// In-process host for the Virtual AGC emulator core (see agc_host.h).
//
// The core (agc_engine.c, agc_engine_init.c, agc_utilities.c) expects the
// host program to provide a handful of functions; they are implemented here.

#include <stdio.h>
#include <string.h>

#include "yaAGC.h"
#include "agc_engine.h"
#include "agc_host.h"
#include "coop.h"

// Generated from the yaYUL output by tools/embed_rope.py.
extern const unsigned char rope_image[];
extern const unsigned int rope_image_len;

static agc_t agc;
static AgcOutputs out;
static DskyState dsky;
static double mct_debt;
static uint64_t mct_count;

// Pending host -> AGC input events.
enum { EV_KEY, EV_PRO };
typedef struct { int type, value; } InputEvent;
#define EVQ_SIZE 64
static InputEvent evq[EVQ_SIZE];
static int evq_head, evq_tail;
// Minimum spacing between delivered events, so each keystroke gets its own
// KEYRUPT handled before the next arrives (~40 ms, a fast human typist).
#define EV_SPACING_MCT 3400
static unsigned evq_cooldown;

// Pending counter increments (PIPA, CDU, THRUST...) delivered one per MCT,
// like the real hardware's cycle stealing.
static int pend[0200];
static unsigned char pend_kind[0200];
static int pend_scan;
static int pend_total;

static void (*out_hook)(int channel, int value);

static void evq_push(int type, int value)
{
    int next = (evq_tail + 1) % EVQ_SIZE;
    if (next == evq_head)
        return;  // full; drop rather than block
    evq[evq_tail].type = type;
    evq[evq_tail].value = value;
    evq_tail = next;
}

// ---------------------------------------------------------------------------
// Rope loading (same layout rules as agc_load_binfile, but from memory).

static int load_rope(agc_t *s, const unsigned char *img, unsigned int len)
{
    if (len & 1 || len / 2 > 36 * 02000)
        return 1;
    s->CheckParity = 0;
    memset(&s->Parities, 0, sizeof s->Parities);
    // File order of banks is 2, 3, 0, 1, 4, 5, ..., 35.
    static const int first_banks[4] = { 2, 3, 0, 1 };
    for (unsigned int i = 0; i < len / 2; i++) {
        unsigned int file_bank = i / 02000, j = i % 02000;
        int bank = file_bank < 4 ? first_banks[file_bank] : (int)file_bank;
        uint16_t raw = (uint16_t)(img[2 * i] << 8 | img[2 * i + 1]);
        uint8_t parity = raw & 1;
        s->Fixed[bank][j] = raw >> 1;
        s->Parities[(bank * 02000 + j) / 32] |= (uint32_t)parity << (j % 32);
        if (parity)
            s->CheckParity = 1;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// DSKY decoding.

static void decode_relay(int row, uint16_t v)
{
    uint8_t left = (v >> 5) & 037, right = v & 037;
    int b = (v & 02000) != 0;
    switch (row) {
    case 11: dsky.prog[0] = left; dsky.prog[1] = right; break;
    case 10: dsky.verb[0] = left; dsky.verb[1] = right; break;
    case 9:  dsky.noun[0] = left; dsky.noun[1] = right; break;
    case 8:  dsky.reg[0][0] = right; break;
    case 7:  dsky.reg[0][1] = left; dsky.reg[0][2] = right;
             dsky.sign[0] = (dsky.sign[0] & ~2) | (b << 1); break;
    case 6:  dsky.reg[0][3] = left; dsky.reg[0][4] = right;
             dsky.sign[0] = (dsky.sign[0] & ~1) | b; break;
    case 5:  dsky.reg[1][0] = left; dsky.reg[1][1] = right;
             dsky.sign[1] = (dsky.sign[1] & ~2) | (b << 1); break;
    case 4:  dsky.reg[1][2] = left; dsky.reg[1][3] = right;
             dsky.sign[1] = (dsky.sign[1] & ~1) | b; break;
    case 3:  dsky.reg[1][4] = left; dsky.reg[2][0] = right; break;
    case 2:  dsky.reg[2][1] = left; dsky.reg[2][2] = right;
             dsky.sign[2] = (dsky.sign[2] & ~2) | (b << 1); break;
    case 1:  dsky.reg[2][3] = left; dsky.reg[2][4] = right;
             dsky.sign[2] = (dsky.sign[2] & ~1) | b; break;
    case 12:  // LM annunciators driven by the software
        dsky.prio_disp   = (v & 0001) != 0;
        dsky.no_dap      = (v & 0002) != 0;
        dsky.vel         = (v & 0004) != 0;
        dsky.no_att      = (v & 0010) != 0;
        dsky.alt         = (v & 0020) != 0;
        dsky.gimbal_lock = (v & 0040) != 0;
        dsky.tracker     = (v & 0200) != 0;
        dsky.prog_lamp   = (v & 0400) != 0;
        break;
    default: break;
    }
}

char agc_digit_char(uint8_t code)
{
    switch (code) {
    case 0:  return ' ';
    case 21: return '0'; case 3:  return '1'; case 25: return '2';
    case 27: return '3'; case 15: return '4'; case 30: return '5';
    case 28: return '6'; case 19: return '7'; case 29: return '8';
    case 31: return '9';
    default: return '?';
    }
}

// ---------------------------------------------------------------------------
// Functions required by the emulator core.

// Rotational hand controller deflections, loaded into RHCP/RHCY/RHCR when the
// software requests a reading (channel 013 bits 8 and 9).
static int16_t rhc_pitch, rhc_yaw, rhc_roll;

// Hand controller deflection, -1..1 per axis; ~42 counts at full deflection.
void agc_host_set_rhc(double pitch, double yaw, double roll)
{
    if (coop_role == COOP_CLIENT) { coop_client_set_rhc(pitch, yaw, roll); return; }
    double v[3] = { pitch, yaw, roll };
    int16_t *dst[3] = { &rhc_pitch, &rhc_yaw, &rhc_roll };
    int out = 0;
    for (int i = 0; i < 3; i++) {
        double x = v[i] > 1 ? 1 : v[i] < -1 ? -1 : v[i];
        if (x > -0.04 && x < 0.04) x = 0;   // detent
        else out = 1;
        int c = (int)(x * 42.0 + (x > 0 ? 0.5 : -0.5));
        *dst[i] = (int16_t)(c >= 0 ? c : ((~(-c)) & 077777));   // ones' complement
    }
    // Channel 031 bit 15 (active low): attitude controller out of detent.
    agc.InputChannel[031] = (int16_t)(out ? (agc.InputChannel[031] & ~040000) : (agc.InputChannel[031] | 040000));
}

void ChannelOutput(agc_t *State, int Channel, int Value)
{
    // Channel 07 is the superbank register, internal to the CPU. The core
    // relies on the host to latch it (as yaAGC's SocketAPI.c does); without
    // this, fixed banks 40-43 are unreachable and Luminary runs bank 30-33
    // code in their place.
    if (Channel == 7) {
        State->InputChannel[7] = State->OutputChannel7 = (int16_t)(Value & 0160);
        return;
    }
    if (Channel == 013 && (Value & 0600) == 0600) {
        State->Erasable[0][042] = rhc_pitch;
        State->Erasable[0][043] = rhc_yaw;
        State->Erasable[0][044] = rhc_roll;
    }
    if (out_hook)
        out_hook(Channel, Value);
    if (Channel < 0 || Channel >= 0200)
        return;  // counter output pulses (0x80 | n) only go to the hook
    out.ch[Channel] = (uint16_t)Value;
    if (Channel == 010) {
        int row = (Value >> 11) & 017;
        out.relay[row] = Value & 03777;
        decode_relay(row, (uint16_t)Value);
    } else if (Channel == 011) {
        dsky.comp_acty   = (Value & 0002) != 0;
        dsky.uplink_acty = (Value & 0004) != 0;
    } else if (Channel == 0163) {
        dsky.temp         = (Value & DSKY_TEMP) != 0;
        dsky.key_rel      = (Value & DSKY_KEY_REL) != 0;
        dsky.vn_flash_off = (Value & DSKY_VN_FLASH) != 0;
        dsky.opr_err      = (Value & DSKY_OPER_ERR) != 0;
        dsky.restart      = (Value & DSKY_RESTART) != 0;
        dsky.stby         = (Value & DSKY_STBY) != 0;
    }
}

// Called once per MCT. Delivers at most one counter increment (returning 1,
// which makes the CPU skip an instruction cycle, as cycle stealing does) or
// one queued DSKY event.
int ChannelInput(agc_t *State)
{
    if (evq_cooldown)
        evq_cooldown--;

    if (pend_total) {
        for (int n = 0; n < 0200; n++) {
            int c = (pend_scan + n) & 0177;
            if (!pend[c])
                continue;
            pend_scan = (c + 1) & 0177;
            int up = pend[c] > 0;
            int type;
            switch (pend_kind[c]) {
            case AGC_PULSE_CDU:      type = up ? 1 : 3; break;
            case AGC_PULSE_CDU_FAST: type = up ? 021 : 023; break;
            case AGC_PULSE_DINC:     type = 4; up = 1; break;
            default:                 type = up ? 0 : 2; break;
            }
            pend[c] += up ? -1 : 1;
            pend_total--;
            UnprogrammedIncrement(State, c, type);
            return 1;
        }
        pend_total = 0;  // inconsistent count; resynchronise
    }

    if (evq_cooldown || evq_head == evq_tail)
        return 0;
    evq_cooldown = EV_SPACING_MCT;
    InputEvent ev = evq[evq_head];
    evq_head = (evq_head + 1) % EVQ_SIZE;
    if (ev.type == EV_KEY) {
        WriteIO(State, 015, ev.value);
        State->InterruptRequests[5] = 1;  // KEYRUPT1
        if (ev.value == KEY_AGC_RSET)
            State->RestartLight = 0;      // hardwired in the real DSKY
    } else if (ev.type == EV_PRO) {
        int v = ReadIO(State, 032);
        v = ev.value ? (v & ~020000) : (v | 020000);
        WriteIO(State, 032, v);
    }
    return 0;
}

void ChannelRoutine(agc_t *State) { (void)State; }

// RequestRadarData() is provided by the vehicle simulation (lm.c).

void ShiftToDeda(agc_t *State, int Data) { (void)State; (void)Data; }

void BacktraceAdd(agc_t *State, int Cause) { (void)State; (void)Cause; }

FILE *rfopen(const char *Filename, const char *mode) { return fopen(Filename, mode); }

// ---------------------------------------------------------------------------
// Public API.

int agc_host_init(void)
{
    memset(&out, 0, sizeof out);
    memset(&dsky, 0, sizeof dsky);
    mct_debt = 0;
    mct_count = 0;
    evq_head = evq_tail = 0;
    evq_cooldown = 0;
    memset(pend, 0, sizeof pend);
    memset(pend_kind, 0, sizeof pend_kind);
    pend_total = 0;
    if (agc_engine_init(&agc, NULL, NULL, 0) != 0)
        return 1;
    return load_rope(&agc, rope_image, rope_image_len);
}

void agc_host_run(double seconds)
{
    mct_debt += seconds * AGC_MCT_PER_SECOND;
    while (mct_debt >= 1.0) {
        agc_engine(&agc);
        mct_count++;
        mct_debt -= 1.0;
    }
}

void agc_host_key(int keycode)
{
    if (coop_role == COOP_CLIENT) { coop_forward(CMD_KEY, keycode, 0); return; }
    evq_push(EV_KEY, keycode);
}
void agc_host_pro(int pressed)
{
    if (coop_role == COOP_CLIENT) { coop_forward(CMD_PRO, pressed, 0); return; }
    evq_push(EV_PRO, pressed);
}

const DskyState *agc_host_dsky(void) { return &dsky; }
void agc_host_set_dsky(const DskyState *d) { dsky = *d; }
const AgcOutputs *agc_host_outputs(void) { return &out; }
uint64_t agc_host_cycles(void) { return mct_count; }

double agc_host_clock(void)
{
    // TIME2 (024) holds the high 14 bits, TIME1 (025) the low 14 bits.
    long t2 = agc.Erasable[0][024] & 037777, t1 = agc.Erasable[0][025] & 037777;
    return (double)(t2 << 14 | t1) / 100.0;
}

int agc_host_read_channel(int channel)
{
    if (channel < 0 || channel >= NUM_CHANNELS)
        return 0;
    return agc.InputChannel[channel] & 077777;
}

void agc_host_set_output_hook(void (*fn)(int channel, int value)) { out_hook = fn; }

void agc_host_pulse(int counter, int count, int kind)
{
    counter &= 0177;
    if (pend[counter] && pend_kind[counter] != kind) {
        pend_total -= pend[counter] < 0 ? -pend[counter] : pend[counter];
        pend[counter] = 0;
    }
    pend_kind[counter] = (unsigned char)kind;
    int before = pend[counter] < 0 ? -pend[counter] : pend[counter];
    pend[counter] += count;
    int after = pend[counter] < 0 ? -pend[counter] : pend[counter];
    pend_total += after - before;
}

int agc_host_pending(int counter) { return pend[counter & 0177]; }

void agc_host_set_input_bits(int channel, int mask, int value)
{
    agc.InputChannel[channel] = (int16_t)((agc.InputChannel[channel] & ~mask) | (value & mask));
}

// Debug access to the raw CPU state.
agc_t *agc_host_state(void) { return &agc; }

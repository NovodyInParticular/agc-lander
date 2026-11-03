// In-process host for the Virtual AGC emulator core.
//
// Replaces yaAGC's SocketAPI.c: instead of talking to peripherals over TCP,
// the AGC's i/o channels are read and written directly by the game.
#ifndef AGC_HOST_H
#define AGC_HOST_H

#include <stdint.h>

// AGC timing: one memory cycle time (MCT) is 12 / 1.024 MHz = 11.71875 us.
#define AGC_MCT_PER_SECOND (1024000.0 / 12.0)

// DSKY keycodes as seen by the AGC on input channel 015.
enum {
    KEY_AGC_0 = 020, KEY_AGC_1 = 001, KEY_AGC_2 = 002, KEY_AGC_3 = 003,
    KEY_AGC_4 = 004, KEY_AGC_5 = 005, KEY_AGC_6 = 006, KEY_AGC_7 = 007,
    KEY_AGC_8 = 010, KEY_AGC_9 = 011,
    KEY_AGC_VERB = 021, KEY_AGC_RSET = 022, KEY_AGC_KEYREL = 031,
    KEY_AGC_PLUS = 032, KEY_AGC_MINUS = 033, KEY_AGC_ENTR = 034,
    KEY_AGC_CLR = 036, KEY_AGC_NOUN = 037,
};

// Decoded DSKY state, updated from AGC output channels.
typedef struct {
    // 5-bit seven-segment codes, as sent by the AGC (0 = blank).
    uint8_t prog[2], verb[2], noun[2];
    uint8_t reg[3][5];
    // Register signs: bit 1 = plus, bit 0 = minus (as in yaDSKY2).
    uint8_t sign[3];

    // Annunciator lamps.
    int comp_acty, uplink_acty, temp, no_att, gimbal_lock, stby, prog_lamp,
        key_rel, restart, opr_err, tracker, prio_disp, alt, no_dap, vel;
    int vn_flash_off;   // 1 while verb/noun digits are blanked by flashing
} DskyState;

// Raw view of the AGC's last-written output channels.
typedef struct {
    uint16_t relay[16];  // channel 010, indexed by relay row (AAAA)
    uint16_t ch[0200];   // channels 0..0177 (incl. fictitious 0163)
} AgcOutputs;

// Load the embedded rope, initialise the CPU. Returns 0 on success.
int agc_host_init(void);

// Run the AGC for `seconds` of simulated time (fractional MCTs carry over).
void agc_host_run(double seconds);

// Queue a DSKY key (KEY_AGC_*). Delivered via channel 015 + KEYRUPT1.
void agc_host_key(int keycode);

// PRO key is a discrete on channel 032 bit 14 (active low).
void agc_host_pro(int pressed);

const DskyState *agc_host_dsky(void);
void agc_host_set_dsky(const DskyState *d);   // outside-view instance: mirror the host's DSKY
const AgcOutputs *agc_host_outputs(void);
uint64_t agc_host_cycles(void);  // memory cycles (MCT) executed

// AGC clock (TIME2:TIME1, 10 ms ticks) in seconds since power-on/fresh start.
double agc_host_clock(void);

// Current value of an i/o channel as the CPU sees it (inputs and outputs).
int agc_host_read_channel(int channel);

// Called for every AGC channel output, including counter output pulses
// (channel 0x80 | counter with 015 = POUT, 016 = MOUT, 017 = ZOUT) and the
// fictitious IMU channels 0174-0177.
void agc_host_set_output_hook(void (*fn)(int channel, int value));

// Counter input pulses, delivered one per memory cycle.
enum {
    AGC_PULSE_PINC = 0,   // count > 0: PINC, < 0: MINC (PIPAs, timers)
    AGC_PULSE_CDU,        // PCDU / MCDU, slow rate
    AGC_PULSE_CDU_FAST,   // PCDU / MCDU, fast rate
    AGC_PULSE_DINC,       // count = number of DINC requests (THRUST, ALTM...)
};
void agc_host_pulse(int counter, int count, int kind);
int agc_host_pending(int counter);

// Rotational hand controller (ACA), -1..1 per axis (pitch=Q, yaw=P, roll=R).
void agc_host_set_rhc(double pitch, double yaw, double roll);

// Set discrete input bits: channel = (channel & ~mask) | (value & mask).
void agc_host_set_input_bits(int channel, int mask, int value);

// Convert a 5-bit DSKY digit code to '0'..'9', or ' ' for blank, '?' for other.
char agc_digit_char(uint8_t code);

#endif

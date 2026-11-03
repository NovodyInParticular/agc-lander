// Two-player co-op over UDP. See coop.h.

#include "coop.h"

#include <stdio.h>
#include <string.h>


#include "agc_host.h"
#include "lm.h"
#include "net.h"

int coop_role = COOP_OFF;
static double now_s;
void coop_tick(double t) { now_s = t; }
#define GetTime() now_s

#define CMD_MAGIC 0x41474343u   // client -> host
#define MAXQ 32

typedef struct { unsigned seq; int type, a, b; } Cmd;
typedef struct {
    unsigned magic;
    unsigned version;
    float rhc[3];
    int ncmd;
    Cmd cmd[MAXQ];
} CmdPkt;

static int sock = -1;
static NetAddr peer;
static int have_peer;
static double last_heard;
static char host_name[128];

// Client.
static Cmd queue[MAXQ];
static int nq;
static unsigned next_seq = 1, acked;
static float my_rhc[3];

// Host.
static unsigned applied;
static float peer_rhc[3];

#define PROTO_VERSION 2   // 2: landing result + hints in StatePkt

int coop_start_host(void)
{
    coop_stop();
    sock = net_socket(COOP_PORT, 1);
    if (sock < 0) return -1;
    coop_role = COOP_HOST;
    have_peer = 0;
    applied = 0;
    return 0;
}

int coop_start_client(const char *host)
{
    coop_stop();
    if (net_resolve(host, COOP_PORT, &peer) != 0) return -1;
    sock = net_socket(0, 1);
    if (sock < 0) return -1;
    snprintf(host_name, sizeof host_name, "%s", host);
    have_peer = 1;
    coop_role = COOP_CLIENT;
    nq = 0;
    next_seq = 1;
    acked = 0;
    last_heard = 0;
    return 0;
}

void coop_stop(void)
{
    if (sock >= 0) net_close_socket(sock);
    sock = -1;
    coop_role = COOP_OFF;
    have_peer = 0;
}

int coop_peer_connected(void) { return have_peer && last_heard > 0 && GetTime() - last_heard < 3.0; }

// --- Client -----------------------------------------------------------------------

void coop_forward(int type, int a, int b)
{
    if (nq >= MAXQ) {   // shouldn't happen; drop the oldest
        memmove(queue, queue + 1, sizeof(Cmd) * (MAXQ - 1));
        nq--;
    }
    queue[nq++] = (Cmd){ next_seq++, type, a, b };
}

void coop_client_set_rhc(double p, double y, double r)
{
    my_rhc[0] = (float)p;
    my_rhc[1] = (float)y;
    my_rhc[2] = (float)r;
}

void coop_client_ack(unsigned ack)
{
    acked = ack;
    int k = 0;
    for (int i = 0; i < nq; i++)
        if (queue[i].seq > acked) queue[k++] = queue[i];
    nq = k;
}

void coop_client_send(void)
{
    if (coop_role != COOP_CLIENT) return;
    CmdPkt p;
    memset(&p, 0, sizeof p);
    p.magic = CMD_MAGIC;
    p.version = PROTO_VERSION;
    memcpy(p.rhc, my_rhc, sizeof p.rhc);
    p.ncmd = nq;
    memcpy(p.cmd, queue, sizeof(Cmd) * nq);
    net_sendto(sock, &peer, &p, sizeof p);
}

int coop_client_poll_state(void *state, int max)
{
    if (coop_role != COOP_CLIENT) return 0;
    static unsigned char buf[8192];
    int n, got = 0;
    NetAddr from;
    while ((n = net_recvfrom(sock, buf, sizeof buf, &from)) > 0)
        if (n <= max) {
            memcpy(state, buf, n);
            got = n;
        }
    if (got) last_heard = GetTime();
    return got;
}

// --- Host -------------------------------------------------------------------------

static void apply(const Cmd *c)
{
    switch (c->type) {
    case CMD_KEY: agc_host_key(c->a); break;
    case CMD_PRO: agc_host_pro(c->a); break;
    case CMD_ENGARM: lm_set_engine_arm(c->a); break;
    case CMD_ATTMODE: lm_set_attitude_mode(c->a); break;
    case CMD_ROD: lm_rod_click(c->a); break;
    case CMD_STOP: lm_engine_stop(); break;
    case CMD_THC: lm_set_thc(c->a, c->b); break;
    }
}

void coop_host_poll(void)
{
    if (coop_role != COOP_HOST) return;
    CmdPkt p;
    NetAddr from;
    int n;
    while ((n = net_recvfrom(sock, &p, sizeof p, &from)) > 0) {
        if (n != (int)sizeof p || p.magic != CMD_MAGIC || p.version != PROTO_VERSION) continue;
        if (!have_peer || from.ip != peer.ip || from.port != peer.port) {
            peer = from;
            have_peer = 1;
            applied = 0;
        }
        last_heard = GetTime();
        memcpy(peer_rhc, p.rhc, sizeof peer_rhc);
        if (p.ncmd < 0 || p.ncmd > MAXQ) continue;
        for (int i = 0; i < p.ncmd; i++)
            if (p.cmd[i].seq > applied) {
                apply(&p.cmd[i]);
                applied = p.cmd[i].seq;
            }
    }
    if (!coop_peer_connected()) memset(peer_rhc, 0, sizeof peer_rhc);
}

void coop_host_rhc(double *p, double *y, double *r)
{
    *p += peer_rhc[0];
    *y += peer_rhc[1];
    *r += peer_rhc[2];
}

unsigned coop_host_ack(void) { return applied; }

void coop_host_send_state(const void *state, int len)
{
    if (coop_role != COOP_HOST || !have_peer) return;
    net_sendto(sock, &peer, state, len);
}

const char *coop_status(void)
{
    static char b[160];
    if (coop_role == COOP_HOST && coop_peer_connected())
        snprintf(b, sizeof b, "CO-OP  you: CDR   LMP connected (%s)", net_addr_str(&peer));
    else if (coop_role == COOP_HOST)
        snprintf(b, sizeof b, "CO-OP  you: CDR   waiting for the LMP (UDP port %d)", COOP_PORT);
    else if (coop_role == COOP_CLIENT)
        snprintf(b, sizeof b, coop_peer_connected() ? "CO-OP  you: LMP   connected to %s" : "CO-OP  you: LMP   connecting to %s ...", host_name);
    else
        b[0] = 0;
    return b;
}

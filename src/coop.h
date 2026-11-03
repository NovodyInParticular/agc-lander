// Two-player co-op: the host is the Commander (runs the AGC and the vehicle),
// the joining player is the LM Pilot (DSKY, panels, switches). The LMP's
// actions are forwarded to the host reliably; the host streams the state back.
#ifndef COOP_H
#define COOP_H

enum { COOP_OFF, COOP_HOST, COOP_CLIENT };
extern int coop_role;

enum { CMD_KEY = 1, CMD_PRO, CMD_ENGARM, CMD_ATTMODE, CMD_ROD, CMD_STOP, CMD_THC };

int coop_start_host(void);                 // 0 on success
int coop_start_client(const char *host);   // 0 on success
void coop_stop(void);
void coop_tick(double now_seconds);       // call once per frame with the wall clock

// Client side: queue an action for the host (called from the lm/agc setters).
void coop_forward(int type, int a, int b);
void coop_client_set_rhc(double pitch, double yaw, double roll);

// Host side, per frame: apply the LMP's actions; the LMP's hand-controller deflection.
void coop_host_poll(void);
void coop_host_rhc(double *pitch, double *yaw, double *roll);
int coop_peer_connected(void);             // heard from the other side recently
unsigned coop_host_ack(void);              // last command applied (goes into the state)

// State stream (opaque bytes; StatePkt lives in main.c).
void coop_host_send_state(const void *state, int len);
int coop_client_poll_state(void *state, int max);   // bytes of the newest state, 0 if none
void coop_client_ack(unsigned ack);
void coop_client_send(void);                         // inputs + unacked commands, every frame

const char *coop_status(void);             // one line for the HUD

#endif

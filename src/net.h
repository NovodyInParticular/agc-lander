// Minimal non-blocking UDP: the second-monitor outside view and two-player co-op.
#ifndef NET_H
#define NET_H

#define NET_PORT 47011     // outside view (localhost)
#define COOP_PORT 47012    // two-player co-op (LAN / internet)

typedef struct { unsigned int ip; unsigned short port; } NetAddr;   // network byte order

int net_socket(int port, int any_interface);   // handle or -1; port 0 = ephemeral
int net_resolve(const char *host, int port, NetAddr *out);
void net_sendto(int h, const NetAddr *to, const void *data, int len);
int net_recvfrom(int h, void *buf, int max, NetAddr *from);   // 0 when nothing waiting
void net_close_socket(int h);
const char *net_addr_str(const NetAddr *a);

// Outside-view link.
int net_open_sender(const char *host, int port);
void net_send(const void *data, int len);
int net_open_listener(int port);   // binds 127.0.0.1:port
int net_recv(void *buf, int max);
void net_close(void);

#endif

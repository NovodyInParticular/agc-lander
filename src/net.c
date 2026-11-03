// Minimal non-blocking UDP, Linux and Windows.

#include "net.h"

#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET sock_t;
#define BADSOCK INVALID_SOCKET
static void net_init(void)
{
    static int ready;
    if (!ready) {
        WSADATA w;
        WSAStartup(MAKEWORD(2, 2), &w);
        ready = 1;
    }
}
static void set_nonblock(sock_t s) { u_long one = 1; ioctlsocket(s, FIONBIO, &one); }
#define CLOSESOCK closesocket
typedef int socklen_t_;
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int sock_t;
#define BADSOCK (-1)
static void net_init(void) {}
static void set_nonblock(sock_t s) { fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK); }
#define CLOSESOCK close
typedef socklen_t socklen_t_;
#endif

#define MAXS 4
static sock_t socks[MAXS] = { BADSOCK, BADSOCK, BADSOCK, BADSOCK };

int net_socket(int port, int any_interface)
{
    net_init();
    int h = -1;
    for (int i = 0; i < MAXS; i++)
        if (socks[i] == BADSOCK) { h = i; break; }
    if (h < 0) return -1;
    sock_t s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s == BADSOCK) return -1;
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((unsigned short)port);
    a.sin_addr.s_addr = htonl(any_interface ? INADDR_ANY : INADDR_LOOPBACK);
    if (bind(s, (struct sockaddr *)&a, sizeof a) != 0) {
        CLOSESOCK(s);
        return -1;
    }
    set_nonblock(s);
    socks[h] = s;
    return h;
}

int net_resolve(const char *host, int port, NetAddr *out)
{
    net_init();
    struct addrinfo hints, *res = NULL;
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    char ps[16];
    snprintf(ps, sizeof ps, "%d", port);
    if (getaddrinfo(host, ps, &hints, &res) != 0 || !res) return -1;
    struct sockaddr_in *sa = (struct sockaddr_in *)res->ai_addr;
    out->ip = sa->sin_addr.s_addr;
    out->port = sa->sin_port;
    freeaddrinfo(res);
    return 0;
}

void net_sendto(int h, const NetAddr *to, const void *data, int len)
{
    if (h < 0 || socks[h] == BADSOCK) return;
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = to->ip;
    a.sin_port = to->port;
    sendto(socks[h], (const char *)data, len, 0, (struct sockaddr *)&a, sizeof a);
}

int net_recvfrom(int h, void *buf, int max, NetAddr *from)
{
    if (h < 0 || socks[h] == BADSOCK) return 0;
    struct sockaddr_in a;
    socklen_t_ al = sizeof a;
    int n = (int)recvfrom(socks[h], (char *)buf, max, 0, (struct sockaddr *)&a, &al);
    if (n <= 0) return 0;
    if (from) { from->ip = a.sin_addr.s_addr; from->port = a.sin_port; }
    return n;
}

void net_close_socket(int h)
{
    if (h < 0 || socks[h] == BADSOCK) return;
    CLOSESOCK(socks[h]);
    socks[h] = BADSOCK;
}

const char *net_addr_str(const NetAddr *a)
{
    static char b[32];
    const unsigned char *p = (const unsigned char *)&a->ip;
    snprintf(b, sizeof b, "%u.%u.%u.%u", p[0], p[1], p[2], p[3]);
    return b;
}

// --- Outside-view link (localhost) ---------------------------------------------

static int tx = -1, rx = -1;
static NetAddr tx_addr;

int net_open_sender(const char *host, int port)
{
    if (tx < 0) tx = net_socket(0, 0);
    if (tx < 0) return -1;
    return net_resolve(host, port, &tx_addr);
}

void net_send(const void *data, int len) { net_sendto(tx, &tx_addr, data, len); }

int net_open_listener(int port)
{
    rx = net_socket(port, 0);
    return rx < 0 ? -1 : 0;
}

int net_recv(void *buf, int max) { return net_recvfrom(rx, buf, max, NULL); }

void net_close(void)
{
    net_close_socket(tx);
    net_close_socket(rx);
    tx = rx = -1;
}

#include <svdpi.h>
#include <winsock2.h>
#include <stdio.h>

#pragma comment(lib, "ws2_32.lib")

static SOCKET server_fd = INVALID_SOCKET;
static SOCKET client_fd = INVALID_SOCKET;

extern "C" void jtag_server_init(int port) {
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    
    bind(server_fd, (struct sockaddr *)&addr, sizeof(addr));
    listen(server_fd, 3);
    
    // Set non-blocking mode cho Server Socket
    u_long mode = 1;
    ioctlsocket(server_fd, FIONBIO, &mode);
    
    printf("[JTAG DPI] Waiting for OpenOCD connection at port %d (Non-blocking)...\n", port);
}

extern "C" void jtag_tick(unsigned char *tms, unsigned char *tdi, unsigned char *tck, unsigned char tdo) {
    // Nếu chưa có client, thử accept
    if (client_fd == INVALID_SOCKET) {
        client_fd = accept(server_fd, NULL, NULL);
        if (client_fd != INVALID_SOCKET) {
            // Set non-blocking cho Client Socket
            u_long mode = 1;
            ioctlsocket(client_fd, FIONBIO, &mode);
            printf("[JTAG DPI] OpenOCD connected successfully!\n");
        }
        return; 
    }

    char cmd;
    int res = recv(client_fd, &cmd, 1, 0);
    
    if (res > 0) {
        if (cmd >= '0' && cmd <= '7') {
            int v = cmd - '0';
            *tdi = v & 1; 
            *tms = (v >> 1) & 1; 
            *tck = (v >> 2) & 1;
        } else if (cmd == 'R') {
            char resp = tdo ? '1' : '0';
            send(client_fd, &resp, 1, 0);
        } else if (cmd == 'Q') {
            printf("[JTAG DPI] OpenOCD send stop.\n");
            closesocket(client_fd);
            client_fd = INVALID_SOCKET;
        }
    } else if (res == 0 || (res < 0 && WSAGetLastError() != WSAEWOULDBLOCK)) {
        printf("[JTAG DPI] Lost connection with OpenOCD.\n");
        closesocket(client_fd);
        client_fd = INVALID_SOCKET;
    }
}
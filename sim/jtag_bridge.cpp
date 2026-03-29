#include <svdpi.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>

#pragma comment(lib, "ws2_32.lib")

static SOCKET server_fd = INVALID_SOCKET;
static SOCKET client_fd = INVALID_SOCKET;

extern "C" void jtag_server_init(int port) {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return;

    server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    bind(server_fd, (struct sockaddr *)&addr, sizeof(addr));
    listen(server_fd, 1);
    unsigned long mode = 1;
    ioctlsocket(server_fd, FIONBIO, &mode);
    printf("[JTAG DPI] Server listening on port %d...\n", port);
}

extern "C" void jtag_tick(unsigned char *tms, unsigned char *tdi, unsigned char *tck, unsigned char tdo) {
    if (client_fd == INVALID_SOCKET) {
        struct sockaddr_in client_addr;
        int addr_len = sizeof(client_addr);
        client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &addr_len);
        if (client_fd != INVALID_SOCKET) {
            unsigned long mode = 1;
            ioctlsocket(client_fd, FIONBIO, &mode);
            printf("[JTAG DPI] OpenOCD connected!\n");
        }
        return;
    }

    char cmd;
    int res = recv(client_fd, &cmd, 1, 0);
    if (res > 0) {
        // 1. Lệnh điều khiển tín hiệu (0-7)
        if (cmd >= '0' && cmd <= '7') {
            int v = cmd - '0';
            *tdi = v & 1;
            *tms = (v >> 1) & 1;
            *tck = (v >> 2) & 1;
        } 
        // 2. Lệnh đọc TDO (CHỈ 'R' VIẾT HOA)
        else if (cmd == 'R') {
            char resp = tdo ? '1' : '0';
            send(client_fd, &resp, 1, 0);
        } 
        // 3. Lệnh Reset ('r' viết thường) -> KHÔNG ĐƯỢC SEND PHẢN HỒI
        else if (cmd == 'r') {
            // OpenOCD chỉ muốn reset tín hiệu, không đợi data trả về
            // Nếu ta send ở đây, OpenOCD sẽ đọc nhầm nó là bit đầu tiên của IDCODE
        }
        // 4. Thoát
        else if (cmd == 'Q' || cmd == 'q') {
            closesocket(client_fd);
            client_fd = INVALID_SOCKET;
        }
    } else if (res == 0) {
        closesocket(client_fd);
        client_fd = INVALID_SOCKET;
    }
}
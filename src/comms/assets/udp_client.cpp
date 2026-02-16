
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>   // std::stoi
#include <cstdio>   // perror

int main(int argc, char** argv) {
    const char* server_ip = (argc >= 2) ? argv[1] : "127.0.0.1";
    const char* message   = (argc >= 3) ? argv[2] : "hello";
    int local_port        = (argc >= 4) ? std::stoi(argv[3]) : 0;

    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return 1; }

    // Optional: bind source port (so sender.sin_port becomes predictable)
    if (local_port > 0) {
        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_port = htons(static_cast<uint16_t>(local_port));
        local.sin_addr.s_addr = htonl(INADDR_ANY);

        if (::bind(sock, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
            perror("bind (client local port)");
            ::close(sock);
            return 1;
        }
    }

    // Destination (server) address — ALWAYS needed
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(5000);
    if (::inet_pton(AF_INET, server_ip, &dst.sin_addr) != 1) {
        std::cerr << "inet_pton failed for " << server_ip << "\n";
        ::close(sock);
        return 1;
    }

    // Send
    const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(message);
    std::size_t len = std::strlen(message);

    ssize_t sent = ::sendto(sock, data, len, 0,
                            reinterpret_cast<const sockaddr*>(&dst), sizeof(dst));
    if (sent < 0) { perror("sendto"); ::close(sock); return 1; }

    std::cout << "Sent " << sent << " bytes to " << server_ip << ":5000"
              << " (src port " << local_port << ")\n";

    // Optional receive (echo)
    std::uint8_t buffer[2048];
    sockaddr_in from{};
    socklen_t from_len = sizeof(from);

    ssize_t n = ::recvfrom(sock, buffer, sizeof(buffer), 0,
                           reinterpret_cast<sockaddr*>(&from), &from_len);
    if (n < 0) { perror("recvfrom"); ::close(sock); return 1; }

    char ip[INET_ADDRSTRLEN]{};
    ::inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
    std::cout << "Reply from " << ip << ":" << ntohs(from.sin_port) << "\n";
    std::cout << "Reply: " << std::string(reinterpret_cast<char*>(buffer),
                                          reinterpret_cast<char*>(buffer) + n)
              << "\n";

    ::close(sock);
    return 0;
}

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <string>

#include "test_payloads/imu_payload.hpp"
#include "test_payloads/gps_payload.hpp"
#include "test_payloads/ph_payload.hpp"

static PBEnvelope build_envelope(const std::string& type) {
  if (type == "gps") return make_gps_envelope();
  if (type == "ph")  return make_ph_envelope();
  return make_imu_envelope(); // default
}

int main(int argc, char** argv) {
  const char* server_ip      = (argc >= 2) ? argv[1] : "127.0.0.1";
  int         local_port     = (argc >= 3) ? std::stoi(argv[2]) : 0;
  std::string payload_type   = (argc >= 4) ? argv[3] : "imu";

  int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) { perror("socket"); return 1; }

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

  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  dst.sin_port = htons(5000);
  if (::inet_pton(AF_INET, server_ip, &dst.sin_addr) != 1) {
    std::cerr << "inet_pton failed for " << server_ip << "\n";
    ::close(sock);
    return 1;
  }

  PBEnvelope envelope = build_envelope(payload_type);

  std::string payload;
  if (!envelope.SerializeToString(&payload)) {
    std::cerr << "SerializeToString failed\n";
    ::close(sock);
    return 1;
  }

  ssize_t sent = ::sendto(sock, payload.data(), payload.size(), 0,
                          reinterpret_cast<const sockaddr*>(&dst), sizeof(dst));
  if (sent < 0) { perror("sendto"); ::close(sock); return 1; }

  std::cout << "Sent " << sent << " bytes (PBEnvelope/" << payload_type
            << ") to " << server_ip << ":5000\n";

  ::close(sock);
  return 0;
}

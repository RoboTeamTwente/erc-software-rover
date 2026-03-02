#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "comms/udp/packet_header.hpp"

#include "components/sensorboard/imu_sensor.pb.h"
#include "components/common/sensor.pb.h"

static constexpr uint16_t MSG_TYPE_IMU = 1;

int main(int argc, char** argv) {
  const char* server_ip = (argc >= 2) ? argv[1] : "127.0.0.1";
  int local_port        = (argc >= 3) ? std::stoi(argv[2]) : 0;

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

  // ---- Construct protobuf ----
  IMUSensorInformation imu;
  imu.set_accel_x(0.12f);
  imu.set_accel_y(-9.81f);
  imu.set_accel_z(0.05f);

  imu.set_gyro_x(0.01f);
  imu.set_gyro_y(0.02f);
  imu.set_gyro_z(0.03f);

  imu.set_mag_x(30.0f);
  imu.set_mag_y(1.5f);
  imu.set_mag_z(-44.2f);

  imu.set_is_calibrated(true);
  imu.set_state(SENSOR_OPERATING);
  imu.set_error_code(IMU_NO_ERROR);

  // Serialize protobuf
  std::string payload;
  if (!imu.SerializeToString(&payload)) {
    std::cerr << "SerializeToString failed\n";
    ::close(sock);
    return 1;
  }

  if (payload.size() > 0xFFFF) {
    std::cerr << "Payload too large for uint16 payload_len: " << payload.size() << "\n";
    ::close(sock);
    return 1;
  }

  // ---- Build header (network byte order) ----
  static uint32_t seq_host = 0;
  PacketHeader hdr{};
  hdr.msg_type = htons(MSG_TYPE_IMU);
  hdr.payload_length = htons(static_cast<uint16_t>(payload.size()));
  hdr.seq = htonl(seq_host++);

  // ---- Build datagram = header + payload ----
  std::vector<uint8_t> datagram(sizeof(PacketHeader) + payload.size());
  std::memcpy(datagram.data(), &hdr, sizeof(PacketHeader));
  std::memcpy(datagram.data() + sizeof(PacketHeader), payload.data(), payload.size());

  // Send framed packet
  ssize_t sent = ::sendto(sock, datagram.data(), datagram.size(), 0,
                          reinterpret_cast<const sockaddr*>(&dst), sizeof(dst));
  if (sent < 0) { perror("sendto"); ::close(sock); return 1; }

  std::cout << "Sent " << sent << " bytes (hdr " << sizeof(PacketHeader)
            << " + payload " << payload.size() << ") to "
            << server_ip << ":5000 (src port " << local_port << ")\n";

  ::close(sock);
  return 0;
}

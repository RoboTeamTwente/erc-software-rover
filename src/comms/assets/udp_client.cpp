#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

#include "components/sensorboard/imu_sensor.pb.h"
#include "components/common/sensor.pb.h"

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

  // Construct protobuf
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

  // Serialize
  std::string bytes;
  if (!imu.SerializeToString(&bytes)) {
    std::cerr << "SerializeToString failed\n";
    ::close(sock);
    return 1;
  }

  // Send protobuf bytes
  ssize_t sent = ::sendto(sock, bytes.data(), bytes.size(), 0,
                          reinterpret_cast<const sockaddr*>(&dst), sizeof(dst));
  if (sent < 0) { perror("sendto"); ::close(sock); return 1; }

  std::cout << "Sent " << sent << " protobuf bytes to " << server_ip << ":5000"
            << " (src port " << local_port << ")\n";

  ::close(sock);
  return 0;
}

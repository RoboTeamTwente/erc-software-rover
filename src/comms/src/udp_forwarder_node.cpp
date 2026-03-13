#include <rclcpp/rclcpp.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <thread>
#include <vector>
#include <cstdint>
#include <stdexcept>
#include <cstring>

#include <unordered_map>
#include <memory>

// Envelope + handlers
#include "components/common/envelope.pb.h"
#include "comms/udp/handler.hpp"
#include "comms/udp/handlers/imu_handler.hpp"

class UdpForwarderNode : public rclcpp::Node {
public:
  UdpForwarderNode() : Node("udp_forwarder"), running_(true) {
    listen_port_ = this->declare_parameter<int>("listen_port", 5000);
    dst_a_port_  = this->declare_parameter<int>("dst_a_port", 6000);
    dst_b_port_  = this->declare_parameter<int>("dst_b_port", 6001);
    dst_ip_      = this->declare_parameter<std::string>("dst_ip", "127.0.0.1");

    sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) throw std::runtime_error("socket() failed");

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(static_cast<uint16_t>(listen_port_));
    local.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(sock_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0)
      throw std::runtime_error("bind() failed");

    dstA_ = make_dst(dst_ip_, dst_a_port_);
    dstB_ = make_dst(dst_ip_, dst_b_port_);

    // Register handlers keyed by PBEnvelope::PayloadCase
    handlers_.emplace(
      static_cast<int>(PBEnvelope::kImuInfo),
      std::make_unique<ImuHandler>(this, "imu_data", 10));

    rx_thread_ = std::thread([this] { this->rx_loop(); });

    RCLCPP_INFO(get_logger(),
      "Listening on UDP :%d, forwarding to %s:%d and %s:%d",
      listen_port_, dst_ip_.c_str(), dst_a_port_, dst_ip_.c_str(), dst_b_port_);
  }

  ~UdpForwarderNode() override {
    running_ = false;
    if (sock_ >= 0) ::close(sock_);
    if (rx_thread_.joinable()) rx_thread_.join();
  }

private:
  static sockaddr_in make_dst(const std::string& ip, int port) {
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, ip.c_str(), &dst.sin_addr) != 1)
      throw std::runtime_error("inet_pton failed");
    return dst;
  }

  void rx_loop() {
    std::vector<std::uint8_t> buffer(2048);

    while (running_ && rclcpp::ok()) {
      sockaddr_in sender{};
      socklen_t sender_len = sizeof(sender);

      ssize_t n = ::recvfrom(sock_, buffer.data(), buffer.size(), 0,
                             reinterpret_cast<sockaddr*>(&sender), &sender_len);

      if (n < 0) {
        if (!running_) break;
        continue;
      }

      // Forward raw datagram (downstream consumers also speak PBEnvelope)
      ::sendto(sock_, buffer.data(), n, 0,
               reinterpret_cast<sockaddr*>(&dstA_), sizeof(dstA_));
      ::sendto(sock_, buffer.data(), n, 0,
               reinterpret_cast<sockaddr*>(&dstB_), sizeof(dstB_));

      // Parse the datagram as a PBEnvelope
      PBEnvelope envelope;
      if (!envelope.ParseFromArray(buffer.data(), static_cast<int>(n))) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Failed to parse PBEnvelope (%zd bytes)", n);
        continue;
      }

      const int payload_case = static_cast<int>(envelope.payload_case());

      if (payload_case == 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Received PBEnvelope with no payload set");
        continue;
      }

      auto it = handlers_.find(payload_case);
      if (it == handlers_.end()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "No handler for PBEnvelope payload_case=%d", payload_case);
        continue;
      }

      it->second->handle(envelope);
    }
  }

  int sock_{-1};
  std::atomic<bool> running_;
  std::thread rx_thread_;

  int listen_port_{5000}, dst_a_port_{6000}, dst_b_port_{6001};
  std::string dst_ip_{"127.0.0.1"};
  sockaddr_in dstA_{}, dstB_{};

  std::unordered_map<int, std::unique_ptr<Handler>> handlers_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UdpForwarderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

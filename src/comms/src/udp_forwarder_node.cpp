#include <rclcpp/rclcpp.hpp> // gives rclcpp::Node, rclcpp::init, etc.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <thread>
#include <vector>

// define a node that listens on a UDP port and forwards received packets to two other UDP ports
class UdpForwarderNode : public rclcpp::Node {
public:
  UdpForwarderNode() : Node("udp_forwarder"), running_(true) {
    // parameters (so you don’t hardcode ports)
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

    // build destinations
    dstA_ = make_dst(dst_ip_, dst_a_port_);
    dstB_ = make_dst(dst_ip_, dst_b_port_);

    // start receiver thread
    rx_thread_ = std::thread([this] { this->rx_loop(); });

    RCLCPP_INFO(get_logger(), "Listening on UDP :%d, forwarding to %s:%d and %s:%d",
                listen_port_, dst_ip_.c_str(), dst_a_port_, dst_ip_.c_str(), dst_b_port_);
  }

  ~UdpForwarderNode() override {
    running_ = false;
    if (sock_ >= 0) ::close(sock_);     // unblocks recvfrom
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
        if (!running_) break; // likely unblocked by close()
        continue;
      }

      // forward
      ::sendto(sock_, buffer.data(), n, 0,
               reinterpret_cast<sockaddr*>(&dstA_), sizeof(dstA_));
      ::sendto(sock_, buffer.data(), n, 0,
               reinterpret_cast<sockaddr*>(&dstB_), sizeof(dstB_));

      // later: parse protobuf, publish ROS topics, etc.
    }
  }

  int sock_{-1};
  std::atomic<bool> running_;
  std::thread rx_thread_;

  int listen_port_{5000}, dst_a_port_{6000}, dst_b_port_{6001};
  std::string dst_ip_{"127.0.0.1"};
  sockaddr_in dstA_{}, dstB_{};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UdpForwarderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

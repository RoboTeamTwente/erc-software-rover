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
#include "comms/udp/handlers/sensor_board/imu_handler.hpp"
#include "comms/udp/handlers/sensor_board/gps_handler.hpp"
#include "comms/udp/handlers/sensor_board/ph_handler.hpp"
#include "comms/udp/handlers/sensor_board/diagnostics_handler.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rtt_behaviour/action/collect_task.hpp"
#include "rtt_behaviour/action/hardware_command.hpp"

class UdpForwarderNode : public rclcpp::Node {
public:

using CollectTask     = rtt_behaviour::action::CollectTask;
  using HardwareCommand = rtt_behaviour::action::HardwareCommand;
  using GoalHandleHW    = rclcpp_action::ServerGoalHandle<HardwareCommand>;

  UdpForwarderNode() : Node("udp_forwarder"), running_(true) {
    bt_client_ = rclcpp_action::create_client<CollectTask>(
      this, "bt_collect_task");

    hw_server_ = rclcpp_action::create_server<HardwareCommand>(
      this, "hardware_command",
      [this](auto uuid, auto goal) {
        (void)uuid; (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](auto handle) {
        (void)handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](auto handle) { handle_hw_accepted(handle); }
    );

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
    handlers_.emplace(
      static_cast<int>(PBEnvelope::kGpsInfo),
      std::make_unique<GpsHandler>(this, "gps_data", 10));
    handlers_.emplace(
      static_cast<int>(PBEnvelope::kPhInfo),
      std::make_unique<PhHandler>(this, "ph_data", 10));
    handlers_.emplace(
      static_cast<int>(PBEnvelope::kSensorDiag),
      std::make_unique<DiagnosticsHandler>(this, "sensor_board_diagnostics", 10));

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

void on_task_received(const std::string& task_name,
                        const std::string& target_id) {
    if (!bt_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "BT Manager action server not available");
      return;
    }

    auto goal = CollectTask::Goal{};
    goal.task_name = task_name;
    goal.target_id = target_id;

    auto opts = rclcpp_action::Client<CollectTask>::SendGoalOptions{};
    opts.result_callback = [this](const auto& result) {
      RCLCPP_INFO(get_logger(), "BT task finished: %s — %s",
        result.result->success ? "SUCCESS" : "FAILURE",
        result.result->message.c_str());
      //TODO send the result back to base station
    };

    bt_client_->async_send_goal(goal, opts);
  }
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
#include "comms/udp/handlers/sensor_board/imu_handler.hpp"
#include "comms/udp/handlers/sensor_board/gps_handler.hpp"
#include "comms/udp/handlers/sensor_board/ph_handler.hpp"
#include "comms/udp/handlers/sensor_board/diagnostics_handler.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rtt_behaviour/action/collect_task.hpp"
#include "rtt_behaviour/action/hardware_command.hpp"

class UdpForwarderNode : public rclcpp::Node {
public:

using CollectTask     = rtt_behaviour::action::CollectTask;
  using HardwareCommand = rtt_behaviour::action::HardwareCommand;
  using GoalHandleHW    = rclcpp_action::ServerGoalHandle<HardwareCommand>;

  UdpForwarderNode() : Node("udp_forwarder"), running_(true) {
    bt_client_ = rclcpp_action::create_client<CollectTask>(
      this, "bt_collect_task");

    hw_server_ = rclcpp_action::create_server<HardwareCommand>(
      this, "hardware_command",
      [this](auto uuid, auto goal) {
        (void)uuid; (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](auto handle) {
        (void)handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](auto handle) { handle_hw_accepted(handle); }
    );

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
    handlers_.emplace(
      static_cast<int>(PBEnvelope::kGpsInfo),
      std::make_unique<GpsHandler>(this, "gps_data", 10));
    handlers_.emplace(
      static_cast<int>(PBEnvelope::kPhInfo),
      std::make_unique<PhHandler>(this, "ph_data", 10));
    handlers_.emplace(
      static_cast<int>(PBEnvelope::kSensorDiag),
      std::make_unique<DiagnosticsHandler>(this, "sensor_board_diagnostics", 10));

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

void on_task_received(const std::string& task_name,
                        const std::string& target_id) {
    if (!bt_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "BT Manager action server not available");
      return;
    }

    auto goal = CollectTask::Goal{};
    goal.task_name = task_name;
    goal.target_id = target_id;

    auto opts = rclcpp_action::Client<CollectTask>::SendGoalOptions{};

    //receiving feedback
    opts.feedback_callback = [this](
        auto /*goal_handle*/, 
        const std::shared_ptr<const CollectTask::Feedback> feedback) 
    {
        RCLCPP_INFO(this->get_logger(), "BT Progress: %s at %.2f%%", 
                    feedback->current_node.c_str(), 
                    feedback->progress * 100.0);
    };

    //receiving final result
    opts.result_callback = [this](const auto& result) {
      RCLCPP_INFO(get_logger(), "BT task finished: %s — %s",
        result.result->success ? "SUCCESS" : "FAILURE",
        result.result->message.c_str());
      //TODO send the result back to base station
    };

    bt_client_->async_send_goal(goal, opts);
  }

private:
void handle_hw_accepted(
      std::shared_ptr<GoalHandleHW> handle) {
    // Spin off a thread — never block the executor
    std::thread([this, handle]() {
      const auto& goal = handle->get_goal();

      RCLCPP_INFO(get_logger(), "HW command received: [%s] param=%.2f",
        goal->command_type.c_str(), goal->param_float);

     //TODO comms to hardware to comms communication

      // Stub: simulate hardware round-trip latency
      std::this_thread::sleep_for(std::chrono::milliseconds(300));

      auto result = std::make_shared<HardwareCommand::Result>();
      result->success = true;
      result->message = "stub ok";
      handle->succeed(result);
    }).detach();
  }


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

  rclcpp_action::Client<CollectTask>::SharedPtr     bt_client_;
  rclcpp_action::Server<HardwareCommand>::SharedPtr hw_server_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UdpForwarderNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

private:
void handle_hw_accepted(
      std::shared_ptr<GoalHandleHW> handle) {
    // Spin off a thread — never block the executor
    std::thread([this, handle]() {
      const auto& goal = handle->get_goal();

      RCLCPP_INFO(get_logger(), "HW command received: [%s] param=%.2f",
        goal->command_type.c_str(), goal->param_float);

     //TODO comms to hardware to comms communication

      // Stub: simulate hardware round-trip latency
      std::this_thread::sleep_for(std::chrono::milliseconds(300));

      auto result = std::make_shared<HardwareCommand::Result>();
      result->success = true;
      result->message = "stub ok";
      handle->succeed(result);
    }).detach();
  }


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

  rclcpp_action::Client<CollectTask>::SharedPtr     bt_client_;
  rclcpp_action::Server<HardwareCommand>::SharedPtr hw_server_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UdpForwarderNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

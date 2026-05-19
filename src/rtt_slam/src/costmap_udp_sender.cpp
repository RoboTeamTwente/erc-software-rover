#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <vector>
#include <string>
#include <cstring>
#include <mutex>

static constexpr size_t MAX_PAYLOAD_SIZE = 60000;
static constexpr size_t HEADER_SIZE = 16;

class CostmapUDPSender : public rclcpp::Node
{
public:
    CostmapUDPSender(const std::string & remote_ip, int remote_port)
    : Node("costmap_udp_sender"), seq_(0)
    {
        this->declare_parameter<std::string>("remote_ip", remote_ip);
        this->declare_parameter<int>("remote_port", remote_port);
        this->declare_parameter<std::string>("topic", "/local_costmap/costmap");

        remote_ip_   = this->get_parameter("remote_ip").as_string();
        remote_port_ = this->get_parameter("remote_port").as_int();
        topic_       = this->get_parameter("topic").as_string();

        // Create UDP socket
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create UDP socket");
            throw std::runtime_error("socket failed");
        }

        std::memset(&remote_addr_, 0, sizeof(remote_addr_));
        remote_addr_.sin_family = AF_INET;
        remote_addr_.sin_port   = htons(remote_port_);
        inet_pton(AF_INET, remote_ip_.c_str(), &remote_addr_.sin_addr);

        sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            topic_,
            10,
            std::bind(&CostmapUDPSender::costmapCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Streaming %s -> %s:%d",
            topic_.c_str(),
            remote_ip_.c_str(),
            remote_port_
        );
    }

    ~CostmapUDPSender()
    {
        close(sock_);
    }

private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        uint32_t seq;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            seq = seq_++;
        }

        // Serialize message (CDR)
        rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> serializer;
        rclcpp::SerializedMessage serialized_msg;
        serializer.serialize_message(msg.get(), &serialized_msg);

        const uint8_t * raw_data = serialized_msg.get_rcl_serialized_message().buffer;
        size_t raw_size = serialized_msg.get_rcl_serialized_message().buffer_length;

        // Split into chunks
        size_t total_chunks = (raw_size + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE;

        for (size_t i = 0; i < total_chunks; ++i) {
            size_t offset = i * MAX_PAYLOAD_SIZE;
            size_t chunk_size = std::min(MAX_PAYLOAD_SIZE, raw_size - offset);

            std::vector<uint8_t> packet;
            packet.resize(HEADER_SIZE + chunk_size);

            // Header: [seq][total][index][payload_len] (little endian)
            uint32_t header[4] = {
                seq,
                static_cast<uint32_t>(total_chunks),
                static_cast<uint32_t>(i),
                static_cast<uint32_t>(chunk_size)
            };

            std::memcpy(packet.data(), header, HEADER_SIZE);
            std::memcpy(packet.data() + HEADER_SIZE, raw_data + offset, chunk_size);

            ssize_t sent = sendto(
                sock_,
                packet.data(),
                packet.size(),
                0,
                reinterpret_cast<struct sockaddr*>(&remote_addr_),
                sizeof(remote_addr_)
            );

            if (sent < 0) {
                RCLCPP_WARN(this->get_logger(), "UDP send error");
            }
        }

        RCLCPP_DEBUG(
            this->get_logger(),
            "seq=%u chunks=%zu bytes=%zu",
            seq, total_chunks, raw_size
        );
    }

    // ROS
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;

    // Networking
    int sock_;
    struct sockaddr_in remote_addr_;

    // Params
    std::string remote_ip_;
    int remote_port_;
    std::string topic_;

    // State
    std::mutex mutex_;
    uint32_t seq_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Default args (can be overridden by ROS params)
    std::string remote_ip = "145.126.41.72";
    int remote_port = 4500;

    auto node = std::make_shared<CostmapUDPSender>(remote_ip, remote_port);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

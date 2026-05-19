
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/qos.hpp>
#include <vector>
#include <cstdint>

// Costmap cells >= this are treated as obstacles
static constexpr int8_t OBSTACLE_THRESHOLD = 65;
// Value written into global map for permanent obstacles
static constexpr int8_t OCCUPIED_VALUE = 100;

class ObstacleMapMerger : public rclcpp::Node
{
public:
  ObstacleMapMerger()
  : Node("obstacle_map_merger"), map_received_(false)
  {
    // Latched QoS — matches map_server publisher
    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local();
    latched_qos.reliable();

    // Subscribe to static map (latched — received immediately on connect)
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", latched_qos,
      std::bind(&ObstacleMapMerger::mapCallback, this, std::placeholders::_1));

    // Subscribe to local costmap updates
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", 10,
      std::bind(&ObstacleMapMerger::costmapCallback, this, std::placeholders::_1));

    // Publish updated map on /map with latched QoS
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", latched_qos);

    RCLCPP_INFO(get_logger(), "ObstacleMapMerger ready. Waiting for /map...");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (map_received_) {
      // We own /map now — ignore any further map_server publishes
      return;
    }

    RCLCPP_INFO(get_logger(),
      "Global map received: %dx%d @ %.3fm/cell, origin (%.2f, %.2f)",
      msg->info.width, msg->info.height, msg->info.resolution,
      msg->info.origin.position.x, msg->info.origin.position.y);

    global_map_  = *msg;
    global_data_ = std::vector<int8_t>(msg->data.begin(), msg->data.end());
    map_received_ = true;

    // Unsubscribe so map_server can't overwrite our changes
    map_sub_.reset();
  }

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (!map_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Local costmap update received but global map not yet loaded. Skipping.");
      return;
    }

    if (mergeObstacles(*msg)) {
      // Republish updated global map
      global_map_.data = global_data_;
      global_map_.header.stamp = now();
      map_pub_->publish(global_map_);
      RCLCPP_DEBUG(get_logger(), "Global map updated with new permanent obstacles.");
    }
  }

  bool mergeObstacles(const nav_msgs::msg::OccupancyGrid & local_costmap)
  {
    const auto & gm = global_map_.info;
    const auto & lm = local_costmap.info;
    const auto & local_data = local_costmap.data;

    bool any_updated = false;

    for (unsigned int local_idx = 0; local_idx < local_data.size(); ++local_idx) {
      if (local_data[local_idx] < OBSTACLE_THRESHOLD) {
        continue;
      }

      // Convert flat local index → row/col
      const unsigned int local_row = local_idx / lm.width;
      const unsigned int local_col = local_idx % lm.width;

      // Local costmap cell centre in world (map) frame
      const double world_x = lm.origin.position.x + (local_col + 0.5) * lm.resolution;
      const double world_y = lm.origin.position.y + (local_row + 0.5) * lm.resolution;

      // Project into global map cell indices
      const int global_col = static_cast<int>(
        (world_x - gm.origin.position.x) / gm.resolution);
      const int global_row = static_cast<int>(
        (world_y - gm.origin.position.y) / gm.resolution);

      // Bounds check
      if (global_col < 0 || global_col >= static_cast<int>(gm.width) ||
          global_row < 0 || global_row >= static_cast<int>(gm.height))
      {
        continue;
      }

      const unsigned int global_flat =
        static_cast<unsigned int>(global_row) * gm.width +
        static_cast<unsigned int>(global_col);

      if (global_data_[global_flat] != OCCUPIED_VALUE) {
        global_data_[global_flat] = OCCUPIED_VALUE;
        any_updated = true;
      }
    }

    if (any_updated) {
      RCLCPP_INFO(get_logger(), "Permanently marked new obstacle cells in global map.");
    }
    return any_updated;
  }

  // State
  bool map_received_;
  nav_msgs::msg::OccupancyGrid     global_map_;
  std::vector<int8_t>              global_data_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleMapMerger>());
  rclcpp::shutdown();
  return 0;
}

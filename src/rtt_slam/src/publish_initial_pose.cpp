#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <cmath>

// -----------------------------------------------------------------------
// CONFIGURE YOUR HARDCODED START POSE HERE
// Units: metres (x, y) and radians (yaw), in the 'map' frame
//
// How to find these values:
//   - Open your given_map.pgm in GIMP or similar
//   - Find your start pixel and convert:
//       x = pixel_col * resolution + origin_x   (from given_map.yaml)
//       y = (map_height - pixel_row) * resolution + origin_y
//   - Yaw: 0.0 = facing +X, M_PI/2 = facing +Y, M_PI = facing -X
// -----------------------------------------------------------------------
static constexpr double INITIAL_X   =  0.0;   // metres
static constexpr double INITIAL_Y   =  0.0;   // metres
static constexpr double INITIAL_YAW =  0.0;   // radians
// -----------------------------------------------------------------------

class InitialPosePublisher : public rclcpp::Node
{
public:
  InitialPosePublisher()
  : Node("initial_pose_publisher")
  {
    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);

    // Fire once after a short delay so RTAB-Map is ready to receive it
    timer_ = create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&InitialPosePublisher::publishOnce, this));
  }

private:
  void publishOnce()
  {
    timer_->cancel();  // one-shot

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp    = now();
    msg.header.frame_id = "map";

    msg.pose.pose.position.x = INITIAL_X;
    msg.pose.pose.position.y = INITIAL_Y;
    msg.pose.pose.position.z = 0.0;

    // Yaw → quaternion (2D, so only z and w are non-zero)
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = std::sin(INITIAL_YAW / 2.0);
    msg.pose.pose.orientation.w = std::cos(INITIAL_YAW / 2.0);

    // Diagonal covariance [x, y, z, roll, pitch, yaw]
    // Moderate uncertainty: ~0.5 m std dev in position, ~15 deg in yaw
    msg.pose.covariance.fill(0.0);
    msg.pose.covariance[0]  = 0.25;   // x variance
    msg.pose.covariance[7]  = 0.25;   // y variance
    msg.pose.covariance[35] = 0.068;  // yaw variance

    pub_->publish(msg);

    RCLCPP_INFO(get_logger(),
      "Published initial pose: x=%.2f  y=%.2f  yaw=%.3f rad",
      INITIAL_X, INITIAL_Y, INITIAL_YAW);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitialPosePublisher>());
  rclcpp::shutdown();
  return 0;
}

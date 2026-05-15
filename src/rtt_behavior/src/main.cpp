#include <ament_index_cpp/get_package_share_directory.hpp>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "ex.hpp"

#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    rclcpp::init(argc, argv);

// Create the node that will own the BT
    auto node = std::make_shared<rclcpp::Node>("bt_manager");
    BT::BehaviorTreeFactory factory;

    BT::RosNodeParams params;
    params.nh = node;
    // Register all custom nodes
    factory.registerNodeType<MyRobotNodes::Approach>("Approach", params);
    factory.registerNodeType<MyRobotNodes::IsNear>("IsNear");
    factory.registerNodeType<MyRobotNodes::IsGripperOpen>("IsGripperOpen");
    factory.registerNodeType<MyRobotNodes::OpenGripper>("OpenGripper");
    factory.registerNodeType<MyRobotNodes::IsGripperClosed>("IsGripperClosed");
    factory.registerNodeType<MyRobotNodes::CloseGripper>("CloseGripper");
    factory.registerNodeType<MyRobotNodes::ExtractWithGripper>(
        "ExtractWithGripper");
    factory.registerNodeType<MyRobotNodes::IsEmpty>("IsEmpty");
    factory.registerNodeType<MyRobotNodes::Contains>("Contains");
    factory.registerNodeType<MyRobotNodes::IsContainerOpen>("IsContainerOpen");
    factory.registerNodeType<MyRobotNodes::OpenContainer>("OpenContainer");
    factory.registerNodeType<MyRobotNodes::IsContainerClosed>(
        "IsContainerClosed");
    factory.registerNodeType<MyRobotNodes::CloseContainer>("CloseContainer");
    factory.registerNodeType<MyRobotNodes::IsArmResting>("IsArmResting");
    factory.registerNodeType<MyRobotNodes::RestArm>("RestArm");

    std::string package_share =
        ament_index_cpp::get_package_share_directory("bt_example");
    std::string xml_path = package_share + "/config/my_tree.xml";

    auto tree = factory.createTreeFromFile(xml_path);

    
    auto subscription = node->create_subscription<std_msgs::msg::String>(
    "gripper_status", 10,
    [&tree](const std_msgs::msg::String::SharedPtr msg) {
        tree.rootBlackboard()->set("gripper_state", msg->data);
    });

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.tickOnce();
        
        // Process ROS 2 communication (Action feedback, responses)
        rclcpp::spin_some(node); 
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}

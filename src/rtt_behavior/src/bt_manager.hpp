#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rtt_behavior/action/collect_task.hpp"
#include "ex.hpp"

class BtManagerNode : public rclcpp::Node {
   public:
    using CollectTask = rtt_behavior::action::CollectTask;
    using GoalHandleCollect = rclcpp_action::ServerGoalHandle<CollectTask>;

    BtManagerNode() : Node("bt_manager") {
        factory_.registerNodeType<MyRobotNodes::IsNear>("IsNear");
        factory_.registerNodeType<MyRobotNodes::IsGripperOpen>("IsGripperOpen");
        factory_.registerNodeType<MyRobotNodes::OpenGripper>("OpenGripper");
        factory_.registerNodeType<MyRobotNodes::IsGripperClosed>(
            "IsGripperClosed");
        factory_.registerNodeType<MyRobotNodes::CloseGripper>("CloseGripper");
        factory_.registerNodeType<MyRobotNodes::ExtractWithGripper>(
            "ExtractWithGripper");
        factory_.registerNodeType<MyRobotNodes::IsEmpty>("IsEmpty");
        factory_.registerNodeType<MyRobotNodes::Contains>("Contains");
        factory_.registerNodeType<MyRobotNodes::IsContainerOpen>(
            "IsContainerOpen");
        factory_.registerNodeType<MyRobotNodes::OpenContainer>("OpenContainer");
        factory_.registerNodeType<MyRobotNodes::IsContainerClosed>(
            "IsContainerClosed");
        factory_.registerNodeType<MyRobotNodes::CloseContainer>(
            "CloseContainer");
        factory_.registerNodeType<MyRobotNodes::IsArmResting>("IsArmResting");
        factory_.registerNodeType<MyRobotNodes::RestArm>("RestArm");

        bt_server_ = rclcpp_action::create_server<CollectTask>(
            this, "bt_collect_task",

            [this](auto uuid, auto goal) {
                (void)uuid;
                (void)goal;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },

            [this](auto handle) {
                (void)handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            },

            [this](auto handle) { handle_task_accepted(handle); });
    }

    void createTree() {
        std::string package_share =
            ament_index_cpp::get_package_share_directory("rtt_behavior");
        std::string xml_path = package_share + "/config/my_tree.xml";

        params_.nh = shared_from_this();  
        factory_.registerNodeType<MyRobotNodes::Approach>("Approach", params_);
        tree_ = factory_.createTreeFromFile(xml_path);
    }

   private:
    void handle_task_accepted(std::shared_ptr<GoalHandleCollect> handle) {
        const auto &goal = handle->get_goal();  
        std::thread([this, handle, goal]() {
            RCLCPP_INFO(get_logger(), "BT task received: task=[%s] target=[%s]",
                        goal->task_name.c_str(), goal->target_id.c_str());

            createTree();

            BT::NodeStatus status = BT::NodeStatus::RUNNING;
            while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
                status = tree_.tickOnce();

                rclcpp::spin_some(shared_from_this());

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            auto result = std::make_shared<CollectTask::Result>();
            result->success =
                (status == BT::NodeStatus::SUCCESS); 
            result->message =
                result->success ? "task completed" : "task failed";
            handle->succeed(result);
        }).detach();
    }

    rclcpp_action::Server<CollectTask>::SharedPtr bt_server_;
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    BT::RosNodeParams params_;
};

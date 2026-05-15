#ifndef MOVE_ROBOT_HPP
#define MOVE_ROBOT_HPP

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp" // Added this
#include "behaviortree_ros2/bt_action_node.hpp"
#include "turtlesim/action/rotate_absolute.hpp"

namespace MyRobotNodes {
using RotateAbsolute = turtlesim::action::RotateAbsolute;
    
inline bool gripperOpen = true;
inline bool containerOpen = false;
inline bool armResting = true;
inline bool containerHasObject = false;

// Condition: Checks if entity is empty
class IsEmpty : public BT::ConditionNode {
   public:
    IsEmpty(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("entity")};
    }
    BT::NodeStatus tick() override;
};

// Condition: Checks if entity contains content
class Contains : public BT::ConditionNode {
   public:
    Contains(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("entity"),
                BT::InputPort<std::string>("content")};
    }
    BT::NodeStatus tick() override;
};

class IsContainerOpen : public BT::ConditionNode {
   public:
    IsContainerOpen(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class OpenContainer : public BT::SyncActionNode {
   public:
    OpenContainer(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class IsContainerClosed : public BT::ConditionNode {
   public:
    IsContainerClosed(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class CloseContainer : public BT::SyncActionNode {
   public:
    CloseContainer(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class IsArmResting : public BT::ConditionNode {
   public:
    IsArmResting(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class RestArm : public BT::SyncActionNode {
   public:
    RestArm(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class IsNear : public BT::ConditionNode {
   public:
    IsNear(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("entity"),
                BT::InputPort<std::string>("target"),
                BT::InputPort<int>("distance")};
    }

    BT::NodeStatus tick() override;

   private:
    int _distance;
};

class Approach : public BT::RosActionNode<RotateAbsolute> {
public:
    // Use the base class alias to simplify the override signatures
    using BaseClass = BT::RosActionNode<RotateAbsolute>;

    Approach(const std::string& name, 
             const BT::NodeConfig& config, 
             const BT::RosNodeParams& params)
        : BaseClass(name, config, params) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<float>("theta")};
    }

    // 4. Use 'BaseClass::' to help the compiler resolve the types
    bool setGoal(BaseClass::Goal& goal) override;

    BT::NodeStatus onResultReceived(const BaseClass::WrappedResult& wr) override;

    void onHalt() override;
};

class ExtractWithGripper : public BT::StatefulActionNode {
   public:
    ExtractWithGripper(const std::string &name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<int>("distance")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

   private:
    int _distance;
};

class IsGripperOpen : public BT::ConditionNode {
   public:
    IsGripperOpen(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override;
};

class OpenGripper : public BT::SyncActionNode {
   public:
    OpenGripper(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override;
};

class IsGripperClosed : public BT::ConditionNode {
   public:
    IsGripperClosed(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override;
};

class CloseGripper : public BT::SyncActionNode {
   public:
    CloseGripper(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override;
};

}  // namespace MyRobotNodes

#endif

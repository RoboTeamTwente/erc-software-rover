#ifndef MOVE_ROBOT_HPP
#define MOVE_ROBOT_HPP

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"

namespace MyRobotNodes {
    inline bool gripperOpen = true;
    inline bool containerOpen = false;
    inline bool armResting = true;
    inline bool containerHasObject = false;

    // Condition: Checks if entity is empty
    class IsEmpty : public BT::ConditionNode {
    public:
        IsEmpty(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) {}
        static BT::PortsList providedPorts() { return { BT::InputPort<std::string>("entity") }; }
        BT::NodeStatus tick() override;
    };

    // Condition: Checks if entity contains content
    class Contains : public BT::ConditionNode {
    public:
        Contains(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) {}
        static BT::PortsList providedPorts() { 
            return { BT::InputPort<std::string>("entity"), BT::InputPort<std::string>("content") }; 
        }
        BT::NodeStatus tick() override;
    };

    class IsContainerOpen : public BT::ConditionNode {
    public:
        IsContainerOpen(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) {}
        static BT::PortsList providedPorts() { return {}; }
        BT::NodeStatus tick() override;
    };

    class OpenContainer : public BT::SyncActionNode {
    public:
        OpenContainer(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}
        static BT::PortsList providedPorts() { return {}; }
        BT::NodeStatus tick() override;
    };

    class IsContainerClosed : public BT::ConditionNode {
    public:
        IsContainerClosed(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) {}
        static BT::PortsList providedPorts() { return {}; }
        BT::NodeStatus tick() override;
    };

    class CloseContainer : public BT::SyncActionNode {
    public:
        CloseContainer(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}
        static BT::PortsList providedPorts() { return {}; }
        BT::NodeStatus tick() override;
    };

    class IsArmResting : public BT::ConditionNode {
    public:
        IsArmResting(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) {}
        static BT::PortsList providedPorts() { return {}; }
        BT::NodeStatus tick() override;
    };

    class RestArm : public BT::SyncActionNode {
    public:
        RestArm(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}
        static BT::PortsList providedPorts() { return {}; }
        BT::NodeStatus tick() override;
    };

class IsNear : public BT::ConditionNode {
public:
    IsNear(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("entity"),
                 BT::InputPort<std::string>("target"),
                 BT::InputPort<int>("distance") };
    }

    BT::NodeStatus tick() override;
private:
    int _distance;
};

class Approach : public BT::StatefulActionNode {
public:

    Approach(const std::string& name, const BT::NodeConfig& config);
    
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("entity"),
                 BT::InputPort<std::string>("target"),
                 BT::InputPort<int>("distance"),
                 BT::InputPort<int>("angle"),
                 BT::InputPort<double>("speed") };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    int _distance;
};

class ExtractWithGripper : public BT::StatefulActionNode {
public:
    ExtractWithGripper(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    
    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("distance") };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    int _distance;
};



class IsGripperOpen : public BT::ConditionNode {
public:
    IsGripperOpen(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {};
    }

    BT::NodeStatus tick() override;
};

class OpenGripper : public BT::SyncActionNode {
public:
    OpenGripper(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {};
    }

    BT::NodeStatus tick() override;
};

class IsGripperClosed : public BT::ConditionNode {
public:
    IsGripperClosed(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {};
    }

    BT::NodeStatus tick() override;
};

class CloseGripper : public BT::SyncActionNode {
public:
    CloseGripper(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {};
    }

    BT::NodeStatus tick() override;
};


} // namespace MyRobotNodes

#endif
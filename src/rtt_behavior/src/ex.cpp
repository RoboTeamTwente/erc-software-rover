#include "ex.hpp"

#include <iostream>

const int MIN_DISTANCE = 5;

namespace MyRobotNodes {

bool Approach::setGoal(BaseClass::Goal& goal) {
    float angle = 0;
    if (!getInput<float>("theta", angle)) return false;
    goal.theta = angle;
    return true;
}

BT::NodeStatus Approach::onResultReceived(const BaseClass::WrappedResult& wr) {
    if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

void Approach::onHalt() {
    // Logic for when the BT stops the action mid-way
}

BT::NodeStatus IsNear::tick() {
    std::string entity, target;
    getInput("entity", entity);
    getInput("target", target);
    if (!getInput<int>("distance", _distance)) _distance = MIN_DISTANCE;

    std::cout << "[ INFO ] Proximity Check: " << entity << " to " << target
              << " | Dist: " << _distance << "cm" << std::endl;

    return (_distance <= MIN_DISTANCE) ? BT::NodeStatus::SUCCESS
                                       : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsGripperOpen::tick() {
    std::cout << "[ INFO ] Checking if gripper is open... " << std::endl;
    if (gripperOpen) std::cout << "[ INFO ] The gripper is open." << std::endl;
    return gripperOpen ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus OpenGripper::tick() {
    gripperOpen = true;
    std::cout << "[ INFO ] Opened the gripper. " << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CloseGripper::tick() {
    gripperOpen = false;
    std::cout << "[ INFO ] Closed the gripper. " << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus IsGripperClosed::tick() {
    std::cout << "[ INFO ] Checking if gripper is closed... " << gripperOpen
              << std::endl;
    if (!gripperOpen)
        std::cout << "[ INFO ] The gripper is closed." << std::endl;
    return !gripperOpen ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus ExtractWithGripper::onStart() {
    if (!getInput<int>("distance", _distance)) return BT::NodeStatus::FAILURE;

    std::cout << "[ Extraction ] Starting mission: probe extraction..."
              << std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExtractWithGripper::onRunning() {
    _distance--;
    std::cout << "[ Extraction ] Moving gripper... " << _distance
              << " centimeters left." << std::endl;

    return (_distance <= 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void ExtractWithGripper::onHalted() {
    std::cout << "[ Extraction ] HALTED!" << std::endl;
}

BT::NodeStatus IsEmpty::tick() {
    std::string entity;
    getInput("entity", entity);
    if (entity == "container") {
        return !containerHasObject ? BT::NodeStatus::SUCCESS
                                   : BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Contains::tick() {
    std::string entity, content;
    getInput("entity", entity);
    getInput("content", content);

    if (entity == "container" && content == "object")
        return containerHasObject ? BT::NodeStatus::SUCCESS
                                  : BT::NodeStatus::FAILURE;
    if (entity == "gripper" && content == "object")
        return !gripperOpen ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;

    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsContainerOpen::tick() {
    return containerOpen ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus OpenContainer::tick() {
    containerOpen = true;
    std::cout << "[ INFO ] Container opened." << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus IsContainerClosed::tick() {
    return !containerOpen ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus CloseContainer::tick() {
    containerOpen = false;
    std::cout << "[ INFO ] Container closed." << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus IsArmResting::tick() {
    return armResting ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus RestArm::tick() {
    armResting = true;
    std::cout << "[ INFO ] Arm moved to rest position." << std::endl;
    return BT::NodeStatus::SUCCESS;
}

}  // namespace MyRobotNodes

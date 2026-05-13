#include "ex.hpp"
#include <iostream>

const int MIN_DISTANCE = 5;

namespace MyRobotNodes {

Approach::Approach(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus Approach::onStart() {
    std::string entity, target;
    double speed = 0.0;
    int angle = 0;

    getInput("entity", entity);
    getInput("target", target);
    getInput("speed", speed);
    getInput("angle", angle);
    if (!getInput<int>("distance", _distance)) _distance = 10;

    std::cout << "[ Approach ] " << entity << " moving to " << target 
              << " at speed " << speed << ". Dist: " << _distance << "m." << std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Approach::onRunning() {
    _distance--;
    std::cout << "[ Approach ] Moving... " << _distance << " centimeters left." << std::endl;
    
    return ( _distance <= 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void Approach::onHalted() { std::cout << "[ Approach ] HALTED!" << std::endl; }


BT::NodeStatus IsNear::tick() {
    std::string entity, target;
    getInput("entity", entity);
    getInput("target", target);
    if(!getInput<int>("distance", _distance)) _distance = MIN_DISTANCE;

    std::cout << "[ INFO ] Proximity Check: " << entity << " to " << target 
              << " | Dist: " << _distance << "cm" << std::endl;
    
    return (_distance <= MIN_DISTANCE) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsGripperOpen::tick() {
    std::cout << "[ INFO ] Checking if gripper is open... " << std::endl;
    if(gripperOpen) std::cout<< "[ INFO ] The gripper is open." << std::endl;
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
    std::cout << "[ INFO ] Checking if gripper is closed... "<< gripperOpen << std::endl; 
    if(!gripperOpen) std::cout<< "[ INFO ] The gripper is closed." << std::endl;
    return !gripperOpen ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus ExtractWithGripper::onStart() {
    if(!getInput<int>("distance", _distance)) return BT::NodeStatus::FAILURE;

    std::cout << "[ Extraction ] Starting mission: probe extraction..." << std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExtractWithGripper::onRunning() {
    _distance--;
    std::cout << "[ Extraction ] Moving gripper... " << _distance <<" centimeters left." << std::endl;
    
    return ( _distance <= 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void ExtractWithGripper::onHalted() { std::cout << "[ Extraction ] HALTED!" << std::endl; }

BT::NodeStatus IsEmpty::tick() {
    std::string entity;
    getInput("entity", entity);
    if (entity == "container") {
        return !containerHasObject ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Contains::tick() {
    std::string entity, content;
    getInput("entity", entity);
    getInput("content", content);
    
    if (entity == "container" && content == "object") return containerHasObject ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    if (entity == "gripper" && content == "object") return !gripperOpen ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsContainerOpen::tick() { return containerOpen ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE; }
BT::NodeStatus OpenContainer::tick() { 
    containerOpen = true; 
    std::cout << "[ INFO ] Container opened." << std::endl;
    return BT::NodeStatus::SUCCESS; 
}

BT::NodeStatus IsContainerClosed::tick() { return !containerOpen ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE; }
BT::NodeStatus CloseContainer::tick() { 
    containerOpen = false; 
    std::cout << "[ INFO ] Container closed." << std::endl;
    return BT::NodeStatus::SUCCESS; 
}

BT::NodeStatus IsArmResting::tick() { return armResting ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE; }
BT::NodeStatus RestArm::tick() { 
    armResting = true; 
    std::cout << "[ INFO ] Arm moved to rest position." << std::endl;
    return BT::NodeStatus::SUCCESS; 
}


} // namespace MyRobotNodes
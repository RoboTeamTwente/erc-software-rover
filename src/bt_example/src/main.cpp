#include "behaviortree_cpp/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ex.hpp"
#include <thread>

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    BT::BehaviorTreeFactory factory;

    // Register all custom nodes
    factory.registerNodeType<MyRobotNodes::Approach>("Approach");
    factory.registerNodeType<MyRobotNodes::IsNear>("IsNear");
    factory.registerNodeType<MyRobotNodes::IsGripperOpen>("IsGripperOpen");
    factory.registerNodeType<MyRobotNodes::OpenGripper>("OpenGripper");
    factory.registerNodeType<MyRobotNodes::IsGripperClosed>("IsGripperClosed");
    factory.registerNodeType<MyRobotNodes::CloseGripper>("CloseGripper");
    factory.registerNodeType<MyRobotNodes::ExtractWithGripper>("ExtractWithGripper");
    factory.registerNodeType<MyRobotNodes::IsEmpty>("IsEmpty");
    factory.registerNodeType<MyRobotNodes::Contains>("Contains");
    factory.registerNodeType<MyRobotNodes::IsContainerOpen>("IsContainerOpen");
    factory.registerNodeType<MyRobotNodes::OpenContainer>("OpenContainer");
    factory.registerNodeType<MyRobotNodes::IsContainerClosed>("IsContainerClosed");
    factory.registerNodeType<MyRobotNodes::CloseContainer>("CloseContainer");
    factory.registerNodeType<MyRobotNodes::IsArmResting>("IsArmResting");
    factory.registerNodeType<MyRobotNodes::RestArm>("RestArm");

    std::string package_share = ament_index_cpp::get_package_share_directory("bt_example");
    std::string xml_path = package_share + "/config/my_tree.xml";

    auto tree = factory.createTreeFromFile(xml_path);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING) {
        status = tree.tickOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}
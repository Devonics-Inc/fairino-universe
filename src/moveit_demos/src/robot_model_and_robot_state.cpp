#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);

    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rclcpp::Node>("moveit_demo",options);

    const auto & LOGGER = node->get_logger();


    robot_model_loader::RobotModelLoader rml(node);
    const moveit::core::RobotModelPtr& kml =  rml->getModel();
    RCLCPP_INFO(LOGGER,"Model frame: %s",kml->getModelFrame().c_str());


    moveit::core::RobotStatePtr krs(new moveit::core::RobotState(kml));
    krs->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kml->getJointModelGroup("arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();


    return 0 ;
}
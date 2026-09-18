
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char** argv)
{



    rclcpp::init(argc,argv);

    auto const node = std::make_shared<rclcpp::Node>("moveit_demo",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto const logger = rclcpp::get_logger("moveit_demo");

    
    // Create the MoveGroupInterface
    auto move_group_interface = MoveGroupInterface(node, "arm");


    auto const tr_pose = []{
        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = 0.28;
        pose.position.y = -0.2;
        pose.position.z = 0.5;
        return pose;
    }();

    move_group_interface.setPoseTarget(tr_pose);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
// Shared belt geometry and parameters. Every node loads the same conveyor.yaml,
// so they all agree on where the belt, the pick point and the queue slots are.
#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace conveyor_sim
{

constexpr double kEps = 1e-6;
constexpr double kLift = 0.002;  // tiny lift off the belt to avoid surface-contact collisions

struct ConveyorParams
{
  std::string frame_id;
  double belt_height;
  double belt_y;
  double belt_width;
  double start_x;               // where boxes appear
  double pick_x;                // where the lead box stops
  std::vector<double> box_size; // x, y, z
  double gap;                   // spacing between queued boxes

  // Distance between the centers of two neighbouring queue slots.
  double pitch() const {return box_size[0] + gap;}

  // Height of a box center resting on the belt.
  double boxZ() const {return belt_height + box_size[2] / 2.0 + kLift;}

  // Slot 0 is the pick point, slot 1 is right behind it, and so on.
  double slotX(std::size_t k) const {return pick_x - static_cast<double>(k) * pitch();}

  // How many slots fit between the pick point and the belt start.
  std::size_t numSlots() const
  {
    return static_cast<std::size_t>(std::floor((pick_x - start_x) / pitch() + kEps)) + 1;
  }

  geometry_msgs::msg::Pose boxPose(double x) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = belt_y;
    pose.position.z = boxZ();
    pose.orientation.w = 1.0;
    return pose;
  }
};

inline ConveyorParams declareConveyorParams(rclcpp::Node & node)
{
  ConveyorParams p;
  p.frame_id = node.declare_parameter<std::string>("frame_id", "world");
  p.belt_height = node.declare_parameter<double>("belt_height", 0.40);
  p.belt_y = node.declare_parameter<double>("belt_y", -0.60);
  p.belt_width = node.declare_parameter<double>("belt_width", 0.40);
  p.start_x = node.declare_parameter<double>("start_x", -0.80);
  p.pick_x = node.declare_parameter<double>("pick_x", 0.0);
  p.box_size = node.declare_parameter<std::vector<double>>(
    "box_size", std::vector<double>{0.30, 0.20, 0.15});
  p.gap = node.declare_parameter<double>("gap", 0.05);

  if (p.box_size.size() != 3) {
    throw std::invalid_argument("box_size must have exactly 3 values [x, y, z]");
  }
  if (p.pick_x <= p.start_x) {
    throw std::invalid_argument("pick_x must be greater than start_x");
  }
  return p;
}

// Latched QoS: a node that starts late still receives the last published message.
inline rclcpp::QoS latchedQos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}

}  // namespace conveyor_sim

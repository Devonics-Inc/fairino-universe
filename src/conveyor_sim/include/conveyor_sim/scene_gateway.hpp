// Pure helpers for building MoveIt planning-scene messages.
//
// Rules for this header:
//  - only free functions that build messages from their arguments
//  - no node state (no p_, no publishers, no logger)
//  - everything `inline`, because several .cpp files include this header
//  - full type names, no `using` (a `using` here would leak into every includer)
#pragma once

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/object_color.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace conveyor_sim
{

// ADD a box-shaped object of size `dims` (x, y, z) at `pose`.
inline moveit_msgs::msg::CollisionObject makeCollisionObjectBox(
  const std::string & frame_id, const std::string & id,
  const geometry_msgs::msg::Pose & pose, const std::array<double, 3> & dims)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = frame_id;
  obj.id = id;
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  obj.pose = pose;

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = shape_msgs::msg::SolidPrimitive::BOX;
  prim.dimensions = {dims[0], dims[1], dims[2]};
  obj.primitives.push_back(prim);

  geometry_msgs::msg::Pose identity;
  identity.orientation.w = 1.0;
  obj.primitive_poses.push_back(identity);
  return obj;
}

// MOVE an existing object to `pose`.
inline moveit_msgs::msg::CollisionObject makeCollisionObjectMove(
  const std::string & frame_id, const std::string & id,
  const geometry_msgs::msg::Pose & pose)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = frame_id;
  obj.id = id;
  obj.operation = moveit_msgs::msg::CollisionObject::MOVE;
  obj.pose = pose;
  return obj;
}

// Color for one object.
inline moveit_msgs::msg::ObjectColor makeObjectColor(
  const std::string & id, const std_msgs::msg::ColorRGBA & color)
{
  moveit_msgs::msg::ObjectColor c;
  c.id = id;
  c.color = color;
  return c;
}

// A PlanningScene diff. robot_state.is_diff is always set: without it MoveIt treats the
// empty robot state as a full one and resets the robot in the scene.
inline moveit_msgs::msg::PlanningScene makeSceneDiff(
  std::vector<moveit_msgs::msg::CollisionObject> objs = {},
  std::vector<moveit_msgs::msg::ObjectColor> colors = {})
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  scene.world.collision_objects = std::move(objs);
  scene.object_colors = std::move(colors);
  return scene;
}

}  // namespace conveyor_sim
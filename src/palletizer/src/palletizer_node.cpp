// palletizer: picks each box the conveyor presents and stacks it on a pallet, in MoveIt.
//
// - Listens to conveyor/box_at_pick (id) and conveyor/box_pick_pose (pose, box center).
// - For each new box: pre-grasp -> straight down -> attach -> lift -> pre-place ->
//   straight down -> detach -> retreat. Slots fill row by row, then layer by layer.
// - Attaching the box is what makes the proximity sensor fire box_removed, so the
//   conveyor brings the next box while the arm is still placing.
// - Stops (does not retry) on any planning/execution failure, so it never grabs twice.

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using moveit::planning_interface::MoveGroupInterface;

class Palletizer
{
public:
  explicit Palletizer(const rclcpp::Node::SharedPtr & node)
  : node_(node), log_(node->get_logger())
  {
    group_ = param<std::string>("planning_group", "fairino5_v6_group");
    ee_link_ = param<std::string>("ee_link", "wrist3_link");
    frame_id_ = param<std::string>("frame_id", "world");
    touch_links_ = param<std::vector<std::string>>("touch_links", {});
    tool_length_ = param<double>("tool_length", 0.0);
    approach_ = param<double>("approach_height", 0.10);
    box_size_ = param<std::vector<double>>("box_size", {0.2, 0.2, 0.2});
    pallet_top_ = param<std::vector<double>>("pallet_top", {0.0, 0.6, 0.15});
    pallet_size_ = param<std::vector<double>>("pallet_size", {0.8, 0.6, 0.15});
    rows_ = param<int64_t>("rows", 2);
    cols_ = param<int64_t>("cols", 2);
    layers_ = param<int64_t>("layers", 2);
    gap_ = param<double>("gap", 0.01);
    const auto rpy = param<std::vector<double>>("grasp_rpy", {M_PI, 0.0, 0.0});
    grasp_q_ = rpyToQuat(rpy.at(0), rpy.at(1), rpy.at(2));

    arm_ = std::make_unique<MoveGroupInterface>(node_, group_);
    arm_->setEndEffectorLink(ee_link_);
    arm_->setPoseReferenceFrame(frame_id_);
    arm_->setPlanningTime(5.0);
    arm_->setMaxVelocityScalingFactor(param<double>("velocity_scaling", 0.3));
    arm_->setMaxAccelerationScalingFactor(param<double>("acceleration_scaling", 0.3));

    id_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "conveyor/box_at_pick", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        box_id_ = m->data;
      });
    pose_sub_ = node_->create_subscription<PoseStamped>(
      "conveyor/box_pick_pose", 10, [this](PoseStamped::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        box_pose_ = m->pose;
        pose_time_ = std::chrono::steady_clock::now();
      });

    hold_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "conveyor/hold", rclcpp::QoS(1).transient_local().reliable());
    setHold(false);
  }

  void run()
  {
    addPallet();
    const int64_t total = rows_ * cols_ * layers_;
    RCLCPP_INFO(log_, "Palletizing %ld boxes (%ldx%ld x %ld layers)",
      total, rows_, cols_, layers_);

    for (int64_t slot = 0; slot < total && rclcpp::ok(); ++slot) {
      std::string id;
      Pose box;
      if (!waitForBox(id, box)) {
        return;
      }
      RCLCPP_INFO(log_, "[%ld/%ld] picking %s", slot + 1, total, id.c_str());
      if (!pickAndPlace(id, box, placePose(slot))) {
        RCLCPP_ERROR(log_, "Cycle failed on %s, stopping", id.c_str());
        return;
      }
      last_picked_ = id;
    }
    RCLCPP_INFO(log_, "Pallet complete");
  }

private:
  template<typename T>
  T param(const std::string & name, const T & def)
  {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<T>(name, def);
    }
    return node_->get_parameter(name).get_value<T>();
  }

  // Wait for a box at the pick point that we haven't picked yet, with a fresh pose.
  bool waitForBox(std::string & id, Pose & box)
  {
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lk(mtx_);
        const bool fresh = std::chrono::steady_clock::now() - pose_time_ < 500ms;
        if (!box_id_.empty() && box_id_ != last_picked_ && fresh) {
          id = box_id_;
          box = box_pose_;
          return true;
        }
      }
      std::this_thread::sleep_for(50ms);
    }
    return false;
  }

  bool pickAndPlace(const std::string & id, const Pose & box, const Pose & place)
  {
    const double half_h = box_size_[2] / 2.0;
    const Pose grasp = toolPose(box.position.x, box.position.y, box.position.z + half_h);
    const Pose drop = toolPose(place.position.x, place.position.y, place.position.z + half_h);
    Pose pre_grasp = grasp;
    pre_grasp.position.z += approach_;
    Pose pre_drop = drop;
    pre_drop.position.z += approach_;

    setHold(true);                        // stop the belt while we work at the pick point

    if (!moveTo(pre_grasp, "pre-grasp") || !straight(grasp, "descend to box")) {
      return false;
    }

    arm_->attachObject(id, ee_link_, touch_links_);
    std::this_thread::sleep_for(300ms);   // let move_group apply the attach


    //setHold(false);
    if (!straight(pre_grasp, "lift") ||
      !moveTo(pre_drop, "pre-place") ||
      !straight(drop, "descend to pallet"))
    {
      arm_->detachObject(id);
      return false;
    }

    arm_->detachObject(id);               // box stays in the scene, on the stack
    std::this_thread::sleep_for(300ms);
    setHold(false);
    return straight(pre_drop, "retreat");
  }

  void setHold(bool on)
  {
    std_msgs::msg::Bool msg;
    msg.data = on;
    hold_pub_->publish(msg);
  }
  // Free-space move (planned, collision-checked).
  bool moveTo(const Pose & target, const char * what)
  {
    arm_->setStartStateToCurrentState();
    arm_->setPoseTarget(target, ee_link_);
    MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(arm_->plan(plan))) {
      RCLCPP_ERROR(log_, "Planning failed: %s", what);
      return false;
    }
    if (!static_cast<bool>(arm_->execute(plan))) {
      RCLCPP_ERROR(log_, "Execution failed: %s", what);
      return false;
    }
    return true;
  }

  // Short vertical move. Collision checking is off on purpose: these segments start or
  // end in contact (gripper on box, box on belt, box on stack), which MoveIt reports as
  // a collision. They are straight up/down over a known-clear spot.
  bool straight(const Pose & target, const char * what)
  {
    arm_->setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory traj;
    const double frac = arm_->computeCartesianPath({target}, 0.005, 0.0, traj, false);
    if (frac < 0.99) {
      RCLCPP_ERROR(log_, "Cartesian path %s only %.0f%% feasible", what, frac * 100.0);
      return false;
    }
    if (!static_cast<bool>(arm_->execute(traj))) {
      RCLCPP_ERROR(log_, "Execution failed: %s", what);
      return false;
    }
    return true;
  }

  // ee_link pose for touching a surface at height z_contact, tool pointing down.
  Pose toolPose(double x, double y, double z_contact) const
  {
    Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z_contact + tool_length_;
    p.orientation = grasp_q_;
    return p;
  }

  // Box center for a slot: row by row, then layer by layer, centered on the pallet.
  Pose placePose(int64_t slot) const
  {
    const int64_t per_layer = rows_ * cols_;
    const int64_t layer = slot / per_layer;
    const int64_t k = slot % per_layer;
    const int64_t r = k / cols_;
    const int64_t c = k % cols_;
    const double sx = box_size_[0] + gap_;
    const double sy = box_size_[1] + gap_;

    Pose p;
    p.position.x = pallet_top_[0] + (r - (rows_ - 1) / 2.0) * sx;
    p.position.y = pallet_top_[1] + (c - (cols_ - 1) / 2.0) * sy;
    p.position.z = pallet_top_[2] + layer * box_size_[2] + box_size_[2] / 2.0;
    p.orientation.w = 1.0;
    return p;
  }

  void addPallet()
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame_id_;
    obj.id = "pallet";
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = {pallet_size_[0], pallet_size_[1], pallet_size_[2]};
    obj.primitives.push_back(prim);

    Pose pose;
    pose.position.x = pallet_top_[0];
    pose.position.y = pallet_top_[1];
    pose.position.z = pallet_top_[2] - pallet_size_[2] / 2.0;
    pose.orientation.w = 1.0;
    obj.primitive_poses.push_back(pose);

    scene_.applyCollisionObject(obj);
    RCLCPP_INFO(log_, "Pallet added to planning scene");
  }

  static geometry_msgs::msg::Quaternion rpyToQuat(double r, double p, double y)
  {
    const double cr = std::cos(r / 2), sr = std::sin(r / 2);
    const double cp = std::cos(p / 2), sp = std::sin(p / 2);
    const double cy = std::cos(y / 2), sy = std::sin(y / 2);
    geometry_msgs::msg::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger log_;
  std::unique_ptr<MoveGroupInterface> arm_;
  moveit::planning_interface::PlanningSceneInterface scene_;

  std::string group_, ee_link_, frame_id_;
  std::vector<std::string> touch_links_;
  double tool_length_, approach_, gap_;
  std::vector<double> box_size_, pallet_top_, pallet_size_;
  int64_t rows_, cols_, layers_;
  geometry_msgs::msg::Quaternion grasp_q_;

  std::mutex mtx_;
  std::string box_id_, last_picked_;
  Pose box_pose_;
  std::chrono::steady_clock::time_point pose_time_{};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr id_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hold_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("palletizer", opts);

  // MoveGroupInterface blocks, so callbacks spin on their own thread.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() {exec.spin();});

  {
    Palletizer palletizer(node);
    palletizer.run();
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}

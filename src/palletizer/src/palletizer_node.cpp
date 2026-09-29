// palletizer_node: executes the targets pallet_planner sends. It never decides where a box goes.
//
// Input   pallet/target        palletizer_msgs/PalletTarget (box id, pick pose, place pose, size)
// Output  pallet/place_result  palletizer_msgs/PlaceResult
//         conveyor/hold        std_msgs/Bool (latched), true for the whole cycle
//
// Cycle: pre-grasp -> straight down -> attach -> lift -> pre-place -> straight down ->
//        detach -> retreat. Stops after a failed cycle so it never grabs a box twice.

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>

#include "geometry_msgs/msg/pose.hpp"
#include "palletizer_msgs/msg/pallet_target.hpp"
#include "palletizer_msgs/msg/place_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Quaternion;
using moveit::planning_interface::MoveGroupInterface;
using palletizer_msgs::msg::PalletTarget;
using palletizer_msgs::msg::PlaceResult;

class Palletizer
{
public:
  explicit Palletizer(const rclcpp::Node::SharedPtr & node)
  : node_(node), log_(node->get_logger())
  {
    group_ = param<std::string>("planning_group", "fairino5_v6_group");
    ee_link_ = param<std::string>("ee_link", "wrist3_link");
    touch_links_ = param<std::vector<std::string>>("touch_links", {});
    tool_length_ = param<double>("tool_length", 0.10);
    grasp_clearance_ = param<double>("grasp_clearance", 0.005);
    approach_ = param<double>("approach_height", 0.10);
    const auto rpy = param<std::vector<double>>("grasp_rpy", {M_PI, 0.0, 0.0});
    grasp_q_ = rpyToQuat(rpy.at(0), rpy.at(1), rpy.at(2));

    arm_ = std::make_unique<MoveGroupInterface>(node_, group_);
    arm_->setEndEffectorLink(ee_link_);
    arm_->setPlanningTime(5.0);
    arm_->setMaxVelocityScalingFactor(param<double>("velocity_scaling", 0.3));
    arm_->setMaxAccelerationScalingFactor(param<double>("acceleration_scaling", 0.3));

    target_sub_ = node_->create_subscription<PalletTarget>(
      "pallet/target", 10, [this](PalletTarget::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (m->box_id == busy_id_ || m->box_id == done_id_) {
          return;   // planner re-sends the pending target every second
        }
        next_ = *m;
      });
    result_pub_ = node_->create_publisher<PlaceResult>("pallet/place_result", 10);
    hold_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "conveyor/hold", rclcpp::QoS(1).reliable().transient_local());
  }

  void run()
  {
    setHold(false);
    RCLCPP_INFO(log_, "Waiting for targets on pallet/target");

    while (rclcpp::ok()) {
      PalletTarget t;
      if (!waitForTarget(t)) {
        return;
      }
      RCLCPP_INFO(log_, "Slot %u (layer %u): %s", t.slot, t.layer, t.box_id.c_str());

      setHold(true);                    // belt stays still for the whole cycle
      const bool ok = pickAndPlace(t);
      if (ok) {
        setHold(false);                 // box is on the pallet and the arm is clear
      }
      // On failure the belt stays HELD: the arm may be stuck at the pick point.

      PlaceResult r;
      r.box_id = t.box_id;
      r.slot = t.slot;
      r.success = ok;
      r.message = ok ? "placed" : last_error_;
      result_pub_->publish(r);
      {
        std::lock_guard<std::mutex> lk(mtx_);
        busy_id_.clear();
        done_id_ = t.box_id;
      }

      if (!ok) {
        RCLCPP_ERROR(log_, "Cycle failed on %s (%s), stopping. Belt left HELD; release with: "
          "ros2 topic pub --once /conveyor/hold std_msgs/msg/Bool \"{data: false}\" "
          "--qos-durability transient_local", t.box_id.c_str(), last_error_.c_str());
        std::this_thread::sleep_for(500ms);   // let the result go out before shutdown
        return;
      }
    }
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

  bool waitForTarget(PalletTarget & t)
  {
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lk(mtx_);
        if (next_) {
          t = *next_;
          next_.reset();
          busy_id_ = t.box_id;
          return true;
        }
      }
      std::this_thread::sleep_for(50ms);
    }
    return false;
  }

  bool pickAndPlace(const PalletTarget & t)
  {
    arm_->setPoseReferenceFrame(t.header.frame_id);
    const double half_h = t.box_size.z / 2.0;
    const Pose grasp = toolPose(t.pick_pose, half_h);
    const Pose drop = toolPose(t.place_pose, half_h);
    RCLCPP_INFO(log_,
      "Box top z=%.3f -> %s target z=%.3f (tool_length %.3f + clearance %.3f)",
      t.pick_pose.position.z + half_h, ee_link_.c_str(), grasp.position.z,
      tool_length_, grasp_clearance_);
    Pose pre_grasp = grasp;
    pre_grasp.position.z += approach_;
    Pose pre_drop = drop;
    pre_drop.position.z += approach_;

    if (!moveTo(pre_grasp, "pre-grasp") || !straight(grasp, "descend to box")) {
      return false;
    }

    arm_->attachObject(t.box_id, ee_link_, touch_links_);
    std::this_thread::sleep_for(300ms);   // let move_group apply the attach

    if (!straight(pre_grasp, "lift") ||
      !moveTo(pre_drop, "pre-place") ||
      !straight(drop, "descend to pallet"))
    {
      // Keep the box attached, like a real gripper that is still holding it.
      // Detaching here would leave it floating in mid-air.
      return false;
    }

    arm_->detachObject(t.box_id);         // box stays in the scene, on the stack
    std::this_thread::sleep_for(300ms);
    return straight(pre_drop, "retreat");
  }

  bool moveTo(const Pose & target, const char * what)
  {
    arm_->setStartStateToCurrentState();
    arm_->setPoseTarget(target, ee_link_);
    MoveGroupInterface::Plan plan;
    bool planned = false;
    for (int attempt = 1; attempt <= 3 && !planned; ++attempt) {
      planned = static_cast<bool>(arm_->plan(plan));
      if (!planned) {
        RCLCPP_WARN(log_, "Planning %s failed (attempt %d/3)", what, attempt);
      }
    }
    if (!planned) {
      return fail(std::string("planning failed: ") + what);
    }
    if (!static_cast<bool>(arm_->execute(plan))) {
      return fail(std::string("execution failed: ") + what);
    }
    return true;
  }

  // Short vertical move, collision checking off: these segments start or end in contact.
  bool straight(const Pose & target, const char * what)
  {
    arm_->setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory traj;
    const double frac = arm_->computeCartesianPath({target}, 0.005, 0.0, traj, false);
    if (frac < 0.99) {
      return fail(std::string("cartesian path infeasible: ") + what +
               " (" + std::to_string(static_cast<int>(frac * 100)) + "%)");
    }
    if (!static_cast<bool>(arm_->execute(traj))) {
      return fail(std::string("execution failed: ") + what);
    }
    return true;
  }

  bool fail(const std::string & why)
  {
    last_error_ = why;
    RCLCPP_ERROR(log_, "%s", why.c_str());
    return false;
  }

  // ee_link pose with the tool face just above the top of a box, tool down, turned to the
  // box's yaw. tool_length = ee_link origin -> tool face; grasp_clearance = gap left between
  // the tool face and the box top when attaching (and above the stack when placing).
  Pose toolPose(const Pose & box_center, double half_h) const
  {
    Pose p;
    p.position.x = box_center.position.x;
    p.position.y = box_center.position.y;
    p.position.z = box_center.position.z + half_h + grasp_clearance_ + tool_length_;
    p.orientation = multiply(yawQuat(yawOf(box_center.orientation)), grasp_q_);
    return p;
  }

  void setHold(bool on)
  {
    std_msgs::msg::Bool m;
    m.data = on;
    hold_pub_->publish(m);
  }

  static Quaternion rpyToQuat(double r, double p, double y)
  {
    const double cr = std::cos(r / 2), sr = std::sin(r / 2);
    const double cp = std::cos(p / 2), sp = std::sin(p / 2);
    const double cy = std::cos(y / 2), sy = std::sin(y / 2);
    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
  }

  static Quaternion yawQuat(double yaw) {return rpyToQuat(0.0, 0.0, yaw);}

  static double yawOf(const Quaternion & q)
  {
    return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  }

  static Quaternion multiply(const Quaternion & a, const Quaternion & b)
  {
    Quaternion q;
    q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return q;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger log_;
  std::unique_ptr<MoveGroupInterface> arm_;

  std::string group_, ee_link_;
  std::vector<std::string> touch_links_;
  double tool_length_, grasp_clearance_, approach_;
  Quaternion grasp_q_;

  std::mutex mtx_;
  std::optional<PalletTarget> next_;
  std::string busy_id_, done_id_, last_error_;

  rclcpp::Subscription<PalletTarget>::SharedPtr target_sub_;
  rclcpp::Publisher<PlaceResult>::SharedPtr result_pub_;
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
// proximity_sensor: a virtual photo-eye at the pick point. Reads the scene, never writes it.
//
// - Listens to conveyor/box_at_pick to learn which box to watch.
// - Keeps a live copy of the planning scene with a PlanningSceneMonitor and checks where
//   that box is, whether it still sits on the belt or is attached to the gripper.
// - Once the box has been seen at the pick point and then stays more than `clear_dist`
//   away (or vanishes from the scene) for `debounce_cycles` checks in a row, publishes
//   its id on conveyor/box_removed.
//
// Needs robot_description and robot_description_semantic as parameters (see the launch file).

#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "conveyor_sim/conveyor_common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace conveyor_sim
{

class ProximitySensor : public rclcpp::Node
{
public:
  ProximitySensor()
  : Node("proximity_sensor"), p_(declareConveyorParams(*this))
  {
    clear_dist_ = declare_parameter<double>("clear_dist", 0.08);    // m from the pick point
    debounce_ = declare_parameter<int64_t>("debounce_cycles", 3);
    rate_ = declare_parameter<double>("sensor_rate", 20.0);
    pick_pos_ = Eigen::Vector3d(p_.pick_x, p_.belt_y, p_.boxZ());

    removed_pub_ = create_publisher<std_msgs::msg::String>("conveyor/box_removed", 10);
    
    // create a subscribtion to box_at_pick topic
    at_pick_sub_ = create_subscription<std_msgs::msg::String>(
      "conveyor/box_at_pick", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {onAtPick(msg->data);});
  }

  // The monitor needs shared_from_this(), which isn't valid inside the constructor.
  void init()
  {
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      shared_from_this(), "robot_description");
    if (!psm_->getPlanningScene()) {
      throw std::runtime_error("Could not load the robot model: is robot_description set?");
    }
    psm_->startSceneMonitor("/monitored_planning_scene");  // diffs published by move_group
    psm_->startStateMonitor();                             // joint states, for attached boxes
    psm_->requestPlanningSceneState("/get_planning_scene"); // full scene once at startup

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_), [this]() {check();});
    RCLCPP_INFO(get_logger(), "Proximity sensor ready");
  }

private:
  void onAtPick(const std::string & id)
  {
    if (id.empty() || id == watched_ || id == last_removed_) {
      return;
    }
    watched_ = id;
    seen_at_pick_ = false;
    clear_count_ = 0;
    RCLCPP_INFO(get_logger(), "Watching %s", id.c_str());
  }


  
  void check()
  {
    if (watched_.empty()) {
      return;
    }

    bool in_scene = false;
    double dist = 0.0;
    {
      planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
      // Works for a box on the belt (world object) and a box attached to the gripper.
      if (scene->knowsFrameTransform(watched_)) {
        in_scene = true;
        dist = (scene->getFrameTransform(watched_).translation() - pick_pos_).norm();
      }
    }

    // Don't report anything until our copy of the scene has seen the box at the pick point.
    // Otherwise a scene that simply hasn't caught up yet would look like "removed".
    if (!seen_at_pick_) {
      seen_at_pick_ = in_scene && dist <= clear_dist_;
      return;
    }

    const bool clear = !in_scene || dist > clear_dist_;
    clear_count_ = clear ? clear_count_ + 1 : 0;

    if (clear_count_ >= debounce_) {
      std_msgs::msg::String msg;
      msg.data = watched_;
      removed_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "%s left the pick zone", watched_.c_str());
      last_removed_ = watched_;
      watched_.clear();
      clear_count_ = 0;
    }
  }

  ConveyorParams p_;
  double clear_dist_;
  int64_t debounce_;
  double rate_;
  Eigen::Vector3d pick_pos_;

  std::string watched_;
  std::string last_removed_;
  bool seen_at_pick_{false};
  int64_t clear_count_{0};

  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr removed_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr at_pick_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace conveyor_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<conveyor_sim::ProximitySensor>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

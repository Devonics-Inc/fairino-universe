// conveyor_manager: the orchestrator. The only node that owns the queue.
//
// - Keeps the belt filled to `num_boxes` by calling conveyor/spawn_box, one box at a time,
//   and only when the belt start is clear.
// - Assigns every box a queue slot and publishes them on conveyor/box_targets (latched).
// - When the proximity sensor reports conveyor/box_removed for the lead box, drops it,
//   shifts every other box forward one slot, and lets the fill logic spawn a replacement.
// - Publishes conveyor/box_present (Bool) and conveyor/box_at_pick (String id) for the
//   palletizer.

#include <cmath>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "conveyor_interfaces/msg/box_state_array.hpp"
#include "conveyor_interfaces/msg/box_target_array.hpp"
#include "conveyor_sim/conveyor_common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"     

using conveyor_interfaces::msg::BoxStateArray;
using conveyor_interfaces::msg::BoxTarget;
using conveyor_interfaces::msg::BoxTargetArray;
using std_srvs::srv::Trigger;

namespace conveyor_sim
{

class ConveyorManager : public rclcpp::Node
{
public:
  ConveyorManager()
  : Node("conveyor_manager"), p_(declareConveyorParams(*this))
  {
    num_boxes_ = declare_parameter<int64_t>("num_boxes", 3);   // boxes kept on the belt
    max_boxes_ = declare_parameter<int64_t>("max_boxes", 50);  // total boxes for the run
    const double rate = declare_parameter<double>("manager_rate", 10.0);

    if (num_boxes_ < 1 || static_cast<std::size_t>(num_boxes_) > p_.numSlots()) {
      throw std::invalid_argument(
              "num_boxes must be between 1 and " + std::to_string(p_.numSlots()) +
              " (the number of slots that fit between start_x and pick_x)");
    }

    targets_pub_ = create_publisher<BoxTargetArray>("conveyor/box_targets", latchedQos());
    present_pub_ = create_publisher<std_msgs::msg::Bool>("conveyor/box_present", 10);
    at_pick_pub_ = create_publisher<std_msgs::msg::String>("conveyor/box_at_pick", 10);

    states_sub_ = create_subscription<BoxStateArray>(
      "conveyor/box_states", 10,
      [this](BoxStateArray::ConstSharedPtr msg) {onStates(*msg);});
    removed_sub_ = create_subscription<std_msgs::msg::String>(
      "conveyor/box_removed", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {onRemoved(msg->data);});

    spawn_client_ = create_client<Trigger>("conveyor/spawn_box");

    pick_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("conveyor/box_pick_pose", 10);

    publishTargets();  // start from an empty belt
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this]() {tick();});
  }

private:
  void tick()
  {
    maybeSpawn();
    publishStatus();
  }

  // ---------- filling the belt ----------
  void maybeSpawn()
  {
    if (spawn_in_flight_ ||
      static_cast<int64_t>(queue_.size()) >= num_boxes_ ||
      spawned_ >= max_boxes_ ||
      !entryClear())
    {
      return;
    }
    if (!spawn_client_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for conveyor/spawn_box");
      return;
    }

    spawn_in_flight_ = true;
    spawn_client_->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this](rclcpp::Client<Trigger>::SharedFuture future) {onSpawned(*future.get());});
  }

  void onSpawned(const Trigger::Response & res)
  {
    spawn_in_flight_ = false;
    if (!res.success) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Spawn failed: %s (will retry)", res.message.c_str());
      return;
    }
    ++spawned_;
    queue_.push_back(res.message);
    publishTargets();
  }

  // The belt start is clear once the last box has moved at least one pitch away from it.
  bool entryClear() const
  {
    if (queue_.empty()) {
      return true;
    }
    auto it = positions_.find(queue_.back());
    if (it == positions_.end()) {
      return false;  // just spawned, advancer hasn't reported it yet
    }
    return it->second - p_.start_x >= p_.pitch() - kEps;
  }

  // ---------- reacting to the sensor ----------
  void onRemoved(const std::string & id)
  {
    if (queue_.empty() || queue_.front() != id) {
      RCLCPP_WARN(get_logger(), "Ignoring removal of '%s': not the lead box", id.c_str());
      return;
    }
    queue_.pop_front();
    positions_.erase(id);
    RCLCPP_INFO(get_logger(), "%s removed, advancing queue", id.c_str());
    publishTargets();
  }

  void onStates(const BoxStateArray & msg)
  {
    positions_.clear();
    for (const auto & b : msg.boxes) {
      positions_[b.id] = b.x;
    }
  }

  // ---------- outputs ----------
  void publishTargets()
  {
    BoxTargetArray msg;
    for (std::size_t k = 0; k < queue_.size(); ++k) {
      BoxTarget t;
      t.id = queue_[k];
      t.target_x = p_.slotX(k);
      msg.targets.push_back(t);
    }
    targets_pub_->publish(msg);
  }

  void publishStatus()
  {
    bool at_pick = false;
    double lead_x = 0.0;
    if (!queue_.empty()) {
      auto it = positions_.find(queue_.front());
      if (it != positions_.end() && std::abs(it->second - p_.pick_x) < kEps) {
        at_pick = true;
        lead_x = it->second;
      }
    }


    
    std_msgs::msg::Bool present;  
    
    present.data = at_pick;
    present_pub_->publish(present);

    std_msgs::msg::String id;
    id.data = at_pick ? queue_.front() : "";
    at_pick_pub_->publish(id);

    // Only publish a pose while a box is actually at the pick point
    if (at_pick) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = now();
      pose.header.frame_id = p_.frame_id;
      pose.pose = p_.boxPose(lead_x);        // x, belt_y, box center z
      pick_pose_pub_->publish(pose);
    }
  }
  ConveyorParams p_;
  int64_t num_boxes_;
  int64_t max_boxes_;

  std::deque<std::string> queue_;                       // lead box first
  std::unordered_map<std::string, double> positions_;   // latest x from the advancer
  int64_t spawned_{0};
  bool spawn_in_flight_{false};

  rclcpp::Publisher<BoxTargetArray>::SharedPtr targets_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr present_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr at_pick_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pick_pose_pub_;
  rclcpp::Subscription<BoxStateArray>::SharedPtr states_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr removed_sub_;
  rclcpp::Client<Trigger>::SharedPtr spawn_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace conveyor_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<conveyor_sim::ConveyorManager>());
  rclcpp::shutdown();
  return 0;
}

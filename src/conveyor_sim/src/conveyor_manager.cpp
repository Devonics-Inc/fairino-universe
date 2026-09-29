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


// this node is responsible for the following 
// - publish box targets (box_id, box_target)
// - determine whether to send a spwan request or not 
// - hold infomration regarding the total number of spawned boxes and removed
// -
class ConveyorManager : public rclcpp::Node
{
public:
  ConveyorManager()
  : Node("conveyor_manager"), p_(declareConveyorParams(*this))
  {
    // capture the max number of boxes on belt during simulation 
    maxBoxesOnBelt_ = declare_parameter<int64_t>("num_boxes", 3);
    num_boxes_ = declare_parameter<int64_t>("num_boxes", 3);   // boxes kept on the belt

    // capture the total number of boxes on the pallet
    totalPalletBoxes_ = declare_parameter<int64_t>("max_boxes", 50); 
    max_boxes_ = declare_parameter<int64_t>("max_boxes", 50);  // total boxes for the run
    const double rate = declare_parameter<double>("manager_rate", 10.0);

    // make sure that the max boxes on belts are geometrically feasible 
    validateNumberOfBoxes();


    // create a publisher for the box targets {box_id_   box_target_x}
    boxTargets_Pub = create_publisher<BoxTargetArray>("conveyor/box_targets", latchedQos());
    
    // the targets is usually consumed by the adavncer 


    // if there is any object at the pick up this publisher inform the robot with the coordinates
    // and whether there is box or not 
    isBoxAtPickUp_pub_ = create_publisher<std_msgs::msg::Bool>("conveyor/box_present", 10);
    // id of the box at the pickup 
    at_pick_pub_ = create_publisher<std_msgs::msg::String>("conveyor/box_at_pick", 10);

    // subscriber to the state of the box 
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
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / 30), [this]() {tick();});
  }

private:

  void tick()
  {
    trySpawnNextBox();
    publishStatus();
  }

  void validateNumberOfBoxes()
  {
    if (num_boxes_ < 1 || static_cast<std::size_t>(num_boxes_) > p_.numSlots()) {
      throw std::invalid_argument(
              "num_boxes must be between 1 and " + std::to_string(p_.numSlots()) +
              " (the number of slots that fit between start_x and pick_x)");
    }
  }


  // isMaxCapacityReached()
  // this function returns whther we reached max capacity or not 
  // within the current belt configuration 
  bool isMaxCapacityReached()
  {
    return static_cast<int64_t>(queue_.size()) >= num_boxes_ ;
  }

  bool spawnedAllBoxes()
  {
    return spawned_ >= max_boxes_;
  }

  bool isSpawnServiceNotReady()
  {
    if(spawn_client_->service_is_ready())
    {      
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for conveyor/spawn_box");

      return false;
    }
    else
    {
      return true;
    }
  }

  // trySpawnNextBox
  // this function validates wether we need to spawn something for this tick or not
  // then sends a request for the spawn service 
  void trySpawnNextBox()
  {
    // validate that we can or should spawn a new box ! 
    // if we already reqeusted a spawn from the box_Spawner or we reached max number of boxes
    // or we already spawned the max number of boxes 
    if (isPreviousSpawnInProcess_ ||
        isMaxCapacityReached() ||
        spawnedAllBoxes() ||
        isSpawnAreaNotClear() ||
        isSpawnServiceNotReady()
      
      )

    {
      // skip the rest
      return;
    }


    // set the variable that represent process to true and send spawn reqeust
    isPreviousSpawnInProcess_ = true;
    spawn_client_->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this](rclcpp::Client<Trigger>::SharedFuture future) {onSpawned(*future.get());});
  }

  void onSpawned(const Trigger::Response & res)
  {
    isPreviousSpawnInProcess_ = false;
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
  bool isSpawnAreaNotClear() const
  {

    bool notclear = true;
    bool clear = false;

    // if we have zero boxes on the belt, the area is clear(not clear = false)
    if (queue_.empty()) {
      return clear;
    }

    auto it = positions_.find(queue_.back());
    if (it == positions_.end()) {
      return notclear;  // just spawned, advancer hasn't moved it yet
    }
    return !(it->second - p_.start_x >= p_.pitch() - kEps);
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


  // update the positions values
  void onStates(const BoxStateArray & msg)
  {
    positions_.clear();
    for (const auto & b : msg.boxes) {
      positions_[b.id] = b.x;
    }
  }

  // this function build array of targets and publish them using boxTargets_Pub
  void publishTargets()
  {
    BoxTargetArray msg;
    for (std::size_t k = 0; k < queue_.size(); ++k) {
      BoxTarget t;
      t.id = queue_[k];
      t.target_x = p_.slotX(k);
      msg.targets.push_back(t);
    }
    boxTargets_Pub->publish(msg);
  }

  void publishStatus()
  {
    bool at_pick = false;
    double lead_x = 0.0;

    // only if the queue have elements check if the first element reached location
    if (!queue_.empty()) {
      auto it = positions_.find(queue_.front());
      if (it != positions_.end() && std::abs(it->second - p_.pick_x) < kEps) {
        at_pick = true;
        lead_x = it->second;
      }
    }


    
    std_msgs::msg::Bool present;  
    
    present.data = at_pick;
    isBoxAtPickUp_pub_->publish(present);

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

  int64_t maxBoxesOnBelt_ ;
  int64_t totalPalletBoxes_;
  std::deque<std::string> queue_;                       // lead box first

  // unordered map that connect between id and position
  std::unordered_map<std::string, double> positions_;   // latest x from the advancer
  int64_t spawned_{0};
  bool isPreviousSpawnInProcess_{false};

  rclcpp::Publisher<BoxTargetArray>::SharedPtr boxTargets_Pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr isBoxAtPickUp_pub_;
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

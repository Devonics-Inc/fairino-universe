// box_advancer: moves boxes along the belt, and nothing else.
//
// - Subscribes to conveyor/box_targets: the full list of boxes (lead first) and the
//   slot each one should stop at. It doesn't decide the queue, it just follows it.
// - Every tick, moves each box toward its target at belt speed, never closer than one
//   pitch to the box ahead, and publishes MOVE diffs to /planning_scene.
// - Publishes conveyor/box_states (current x of every box).

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "conveyor_interfaces/msg/box_state_array.hpp"
#include "conveyor_interfaces/msg/box_target_array.hpp"
#include "conveyor_sim/conveyor_common.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using conveyor_interfaces::msg::BoxState;
using conveyor_interfaces::msg::BoxStateArray;
using conveyor_interfaces::msg::BoxTargetArray;
using moveit_msgs::msg::CollisionObject;
using moveit_msgs::msg::PlanningScene;

namespace conveyor_sim
{

// this node is responsible for 
//  publish scene updates (box_location_   box_id_)
//  publish box states (id & x)
//  listen to the target need to be reached by each box 
class BoxAdvancer : public rclcpp::Node
{
public:
  BoxAdvancer()
  : Node("box_advancer"), p_(declareConveyorParams(*this))
  {
    // configure the belt speed 
    speed_ = declare_parameter<double>("speed", 0.15);   // m/s
    const double rate = declare_parameter<double>("rate", 50.0);
    dt_ = 1.0 / rate;
    

    // publisher for scene updates
    scene_pub_ = create_publisher<PlanningScene>("/planning_scene", 10);

    // publisher for box states (id , distance)
    states_pub_ = create_publisher<BoxStateArray>("conveyor/box_states", 10);


    // subscriper to each box targets 
    targets_sub_ = create_subscription<BoxTargetArray>(
      "conveyor/box_targets", latchedQos(),
      [this](BoxTargetArray::ConstSharedPtr msg) {onTargets(*msg);});

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(dt_), [this]() {step();});


    // conveyor/hold: true = belt stopped. Latched, so a late start still sees the last state.
    hold_sub_ = create_subscription<std_msgs::msg::Bool>(
      "conveyor/hold", rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data != hold_) {
          RCLCPP_INFO(get_logger(), "Belt %s", msg->data ? "HELD" : "released");
        }
        hold_ = msg->data;
      });
  }

private:
  struct Box
  {
    std::string id;
    double x;
    double target;
  };


// onTargets()
// Take the new list of boxes that should currently exist, 
// update their positions while preserving existing positions, 
// and remove boxes that disappeared from the list.
  void onTargets(const BoxTargetArray & msg)
  {
    // create a new vector of the type box 
    std::vector<Box> next;
    // Allocate enough memeory to capture all elements 
    next.reserve(msg.targets.size());
    
    for (const auto & t : msg.targets) {
      // get iterator toward the element where new_msg_target_id == old_box_id
      auto it = std::find_if(
        boxes_.begin(), boxes_.end(), [&](const Box & b) {return b.id == t.id;});
      
      // only if first time to see this box id give it the start_x
      const double x = (it != boxes_.end()) ? it->x : p_.start_x;
      next.push_back({t.id, x, t.target_x});
    }
    // make boxes_ take the next content 
    boxes_ = std::move(next);
  }

  void step()
  {
    std::vector<CollisionObject> objs;

    for (std::size_t i = 0; !hold_ &&  i < boxes_.size(); ++i) {
      Box & b = boxes_[i];
      double allowed = b.target;
      if (i > 0) {
        allowed = std::min(allowed, boxes_[i - 1].x - p_.pitch());  // never hit the box ahead
      }
      const double new_x = std::min(b.x + speed_ * dt_, allowed);
      if (new_x > b.x + kEps) {
        b.x = new_x;
        objs.push_back(makeMove(b));
      }
    }

    if (!objs.empty()) {
      PlanningScene scene;
      scene.is_diff = true;
      scene.robot_state.is_diff = true;
      scene.world.collision_objects = std::move(objs);
      scene_pub_->publish(scene);
    }

    BoxStateArray states;
    for (const auto & b : boxes_) {
      BoxState s;
      s.id = b.id;
      s.x = b.x;
      states.boxes.push_back(s);
    }
    states_pub_->publish(states);
  }

  CollisionObject makeMove(const Box & b) const
  {
    CollisionObject obj;
    obj.header.frame_id = p_.frame_id;
    obj.id = b.id;
    obj.operation = CollisionObject::MOVE;
    obj.pose = p_.boxPose(b.x);
    return obj;
  }

  ConveyorParams p_;
  double speed_;
  double dt_;
  std::vector<Box> boxes_;   // lead box first

  rclcpp::Publisher<PlanningScene>::SharedPtr scene_pub_;
  rclcpp::Publisher<BoxStateArray>::SharedPtr states_pub_;
  rclcpp::Subscription<BoxTargetArray>::SharedPtr targets_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool hold_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_sub_;
};

}  // namespace conveyor_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<conveyor_sim::BoxAdvancer>());
  rclcpp::shutdown();
  return 0;
}

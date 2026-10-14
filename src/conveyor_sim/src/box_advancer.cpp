// box_advancer: moves boxes along the belt, and nothing else.
//
// - Subscribes to conveyor/box_targets: the full list of boxes (lead first) and the
//   slot each one should stop at. It doesn't decide the queue, it just follows it.
// - Every tick, moves each box toward its target at belt speed, never closer than one
//   pitch to the box ahead.
// - Publishes conveyor/box_states (current x of every box). scene_gateway turns those
//   into MOVE diffs on the planning scene.
// - sim_speed (std_msgs/Float64): multiplier on the belt speed (1.0 = normal).

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "conveyor_interfaces/msg/box_state_array.hpp"
#include "conveyor_interfaces/msg/box_target_array.hpp"
#include "conveyor_sim/conveyor_common.hpp"
#include "conveyor_sim/scene_gateway.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

using conveyor_interfaces::msg::BoxState;
using conveyor_interfaces::msg::BoxStateArray;
using conveyor_interfaces::msg::BoxTargetArray;


namespace conveyor_sim
{

// this node is responsible for mainting the state of the boxes
//  publish box states (id & x)
//  listen to the target need to be reached by each box
//  parameters: beltSpeed_ , rate
//  topics: conveyor/hold (stop/release), sim_speed (speed multiplier)
class BoxAdvancer : public rclcpp::Node
{
public:
  BoxAdvancer()
  : Node("box_advancer"), p_(declareConveyorParams(*this))
  {
    // capture belt speed configuration
    beltSpeed_ = declare_parameter<double>("speed", 0.15);   // m/s


    // publisher for box states (id , distance)
    states_pub_ = create_publisher<BoxStateArray>("conveyor/box_states", 10);


    // subscriber to targets published by the conveyer_manager
    targets_sub_ = create_subscription<BoxTargetArray>(
      "conveyor/box_targets", latchedQos(),
      [this](BoxTargetArray::ConstSharedPtr msg) {onReceivingNewTargets(*msg);});

    // timer logic
    const double rate = declare_parameter<double>("rate", 50.0);
    solverStep_ = 1.0 / rate;
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(solverStep_), [this]() {step();});


    // conveyor/hold: true = belt stopped. Latched, so a late start still sees the last state.
    hold_sub_ = create_subscription<std_msgs::msg::Bool>(
      "conveyor/hold", rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data != hold_) {
          RCLCPP_INFO(get_logger(), "Belt %s", msg->data ? "HELD" : "released");
        }
        hold_ = msg->data;
      });

    // sim_speed: multiplier on the belt speed (1.0 = normal, 2.0 = twice as fast).
    // Latched, so a late start still sees the last value. Use conveyor/hold to stop the belt.
    sim_speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      "sim_speed", rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {onSimSpeed(msg->data);});
  }

private:
  struct Box
  {
    std::string id;
    double x;
    double target;
  };


// onReceivingNewTargets()
// Take the new target list of boxes that should currently exist,
// update their positions while preserving existing positions,
// and remove boxes that disappeared from the list.
  void onReceivingNewTargets(const BoxTargetArray & msg)
  {
    // create a new vector of the type box  & ensure enough memory is allocated
    std::vector<Box> updatedBoxVector;
    updatedBoxVector.reserve(msg.targets.size());

    // loop over each box in the message
    for (const auto & t : msg.targets) {
      // get iterator toward the element where new_msg_target_id == old_box_id
      auto it = std::find_if(
        boxes_.begin(), boxes_.end(), [&](const Box & b) {return b.id == t.id;});

      // only if first time to see this box id give it the start_x
      const double x = (it != boxes_.end()) ? it->x : p_.start_x;
      updatedBoxVector.push_back({t.id, x, t.target_x});
    }
    // make boxes_ take the next content
    boxes_ = std::move(updatedBoxVector);
  }

  // reject values that would stop or reverse the belt; hold is the way to stop it
  void onSimSpeed(double factor)
  {
    if (!std::isfinite(factor) || factor <= 0.0) {
      RCLCPP_WARN(get_logger(), "sim_speed %.3f ignored (must be > 0)", factor);
      return;
    }
    simSpeed_ = factor;
    RCLCPP_INFO(
      get_logger(), "sim_speed x%.2f -> belt %.3f m/s", simSpeed_, beltSpeed_ * simSpeed_);
  }


  // function call back on each step
  // advance box at each tick toward their targets
  void step()
  {
    // distance a box can travel this tick
    const double stepDistance = beltSpeed_ * simSpeed_ * solverStep_;

    // loop on the boxes as long as hold_ flag not raised
    for (std::size_t i = 0; !hold_ && i < boxes_.size(); ++i) {
      Box & b = boxes_[i];
      // allowed variable by default is the target movement unless we modify it later
      double allowed = b.target;

      // the allowed variable is only modified if we are in the second box
      // allowed is a cap on how far we can advance a box
      if (i > 0) {
        allowed = std::min(allowed, boxes_[i - 1].x - p_.pitch());  // never hit the box ahead
      }

      const double new_x = std::min(b.x + stepDistance, allowed);
      // if there is any small change in the state of the box we need to do an update
      if (new_x > b.x + kEps) {
        b.x = new_x;
      }
    }

    BoxStateArray states;
    states.boxes.reserve(boxes_.size());
    for (const auto & b : boxes_) {
      BoxState s;
      s.id = b.id;
      s.x = b.x;
      states.boxes.push_back(s);
    }
    states_pub_->publish(states);
  }


  ConveyorParams p_;
  double beltSpeed_;
  double simSpeed_{1.0};   // multiplier from the sim_speed topic
  double solverStep_;
  std::vector<Box> boxes_;   // lead box first

  rclcpp::Publisher<BoxStateArray>::SharedPtr states_pub_;
  rclcpp::Subscription<BoxTargetArray>::SharedPtr targets_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool hold_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sim_speed_sub_;
};

}  // namespace conveyor_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // single-threaded: hold_, simSpeed_ and boxes_ are never touched concurrently
  rclcpp::spin(std::make_shared<conveyor_sim::BoxAdvancer>());
  rclcpp::shutdown();
  return 0;
}
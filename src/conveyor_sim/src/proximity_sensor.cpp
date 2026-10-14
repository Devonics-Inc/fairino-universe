// proximity_sensor: a virtual photo-eye at the pick point. Reads the scene, never writes it.
//
// - Listens to conveyor/box_at_pick to learn which box to watch.
// - Polls move_group's /get_planning_scene for world objects only, so it needs no robot
//   model: a box on the belt is a world object; once picked it is attached to the gripper
//   (no longer a world object) or moved/removed.
// - Once the box has been seen at the pick point and then stays more than `clear_dist`
//   away (or is no longer a world object) for `debounce_cycles` checks in a row,
//   publishes its id on conveyor/box_removed.

#include <cmath>
#include <memory>
#include <string>

#include "conveyor_sim/conveyor_common.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using moveit_msgs::msg::PlanningSceneComponents;
using moveit_msgs::srv::GetPlanningScene;

namespace conveyor_sim
{

// conveyor manager publish the box id at the pick up point | the conveyor queue of boxes id and state 

// this node is responsible for 
// - judge if that box left the pick point (conveyor/box_removed)
// - if so tell the manager that the box is removed by publishing the box_id
class ProximitySensor : public rclcpp::Node
{
public:
  ProximitySensor()
  : Node("proximity_sensor"), p_(declareConveyorParams(*this))
  {
    // distance needed to consider the object cleared from the belt 
    clear_dist_ = declare_parameter<double>("clear_dist", 0.08);    // m from the pick point

    // number of times needed to check for box removal from the belt 
    debounce_ = declare_parameter<int64_t>("debounce_cycles", 3);

    // rate at which proximity gets checked
    rate_ = declare_parameter<double>("rate", 20.0);

    // position for pick and place
    pick_x_ = p_.pick_x;
    pick_y_ = p_.belt_y;
    pick_z_ = p_.boxZ();

    // publish the id of the box removed
    removed_pub_ = create_publisher<std_msgs::msg::String>("conveyor/box_removed", 10);

    // create a subscription to the id of a box arriving at pickup
    at_pick_sub_ = create_subscription<std_msgs::msg::String>(
      "conveyor/box_at_pick", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {onAtPick(msg->data);});

    // move_group's scene query service
    scene_client_ = create_client<GetPlanningScene>("/get_planning_scene");

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_), [this]() {check();});
    RCLCPP_INFO(get_logger(), "Proximity sensor ready");
  }

private:

  void onAtPick(const std::string & id)
  {
    // ignore empty id, or any old id we might saw before (watched , last_removed)
    if (id.empty() || id == watched_ || id == last_removed_) {
      return;
    }
    watched_ = id;
    seen_at_pick_ = false;
    clear_count_ = 0;
    RCLCPP_INFO(get_logger(), "Watching %s", id.c_str());
  }

  // ask move_group for the world objects (non-blocking, one request at a time)
  void check()
  {
    if (watched_.empty() || request_pending_) {
      return;
    }
    if (!scene_client_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for /get_planning_scene");
      return;
    }

    auto req = std::make_shared<GetPlanningScene::Request>();
    req->components.components =
      PlanningSceneComponents::WORLD_OBJECT_NAMES | PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

    request_pending_ = true;
    const std::string id = watched_;
    scene_client_->async_send_request(
      req, [this, id](rclcpp::Client<GetPlanningScene>::SharedFuture future) {
        request_pending_ = false;
        onScene(id, *future.get());
      });
  }

  void onScene(const std::string & id, const GetPlanningScene::Response & res)
  {
    // the watched box changed while the request was in flight: this answer is stale
    if (id != watched_) {
      return;
    }

    // look for the box among the world objects
    bool on_belt = false;
    double dist = 0.0;
    for (const auto & obj : res.scene.world.collision_objects) {
      if (obj.id == id) {
        const auto & p = obj.pose.position;
        dist = std::sqrt(
          (p.x - pick_x_) * (p.x - pick_x_) +
          (p.y - pick_y_) * (p.y - pick_y_) +
          (p.z - pick_z_) * (p.z - pick_z_));
        on_belt = true;
        break;
      }
    }

    // Don't report anything until the scene has shown the box at the pick point.
    // Otherwise a scene that simply hasn't caught up yet would look like "removed".
    if (!seen_at_pick_) {
      seen_at_pick_ = on_belt && dist <= clear_dist_;
      return;
    }

    // attached to the gripper / removed (not a world object), or moved away from the pick point
    const bool clear = !on_belt || dist > clear_dist_;
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
  double pick_x_;
  double pick_y_;
  double pick_z_;

  std::string watched_;
  std::string last_removed_;
  bool seen_at_pick_{false};
  int64_t clear_count_{0};
  bool request_pending_{false};

  rclcpp::Client<GetPlanningScene>::SharedPtr scene_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr removed_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr at_pick_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace conveyor_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<conveyor_sim::ProximitySensor>());
  rclcpp::shutdown();
  return 0;
}
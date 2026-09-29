// box_spawner: adds objects to the planning scene, and nothing else.
//
// - On startup, adds the static conveyor belt.
// - Offers conveyor/spawn_box (std_srvs/Trigger). Each call adds ONE box at the belt
//   start and replies with its id in `message`.
// - Uses move_group's /apply_planning_scene service, which only returns once the object
//   is really in the scene. That guarantees the advancer's first MOVE never arrives
//   before the ADD.

#include <array>
#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "conveyor_sim/conveyor_common.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using moveit_msgs::msg::CollisionObject;
using moveit_msgs::srv::ApplyPlanningScene;
using shape_msgs::msg::SolidPrimitive;
using std_srvs::srv::Trigger;

namespace conveyor_sim
{



// box_spawner is responsible only for putting the conveyor belt
// and boxes into MoveIt's Planning Scene. 
// It does NOT move boxes, track them, or control the robot.

class BoxSpawner : public rclcpp::Node
{
public:
  BoxSpawner()
  : Node("box_spawner"), p_(declareConveyorParams(*this))
  {
    // The spawn service waits for move_group's reply inside its callback. The client
    // therefore lives in its own callback group, and main() uses a multi-threaded
    // executor, so the reply can be processed while the service callback is waiting.
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    apply_client_ = create_client<ApplyPlanningScene>(
      "/apply_planning_scene", rmw_qos_profile_services_default, client_group_);

    spawn_srv_ = create_service<Trigger>(
      "conveyor/spawn_box",
      [this](const std::shared_ptr<Trigger::Request> /*req*/,
      std::shared_ptr<Trigger::Response> res) {onSpawn(*res);},
      rmw_qos_profile_services_default, service_group_);

    // Same group as the service: a spawn request can never run before the belt exists.
    init_timer_ = create_wall_timer(1s, [this]() {addBelt();}, service_group_);
  }

private:

  // spawn the belt object
  void addBelt()
  {
    // check if the server is ready before sending any requests
    if (!apply_client_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for /apply_planning_scene (is move_group running?)");
      return;
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = (p_.start_x + p_.pick_x) / 2.0;
    pose.position.y = p_.belt_y;
    pose.position.z = p_.belt_height / 2.0;
    pose.orientation.w = 1.0;
    const double length = (p_.pick_x - p_.start_x) + p_.box_size[0] + 0.10;

    if (apply(makeObject("conveyor", pose, {length, p_.belt_width, p_.belt_height}))) {
      belt_ready_ = true;
      init_timer_->cancel();
      RCLCPP_INFO(get_logger(), "Conveyor added to planning scene");
    }
  }

  void onSpawn(Trigger::Response & res)
  {
    if (!belt_ready_) {
      res.success = false;
      res.message = "scene not ready yet";
      return;
    }

    const std::string id = "box_" + std::to_string(count_ + 1);
    const auto pose = p_.boxPose(p_.start_x);
    if (apply(makeObject(id, pose, {p_.box_size[0], p_.box_size[1], p_.box_size[2]}))) {
      ++count_;
      res.success = true;
      res.message = id;
      RCLCPP_INFO(get_logger(), "Spawned %s", id.c_str());
    } else {
      res.success = false;
      res.message = "apply_planning_scene failed";
    }
  }

  // Blocking call to move_group. Safe here because of the callback groups (see ctor).
  bool apply(const CollisionObject & obj)
  {
    auto req = std::make_shared<ApplyPlanningScene::Request>();
    req->scene.is_diff = true;
    req->scene.robot_state.is_diff = true;
    req->scene.world.collision_objects.push_back(obj);

    auto result = apply_client_->async_send_request(req);
    if (result.future.wait_for(5s) != std::future_status::ready) {
      apply_client_->remove_pending_request(result.request_id);
      RCLCPP_WARN(get_logger(), "apply_planning_scene timed out for '%s'", obj.id.c_str());
      return false;
    }
    return result.future.get()->success;
  }

  CollisionObject makeObject(
    const std::string & id, const geometry_msgs::msg::Pose & pose,
    const std::array<double, 3> & dims) const
  {
    CollisionObject obj;
    obj.header.frame_id = p_.frame_id;
    obj.id = id;
    obj.operation = CollisionObject::ADD;
    obj.pose = pose;

    SolidPrimitive prim;
    prim.type = SolidPrimitive::BOX;
    prim.dimensions.resize(3);
    prim.dimensions[SolidPrimitive::BOX_X] = dims[0];
    prim.dimensions[SolidPrimitive::BOX_Y] = dims[1];
    prim.dimensions[SolidPrimitive::BOX_Z] = dims[2];
    obj.primitives.push_back(prim);

    geometry_msgs::msg::Pose identity;
    identity.orientation.w = 1.0;
    obj.primitive_poses.push_back(identity);
    return obj;
  }

  ConveyorParams p_;
  bool belt_ready_{false};
  int64_t count_{0};

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::Client<ApplyPlanningScene>::SharedPtr apply_client_;
  rclcpp::Service<Trigger>::SharedPtr spawn_srv_;
  rclcpp::TimerBase::SharedPtr init_timer_;
};

}  // namespace conveyor_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<conveyor_sim::BoxSpawner>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

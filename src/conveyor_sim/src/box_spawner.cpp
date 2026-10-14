// box_spawner: decides what box to create and when. Never touches the planning scene.
//
// - Offers conveyor/spawn_box (std_srvs/Trigger). Each call builds ONE box at the belt
//   start and asks scene_gateway (scene_gateway/add_object) to add it.
// - Replies with the box id in `message` only once scene_gateway confirmed the box is
//   really in the scene. The id counter only advances on success.
// - Colors, MOVEs and removal are scene_gateway's job.

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "conveyor_sim/conveyor_common.hpp"
#include "conveyor_sim/scene_gateway.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using moveit_msgs::srv::ApplyPlanningScene;
using std_srvs::srv::Trigger;

namespace conveyor_sim
{

class BoxSpawner : public rclcpp::Node
{
public:
  BoxSpawner()
  : Node("box_spawner"), p_(declareConveyorParams(*this))
  {
    // The spawn service waits for scene_gateway's reply inside its callback, so the
    // client lives in its own callback group and main() uses a multi-threaded executor.
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // (box_spawner -c-> scene_gateway) scene_gateway/add_object
    gateway_client_ = create_client<ApplyPlanningScene>(
      "scene_gateway/add_object", rmw_qos_profile_services_default, client_group_);

    // (box_spawner <- conveyor_manager) conveyor/spawn_box
    spawn_srv_ = create_service<Trigger>(
      "conveyor/spawn_box",
      [this](const std::shared_ptr<Trigger::Request> /*req*/,
      std::shared_ptr<Trigger::Response> res) {onSpawn(*res);},
      rmw_qos_profile_services_default, service_group_);
  }

private:
  void onSpawn(Trigger::Response & res)
  {
    res.success = false;
    if (!gateway_client_->service_is_ready()) {
      res.message = "scene_gateway not available";
      return;
    }

    const std::string id = "box_" + std::to_string(count_ + 1);

    // build the box; scene_gateway paints it and tracks it once move_group confirms
    auto req = std::make_shared<ApplyPlanningScene::Request>();
    req->scene.is_diff = true;
    req->scene.robot_state.is_diff = true;
    req->scene.world.collision_objects.push_back(
      makeCollisionObjectBox(
        p_.frame_id, id, p_.boxPose(p_.start_x),
        {p_.box_size[0], p_.box_size[1], p_.box_size[2]}));

    auto result = gateway_client_->async_send_request(req);
    // a bit longer than scene_gateway's own 5 s wait on move_group, so its timeout reports first
    if (result.future.wait_for(6s) != std::future_status::ready) {
      gateway_client_->remove_pending_request(result.request_id);
      res.message = "scene_gateway timed out";
      return;
    }
    if (!result.future.get()->success) {
      // the reason is in scene_gateway's log (belt not ready / move_group failed)
      res.message = "scene_gateway refused the ADD";
      return;
    }

    ++count_;
    res.success = true;
    res.message = id;
    RCLCPP_INFO(get_logger(), "Spawned %s", id.c_str());
  }

  ConveyorParams p_;
  int64_t count_{0};

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::Client<ApplyPlanningScene>::SharedPtr gateway_client_;
  rclcpp::Service<Trigger>::SharedPtr spawn_srv_;
};

}  // namespace conveyor_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<conveyor_sim::BoxSpawner>();
  // multi-threaded: onSpawn() blocks while waiting for scene_gateway's reply
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
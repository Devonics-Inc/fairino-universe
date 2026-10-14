// scene_gateway: the only node that writes to MoveIt's planning scene.

// on startup - add the conveyor belt

// offers scene_gateway/add_object (moveit_msgs/ApplyPlanningScene) to add a box or other object to the scene
// the collision object is added to the scene via move_group's /apply_planning_scene service.

// listens to conveyor/box_states (published by box_advancer) and turns every box whose
// x changed into a MOVE diff on /planning_scene. A box is only moved after its ADD was confirmed,
// so a MOVE can never reach the scene before the object exists.

// owns box colors (conveyor/box_color):
//  - new boxes are painted inside their ADD request (same confirmed call)
//  - on a color change, existing boxes are repainted on the next box_states tick,
//    inside the MOVE diff that tick sends anyway (single publish point)

#include <chrono>
#include <future>
#include <memory>
#include <vector>
#include <mutex>
#include <string>
#include <unordered_map>
#include <cmath>
#include <utility>
#include <optional>
#include <unordered_set>

#include "conveyor_sim/scene_gateway.hpp"
#include "conveyor_sim/conveyor_common.hpp"
#include "conveyor_interfaces/msg/box_state_array.hpp"

#include "moveit_msgs/msg/object_color.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"


using conveyor_interfaces::msg::BoxStateArray;
using namespace std::chrono_literals;
using moveit_msgs::msg::CollisionObject;
using moveit_msgs::srv::ApplyPlanningScene;
using moveit_msgs::msg::ObjectColor;
using moveit_msgs::msg::PlanningScene;
using std_msgs::msg::ColorRGBA;

namespace conveyor_sim
{

    class SceneGateway : public rclcpp::Node
    {
        public:
        SceneGateway() : Node("scene_gateway") , p_(declareConveyorParams(*this))
        {
            //make groups for the callbacks (callbacks of service, client and states subscription)
            service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            client_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            state_sub_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


            // (scene_gateway -c-> move_group) /apply_planning_scene
            // create a move group client to apply planning scene request (eg: spawn objects, move objects)
            move_group_client = create_client<ApplyPlanningScene>("/apply_planning_scene", rmw_qos_profile_services_default, client_group_);

            // (scene_gatway <- any_node)  scene_gateway/add_object
            // offer add_object service to other nodes to add objects to the planning scene via move_group's /apply_planning_scene service
            add_move_group_obj_srv = create_service<ApplyPlanningScene>(
                "scene_gateway/add_object",
                [this](const std::shared_ptr<ApplyPlanningScene::Request> req,
                std::shared_ptr<ApplyPlanningScene::Response> res) {onAddObject(*req, *res);},
                rmw_qos_profile_services_default, service_group_);


            // (scene_gatway -P-> move_group) /planning_scene  | delta in planning scene
            // publish the changes in the box positions to move_group
            change_in_scene_pub_ = create_publisher<PlanningScene>("/planning_scene", 10);


            // (scene_gateway <-P- box_advancer)  conveyor/box_states
            // (jmpto => onBoxPosStateUpdate => convert the box states into collision object changes)
            // NOTE: volatile QoS on purpose. A transient_local subscriber is incompatible
            // with the advancer's volatile publisher and would receive nothing.
            rclcpp::SubscriptionOptions states_opts;
            states_opts.callback_group = state_sub_group;
            box_state_sub_ = create_subscription<BoxStateArray>(
                "conveyor/box_states", 10,
                [this](const BoxStateArray::SharedPtr msg) {onBoxPosStateUpdate(*msg);},
                states_opts);

            // (scene_gateway <-P- any_node)  conveyor/box_color
            // (jmpto => onBoxColorUpdate => store color, mark boxes for repaint)
            // latched, so a color set before this node started still applies
            boxes_color_sub_ = create_subscription<ColorRGBA>(
                "conveyor/box_color", rclcpp::QoS(1).transient_local().reliable(),
                [this](const ColorRGBA::SharedPtr msg) {onBoxColorUpdate(*msg);},
                states_opts);


            // (scene_gateway <-P- proximity_sensor)  conveyor/box_removed
            // a picked box is no longer on the belt: stop moving / coloring it
            box_removed_sub_ = create_subscription<std_msgs::msg::String>(
                "conveyor/box_removed", 10,
                [this](const std_msgs::msg::String::SharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(scope_mutex_);
                    boxes_in_scene.erase(msg->data);
                    color_pending_.erase(msg->data);
                },
                states_opts
            );

            // ToDO : turn this to a seperate function so that convyeor_manager requests it
            // (scene_gatway <- ros timer every 500ms till cancelled)
            // same group as add_object: no ADD can run in parallel with the belt setup
            init_timer_ = create_wall_timer(500ms, [this]() {addBelt();}, service_group_);
        }

        private:

       void publishSceneDiff(std::vector<CollisionObject> objs = {}, std::vector<ObjectColor> colors = {})
        {
            if(objs.empty() && colors.empty())
            {
                return;
            }
            change_in_scene_pub_->publish(makeSceneDiff(std::move(objs), std::move(colors)));
        }
        bool isSceneReady()
        {
            return move_group_client->service_is_ready();
        }

        // (scene_gateway <-P- box_advancer)  conveyor/box_states
        // check which boxes should we update and what can be skipped
        // boxes that moved get a MOVE; boxes waiting for a repaint get their color
        // in the same diff (a repaint of a box standing still needs a same-pose MOVE,
        // because MoveIt only forwards diffs to RViz when geometry is touched)
        void onBoxPosStateUpdate(const BoxStateArray & msg)
        {
            std::vector<CollisionObject> objs_to_update_pos;
            std::vector<ObjectColor> colors;
            {
                std::lock_guard<std::mutex> lock(scope_mutex_);

                for(const auto & box : msg.boxes)
                {
                    // check if the box is already in the scene
                    auto boxes_in_scene_it = boxes_in_scene.find(box.id);
                    if(boxes_in_scene_it == boxes_in_scene.end())
                    {
                        // we can't move a box that we don't have in the scene yet
                        continue;
                    }

                    const bool moved = std::abs(box.x - boxes_in_scene_it->second) > kEps;
                    const bool repaint = color_pending_.erase(box.id) > 0;
                    if(!moved && !repaint)
                    {
                        continue;
                    }

                    // update the location of the box using the state from advancer
                    boxes_in_scene_it->second = box.x;
                    objs_to_update_pos.push_back(makeCollisionObjectMove(p_.frame_id, box.id, p_.boxPose(box.x)));

                    if(repaint && box_color_)
                    {
                        ObjectColor obj_color;
                        obj_color.id = box.id;
                        obj_color.color = *box_color_;
                        colors.push_back(std::move(obj_color));
                    }
                }
            }

            publishSceneDiff(std::move(objs_to_update_pos), std::move(colors));
        }

        // (scene_gateway <-P- any_node)  conveyor/box_color
        // nothing is published here: every known box is repainted on the next
        // box_states tick, inside the MOVE diff that tick sends anyway
        void onBoxColorUpdate(const ColorRGBA & msg)
        {
            std::lock_guard<std::mutex> lock(scope_mutex_);
            box_color_ = msg;
            for(const auto & box : boxes_in_scene)
            {
                color_pending_.insert(box.first);
            }
            RCLCPP_INFO(get_logger(), "Box color set to (%.2f, %.2f, %.2f, %.2f)",
                msg.r, msg.g, msg.b, msg.a);
        }



        void addBelt()
        {
            if(!isSceneReady())
            {
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Waiting for /apply_planning_scene (is move_group running?)");
                return;
            }

            geometry_msgs::msg::Pose BeltPose;
            BeltPose.position.x = (p_.start_x + p_.pick_x) / 2.0;
            BeltPose.position.y = p_.belt_y;
            BeltPose.position.z = p_.belt_height / 2.0;
            BeltPose.orientation.w = 1.0;
            const double length = (p_.pick_x - p_.start_x) + p_.box_size[0] + 0.10;

            CollisionObject obj = makeCollisionObjectBox(p_.frame_id, "conveyor_belt", BeltPose, {length, p_.belt_width, p_.belt_height});
            std::vector<CollisionObject> objs = {obj};
            if(publishCollisionObject(objs))
            {
                RCLCPP_INFO(get_logger(), "Conveyor belt added to planning scene");
                init_timer_->cancel();
                belt_spwaned_ = true;
            }
        }

        // register confirmed objects so their states can become MOVEs
        void updateObjectPositions(
            const std::vector<CollisionObject> & objs, const std::optional<ColorRGBA> & color_at_add)
        {
            std::lock_guard<std::mutex> lock(scope_mutex_);

            for(const auto & obj : objs)
            {
                boxes_in_scene[obj.id] = obj.pose.position.x;
                // the color changed while the ADD was waiting on move_group:
                // repaint this box on the next box_states tick
                if(box_color_ && box_color_ != color_at_add)
                {
                    color_pending_.insert(obj.id);
                }
            }
        }

        // confirmed write (ADD, later REMOVE/ATTACH). Blocks until move_group replies.
        // Safe because the client is in its own callback group (see ctor).
        bool publishCollisionObject(
            const std::vector<CollisionObject> & obj, const std::vector<ObjectColor> & colors = {})
        {
            auto request = std::make_shared<ApplyPlanningScene::Request>();
            request->scene.is_diff = true;
            request->scene.world.collision_objects = obj;
            request->scene.object_colors = colors;
            request->scene.robot_state.is_diff = true;

            auto result = move_group_client->async_send_request(request);

            // wait for timout of 5 seconds before returning false
            // timeout means max time window
            if(result.future.wait_for(5s) != std::future_status::ready)
            {
                move_group_client->remove_pending_request(result.request_id);
                RCLCPP_ERROR(get_logger(), "apply_planning_scene timed out");
                return false;
            }

            // if we get response store the result in a boolean and return it
            const bool resultstatus = result.future.get()->success;
            if(!resultstatus)
            {
                RCLCPP_ERROR(get_logger(), "Failed to apply planning scene");
            }
            return resultstatus;
        }

        // on receiving add object request from other nodes
        void onAddObject(const ApplyPlanningScene::Request & req, ApplyPlanningScene::Response & res)
        {
            // validation before spawning the object
            if(!isSceneReady() || !belt_spwaned_ || !isAddObjectReqNonEmpty(req) || !isAddObjectReqAdditionOnly(req))
            {
                RCLCPP_ERROR(get_logger(), "Invalid Add Object Request");
                res.success = false;
                return;
            }

            const auto & objs = req.scene.world.collision_objects;

            // new boxes are added already painted with the current color (if one was set)
            std::optional<ColorRGBA> color_at_add;
            {
                std::lock_guard<std::mutex> lock(scope_mutex_);
                color_at_add = box_color_;
            }
            std::vector<ObjectColor> colors;
            if(color_at_add)
            {
                for(const auto & obj : objs)
                {
                    ObjectColor obj_color;
                    obj_color.id = obj.id;
                    obj_color.color = *color_at_add;
                    colors.push_back(std::move(obj_color));
                }
            }

            // try publishing the objects requested; if it worked, update response and internal state
            if(publishCollisionObject(objs, colors))
            {
                RCLCPP_INFO(get_logger(), "Object added to planning scene");

                // update the internal state of the object position (boxes_in_scene)
                updateObjectPositions(objs, color_at_add);

                // update the response to true
                res.success = true;
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Failed to add object to planning scene");
                res.success = false;
            }
        }

        bool isAddObjectReqAdditionOnly(const ApplyPlanningScene::Request & req)
        {
            const auto & objs = req.scene.world.collision_objects;
            for(const auto & obj : objs)
            {
                if(obj.operation != CollisionObject::ADD)
                {
                    RCLCPP_ERROR(get_logger(), "Only ADD operation is allowed");
                    return false;
                }
            }
            return true;
        }

        bool isAddObjectReqNonEmpty(const ApplyPlanningScene::Request & req)
        {
            if(req.scene.world.collision_objects.empty())
            {
                RCLCPP_ERROR(get_logger(), "No collision objects in add_object request");
                return false;
            }
            return true;
        }

        bool belt_spwaned_{false};
        ConveyorParams p_;
        rclcpp::CallbackGroup::SharedPtr client_group_;
        rclcpp::CallbackGroup::SharedPtr service_group_;
        rclcpp::CallbackGroup::SharedPtr state_sub_group;
        rclcpp::Publisher<PlanningScene>::SharedPtr change_in_scene_pub_;
        rclcpp::Subscription<BoxStateArray>::SharedPtr box_state_sub_;
        std::optional<ColorRGBA> box_color_;   // unset = MoveIt's default color
        rclcpp::Subscription<ColorRGBA>::SharedPtr boxes_color_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr box_removed_sub_;

        rclcpp::Client<ApplyPlanningScene>::SharedPtr move_group_client;

        // initialization timer to add conveyor belt to the planning scene after move_group is ready
        rclcpp::TimerBase::SharedPtr init_timer_;

        // add move group object service, help us to add object to the scene when other nodes request it
        rclcpp::Service<ApplyPlanningScene>::SharedPtr add_move_group_obj_srv;

        // internal state of the object positions (boxes_in_scene) to be used for MOVE operations
        std::unordered_map<std::string, double> boxes_in_scene;

        // boxes whose color must be refreshed on the next box_states tick
        std::unordered_set<std::string> color_pending_;

        // mutex to protect access to boxes_in_scene, color_pending_ and box_color_
        std::mutex scope_mutex_;
    };

}  // namespace conveyor_sim

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<conveyor_sim::SceneGateway>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
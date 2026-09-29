// pallet_planner: decides WHERE each box goes. It never moves the robot.
//
// Inputs   conveyor/box_at_pick    std_msgs/String            id of the box at the pick point
//          conveyor/box_pick_pose  geometry_msgs/PoseStamped  that box's center
//          pallet/place_result     palletizer_msgs/PlaceResult (from the executor)
// Outputs  pallet/target           palletizer_msgs/PalletTarget (one box at a time)
//          pallet/status           palletizer_msgs/PalletStatus (latched)
//          pallet/slots            visualization_msgs/MarkerArray (latched, for RViz)
//          /planning_scene         the pallet as a collision object
// Service  ~/reset                 std_srvs/Trigger (start a new pallet)
//
// Pattern: fixed 2 x 2 per layer, `layers` high. Order: row by row, then layer by layer.
// A slot is consumed only when the executor reports success, so a failed box never
// leaves a hole in the pattern.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "palletizer_msgs/msg/pallet_status.hpp"
#include "palletizer_msgs/msg/pallet_target.hpp"
#include "palletizer_msgs/msg/place_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Quaternion;
using palletizer_msgs::msg::PalletStatus;
using palletizer_msgs::msg::PalletTarget;
using palletizer_msgs::msg::PlaceResult;

class PalletPlanner : public rclcpp::Node
{
public:
  static constexpr uint32_t kRows = 2;
  static constexpr uint32_t kCols = 2;
  static constexpr uint32_t kPerLayer = kRows * kCols;

  PalletPlanner()
  : Node("pallet_planner")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    pallet_top_ = declare_parameter<std::vector<double>>("pallet_top", {0.0, 0.6, 0.15});
    pallet_size_ = declare_parameter<std::vector<double>>("pallet_size", {0.8, 0.6, 0.15});
    box_size_ = declare_parameter<std::vector<double>>("box_size", {0.2, 0.2, 0.2});
    gap_ = declare_parameter<double>("gap", 0.01);
    layers_ = static_cast<uint32_t>(declare_parameter<int64_t>("layers", 2));
    place_yaw_ = declare_parameter<double>("place_yaw", 0.0);
    pose_timeout_ = declare_parameter<double>("pose_timeout", 0.5);
    spawn_pallet_ = declare_parameter<bool>("spawn_pallet", true);

    if (pallet_top_.size() != 3 || pallet_size_.size() != 3 || box_size_.size() != 3 ||
      layers_ == 0)
    {
      throw std::runtime_error(
              "pallet_top, pallet_size and box_size need 3 values each; layers must be >= 1");
    }
    checkFootprint();

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    target_pub_ = create_publisher<PalletTarget>("pallet/target", 10);
    status_pub_ = create_publisher<PalletStatus>("pallet/status", latched);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("pallet/slots", latched);
    scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

    id_sub_ = create_subscription<std_msgs::msg::String>(
      "conveyor/box_at_pick", 10, [this](std_msgs::msg::String::ConstSharedPtr m) {
        at_pick_id_ = m->data;
        tryAssign();
      });
    pose_sub_ = create_subscription<PoseStamped>(
      "conveyor/box_pick_pose", 10, [this](PoseStamped::ConstSharedPtr m) {
        pick_pose_ = *m;
        pose_time_ = std::chrono::steady_clock::now();
        tryAssign();
      });
    result_sub_ = create_subscription<PlaceResult>(
      "pallet/place_result", 10, [this](PlaceResult::ConstSharedPtr m) {onResult(*m);});

    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        next_slot_ = 0;
        pending_.reset();
        failed_ids_.clear();
        last_placed_id_.clear();
        publishStatus();
        publishMarkers();
        res->success = true;
        res->message = "Pallet reset (boxes already in the planning scene are not removed)";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });

    // Re-send the pending target (executor dedupes) and spawn the pallet once move_group is up.
    timer_ = create_wall_timer(1s, [this]() {tick();});

    publishStatus();
    publishMarkers();
    RCLCPP_INFO(get_logger(), "Pallet plan: %ux%u x %u layers = %u boxes, box %.3fx%.3fx%.3f",
      kRows, kCols, layers_, capacity(), box_size_[0], box_size_[1], box_size_[2]);
  }

private:
  uint32_t capacity() const {return kPerLayer * layers_;}

  // Axis-aligned footprint of one box after place_yaw.
  void footprint(double & fx, double & fy) const
  {
    const double c = std::abs(std::cos(place_yaw_)), s = std::abs(std::sin(place_yaw_));
    fx = c * box_size_[0] + s * box_size_[1];
    fy = s * box_size_[0] + c * box_size_[1];
  }

  void checkFootprint() const
  {
    double fx, fy;
    footprint(fx, fy);
    const double need_x = kRows * fx + (kRows - 1) * gap_;
    const double need_y = kCols * fy + (kCols - 1) * gap_;
    if (need_x > pallet_size_[0] + 1e-6 || need_y > pallet_size_[1] + 1e-6) {
      RCLCPP_WARN(get_logger(),
        "2x2 pattern needs %.3f x %.3f m but pallet is %.3f x %.3f m: boxes will overhang",
        need_x, need_y, pallet_size_[0], pallet_size_[1]);
    }
  }

  // Box center for a slot. The 2x2 block is centered on pallet_top.
  Pose slotPose(uint32_t row, uint32_t col, uint32_t layer) const
  {
    double fx, fy;
    footprint(fx, fy);
    Pose p;
    p.position.x = pallet_top_[0] + (static_cast<double>(row) - (kRows - 1) / 2.0) * (fx + gap_);
    p.position.y = pallet_top_[1] + (static_cast<double>(col) - (kCols - 1) / 2.0) * (fy + gap_);
    p.position.z = pallet_top_[2] + layer * box_size_[2] + box_size_[2] / 2.0;
    p.orientation = yawQuat(place_yaw_);
    return p;
  }

  void tryAssign()
  {
    if (pending_ || at_pick_id_.empty()) {
      return;
    }
    if (at_pick_id_ == last_placed_id_ || failed_ids_.count(at_pick_id_) > 0) {
      return;
    }
    if (next_slot_ >= capacity()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Pallet full, %s waiting. Call ~/reset after unloading.", at_pick_id_.c_str());
      return;
    }
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - pose_time_).count();
    if (age > pose_timeout_) {
      return;   // wait for a fresh pick pose
    }
    if (pick_pose_.header.frame_id != frame_id_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Pick pose frame '%s' != frame_id '%s' (no TF lookup is done)",
        pick_pose_.header.frame_id.c_str(), frame_id_.c_str());
      return;
    }

    PalletTarget t;
    t.header.stamp = now();
    t.header.frame_id = frame_id_;
    t.box_id = at_pick_id_;
    t.slot = next_slot_;
    t.layer = next_slot_ / kPerLayer;
    t.row = (next_slot_ % kPerLayer) / kCols;
    t.col = (next_slot_ % kPerLayer) % kCols;
    t.pick_pose = pick_pose_.pose;
    t.place_pose = slotPose(t.row, t.col, t.layer);
    t.box_size.x = box_size_[0];
    t.box_size.y = box_size_[1];
    t.box_size.z = box_size_[2];

    pending_ = t;
    target_pub_->publish(t);

    double rx, ry, rz;
    toRPY(t.place_pose.orientation, rx, ry, rz);
    RCLCPP_INFO(get_logger(),
      "%s -> slot %u (layer %u, row %u, col %u): "
      "x=%.3f y=%.3f z=%.3f rx=%.3f ry=%.3f rz=%.3f",
      t.box_id.c_str(), t.slot, t.layer, t.row, t.col,
      t.place_pose.position.x, t.place_pose.position.y, t.place_pose.position.z, rx, ry, rz);

    publishStatus();
    publishMarkers();
  }

  void onResult(const PlaceResult & r)
  {
    if (!pending_ || r.box_id != pending_->box_id) {
      RCLCPP_WARN(get_logger(), "Result for %s ignored (pending: %s)", r.box_id.c_str(),
        pending_ ? pending_->box_id.c_str() : "none");
      return;
    }
    if (r.success) {
      last_placed_id_ = r.box_id;
      ++next_slot_;
      RCLCPP_INFO(get_logger(), "%s placed (%u/%u)", r.box_id.c_str(), next_slot_, capacity());
      if (next_slot_ >= capacity()) {
        RCLCPP_INFO(get_logger(), "Pallet full");
      }
    } else {
      failed_ids_.insert(r.box_id);
      RCLCPP_ERROR(get_logger(), "%s failed (%s). Slot %u stays free; box won't be re-assigned.",
        r.box_id.c_str(), r.message.c_str(), pending_->slot);
    }
    pending_.reset();
    publishStatus();
    publishMarkers();
    tryAssign();
  }

  void tick()
  {
    if (pending_) {
      target_pub_->publish(*pending_);
    }
    if (spawn_pallet_ && pallet_sends_ < 3 && scene_pub_->get_subscription_count() > 0) {
      publishPallet();   // sent a few times in case the first is lost during discovery
      ++pallet_sends_;
    }
  }

  void publishStatus()
  {
    PalletStatus s;
    s.placed = next_slot_;
    s.capacity = capacity();
    s.current_layer = std::min(next_slot_ / kPerLayer, layers_ - 1);
    s.full = next_slot_ >= capacity();
    s.pending_box_id = pending_ ? pending_->box_id : "";
    status_pub_->publish(s);
  }

  // Grey = free, orange = assigned, green = placed.
  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    for (uint32_t i = 0; i < capacity(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id_;
      m.header.stamp = now();
      m.ns = "pallet_slots";
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = slotPose((i % kPerLayer) / kCols, (i % kPerLayer) % kCols, i / kPerLayer);
      m.scale.x = box_size_[0] * 0.98;
      m.scale.y = box_size_[1] * 0.98;
      m.scale.z = box_size_[2] * 0.98;
      if (i < next_slot_) {
        m.color.r = 0.2f; m.color.g = 0.8f; m.color.b = 0.2f; m.color.a = 0.8f;
      } else if (pending_ && pending_->slot == i) {
        m.color.r = 1.0f; m.color.g = 0.6f; m.color.b = 0.0f; m.color.a = 0.8f;
      } else {
        m.color.r = 0.7f; m.color.g = 0.7f; m.color.b = 0.7f; m.color.a = 0.15f;
      }
      arr.markers.push_back(m);
    }
    marker_pub_->publish(arr);
  }

  void publishPallet()
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame_id_;
    obj.id = "pallet";
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = {pallet_size_[0], pallet_size_[1], pallet_size_[2]};
    obj.primitives.push_back(prim);

    Pose pose;
    pose.position.x = pallet_top_[0];
    pose.position.y = pallet_top_[1];
    pose.position.z = pallet_top_[2] - pallet_size_[2] / 2.0;
    pose.orientation.w = 1.0;
    obj.primitive_poses.push_back(pose);

    moveit_msgs::msg::PlanningScene ps;
    ps.is_diff = true;
    ps.world.collision_objects.push_back(obj);
    scene_pub_->publish(ps);
    RCLCPP_INFO(get_logger(), "Pallet sent to planning scene");
  }

  static Quaternion yawQuat(double yaw)
  {
    Quaternion q;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
  }

  static void toRPY(const Quaternion & q, double & r, double & p, double & y)
  {
    r = std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
    const double s = 2 * (q.w * q.y - q.z * q.x);
    p = std::abs(s) >= 1 ? std::copysign(M_PI / 2, s) : std::asin(s);
    y = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  }

  // parameters
  std::string frame_id_;
  std::vector<double> pallet_top_, pallet_size_, box_size_;
  double gap_, place_yaw_, pose_timeout_;
  uint32_t layers_;
  bool spawn_pallet_;

  // state
  uint32_t next_slot_{0};
  std::optional<PalletTarget> pending_;
  std::string at_pick_id_, last_placed_id_;
  std::set<std::string> failed_ids_;
  PoseStamped pick_pose_;
  std::chrono::steady_clock::time_point pose_time_{};
  int pallet_sends_{0};

  rclcpp::Publisher<PalletTarget>::SharedPtr target_pub_;
  rclcpp::Publisher<PalletStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr id_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<PlaceResult>::SharedPtr result_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PalletPlanner>());
  rclcpp::shutdown();
  return 0;
}

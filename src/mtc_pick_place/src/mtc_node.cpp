#include <rclcpp/rclcpp.hpp>

// MTC core: the Task class and the exception thrown by task.init()
#include <moveit/task_constructor/task.h>

// Stages we use: one generator (CurrentState) and one propagator (MoveTo)
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
// Stage that moves the IK frame by a relative offset
#include <moveit/task_constructor/stages/move_relative.h>

// Solver that plans straight-line Cartesian motion
#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <geometry_msgs/msg/vector3_stamped.hpp>
// Solver that plans through the normal MoveIt planning pipeline (OMPL)
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <chrono>
#include <string>
#include <thread>
#include <vector>

// short alias so we don't repeat moveit::task_constructor everywhere
namespace mtc = moveit::task_constructor;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_pick_place");

// ---------------------------------------------------------------------------
// Copy the MoveIt settings that only exist as parameters on move_group.
//
// ROS 2 has no global parameter server: each node owns its parameters.
// The URDF and SRDF reach us over topics, but the IK solver, the joint limits
// and the OMPL configuration live only on /move_group, so we copy them here.
// ---------------------------------------------------------------------------
void copyParamsFromMoveGroup(const rclcpp::Node::SharedPtr& node)
{
    // The parameter client spins the node it is given. We use a throwaway
    // helper node so it never clashes with the executor of our main node.
    rclcpp::SyncParametersClient client(node, "move_group");

    // move_group must already be running, otherwise there is nothing to copy
    if (!client.wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(LOGGER, "move_group parameter service not available");
        return;
    }

    // robot_description_kinematics : which IK plugin each group uses
    // robot_description_planning   : joint velocity / acceleration limits
    // ompl                         : planning plugin, adapters, planner configs
    const std::vector<std::string> prefixes = { "robot_description_kinematics",
                                                "robot_description_planning", "ompl" };

    // ask move_group for every parameter name under those prefixes (depth 10)
    const auto names = client.list_parameters(prefixes, 10).names;

    // fetch the values and declare each one on our own node
    for (const auto& param : client.get_parameters(names))
    {
        // skip parameters that are declared on move_group but have no value
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
            continue;

        // don't overwrite something that was passed to us explicitly
        if (!node->has_parameter(param.get_name()))
            node->declare_parameter(param.get_name(), param.get_parameter_value());
    }

    RCLCPP_INFO(LOGGER, "Copied %zu parameters from move_group", names.size());
}

// ---------------------------------------------------------------------------
// Build the task:  CurrentState  ->  MoveTo (pre-grasp)
// ---------------------------------------------------------------------------
mtc::Task createTask(const rclcpp::Node::SharedPtr& node)
{
    mtc::Task task;

    // task.stages() returns the top-level container that holds all stages;
    // this name is what shows up in the RViz "Motion Planning Tasks" panel
    task.stages()->setName("pick_place_milestone_1");

    // load the robot model (URDF + SRDF + kinematics) through this node.
    // The kinematics parameters must already be on the node at this point,
    // which is why copyParamsFromMoveGroup() runs before createTask().
    task.loadRobotModel(node);

    // task-level property: the planning group stages should use by default.
    // Stages opt in to it with configureInitFrom(PARENT, {"group"}).
    // There is no "eef" property because the SRDF defines no end effector.
    task.setProperty("group", "fairino5_v6_group");

    // --- Stage 1: CurrentState (generator) ---------------------------------
    // Asks move_group for the current planning scene and offers it as the
    // starting state of the task. It produces a state, not a motion.
    task.add(std::make_unique<mtc::stages::CurrentState>("current"));

    // --- Solver for free-space motion --------------------------------------
    // PipelinePlanner runs the same OMPL pipeline that the RViz
    // "Plan & Execute" button uses. It reads the ompl.* parameters from the
    // node. Solvers are shared_ptr because several stages can share one.
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);

    // --- Goal: where the flange should be before the approach --------------
    geometry_msgs::msg::PoseStamped pre_grasp;
    pre_grasp.header.frame_id = "base_link";  // pose is expressed in the robot base frame
    pre_grasp.pose.position.x = 0.4;          // metres
    pre_grasp.pose.position.y = 0.1;
    pre_grasp.pose.position.z = 0.35;
    // Quaternion (x=1, y=0, z=0, w=0) is a 180 degree rotation about X.
    // It flips the flange z-axis so it points straight down.
    // Assumption: wrist3_link's z-axis points out of the flange.
    pre_grasp.pose.orientation.x = 1.0;
    pre_grasp.pose.orientation.y = 0.0;
    pre_grasp.pose.orientation.z = 0.0;
    pre_grasp.pose.orientation.w = 0.0;

    // --- Stage 2: MoveTo (propagator) --------------------------------------
    // A propagator takes the state from the previous stage and plans forward
    // from it, so it chains directly after CurrentState with no Connect stage.
    auto move_to_pre_grasp =
        std::make_unique<mtc::stages::MoveTo>("move to pre-grasp", sampling_planner);

    // inherit the "group" property from the task instead of hard-coding it here
    move_to_pre_grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

    // which link must reach the goal pose. Set on the stage (not inherited)
    // because MoveTo stores it as a pose, and this call does the conversion.
    move_to_pre_grasp->setIKFrame("wrist3_link");

    // the target for that link
    move_to_pre_grasp->setGoal(pre_grasp);

    // hand ownership of the stage to the task
    task.add(std::move(move_to_pre_grasp));

    // --- Solver for straight-line motion -----------------------------------
    // CartesianPath interpolates the IK frame along a straight line and solves
    // IK at every step, so the path shape is guaranteed (unlike OMPL).
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

    // distance between interpolated waypoints, in metres (1 cm)
    cartesian_planner->setStepSize(0.01);

    // --- Stage 3: MoveRelative (propagator) --------------------------------
    // Starts from the pre-grasp state produced by the previous stage and moves
    // the flange a relative distance from there.
    auto approach =
        std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);

    // inherit the planning group from the task
    approach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

    // the link whose motion we are describing
    approach->setIKFrame("wrist3_link");

    // try to move 10 cm; accept the result only if at least 8 cm was achieved
    approach->setMinMaxDistance(0.08, 0.10);

    // direction of travel, expressed in the flange's own frame:
    // +z of wrist3_link = straight out of the flange, toward the object.
    // Because it is in the flange frame, it stays correct for any grasp orientation.
    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = "wrist3_link";
    direction.vector.z = 1.0;
    approach->setDirection(direction);

    task.add(std::move(approach));

    return task;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // let the node accept parameters that were not declared in code
    rclcpp::NodeOptions opt;
    opt.automatically_declare_parameters_from_overrides(true);

    // create the ROS 2 node
    auto node = rclcpp::Node::make_shared("mtc_pick_place", opt);

    // must run before createTask(): the robot model reads these parameters
    copyParamsFromMoveGroup(node);

    // spin the node in a background thread. Planning does not need this,
    // but RViz fetches the task and its solutions through services on this node.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor] { executor.spin(); });

    mtc::Task task = createTask(node);

    try
    {
        // check every stage is configured and wire the stages together.
        // Throws InitStageException if a stage is missing something.
        task.init();

        // search for up to 5 complete solutions
        if (task.plan(5))
        {
            RCLCPP_INFO(LOGGER, "Planning succeeded: %zu solution(s)", task.solutions().size());

            // solutions are sorted by cost, so front() is the best one.
            // This publishes it for RViz; it does not move the robot.
            task.introspection().publishSolution(*task.solutions().front());
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "Task planning failed");
        }
    }
    catch (const mtc::InitStageException& e)
    {
        // prints which stage failed to initialise and why
        RCLCPP_ERROR_STREAM(LOGGER, e);
    }

    // keep the node alive so the task stays visible in RViz (Ctrl+C to quit)
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
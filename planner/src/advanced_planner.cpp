#include <pluginlib/class_loader.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node =
        rclcpp::Node::make_shared("advanced_planner", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(motion_planning_api_tutorial_node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    auto const logger = rclcpp::get_logger("advanced_planner");

    const std::string PLANNING_GROUP = "ur_manipulator";
    robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
    const moveit::core::RobotModelPtr &robot_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    const moveit::core::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;






    geometry_msgs::msg::PoseStamped start_pose = move_group_interface.getCurrentPose();

    // Create a MotionPlanRequest
    moveit_msgs::msg::MotionPlanRequest req;
    req.planner_id = "LIN" req.group_name = "ur_manipulator";
    req.max_velocity_scaling_factor =
    req.max_acceleration_scaling_factor =

    moveit_msgs::msg::RobotState start_state;
    start_state.joint_state.name = move_group_interface.getJointNames();
    start_state.joint_state.position = move_group_interface.getCurrentJointValues();
    req.start_state = start_state;

    // Define the goal state
    geometry_msgs::msg::Pose target_pose = start_pose.pose;
    bool move_forward = true;

    while (rclcpp::ok())
    {
        if (move_forward)
        {
            target_pose.position.x += 0.05; // Move forward
        }
        else
        {
            target_pose.position.x -= 0.05; // Move backward
        }

        // Set the goal pose
        moveit_msgs::msg::Constraints goal_constraint;
        moveit_msgs::msg::PositionConstraint position_constraint;
        position_constraint.link_name = move_group_interface.getEndEffectorLink();
        position_constraint.header.frame_id = move_group_interface.getPlanningFrame();
        position_constraint.constraint_region.primitive_poses.push_back(target_pose);
        goal_constraint.position_constraints.push_back(position_constraint);
        req.goal_constraints.push_back(goal_constraint);

        // Plan and execute
        moveit_msgs::msg::MotionPlanResponse res;
        move_group_interface.setStartStateToCurrentState();
        move_group_interface.setPlanningTime(5.0); // Set a suitable planning time
        move_group_interface.asyncExecute(req, res);

        if (res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            move_forward = !move_forward; // Change direction after each move
        }
        else
        {
            RCLCPP_ERROR(logger, "Planning failed!");
            break; // Exit loop if planning fails
        }
    }

    rclcpp::shutdown();
    return 0;
}

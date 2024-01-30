#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("planner", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("planner");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group_interface.setPlannerId("PTP");

  geometry_msgs::msg::PoseStamped start_pose = move_group_interface.getCurrentPose();
  geometry_msgs::msg::Pose target_pose = start_pose.pose;

  RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "Start Pose Position    X %f", start_pose.pose.position.x);
  RCLCPP_INFO(logger, "Start Pose Position    Y %f", start_pose.pose.position.y);
  RCLCPP_INFO(logger, "Start Pose Position    Z %f", start_pose.pose.position.z);
  RCLCPP_INFO(logger, "Start Pose Orientation X %f", start_pose.pose.orientation.x);
  RCLCPP_INFO(logger, "Start Pose Orientation Y %f", start_pose.pose.orientation.y);
  RCLCPP_INFO(logger, "Start Pose Orientation Z %f", start_pose.pose.orientation.z);
  RCLCPP_INFO(logger, "Start Pose Orientation W %f", start_pose.pose.orientation.w);

bool move_forward = true; // Flag to determine the direction of movement
  while (rclcpp::ok()) // Loop while ROS is running
  {
    if (move_forward)
    {
      target_pose.position.x += 0.05; // Move forward
    }
    else
    {
      target_pose.position.x -= 0.05; // Move backward
    }

    move_group_interface.setPoseTarget(target_pose);

    auto const [success, plan] = [&move_group_interface]
    {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

    if (success)
    {
      move_group_interface.execute(plan);
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
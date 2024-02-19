#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

bool move_to_pose(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                  geometry_msgs::msg::Pose target_pose,
                  const rclcpp::Logger &logger)
{
  move_group_interface.setPoseTarget(target_pose);
  auto const [success, plan] = [&move_group_interface]()
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (success)
  {
    move_group_interface.execute(plan);
    return true;
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
    return false;
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("planner", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("planner");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group_interface.setPlannerId("LIN");

  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);

  geometry_msgs::msg::PoseStamped start_pose = move_group_interface.getCurrentPose();
  // geometry_msgs::msg::Pose target_pose = start_pose.pose;

  RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "Start Pose Position    X %f", start_pose.pose.position.x);
  RCLCPP_INFO(logger, "Start Pose Position    Y %f", start_pose.pose.position.y);
  RCLCPP_INFO(logger, "Start Pose Position    Z %f", start_pose.pose.position.z);
  RCLCPP_INFO(logger, "Start Pose Orientation X %f", start_pose.pose.orientation.x);
  RCLCPP_INFO(logger, "Start Pose Orientation Y %f", start_pose.pose.orientation.y);
  RCLCPP_INFO(logger, "Start Pose Orientation Z %f", start_pose.pose.orientation.z);
  RCLCPP_INFO(logger, "Start Pose Orientation W %f", start_pose.pose.orientation.w);

  // bool success;

  // target_pose.position.x = -0.158841;
  // target_pose.position.y = 0.464466;
  // target_pose.position.z = 0.90000;
  // target_pose.position.z += 0.03;
  // target_pose.orientation.x = 0;
  // target_pose.orientation.y = 1;
  // target_pose.orientation.z = 0;
  // target_pose.orientation.w = 0;

  // success = move_to_pose(move_group_interface, target_pose, logger);
  // if (!success)
  //   return 1;

  
  // target_pose.position.z -= 0.03;

  // success = move_to_pose(move_group_interface, target_pose, logger);
  // if (!success)
  //   return 1;

  // target_pose.position.z -= 0.03;
  // success = move_to_pose(move_group_interface, target_pose, logger);
  // if (!success)
  //   return 1;

  // for (uint8_t i = 0; i < 5; i++)
  // {
  //   if (i % 2 == 0)
  //   {
  //     target_pose.position.x -= 0.16;
  //   }
  //   else
  //   {
  //     target_pose.position.x += 0.16;
  //   }
  //   success = move_to_pose(move_group_interface, target_pose, logger);
  //   if (!success)
  //     return 1;

  //   target_pose.position.y -= 0.004;
  //   success = move_to_pose(move_group_interface, target_pose, logger);
  //   if (!success)
  //     return 1;
  // }

  // target_pose.position.x = 0.077657;
  // target_pose.position.y = 0.467640;
  // target_pose.position.z = 0.925300;
  // target_pose.orientation.x = 0;
  // target_pose.orientation.y = 1;
  // target_pose.orientation.z = 0;
  // target_pose.orientation.w = 0;

  // success = move_to_pose(move_group_interface, target_pose, logger);
  // if (!success)
  //   return 1;

  rclcpp::shutdown();
  return 0;
}
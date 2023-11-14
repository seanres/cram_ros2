#!/usr/bin/env python3
import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
)


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    # plan to goal
    logger.info("Planning trajectory")

    if single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    ur = MoveItPy(node_name="moveit_py")
    ur_arm = ur.get_planning_component("ur_manipulator")
    logger.info("MoveItPy instance created")

    # set plan start state to current state
    ur_arm.set_start_state_to_current_state()

    # Get current pose of the end effector
    current_pose = ur_arm.get_current_pose()
    logger.info(f"Current pose: {current_pose}")

    # Create a new pose goal based on the current pose
    new_pose_goal = PoseStamped()
    new_pose_goal.header.frame_id = "base_frame"  # Replace with your robot's base frame
    new_pose_goal.pose.position.x = current_pose.pose.position.x + 0.02  # Add 2 cm in X direction
    new_pose_goal.pose.position.y = current_pose.pose.position.y
    new_pose_goal.pose.position.z = current_pose.pose.position.z
    new_pose_goal.pose.orientation = current_pose.pose.orientation

    # Set the new goal state
    ur_arm.set_goal_state(pose_stamped_msg=new_pose_goal, pose_link="end_effector_link")  # Replace with your end effector link name

    # Plan to the new goal
    plan_and_execute(ur, ur_arm, logger, sleep_time=1.0)

if __name__ == "__main__":
    main()


if __name__ == "__main__":
    main()

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "vitals/msg/end_effector_velocity.hpp" 
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Core>

std::shared_ptr<moveit::core::RobotState> robot_state;
const moveit::core::JointModelGroup* joint_model_group;
rclcpp::Publisher<vitals::msg::EndEffectorVelocity>::SharedPtr velocity_publisher;

void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    robot_state->setVariableValues(*msg);
}

void calculateAndPublishEndEffectorVelocity(rclcpp::Node::SharedPtr node) {
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;

    const moveit::core::LinkModel* end_effector_link = joint_model_group->getLinkModel(joint_model_group->getLinkModelNames().back());
    if (!end_effector_link) {
        RCLCPP_ERROR(node->get_logger(), "End-effector link model not found.");
        return;
    }

    if (robot_state && joint_model_group) {
        bool success = robot_state->getJacobian(joint_model_group,
                                                end_effector_link,
                                                reference_point_position, 
                                                jacobian);

        if (!success) {
            RCLCPP_ERROR(node->get_logger(), "Failed to calculate Jacobian.");
            return;
        }

        // Assuming joint velocities are already updated by the joint state callback
        std::vector<double> joint_velocities;
        robot_state->copyJointGroupVelocities(joint_model_group, joint_velocities);
        Eigen::VectorXd joint_velocities_vector = Eigen::VectorXd::Map(joint_velocities.data(), joint_velocities.size());

        Eigen::VectorXd end_effector_velocity = jacobian * joint_velocities_vector;

        auto velocity_msg = vitals::msg::EndEffectorVelocity();
        velocity_msg.header.stamp = node->get_clock()->now();
        velocity_msg.velocity = std::vector<double>(end_effector_velocity.data(), end_effector_velocity.data() + end_effector_velocity.size());
        velocity_publisher->publish(velocity_msg);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Robot state or joint model group not initialized.");
    }
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("end_effector_velocity_node");
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
    auto kinematic_model = robot_model_loader.getModel();
    robot_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
    joint_model_group = kinematic_model->getJointModelGroup("ur_manipulator");

    auto joint_state_subscriber = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, jointStateCallback);

    velocity_publisher = node->create_publisher<vitals::msg::EndEffectorVelocity>("end_effector_velocity", 10);

    rclcpp::Rate rate(100); // Hz
    while (rclcpp::ok()) {
        calculateAndPublishEndEffectorVelocity(node);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/Dense>
#include <vector>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

class EndEffectorVelocityCalculator : public rclcpp::Node
{
public:
    EndEffectorVelocityCalculator()
        : Node("end_effector_velocity_calculator")
    {
        // Create a subscriber for the joint states topic
        joint_states_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                calculateEndEffectorVelocity(msg);
            });

        // Create a publisher for the end effector velocities
        end_effector_velocity_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/end_effector_velocities", 10);

        // Initialize MoveIt! MoveGroupInterface
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("ur_manipulator");
    }

private:
    void calculateEndEffectorVelocity(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
    {
        // Extract joint velocities from the received JointState message
        std::vector<double> joint_velocities = joint_state_msg->velocity;

        const moveit::core::JointModelGroup *joint_model_group =
            move_group_.getCurrentState()->getJointModelGroup("ur_manipulator");

        // Compute the Jacobian matrix
        Eigen::MatrixXd jacobian;
        move_group_->getJacobian(jacobian, joint_model_group);

        // Convert joint velocities to Eigen vector
        Eigen::VectorXd joint_velocities_eigen(joint_velocities.size());
        for (size_t i = 0; i < joint_velocities.size(); ++i)
        {
            joint_velocities_eigen(i) = joint_velocities[i];
        }

        // Calculate end effector velocity
        Eigen::VectorXd end_effector_velocity = jacobian * joint_velocities_eigen;

        // Create a Float64MultiArray message for publishing
        auto end_effector_velocity_msg = std_msgs::msg::Float64MultiArray();
        end_effector_velocity_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        end_effector_velocity_msg.layout.dim[0].size = end_effector_velocity.size();
        end_effector_velocity_msg.layout.dim[0].stride = 1;
        end_effector_velocity_msg.layout.dim[0].label = "velocity";

        end_effector_velocity_msg.data = std::vector<double>(end_effector_velocity.data(), end_effector_velocity.data() + end_effector_velocity.size());

        // Publish the calculated end effector velocity with timestamps
        end_effector_velocity_msg.header.stamp = joint_state_msg->header.stamp;
        end_effector_velocity_publisher_->publish(end_effector_velocity_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr end_effector_velocity_publisher_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EndEffectorVelocityCalculator>());
    rclcpp::shutdown();
    return 0;
}

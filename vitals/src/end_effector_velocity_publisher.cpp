#include <memory>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>

class EndEffectorVelocityCalculator : public rclcpp::Node
{
public:
  EndEffectorVelocityCalculator() : Node("end_effector_velocity_publisher")
  {
    joint_states_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&EndEffectorVelocityCalculator::calculate, this, std::placeholders::_1));
    last_zero_velocity_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(1); // Initialize in the past
  }

private:
  void calculate(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
  {
    std::vector<double> joint_velocities = joint_state_msg->velocity;
    auto now = std::chrono::steady_clock::now();

    size_t num_joints = joint_velocities.size();
    bool all_velocities_zero = true;
    double zero_threshold = 0.001; // Define what you consider "close to zero"

    for (size_t i = 0; i < num_joints; ++i)
    {
      if (std::fabs(joint_velocities[i]) >= zero_threshold)
      {
        all_velocities_zero = false;
        break; // Exit the loop as soon as one velocity is not close to zero
      }
    }

    // Only log if all velocities are close to zero and debounce time has passed
    if (all_velocities_zero && std::chrono::duration_cast<std::chrono::milliseconds>(now - last_zero_velocity_time_).count() > debounce_time_ms_)
    {
      RCLCPP_INFO(this->get_logger(), "All joint velocities are close to zero at time: %u.%u", 
                  joint_state_msg->header.stamp.sec, joint_state_msg->header.stamp.nanosec);
      last_zero_velocity_time_ = now; // Update the last zero velocity time
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber;
  std::chrono::steady_clock::time_point last_zero_velocity_time_;
  int debounce_time_ms_ = 500; // 500 milliseconds debounce time
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EndEffectorVelocityCalculator>());
  rclcpp::shutdown();
  return 0;
}
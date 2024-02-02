#include <memory>
#include <mutex>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class PlannerNode : public rclcpp::Node
{
public:
    PlannerNode()
        : Node("planner", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
          move_group_interface(std::make_shared<rclcpp::Node>("move_group_interface"), "ur_manipulator")
    {
        this->declare_parameter<std::string>("pose_topic", "/new_pose_topic");
        std::string pose_topic = this->get_parameter("pose_topic").as_string();

        pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, 10, std::bind(&PlannerNode::poseCallback, this, std::placeholders::_1));
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        target_pose_ = *msg;
        new_pose_received_ = true;
    }

    void run()
    {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(this->get_node_base_interface());
        std::thread([&executor]()
                    { executor.spin(); })
            .detach();

        move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group_interface.setPlannerId("PTP");

        while (rclcpp::ok())
        {
            if (new_pose_received_)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                new_pose_received_ = false;

                move_group_interface.setPoseTarget(target_pose_);

                auto const [success, plan] = [&]
                {
                    moveit::planning_interface::MoveGroupInterface::Plan msg;
                    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
                    return std::make_pair(ok, msg);
                }();

                if (success)
                {
                    move_group_interface.execute(plan);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Planning failed!");
                    break;
                }
            }
        }
    }

private:
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber;
    geometry_msgs::msg::PoseStamped target_pose_;
    std::mutex mutex_;
    bool new_pose_received_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannerNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
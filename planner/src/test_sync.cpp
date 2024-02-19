#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "interfaces/srv/arduino_command.hpp"
#include <memory>
#include <string.h>
#include <string>
#include <functional>

using namespace std::chrono_literals;
using ArduinoCommand = interfaces::srv::ArduinoCommand;
using std::placeholders::_1;

class ArduinoCommandClient : public rclcpp::Node
{
public:
    ArduinoCommandClient() : Node("arduino_command_client")
    {
        client_ = this->create_client<ArduinoCommand>("arduino_command_service");
    }

    void send_command(const std::string &command)
    {
        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        RCLCPP_INFO(this->get_logger(), "Sending command: '%s'", command.c_str());

        auto request = std::make_shared<ArduinoCommand::Request>();
        request->command = command;

        auto response_received_callback = [this](rclcpp::Client<ArduinoCommand>::SharedFuture future)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Received response: '%s'", response->response.c_str());
        };

        client_->async_send_request(request, response_received_callback);
    }

private:
    rclcpp::Client<ArduinoCommand>::SharedPtr client_;
};

bool move_to_pose(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                  geometry_msgs::msg::Pose target_pose,
                  const rclcpp::Logger &logger, ArduinoCommandClient &arduino_client, const std::string &command)
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
        if (command != "")
        {
            arduino_client.send_command(command);
        }

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
    auto client_node = std::make_shared<ArduinoCommandClient>();
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

    client_node->send_command("M201 X1000 Y1000 Z200 E8000");  // sets maximum accelerations, mm/sec^2
    client_node->send_command("M203 X200 Y200 Z12 E120");      // sets maximum feedrates, mm / sec
    client_node->send_command("M204 P1250 R1250 T1250");       // sets acceleration (P, T) and retract acceleration (R), mm/sec^2
    client_node->send_command("M205 X8.00 Y8.00 Z0.40 E4.50"); // sets the jerk limits, mm/sec
    client_node->send_command("M205 S0 T0");                   // sets the minimum extruding and travel feed rate, mm/sec
    client_node->send_command("M204 S800");
    client_node->send_command("G91");

    geometry_msgs::msg::PoseStamped start_pose = move_group_interface.getCurrentPose();
    RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    RCLCPP_INFO(logger, "Start Pose Position    X %f", start_pose.pose.position.x);
    RCLCPP_INFO(logger, "Start Pose Position    Y %f", start_pose.pose.position.y);
    RCLCPP_INFO(logger, "Start Pose Position    Z %f", start_pose.pose.position.z);
    RCLCPP_INFO(logger, "Start Pose Orientation X %f", start_pose.pose.orientation.x);
    RCLCPP_INFO(logger, "Start Pose Orientation Y %f", start_pose.pose.orientation.y);
    RCLCPP_INFO(logger, "Start Pose Orientation Z %f", start_pose.pose.orientation.z);
    RCLCPP_INFO(logger, "Start Pose Orientation W %f", start_pose.pose.orientation.w);

    bool success;
    geometry_msgs::msg::Pose target_pose, home_pose;

    home_pose.position.x = 0.040455;
    home_pose.position.y = 0.520148;
    home_pose.position.z = 0.895453;
    home_pose.orientation.x = 0;
    home_pose.orientation.y = 1;
    home_pose.orientation.z = 0;
    home_pose.orientation.w = 0;

    success = move_to_pose(move_group_interface, home_pose, logger, *client_node, "");
    if (!success)
        return 1;

    client_node->send_command("G1 E8 F100");
    client_node->send_command("G1 F1200");

    rclcpp::sleep_for(std::chrono::seconds(5));

    target_pose = home_pose;

    // starting position in gcode is (43.25, 106.571) corrected to (43.41, 106.571)
    double gcodeMovements[][2] = {
        {206.59, 106.571},
        {206.59, 106.179},
        {43.41, 106.178},
        {43.41, 105.786},
        {206.59, 105.786},
        {206.59, 105.393},
        {43.41, 105.393},
        {43.41, 105},
        {206.59, 105},
        {206.59, 104.607},
        {43.41, 104.607},
        {43.41, 104.214},
        {206.59, 104.214},
        {206.59, 103.821},
        {43.41, 103.821},
        {43.41, 103.429}};

    double lastx = 43.41;
    double lasty = 106.571;
    const int arraySize = sizeof(gcodeMovements) / sizeof(gcodeMovements[0]);
    for (uint8_t i = 0; i < arraySize; i++)
    {
        double xMovement = gcodeMovements[i][0];
        double yMovement = gcodeMovements[i][1];

        target_pose.position.x = (-xMovement + 43.41) / 1000.0 + home_pose.position.x;
        target_pose.position.y = (yMovement - 106.571) / 1000.0 + home_pose.position.y;

        if (i % 2 == 0)
        {
            success = move_to_pose(move_group_interface, target_pose, logger, *client_node, "G1 X" + std::to_string(xMovement - lastx) + " Y" + std::to_string(yMovement - lasty) + " E7.9954");
        }   
        else
        {
            success = move_to_pose(move_group_interface, target_pose, logger, *client_node, "G1 X" + std::to_string(xMovement - lastx) + " Y" + std::to_string(yMovement - lasty) + " E.01925");
            std::cout << "Printed length %s in Y" << std::to_string(yMovement - lasty) << "at X = %s" << std::to_string(lastx)<< std::endl;
        }
        
        lastx = xMovement;
        lasty = yMovement;

        if (!success)
            return 1;
    }

    client_node->send_command("G1 E-8 F100");

    geometry_msgs::msg::Pose end_pose = home_pose;
    end_pose.position.x = -0.389110;
    end_pose.position.y = 0.267;
    end_pose.position.z = 0.98;

    success = move_to_pose(move_group_interface, end_pose, logger, *client_node, "");
    if (!success)
        return 1;

    rclcpp::shutdown();
    return 0;
}
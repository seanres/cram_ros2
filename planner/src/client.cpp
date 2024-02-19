#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/arduino_command.hpp"
#include <memory>
#include <string>
#include <functional>

using namespace std::chrono_literals;
using ArduinoCommand = interfaces::srv::ArduinoCommand;
using std::placeholders::_1;

class ArduinoCommandClient : public rclcpp::Node {
public:
  ArduinoCommandClient() : Node("arduino_command_client") {
    client_ = this->create_client<ArduinoCommand>("arduino_command_service");
  }

  void send_command(const std::string & command) {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }


    auto request = std::make_shared<ArduinoCommand::Request>();
    request->command = command;

    auto response_received_callback = [this](rclcpp::Client<ArduinoCommand>::SharedFuture future) {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Received response: '%s'", response->response.c_str());
    };

    client_->async_send_request(request, response_received_callback);
  }
  
private:
  rclcpp::Client<ArduinoCommand>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArduinoCommandClient>();
  RCLCPP_INFO(node->get_logger(), "Enter commands (type 'quit' to exit):");

  std::string command;
  while (rclcpp::ok()) {
    std::cout << "> ";
    std::getline(std::cin, command);
    if (command == "quit" || std::cin.fail()) {
      break;
    }

    node->send_command(command);
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}

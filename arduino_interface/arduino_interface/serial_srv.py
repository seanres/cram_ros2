import rclpy
from rclpy.node import Node
from interfaces.srv import ArduinoCommand
import serial

class ArduinoInterface(Node):
    def __init__(self, port="/dev/ttyACM0"):
        super().__init__('arduino_interface')
        self.ser = serial.Serial(port, 250000, timeout=1)
        self.get_logger().info(f"Connected to Arduino on port {port}")

        self.arduino_service = self.create_service(ArduinoCommand, 'arduino_command_service', self.handle_service_request)

    def handle_service_request(self, request, response):
        try:
            formatted_command = request.command.strip() + "\r\n"
            self.get_logger().info(f"Sending command to Arduino: {formatted_command}")
            self.ser.write(formatted_command.encode())
            
            arduino_response = self.ser.readline().decode('UTF-8').strip()
            response.response = arduino_response
            return response
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {str(e)}")
            response.response = f"ERROR: {str(e)}"
            return response
    
    def shutdown(self):
        if self.ser:
            self.ser.close()
            self.get_logger().info("Serial connection closed.")

def main(args=None):
    rclpy.init(args=args)
    arduino_interface = ArduinoInterface()
    try:
        rclpy.spin(arduino_interface)
    except KeyboardInterrupt:
        passt
        rclpy.shutdown()

if __name__ == '__main__':
    main()

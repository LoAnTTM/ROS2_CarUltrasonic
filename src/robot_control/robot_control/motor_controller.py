import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class CommandNode(Node):
    def __init__(self):
        super().__init__('motor_controller')


        # Initialize serial communication with Arduino
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info("Serial connection established with Arduino.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.ser = None
        
        # Create a subscriber to the 'distance' topic
        self.subscription = self.create_subscription(Float32, 'distance', self.listener_callback, 10)
        

    def listener_callback(self, msg):
        distance = msg.data
        if distance < 20.0:
            command = 'STOP\n'
        else:
            command = 'FORWARD\n'
        self.ser.write(command.encode())
        self.get_logger().info(f"Sent command: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

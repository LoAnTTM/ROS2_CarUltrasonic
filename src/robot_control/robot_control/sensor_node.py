import rclpy
import Jetson.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import Bool

IR_PIN = 31  

class IRSensorNode(Node):
    def __init__(self):
        super().__init__('ir_sensor_node')
        self.publisher_ = self.create_publisher(Bool, 'obstacle_detected', 10)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(IR_PIN, GPIO.IN)

        self.timer = self.create_timer(0.5, self.read_sensor)

    def read_sensor(self):
        obstacle = GPIO.input(IR_PIN) == 0 
        msg = Bool()
        msg.data = obstacle

        if obstacle:
            self.get_logger().info(" IR sensor detected an obstacle!")
        else:
            self.get_logger().info(" No obstacle detected")

        self.publisher_.publish(msg)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IRSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

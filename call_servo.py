import rclpy
import tf2_ros
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from servo_function import servo_it

class Pose_servo(Node):
    def __init__(self):
        super().__init__('pose_servo')

        # Create a TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create servo publisher
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz


    def timer_callback(self):
        goal_reached = servo_it(
            [0.41,-0.48 ,0.71,-0.0097,0.74,-0.37,0.554],
            self.publisher_,
            self.tf_buffer
        )

def main(args=None):
    rclpy.init(args=args)
    
    node = Pose_servo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

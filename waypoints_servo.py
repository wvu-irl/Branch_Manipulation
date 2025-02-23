import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistStamped
from constrained_rrt import RRTStarConstrained
from servo_function import servo_it

class Waypoints_servo(Node):
    def __init__(self):
        super().__init__('tf2_echo_node')

        # Create a TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create servo publisher
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        # Set up a timer to continuously check for the transform
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        # Define parameters for RRT
        start = [0.66, 0.046, 0.6]  # Start point in task space (x, y, z)
        goal = [0.46, -0.47, 0.36]   # Goal point in task space (x, y, z)
        plant_height = 0.6      # Maximum height of the plant
        half_height = 0.25        # Half the height of the plant
      
        q_start=[-0.55,0.57,-0.42,0.42] #0.42
        q_end=[-0.23,0.83,-0.48,0.1] #0.1
        
        # Create RRT* planner
        rrt_star = RRTStarConstrained(start, goal, plant_height, half_height)

        # Plan a path and store it in self.waypoints
        self.path=np.array(rrt_star.plan())
        
        
        num_steps=len(self.path)
        self.quaternions = rrt_star.slerp(q_start, q_end, num_steps)
        self.waypoints=np.hstack((self.path, self.quaternions))
        first=self.waypoints[0]
        last=self.waypoints[-1]
        self.waypoints=np.vstack((first,last))
        self.current_waypoint_index =0


    def timer_callback(self):
        # Check if all waypoints have been processed
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached.")
            return

        try:
            
            # Get the current waypoint
            target_pose = self.waypoints[self.current_waypoint_index]
            [0.41,-0.48 ,0.71,-0.0097,0.74,-0.37,0.554]

            goal_reached = servo_it(target_pose
            ,
            self.publisher_,
            self.tf_buffer
            )

          

            if goal_reached==True:
                self.get_logger().info(f"Reached waypoint {self.current_waypoint_index + 1}")
                self.current_waypoint_index += 1  # Move to next waypoint
  
          
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Could not get transform: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = Waypoints_servo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


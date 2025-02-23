import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistStamped
from constrained_rrt import RRTStarConstrained
from servo_function import servo_it
from geometry_msgs.msg import  WrenchStamped
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Waypoints_servo(Node):
    def __init__(self):
        super().__init__('tf2_echo_node')
        self.start_time =time.time()
        # Create a TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create servo publisher
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        # Set up a timer to continuously check for the transform
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        # Define parameters for RRT
        self.start_init=[0.66,0.046,0.52]
        self.start =[0.66,0.046,0.52]                                                                  
        self.goal =[0.46,-0.47,0.32]
        self.plant_height = 0.52   # Maximum height of the plant
        self.half_height = 0.25     # Half the height of the plant
      
        self.q_start=[-0.55,0.57,-0.42,0.42]
        self.q_end=[-0.23,0.83,-0.48,0.1]
        
        # Create RRT* planner
        self.rrt_star = RRTStarConstrained(self.start, self.goal, self.plant_height, self.half_height)
        self.path_list=[]
        # Plan a path and store it in self.waypoints
        self.path=np.array(self.rrt_star.plan())
        self.path_list.append(self.path)
        self.target_list=[]
        
        self.num_steps=len(self.path)
        self.quaternions = self.rrt_star.slerp(self.q_start, self.q_end, self.num_steps)
        self.waypoints=np.hstack((self.path, self.quaternions))
        
        self.current_waypoint_index =0

        # Subscribe to the force/torque data
        self.force_subscriber = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.force_callback,
            10
        )

        # Variables to store force data
        self.force_current = np.zeros(6)
        self.i=0
        self.j=0
        #variables to store replaning frequency
        self.replan_n=0

    def force_callback(self, msg: WrenchStamped):
        # Extract force/torque data from the WrenchStamped message
        self.force_current = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z
        ])

    def calulate_path_length(self,waypoints):
        # Ensure the path is a NumPy array (in case it isn't)
        path = np.array(waypoints)
    
        # Initialize path length to 0
        total_length = 0
    
        # Iterate through consecutive points in the path and calculate the distance
        for i in range(1, len(path)):
            # Calculate the Euclidean distance between consecutive points
            length = np.linalg.norm(path[i] - path[i-1])
            total_length += length
    
        return total_length

    def plot_paths(self,path_list,target_list):
        fig = plt.figure(figsize=(10, 8))  # Set the figure size
        ax = fig.add_subplot(111, projection='3d')  # Create a 3D axes
    
        # Iterate through each path in the path_list
        for idx, path in enumerate(path_list):#to limit plot of only 10 paths
            # Extract x, y, and z coordinates from the path
            x = path[:, 0]
            y = path[:, 1]
            z = path[:, 2]
            x=x[:3]
            y=y[:3]
            z=z[:3]
            # Plot the 3D path with a line and markers at each waypoint
            ax.plot(x, y, z, marker='o', label=f'Path {idx + 1}')
        
        # Highlight start and goal
        ax.scatter(self.start_init[0], self.start_init[1], self.start_init[2], color='green', s=100, label='Start')
        ax.scatter(self.goal[0], self.goal[1], self.goal[2], color='blue', s=200, label='Goal Region')
        

        # Create and plot a continuous path through the target waypoints
        target_arr = np.array(target_list)  # Convert target_list to a NumPy array if it's not already
        target_x = target_arr[:, 0]
        target_y = target_arr[:, 1]
        target_z = target_arr[:, 2]
    
        # Plot the path through the target points with a thicker line
        ax.plot(target_x, target_y, target_z, color='b',linestyle='--', label='Force Aware Path', linewidth=2)  
        
        ax.set_title(' Paths After Re-Planning')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_zlim(0.0, 0.7)
        ax.legend()  
        plt.show()
            

    def timer_callback(self):
        # Check if all waypoints have been processed
        if self.current_waypoint_index >= len(self.waypoints):
            if self.j==0:
                self.j=1
                self.get_logger().info("All waypoints reached.")
                self.get_logger().info(f"Number of replanning attempt:{self.replan_n}")
                end_time = time.time()
                execution_time = end_time - self.start_time
                self.get_logger().info(f"Total Planning Time:{execution_time}")
                self.get_logger().info(f"targets followed:{self.target_list[:20]}")
                path_length=self.calulate_path_length(self.target_list)
                self.get_logger().info(f"total path length:{path_length}")
                self.plot_paths(self.path_list,self.target_list)
                
            return

        try:
            
            # Get the current waypoint
            target_pose = self.waypoints[self.current_waypoint_index]
            
            #[0.41,-0.48 ,0.71,-0.0097,0.74,-0.37,0.554]

            goal_reached = servo_it(target_pose
            ,
            self.publisher_,
            self.tf_buffer
            )
            #f self.i>0 and (np.linalg.norm(self.force_current[0]) >40 or np.linalg.norm(self.force_current[1]>40) or np.linalg.norm(self.force_current[2]>40)) :
            if self.i>0 and (np.linalg.norm(self.force_current[0]) >40 or np.linalg.norm(self.force_current[1]>40) or np.linalg.norm(self.force_current[2]>40)) :
                print("force is greater than threshold")

                transform: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link',  # target frame
                'tool0',      # source frame
                rclpy.time.Time())  # time point, use current time

                # Extract the current position from the transform
                self.start = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
                ])
            
                self.q_start = np.array([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z, 
                transform.transform.rotation.w
       
                ])
                self.rrt_star=RRTStarConstrained(self.start, self.goal, self.plant_height, self.half_height)
                # Plan a path and store it in self.waypoints
                self.path=np.array(self.rrt_star.plan())
                self.path_list.append(self.path)
                self.num_steps=len(self.path)
                self.quaternions = self.rrt_star.slerp(self.q_start, self.q_end, self.num_steps)
                self.waypoints=np.hstack((self.path, self.quaternions))
                self.current_waypoint_index =0
                self.replan_n+=1


            if goal_reached==True:
                self.i=1
                self.get_logger().info(f"Reached waypoint {self.current_waypoint_index + 1}")
                self.target_list.append(target_pose[:3])
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


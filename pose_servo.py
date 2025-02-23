import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistStamped
from constrained_rrt import RRTStarConstrained
import roboticstoolbox as rtb
import tf_transformations as tft



class Pose_servo(Node):
    def __init__(self, target_pose, gain=1):
        super().__init__('pose_servo')

        # Create a TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create servo publisher
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        
        
        # Set up a timer to continuously check for the transform
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        # provide desired translation and orientation
        target_pose = np.array(target_pose)
        self.translation_desired=target_pose[:3]
        self.quaternion_desired=target_pose[3:]              
        
        self.rotation_matrix_desired = tft.quaternion_matrix(self.quaternion_desired)[:3, :3]
            
        # Create the 4x4 homogeneous transformation matrix for goal
        self.Tg = np.eye(4)
        self.Tg[:3, :3] = self.rotation_matrix_desired  # Set the rotation part
        self.Tg[:3, 3] = self.translation_desired  # Set the translation part

        self.gain = np.array([1.0, 1.0, 1.0, 2.0, 2.0, 2.0])*gain
        

    def timer_callback(self):
        try:
            # Attempt to get the transform between 'base_link' and 'tool0'
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link',  # target frame
                'tool0',      # source frame
                rclpy.time.Time())  # time point, use current time

            # Extract the current position from the transform
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            
            quaternion = np.array([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z, 
                transform.transform.rotation.w
       
            ])
            
            rotation_matrix = tft.quaternion_matrix(quaternion)[:3, :3]
            
            # Create the 4x4 homogeneous transformation matrix
            Te = np.eye(4)
            Te[:3, :3] = rotation_matrix  # Set the rotation part
            Te[:3, 3] = translation  # Set the translation part
            
            

            # Print the transformation matrix
            self.get_logger().info(f'Homogeneous Transformation Matrix (T):\n{Te}')
                                                          
            v_e, _ = rtb.p_servo(Te, self.Tg, self.gain, threshold=0.01, method='angle-axis')
           
            print("ev is ",v_e)

            # Create a new TwistStamped message
            msg = TwistStamped()

            # Set the header with the current timestamp
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'  # Change to your reference frame if needed
            
                        

            # Set the twist (linear and angular velocities)
            msg.twist.linear.x =v_e[0]
            msg.twist.linear.y =v_e[1]
            msg.twist.linear.z =v_e[2]
            msg.twist.angular.x =v_e[3]
            msg.twist.angular.y =v_e[4] 
            msg.twist.angular.z =v_e[5]

            # Publish the message
            self.publisher_.publish(msg)
         
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Could not get transform: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    target_pose = [0.41,-0.48 ,0.71,-0.0097,0.74,-0.37,0.554]
    node = Pose_servo(target_pose)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


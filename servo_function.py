import rclpy
import numpy as np
import tf2_ros
import tf_transformations as tft
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistStamped
import roboticstoolbox as rtb


def servo_it(target_pose, twist_publisher, tf_buffer, gain=1):
    try:
        #! -------- GET CURRENT POSE --------
        transform = tf_buffer.lookup_transform(
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
        current_pose = np.eye(4)
        current_pose[:3, :3] = rotation_matrix  # Set the rotation part
        current_pose[:3, 3] = translation  # Set the translation part

        #! -------- GET TARGET POSE --------
        target_pose = np.array(target_pose)
        translation_desired=target_pose[:3]
        quaternion_desired=target_pose[3:]              
        rotation_matrix_desired = tft.quaternion_matrix(quaternion_desired)[:3, :3]
            
        # Create the 4x4 homogeneous transformation matrix for goal
        Tg = np.eye(4)
        Tg[:3, :3] = rotation_matrix_desired  # Set the rotation part
        Tg[:3, 3] = translation_desired  # Set the translation part

        #! -------- Get Twist command --------   
        gain_array = np.array([1.0, 1.0, 1.0, 2.0, 2.0, 2.0])*gain                                     
        v_e, goal_reached = rtb.p_servo(
            current_pose, Tg, 
            gain_array, 
            threshold=0.1, 
            method='angle-axis'
        )
        
        # print("ev is ",v_e)

        # Create a new TwistStamped message
        msg = TwistStamped()
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.twist.linear.x =v_e[0]
        msg.twist.linear.y =v_e[1]
        msg.twist.linear.z =v_e[2]
        msg.twist.angular.x =v_e[3]
        msg.twist.angular.y =v_e[4] 
        msg.twist.angular.z =v_e[5]

        # -------- Publish the message --------
        twist_publisher.publish(msg)

        return goal_reached
    
    except tf2_ros.TransformException as e:
        print(f"Could not get transform: {e}")
        return False

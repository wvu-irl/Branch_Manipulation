# Branch_Manipulation
This repo provides the strategy to manipulate the branch. It includes the geometric modeling of branches, constrained RRT* to generate way-points, force-aware re-planing and pose servo to follow the generated way-points
# Prerequisites
1. **Install Universal Robots ROS2 Driver**
   ```bash
   sudo apt-get install ros-distro-ur
   ```
2. **Start the driver**
  
     ```bash
   # Replace ur5e with one of ur3, ur3e, ur5, ur5e, ur7e, ur10, ur10e, ur12e, ur16e, ur20, ur30
   # Replace the IP address with the IP address of your actual robot / URSim
   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.56.xxx
     ```
3. **Start Moveit**
   ```bash
   ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true
   ```
4. **Activate Servo mode**
      ```bash
      ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_position_controller
   ```
   ```bash
   ros2 service call /servo_node/start_servo std_srvs/srv/Trigger
   ```
   

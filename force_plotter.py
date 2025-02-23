import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import matplotlib.pyplot as plt
import time  # Import time for real-time data tracking

class ForceTorqueSubscriber(Node):
    def __init__(self):
        super().__init__('force_torque_subscriber')
        # Subscribe to the /force_torque_sensor_broadcaster/wrench topic
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.listener_callback,
            10
        )
        
        # Initialize data lists for force and time
        self.force_x = []
        self.force_y = []
        self.force_z = []
        self.time_data = []

        # Create the plot window
        plt.ion()  # Enable interactive mode for real-time plotting
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.ax.set_title('Force vs Time')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Force (N)')

    def listener_callback(self, msg):
        # Extract the force data from the WrenchStamped message
        current_time = time.time()  # Get the current time using time.time()
        force_x = msg.wrench.force.x
        force_y = msg.wrench.force.y
        force_z = msg.wrench.force.z
        
        # Append new data
        self.time_data.append(current_time)
        self.force_x.append(force_x)
        self.force_y.append(force_y)
        self.force_z.append(force_z)

        # Limit the data to the last 50 samples
        if len(self.time_data) > 100:
            self.time_data.pop(0)
            self.force_x.pop(0)
            self.force_y.pop(0)
            self.force_z.pop(0)

        # Clear the plot and update it with the new data
        self.update_plot()

    def update_plot(self):
        # Clear the current plot
        self.ax.cla()
        
        # Plot the data
        self.ax.plot(self.time_data, self.force_x, label='Force X', color='r')
        self.ax.plot(self.time_data, self.force_y, label='Force Y', color='g')
        self.ax.plot(self.time_data, self.force_z, label='Force Z', color='b')

        # Update the title and labels
        self.ax.set_title('Force vs Time')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Force (N)')
        
        # Set y-axis limits from -100 to 100
        self.ax.set_ylim(-60, 60)


        # Add a legend
        self.ax.legend()

        # Redraw the plot
        plt.draw()
        plt.pause(0.1)  # Pause to allow the plot to update

def main(args=None):
    rclpy.init(args=args)
    force_torque_subscriber = ForceTorqueSubscriber()

    # Spin the ROS 2 node to keep it running and listening for messages
    rclpy.spin(force_torque_subscriber)

    # Shutdown the node when it's done
    force_torque_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


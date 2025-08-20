#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
matplotlib.use('TkAgg')
from matplotlib.animation import FuncAnimation
import numpy as np
import threading

class PathPlotter(Node):
    def __init__(self):
        super().__init__('plot_path')
        
        self.subscription = self.create_subscription(
            Odometry,
            '/iris/dynamics/odometry',
            self.odometry_callback,
            10)
            
        self.fig_xy, self.ax_xy = plt.subplots()
        self.ax_xy.set_xlabel('X Position')
        self.ax_xy.set_ylabel('Y Position')
        self.ax_xy.set_title('Robot Path (XY Projection)')

        self.fig_xz, self.ax_xz = plt.subplots()
        self.ax_xz.set_xlabel('Time')
        self.ax_xz.set_ylabel('Z Position')
        self.ax_xz.set_title('Robot Path (Z vs Time)')

        self.max_data_points = 1000000
        self.x_positions = np.full(self.max_data_points, np.nan)
        self.y_positions = np.full(self.max_data_points, np.nan)
        self.z_positions = np.full(self.max_data_points, np.nan)
        self.timestamps = np.full(self.max_data_points, np.nan)
        self.current_index = 0
        self.save_interval = 1000000
        self.save_counter = 0

    def save_data(self):
        if self.save_counter >= self.save_interval:
            np.savez('robot_path_data.npz', 
                    x=self.x_positions, 
                    y=self.y_positions, 
                    z=self.z_positions, 
                    timestamps=self.timestamps)
            self.save_counter = 0

    def odometry_callback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z
        timestamp = self.get_clock().now().nanoseconds / 1e9

        self.x_positions[self.current_index] = x
        self.y_positions[self.current_index] = y
        self.z_positions[self.current_index] = z
        self.timestamps[self.current_index] = timestamp

        self.current_index = (self.current_index + 1) % self.max_data_points
        self.save_counter += 1
        self.save_data()

    def update_plot(self, frame):
        valid_indices = ~np.isnan(self.x_positions)
        x_filtered = self.x_positions[valid_indices]
        y_filtered = self.y_positions[valid_indices]
        z_filtered = self.z_positions[valid_indices]
        timestamps_filtered = self.timestamps[valid_indices]

        # Only update plots if we have data
        if len(timestamps_filtered) > 0:
            time_elapsed = timestamps_filtered - timestamps_filtered[0]

            self.ax_xy.clear()
            self.ax_xy.plot(x_filtered, y_filtered, 'g')
            self.ax_xy.set_xlabel('X Position')
            self.ax_xy.set_ylabel('Y Position')
            self.ax_xy.set_title('Robot Path (XY Projection)')

            self.ax_xz.clear()
            self.ax_xz.plot(time_elapsed, z_filtered, 'b')
            self.ax_xz.set_xlabel('Time')
            self.ax_xz.set_ylabel('Z Position')
            self.ax_xz.set_title('Depth')
            self.ax_xz.invert_yaxis()

        # Return empty list if no artists were modified
        return []

def main(args=None):
    rclpy.init(args=args)
    
    path_plotter = PathPlotter()
    
    # Create animations
    ani_xy = FuncAnimation(path_plotter.fig_xy, path_plotter.update_plot, 
                          interval=100,
                          frames=None, 
                          repeat=False)
    ani_xz = FuncAnimation(path_plotter.fig_xz, path_plotter.update_plot, 
                          interval=100,
                          frames=None, 
                          repeat=False)
    
    # Create and start the ROS 2 spinning thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(path_plotter,))
    spin_thread.daemon = True
    spin_thread.start()
    
    # Show plots (this will block in the main thread)
    plt.show()
    
    # Cleanup
    path_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


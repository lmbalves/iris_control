import rospy
from nav_msgs.msg import Odometry
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Import 3D plotting toolkit
matplotlib.use('TkAgg')  # Use the TkAgg backend (replace with the appropriate backend)
from matplotlib.animation import FuncAnimation
import numpy as np

# Create blank graphs for XY, XZ, and YZ projections
fig_xy, ax_xy = plt.subplots()
ax_xy.set_xlabel('X Position')
ax_xy.set_ylabel('Y Position')
ax_xy.set_title('Robot Path (XY Projection)')

fig_xz, ax_xz = plt.subplots()
ax_xz.set_xlabel('X Position')
ax_xz.set_ylabel('Z Position')
ax_xz.set_title('Robot Path (XZ Projection)')

fig_yz, ax_yz = plt.subplots()
ax_yz.set_xlabel('Y Position')
ax_yz.set_ylabel('Z Position')
ax_yz.set_title('Robot Path (YZ Projection)')

# Create a 3D graph for XYZ points
fig_3d = plt.figure()
ax_3d = fig_3d.add_subplot(111, projection='3d')
ax_3d.set_xlabel('X Position')
ax_3d.set_ylabel('Y Position')
ax_3d.set_zlabel('Z Position')
ax_3d.set_title('Robot Path (3D Projection)')

# Define the maximum number of data points to keep in the circular buffer
max_data_points = 10000
x_positions = np.full(max_data_points, np.nan)
y_positions = np.full(max_data_points, np.nan)
z_positions = np.full(max_data_points, np.nan)
current_index = 0

# Define the interval to save the data (in data points)
save_interval = 10000  # Save the data every 10,000 points
save_counter = 0

def save_data():
    global save_counter
    global x_positions, y_positions, z_positions

    # Check if it's time to save the data
    if save_counter >= save_interval:
        # Save the data to a file or perform any other desired action
        np.savez('robot_path_data.npz', x=x_positions, y=y_positions, z=z_positions)
        
        # Reset the save counter
        save_counter = 0

def odometry_callback(data):
    global current_index, save_counter
    # Extract position data (x, y, z) from the odometry message
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z

    # Store the position data in the circular buffer
    x_positions[current_index] = x
    y_positions[current_index] = y
    z_positions[current_index] = z

    # Increment the circular buffer index
    current_index = (current_index + 1) % max_data_points

    # Increment the save counter
    save_counter += 1
    save_data()

def update_plot(frame):
    # Filter out the NaN values from the circular buffer
    valid_indices = ~np.isnan(x_positions)
    x_filtered = x_positions[valid_indices]
    y_filtered = y_positions[valid_indices]
    z_filtered = z_positions[valid_indices]

    # Update the XY, XZ, and YZ plots with the filtered data
    ax_xy.clear()  # Clear the previous plot
    ax_xy.plot(x_filtered, y_filtered, 'g')  # Customize the color and style for XY
    ax_xy.set_xlabel('X Position')
    ax_xy.set_ylabel('Y Position')
    ax_xy.set_title('Robot Path (XY Projection)')

    ax_xz.clear()  # Clear the previous plot
    ax_xz.plot(x_filtered, z_filtered, 'b')  # Customize the color and style for XZ
    ax_xz.set_xlabel('X Position')
    ax_xz.set_ylabel('Z Position')
    ax_xz.set_title('Robot Path (XZ Projection')
    ax_xz.invert_yaxis()  # Invert the Y-axis for XZ plot

    ax_yz.clear()  # Clear the previous plot
    ax_yz.plot(y_filtered, z_filtered, 'r')  # Customize the color and style for YZ
    ax_yz.set_xlabel('Y Position')
    ax_yz.set_ylabel('Z Position')
    ax_yz.set_title('Robot Path (YZ Projection)')
    ax_yz.invert_yaxis()  # Invert the Y-axis for YZ plot

    # Update the 3D plot with the filtered data
    ax_3d.clear()  # Clear the previous 3D plot
    ax_3d.plot(x_filtered, y_filtered, z_filtered, c='b', marker='o')  # Customize the color and marker style
    ax_3d.set_xlabel('X Position')
    ax_3d.set_ylabel('Y Position')
    ax_3d.set_zlabel('Z Position')
    ax_3d.set_title('Robot Path (3D Projection)')
    ax_3d.invert_zaxis()  # Invert the Z-axis for 3D plot

def main():
    # Initialize the ROS node
    rospy.init_node('plot_path', anonymous=True)

    # Subscribe to the odometry topic
    rospy.Subscriber("/iris/dynamics/odometry", Odometry, odometry_callback)

    # Create animations that update the XY, XZ, and YZ plots
    ani_xy = FuncAnimation(fig_xy, update_plot, frames=None, repeat=False)
    ani_xz = FuncAnimation(fig_xz, update_plot, frames=None, repeat=False)
    ani_yz = FuncAnimation(fig_yz, update_plot, frames=None, repeat=False)
    ani_3d = FuncAnimation(fig_3d, update_plot, frames=None, repeat=False)

    # Keep the script running to handle callbacks
    plt.pause(0.02)  # Add a slight pause to give Matplotlib time to update

    plt.show()  # Keep the plots open
if __name__ == '__main__':
    main()  # Start the main function


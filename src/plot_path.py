import rospy
from nav_msgs.msg import Odometry
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
matplotlib.use('TkAgg')
from matplotlib.animation import FuncAnimation
import numpy as np


fig_xy, ax_xy = plt.subplots()
ax_xy.set_xlabel('X Position')
ax_xy.set_ylabel('Y Position')
ax_xy.set_title('Robot Path (XY Projection)')

fig_xz, ax_xz = plt.subplots()
ax_xz.set_xlabel('Time')
ax_xz.set_ylabel('Z Position')
ax_xz.set_title('Robot Path (Z vs Time)')

max_data_points = 1000000
x_positions = np.full(max_data_points, np.nan)
y_positions = np.full(max_data_points, np.nan)
z_positions = np.full(max_data_points, np.nan)
timestamps = np.full(max_data_points, np.nan)
current_index = 0


save_interval = 1000000
save_counter = 0

def save_data():
    global save_counter
    global x_positions, y_positions, z_positions, timestamps

    if save_counter >= save_interval:
        np.savez('robot_path_data.npz', x=x_positions, y=y_positions, z=z_positions, timestamps=timestamps)
        save_counter = 0

def odometry_callback(data):
    global current_index, save_counter

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    timestamp = data.header.stamp.to_sec()

    x_positions[current_index] = x
    y_positions[current_index] = y
    z_positions[current_index] = z
    timestamps[current_index] = timestamp


    current_index = (current_index + 1) % max_data_points
    save_counter += 1
    save_data()

def update_plot(frame):

    valid_indices = ~np.isnan(x_positions)
    x_filtered = x_positions[valid_indices]
    y_filtered = y_positions[valid_indices]
    z_filtered = z_positions[valid_indices]
    timestamps_filtered = timestamps[valid_indices]
    time_elapsed = timestamps_filtered - timestamps_filtered[0]


    ax_xy.clear()
    ax_xy.plot(x_filtered, y_filtered, 'g')
    ax_xy.set_xlabel('X Position')
    ax_xy.set_ylabel('Y Position')
    ax_xy.set_title('Robot Path (XY Projection)')

    ax_xz.clear()
    ax_xz.plot(time_elapsed, z_filtered, 'b')
    ax_xz.set_xlabel('Time')
    ax_xz.set_ylabel('Z Position')
    ax_xz.set_title('Depth')
    ax_xz.invert_yaxis()  

def main():

    rospy.init_node('plot_path', anonymous=True)

    rospy.Subscriber("/iris/dynamics/odometry", Odometry, odometry_callback)


    ani_xy = FuncAnimation(fig_xy, update_plot, frames=None, repeat=False)
    ani_xz = FuncAnimation(fig_xz, update_plot, frames=None, repeat=False)


    plt.pause(0.02)

    plt.show() 
if __name__ == '__main__':
    main()


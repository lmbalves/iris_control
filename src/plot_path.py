import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Create a blank graph
fig, ax = plt.subplots()
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_title('Robot Path')

# Initialize empty lists to store position data
x_positions = []
y_positions = []

# Define callback function to handle odometry data
def odometry_callback(data):
    # Extract position data (x, y) from the odometry message
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    # Append the position data to the lists
    x_positions.append(x)
    y_positions.append(y)

def update_plot(frame):
    # Clear the previous plot
    ax.clear()

    # Update the plot with the new data
    ax.scatter(x_positions, y_positions, c='blue')  # Customize the color and style

def main():
    # Initialize the ROS node
    rospy.init_node('plot_path', anonymous=True)

    # Subscribe to the odometry topic
    rospy.Subscriber("/iris/dynamics/odometry", Odometry, odometry_callback)

    # Create an animation that updates the plot
    ani = FuncAnimation(fig, update_plot, frames=None, repeat=False)
    plt.show()  # Keep the plot open
    # Keep the script running to handle callbacks
    rospy.spin()

if __name__ == '__main__':
    main()
    plt.close()

    

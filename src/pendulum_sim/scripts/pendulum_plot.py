import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState



class Plotter():
    def __init__(self):
        plt.ion()

        rospy.init_node("pendulum_plot")
        rospy.loginfo("Starting visualizer")    
        self.positions = []
        self.speed = []
        self.rate = rospy.Rate(100)
        self.update_state_sub = rospy.Subscriber("/joint_states",JointState, self.update_state)

        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('Position')  # Label for the x-axis
        self.ax.set_ylabel('Speed')     # Label for the y-axis
        self.ax.set_title('Position vs Speed')


    def update_state(self,data):
        position = data.position[0]
        velocity = data.velocity[0]
        self.positions.append(position)
        self.speed.append(velocity)
    
    def plot_position_velocity(self):
        self.ax.clear()  # Clear the previous plot
        self.ax.plot(self.positions, self.speed)
        plt.pause(0.01)  # Pause to allow the plot to update
    

if __name__ == "__main__":
    plotter = Plotter()
    while not rospy.is_shutdown():
        plotter.plot_position_velocity()
        plotter.rate.sleep()
        #rospy.spin()
#!/usr/bin/env python3
import rospy
import numpy as np
import time
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class PendulumSim():
    def __init__(self):
        #Ros Node
        rospy.init_node("pendulum_sim", anonymous=False)
        self.rate = rospy.Rate(rospy.get_param("~node_rate",100))
        
        # Publisher
        self.pendulum_pub = rospy.Publisher("/joint_states", JointState, queue_size= 10)
        
        # Subscriber
        self.tau_sub = rospy.Subscriber("/tau", Float32, self.update_tau)

        #Constants
        self.MASS    = 0.75 # kg
        self.GRAVITY = 9.81 # m/s²
        self.LENGTH  = 0.36 # length in meters 
        self.FRICTION =  0.01
        self.INERTIA = (4/3) * self.MASS * (self.LENGTH/2)**2
        
        # System variables
        self.tau = 0.0 # No force :)
        self.angular_speed = 0.0
        self.output = JointState()
        self.output.name = ["joint2"]
        self.output.position = [0.0]
        self.output.velocity = [0.0]

    def correct_reference(self,theta ):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if(result < 0):
            result += 2 * np.pi
        return result - np.pi
    
    def update_tau(self, msg):
        self.tau = msg.data
    
    def simulate_pendulum(self):
        rospy.loginfo("The Pendulum sim is Running")
        pendulum_angle = 0.0
        prev_time  = time.time()
        while not rospy.is_shutdown():
            dt = time.time() - prev_time
            self.angular_speed = 1/self.INERTIA*(self.tau - self.MASS*self.GRAVITY*self.LENGTH*np.cos(pendulum_angle)/2 - self.FRICTION*self.angular_speed)
            pendulum_angle += self.angular_speed * dt
            self.output.position[0] = self.correct_reference(pendulum_angle)
            rospy.loginfo(f"Estimated {pendulum_angle}")
            self.pendulum_pub.publish(self.output)
            prev_time = time.time()
            self.rate.sleep()
         
if __name__=='__main__':
    pendulum_sim = PendulumSim()
    pendulum_sim.simulate_pendulum()
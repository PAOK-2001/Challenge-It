#include <cmath>
#include <chrono>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

using namespace std;
int nodeRate = 100;

float right_speed = 0; 
float left_speed = 0;


void receive_right_speed(const std_msgs::Float32 &received_speed){
    right_speed = received_speed.data;
}

void receive_left_speed(const std_msgs::Float32 &received_speed){
    left_speed = received_speed.data;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "dead_reckoning");
    ros::NodeHandle handler;
    ros::Rate rate(nodeRate);

    ros::Publisher odom_pub = handler.advertise<nav_msgs::Odometry>("/odom", 10);
    ros::Subscriber wl = handler.subscribe("/wr", 10, receive_right_speed);
    ros::Subscriber wr = handler.subscribe("/wl", 10, receive_left_speed);

     float _WHEELBASE, _WHEELRADIUS;
    // Get robot parameters
    ros::param::get("/wheel_base", _WHEELBASE);
    ros::param::get("/wheel_radius", _WHEELRADIUS);

    nav_msgs::Odometry odom;
    float velocity, angular_vel;

    float robot_x = 0;
    float robot_y = 0;
    float robot_theta = 0;
    float dt = 0;

    while(ros::ok){
        chrono::steady_clock::time_point t = chrono::steady_clock::now();

        velocity = _WHEELRADIUS*(right_speed + left_speed)/2;
        angular_vel = _WHEELRADIUS*(right_speed - left_speed)/_WHEELBASE;

        robot_x = robot_x + ((velocity * cos(robot_theta))*dt);
        robot_y = robot_y + ((velocity * sin(robot_theta))*dt);
        robot_theta = robot_theta + (angular_vel * dt);
        
        if(robot_theta < 0) robot_theta = robot_theta + 2*M_PI;

        odom.pose.pose.position.x = robot_x;
        odom.pose.pose.position.y = robot_y;
        odom.pose.pose.orientation.z = robot_theta;

        odom.twist.twist.linear.x = velocity;
        odom.twist.twist.angular.z = angular_vel;


        odom_pub.publish(odom);

        ros::spinOnce();
        rate.sleep();     
        dt = (chrono::duration_cast<chrono::microseconds> (chrono::steady_clock::now() - t).count())/1000000.0; // Time difference in seconds
     
     }
    
}
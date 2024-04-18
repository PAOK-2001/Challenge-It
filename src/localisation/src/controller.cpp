#include <cmath>
#include <chrono>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

using namespace std;

#define THRESHOLD 0.2

int nodeRate = 100;
nav_msgs::Path path;

float robot_x = 0;
float robot_y = 0;
float robot_orientation = 0;

void receive_path(const nav_msgs::Path &received_path){
    path = received_path;
}

void receive_odom(const nav_msgs::Odometry &received_odom){
    robot_x = received_odom.pose.pose.position.x;
    robot_y = received_odom.pose.pose.position.y;
    robot_orientation = received_odom.pose.pose.orientation.z;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "controller");
    ros::NodeHandle handler;
    ros::Rate rate(nodeRate);

    ros::Subscriber reference = handler.subscribe("/path", 10, receive_path);
    ros::Subscriber odom_sub = handler.subscribe("/odom", 10, receive_odom);
    ros::Publisher control_out =  handler.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    float kpr, kit, kpt;
    ros::param::get("/rotational_constant", kpr);
    ros::param::get("/translational_constant", kpt);
    ros::param::get("/translational_integral_constant", kit);

    float _WHEELBASE, _WHEELRADIUS;
    ros::param::get("/wheel_base", _WHEELBASE);
    ros::param::get("/wheel_radius", _WHEELRADIUS);

    float dt = 0;
    float w_max = 8;
    float v_max = (w_max*_WHEELRADIUS)*0.5;
    float angular_max = 1;

    float integral_trr = 0;
    float integral_rr = 0;
    geometry_msgs::Twist output;
    
    while(ros::ok){
          for (auto coord : path.poses){
                float desired_x = coord.pose.position.x;
                float desired_y = coord.pose.position.y;
                float desired_angle;

                float distance = sqrt(pow(desired_y-robot_y,2)+pow(desired_x-robot_x,2));
                while (distance > THRESHOLD){
                    chrono::steady_clock::time_point t = chrono::steady_clock::now();
                    rate.sleep();
                    desired_angle = atan2((desired_y-robot_y),(desired_x - robot_x));
                    distance = sqrt(pow(desired_y-robot_y,2)+pow(desired_x-robot_x,2));
                    float angle_error = (desired_angle - robot_orientation);

                    if(angle_error >  M_PI) angle_error = angle_error - 2*M_PI;
                    else if(angle_error < - M_PI) angle_error = angle_error + 2*M_PI;

                    float angular_velocity = kpr*angle_error;
                    if(angular_velocity > angular_max) angular_velocity = angular_max;
                    else if (angular_velocity < -angular_max) angular_velocity = -angular_max;
                    
                    float velocity = kpt*distance + kit*integral_trr; 
                    velocity = v_max * tanh(velocity);
                    integral_trr+=distance;
            
                    output.linear.x = velocity;
                    output.linear.y = 0;
                    output.linear.z = 0;

                    output.angular.x = 0;
                    output.angular.y = 0;
                    output.angular.z = angular_velocity;

                    dt = (chrono::duration_cast<chrono::microseconds> (chrono::steady_clock::now() - t).count())/1000000.0;
                               
                    control_out.publish(output);
                    ros::spinOnce();
                    
                }
            }
            integral_trr = 0;
            ros::spinOnce();
            rate.sleep();
        }
    
}
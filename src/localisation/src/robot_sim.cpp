#include <chrono>
#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>

using namespace std;

int nodeRate = 100;
geometry_msgs::Twist cmd_vel;
bool is_receiving;

void receive_cmd_vel(const geometry_msgs::Twist &received_speed){
    cmd_vel = received_speed;
    is_receiving = true;
}

void get_wheel_speed(geometry_msgs::Twist cmd_vel, float &right_vel, float &left_vel, float _WHEELBASE, float _WHEELRADIUS){
    float vel = cmd_vel.linear.x;
    float angular_vel = cmd_vel.angular.z;

    right_vel = _WHEELBASE/(2*_WHEELRADIUS) * angular_vel + vel/_WHEELRADIUS;
    left_vel  = vel/_WHEELRADIUS - _WHEELBASE/(2*_WHEELRADIUS)*angular_vel;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "robot_sim");
    ros::NodeHandle handler;
    ros::Rate rate(nodeRate);
    // Init Subscribers
    ros::Subscriber cmd_vel_sub = handler.subscribe("/cmd_vel", 10, receive_cmd_vel);
    // ros::Subscriber initial_pose = handler.subscribe("/intialpose",10, receive_initial_pose)
    // Init Publishers
    ros::Publisher pose_pub = handler.advertise<geometry_msgs::Pose>("/pose", 10);
    ros::Publisher angular_speed_right = handler.advertise<std_msgs::Float32>("/wr", 10);
    ros::Publisher angular_speed_left  = handler.advertise<std_msgs::Float32>("wl", 10);
    float _WHEELBASE, _WHEELRADIUS;
    // Get robot parameters
    ros::param::get("/wheel_base", _WHEELBASE);
    ros::param::get("/wheel_radius", _WHEELRADIUS);
    // Init state variables
    float left_vel = 0;
    float right_vel = 0;
    float robot_x = 0;
    float robot_y = 0;
    float robot_theta = 0;
    float dt = 0;
    geometry_msgs::Pose pose;
    std_msgs::Float32 wr, wl;

    // Initial robot conditions
    while(ros::ok){
        chrono::steady_clock::time_point t = chrono::steady_clock::now();
        get_wheel_speed(cmd_vel, right_vel, left_vel, _WHEELBASE, _WHEELRADIUS);

        robot_x = robot_x + ((cmd_vel.linear.x * cos(robot_theta))*dt);
        robot_y = robot_y + ((cmd_vel.linear.x * sin(robot_theta))*dt);
        robot_theta =robot_theta + (cmd_vel.angular.z * dt);

        // Publish robot's messages
        pose.position.x = robot_x;
        pose.position.y = robot_y;
        pose.orientation.z = robot_theta;
        wr.data = right_vel;
        wl.data = left_vel;


        pose_pub.publish(pose);
        angular_speed_right.publish(wr);
        angular_speed_left.publish(wl);

        ros::spinOnce();
        rate.sleep();     
        dt = (chrono::duration_cast<chrono::microseconds> (chrono::steady_clock::now() - t).count())/1000000.0; // Time difference in seconds
    }
}
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <chrono>

using namespace std;

int nodeRate = 100;
geometry_msgs::Twist cmd_vel;
bool is_receiving

void receive_cmd_vel(const geometry_msgs::Twist &cmd_vel){
    cmd_vel = received_speed.data;
    isReceiving = true;
}




int main(int argc, char *argv[]) {
    ros::init(argc, argv, "robot_sim");
    ros::NodeHandle handler;
    ros::Rate rate(nodeRate);
    // Init Subscribers
    ros::Subscriber cmd_vel_sub = handler.subscribe("/cmd_vel", 10, receive_cmd_vel);
    // ros::Subscriber initial_pose = handler.subscribe("/intialpose",10, receive_initial_pose)
    // Init Publishers
    ros::Publisher pose = handler.advertise<geometry_msgs::PoseStamped>("/pose", 10);
    ros::Publisher angular_speed_right = handler.advertise<std_msgs::Float32>("/wr", 10);
    ros::Publisher angular_speed_left  = handler.advertise<std_msgs::Float32>("wl", 10);
    // Get robot parameters
    float _WHEELBASE, _WHEELRADIUS;

    ros::param::get("/wheel_base", _WHEELBASE);
    ros::param::get("/wheel_radius", _WHEELRADIUS);

    // Initial robot conditions
    while(ros::ok){


        ros::spinOnce();
        rate.sleep();     
    }
}
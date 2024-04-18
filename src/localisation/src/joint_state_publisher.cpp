#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

int nodeRate = 100;

void receive_odom(const nav_msgs::Odometry &received_odom){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "chassis";
    transformStamped.transform.translation.x = received_odom.pose.pose.position.x;
    transformStamped.transform.translation.y = received_odom.pose.pose.position.y;
    transformStamped.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, received_odom.pose.pose.orientation.z);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "joint_states");  
    ros::NodeHandle handler;
    ros::Rate rate(nodeRate);

    ros::Subscriber sub = handler.subscribe("/odom", 10, receive_odom);
    ros::spin();
    return 0;

}
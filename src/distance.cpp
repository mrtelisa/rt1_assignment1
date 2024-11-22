#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <string>
#include <iostream>
#include <cmath>

turtlesim::Pose turtle1_pose, turtle2_pose;
turtlesim::Pose turtle1_prev, turtle2_prev;
double prev_dist = 0.0;

ros::Publisher pub_turtle1;
ros::Publisher pub_turtle2;
ros::Publisher dist_pub;

geometry_msgs::Twist stop_msg;

std::string selected_turtle;

// Callback to update turtle1 position
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle1_prev = turtle1_pose;
    turtle1_pose = *msg;
}

// Callback to update turtle2 position
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle2_prev = turtle2_pose;
    turtle2_pose = *msg;
}

// Calculating the distance between the two turtles
double calculateDist(const turtlesim::Pose& pose1, const turtlesim::Pose& pose2) {
    return std::sqrt(std::pow(pose2.x - pose1.x, 2) + std::pow(pose2.y - pose1.y, 2));
}

// Getting the name of the turtle which is moving
void selectedTurtleCallback(const std_msgs::String::ConstPtr& msg) {
    selected_turtle = msg->data;
}

// Function to stop the turtle
void stopTurtle(double dist){
    bool collision = (dist < 1.0 && dist < prev_dist);
    bool turtle1_border = (turtle1_pose.x >= 9 || turtle1_pose.x <= 2 || turtle1_pose.y >= 9 || turtle1_pose.y <= 2);
    bool turtle2_border = (turtle2_pose.x >= 9 || turtle2_pose.x <= 2 || turtle2_pose.y >= 9 || turtle2_pose.y <= 2);
    
    if(collision){
        ROS_INFO("Stopping turtles due to collision.");
        pub_turtle1.publish(stop_msg);
        pub_turtle2.publish(stop_msg);
    }

    if(turtle1_border){
        ROS_INFO("Stopping turtle1 due to border collision.");
        pub_turtle1.publish(stop_msg);
    }

    if(turtle2_border){
        ROS_INFO("Stopping turtle2 due to border collision.");
        pub_turtle2.publish(stop_msg);
    }
}


int main (int argc, char **argv){

	ros::init(argc, argv, "turtle_distance");  
	ros::NodeHandle nh;
	
	// Initialising publisher and subcriber
    pub_turtle1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    pub_turtle2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 1);
    dist_pub = nh.advertise<std_msgs::Float32>("turtle_distance", 1);

    ros::Subscriber sub_turtle1 = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber sub_turtle2 = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);

    // Subscriber for selected turtle
    ros::Subscriber turtle_sub = nh.subscribe("/selected_turtle", 10, selectedTurtleCallback);

    // Stop message initialization
    stop_msg.linear.x = 0.0;
    stop_msg.linear.y = 0.0;
    stop_msg.angular.z = 0.0;

    ros::Rate rate(10);	
    
    while(ros::ok()){        
        double dist = calculateDist(turtle1_pose, turtle2_pose);

        std_msgs::Float32 dist_msg;
        dist_msg.data = dist; 
        dist_pub.publish(dist_msg);

        stopTurtle(dist);

        prev_dist = dist;
    
        ros::spinOnce();
        rate.sleep();
    }

	return 0;
}


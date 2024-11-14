#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include <cmath>

turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;
std::string selected_turtle;

// Callback to update turtle1 position
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle1_pose = *msg;
}

// Callback to update turtle2 position
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
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
void stopTurtle(ros::Publisher& pub){

    geometry_msgs::Twist stop;
    stop.linear.x = 0;
    stop.linear.y = 0;
    stop.angular.z = 0;
    pub.publish(stop);
}


int main (int argc, char **argv)
{
	ros::init(argc, argv, "turtle_distance");  
	ros::NodeHandle nh;
	
	// Initialising publisher and subcriber
    ros::Publisher pub_turtle1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher pub_turtle2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
    ros::Subscriber sub_turtle1 = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber sub_turtle2 = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);

    // Subscriber for selected turtle
    ros::Publisher distance = nh.advertise<std_msgs::String>("/dist", 10);
    ros::Subscriber turtle_sub = nh.subscribe("/selected_turtle", 10, selectedTurtleCallback);

    ros::Rate rate(10);  	
    
    while(ros::ok()){
        
        ros::spinOnce();

        double dist = calculateDist(turtle1_pose, turtle2_pose);
       
        if (selected_turtle == "turtle1") {

            if (turtle1_pose.x > 10 || turtle1_pose.y > 10 || turtle1_pose.x < 1 || turtle1_pose.y < 1) {

                stopTurtle(pub_turtle1);
                std::cout << "Stopping turtle1 due to position limits.\n";
            }

            if(dist <= 1){

                stopTurtle(pub_turtle1);
                std::cout << "turtle1 too close!\n";
            }

        } else if (selected_turtle == "turtle2") {

            if (turtle2_pose.x > 10 || turtle2_pose.y > 10 || turtle2_pose.x < 1 || turtle2_pose.y < 1) {

                stopTurtle(pub_turtle2);
                std::cout << "Stopping turtle2 due to position limits.\n";
            }

            if(dist <= 1){

                stopTurtle(pub_turtle2);
                std::cout << "turtle2 too close!\n";
            }
        }
    }

	return 0;
}


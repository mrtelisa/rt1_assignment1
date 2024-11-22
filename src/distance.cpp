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

void moveTurtle(ros::Publisher& pub, const turtlesim::Pose& pose){
    geometry_msgs::Twist move;
    if (pose.x <= 1) {
        move.linear.x = 1.0;
    } else if (pose.x >= 10) {
        move.linear.x = -1.0;
    }

    if (pose.y <= 1) {
        move.linear.y = 1.0;
    } else if (pose.y >= 10) {
        move.linear.y = -1.0;
    }

    pub.publish(move);
}

void separateTurtles(ros::Publisher& pub, const turtlesim::Pose& pose1, const turtlesim::Pose& pose2){
    geometry_msgs::Twist move;
    
    if (calculateDist(pose1, pose2) < 1) {
        move.linear.x = (pose1.x < pose2.x) ? -0.5 : 0.5;
        move.linear.y = (pose1.y < pose2.y) ? -0.5 : 0.5;
        pub.publish(move);
    } else {
        stopTurtle(pub);
    } 
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
                moveTurtle(pub_turtle1, turtle1_pose);
                std::cout << "Stopping turtle1 due to position limits.\n";
                stopTurtle(pub_turtle1);
            }
            
            if (dist <= 1){
                separateTurtles(pub_turtle1, turtle1_pose, turtle2_pose);
                std::cout << "turtle1 too close!\n";
                stopTurtle(pub_turtle1);
            }
            

        } else if (selected_turtle == "turtle2") {

            if (turtle2_pose.x > 10 || turtle2_pose.y > 10 || turtle2_pose.x < 1 || turtle2_pose.y < 1) {
                moveTurtle(pub_turtle2, turtle2_pose);
                std::cout << "Stopping turtle2 due to position limits.\n";
                stopTurtle(pub_turtle2);
            }
            
            if (dist <= 1){
                separateTurtles(pub_turtle2, turtle1_pose, turtle2_pose);
                std::cout << "turtle2 too close!\n";
                stopTurtle(pub_turtle2);
            }

        }
        
        rate.sleep();
    }

	return 0;
}


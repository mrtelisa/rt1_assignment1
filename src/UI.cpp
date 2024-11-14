#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>


// Function to make the turtle move as the user wants
void setVel(ros::Publisher& pub, double linear_x, double linear_y, double angular){

    geometry_msgs::Twist move;
    move.linear.x = linear_x;
    move.linear.y = linear_y;
    move.angular.z = angular;
    pub.publish(move);
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
	ros::init(argc, argv, "turtle_control");  
	ros::NodeHandle nh;
	
    // Service client to spawn a new turtle
	ros::ServiceClient client1 =  nh.serviceClient<turtlesim::Spawn>("/spawn");

	// Modify the starting position of "turtle2"
	turtlesim::Spawn srv1;
	srv1.request.x = 2.0;  
	srv1.request.y = 1.0;
	srv1.request.theta = 0.0;
	srv1.request.name = "turtle2";
	client1.call(srv1);
	
	// Initialising publisher and subcriber
    ros::Publisher pub_turtle1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	ros::Publisher pub_turtle2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    // Publisher to let the second node know which turtle is moving
    ros::Publisher turtle_pub = nh.advertise<std_msgs::String>("/selected_turtle", 10);
	     	
    std::string selected_turtle;
    double linear_vel_x, linear_vel_y, angular_vel;

    while(ros::ok()){
        // User input in order to select a turtle
        std::cout << "Select the turtle you want to control (turtle1 or turtle2): ";
        std::cin >> selected_turtle;

        if (selected_turtle != "turtle1" && selected_turtle != "turtle2"){

            std::cout << "Error: non valid name. Please retry.\n";
            continue;
        }

        // Publishing info about the selected_turtle
        std_msgs::String turtle_msg;
        turtle_msg.data = selected_turtle;
        turtle_pub.publish(turtle_msg);

        // User input for setting velocities
        std::cout << "Insert linear velocity along x-axis:";
        std::cin >> linear_vel_x;

        std::cout << "Insert linear velocity along y-axis:";
        std::cin >> linear_vel_y;

        std::cout << "Insert angular velocity:";
        std::cin >> angular_vel;

        // Sending commands to the turtle thanks to the void function
        if (selected_turtle == "turtle1"){
            setVel(pub_turtle1, linear_vel_x, linear_vel_y, angular_vel);
        }

        if (selected_turtle == "turtle2"){
            setVel(pub_turtle2, linear_vel_x, linear_vel_y, angular_vel);
        }

        std::cout << "Command sent to" << selected_turtle << "\n";

        // Waiting 1 second
        ros::Duration(1.0).sleep();

        // Making the selected_turtle stop
        if (selected_turtle == "turtle1"){
            stopTurtle(pub_turtle1);
        }

        if (selected_turtle == "turtle2"){
            stopTurtle(pub_turtle2);
        }
    }

	return 0;
}






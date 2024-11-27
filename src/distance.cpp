#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <string>
#include <iostream>
#include <cmath>

//turtlesim::Pose turtle1_pose, turtle2_pose;
//turtlesim::Pose turtle1_prev, turtle2_prev;
//double prev_dist = 0.0;

ros::Publisher pub_turtle1;
ros::Publisher pub_turtle2;
ros::Publisher dist_pub;

geometry_msgs::Twist stop_msg{};

float x_turtle1[2] = {0,0},
      y_turtle1[2] = {0,0},
      x_turtle2[2] = {0,0},
      y_turtle2[2] = {0,0};

float distance[2] = {0,0};

bool bord_dist1 = false;
bool bord_dist2 = false;
bool stop;

void handle_stopping(){
    if(stop){
        ROS_INFO("Stopping the turtles due to collision.");
        stop =  false;
        pub_turtle1.publish(stop_msg);
        pub_turtle2.publish(stop_msg);
    }

    if(bord_dist1){
        ROS_INFO("Stopping turtle1 due to collision with the border.");
        bord_dist1 = false;
    }
    
    if(bord_dist2){
        ROS_INFO("Stopping turtle2 due to collision with the border.");
        bord_dist2 = false;
    }
}

void set_vel1(const turtlesim::Pose::ConstPtr& msg){
    x_turtle1[1] = x_turtle1[0];
    y_turtle1[1] = y_turtle1[0];
    x_turtle1[0] = msg->x;
    y_turtle1[0] = msg->y;
}

void set_vel2(const turtlesim::Pose::ConstPtr& msg){
    x_turtle2[1] = x_turtle2[0];
    y_turtle2[1] = y_turtle2[0];
    x_turtle2[0] = msg->x;
    y_turtle2[0] = msg->y;
}


int main (int argc, char **argv){

	ros::init(argc, argv, "turtle_distance");  
	ros::NodeHandle nh;

    dist_pub = nh.advertise<std_msgs::Float32>("turtle_distance", 1);
    std_msgs::Float32 distance_msg;

	// Initialising publisher and subcriber
    pub_turtle1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    pub_turtle2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 1);

    ros::Subscriber sub_turtle1 = nh.subscribe("/turtle1/pose", 10, set_vel1);
    ros::Subscriber sub_turtle2 = nh.subscribe("/turtle2/pose", 10, set_vel2);

    ros::Rate rate(10);	
    
    while(ros::ok()){        
        
        distance[1] =  distance[0];
        distance[0] = sqrt(pow(x_turtle1[0] - x_turtle2[0], 2) + pow(y_turtle1[0] - y_turtle2[0], 2));
        
        distance_msg.data =  distance[0];
        dist_pub.publish(distance_msg);

        if(distance[0] < 1 && distance[0] < distance[1]) { 
            stop = true; 
        }

        bool turtle1_border = ((x_turtle1[0] >= 9 && (x_turtle1[0] > x_turtle1[1]))||
                               (x_turtle1[0] <= 2 && (x_turtle1[0] < x_turtle1[1]))||
                               (y_turtle1[0] >= 9 && (y_turtle1[0] > y_turtle1[1]))||
                               (y_turtle1[0] <= 2 && (y_turtle1[0] < y_turtle1[1])));
        bool turtle2_border = ((x_turtle2[0] >= 9 && (x_turtle2[0] > x_turtle2[1]))||
                               (x_turtle2[0] <= 2 && (x_turtle2[0] < x_turtle2[1]))||
                               (y_turtle2[0] >= 9 && (y_turtle2[0] > y_turtle2[1]))||
                               (y_turtle2[0] <= 2 && (y_turtle2[0] < y_turtle2[1])));
        
        if(turtle1_border) {
            bord_dist1 = true;
            pub_turtle1.publish(stop_msg);

        }

        if(turtle2_border) {
            bord_dist2 = true;    
            pub_turtle2.publish(stop_msg);
        }

        handle_stopping();

        ros::spinOnce();
        rate.sleep();
    }

	return 0;
}


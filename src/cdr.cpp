#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <cmath>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <math.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/duration.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <vector>
#include "control_functions.hpp"

using namespace std;

// Set global variables
mavros_msgs::State current_state;
// nav_msgs::Odometry current_pose;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped pose;
std_msgs::Float64 current_heading;
float GYM_OFFSET;
std_msgs::String qr;
bool moving = false;


// set orientation of the drone (drone should always be level)
void voice_cb(const std_msgs::String::ConstPtr& voice)
{
    std_msgs::String v = *voice;
    //boost::algorithm::to_lower(v);
    if (v.data == "takeoff") {
            takeoff(1);
            moving = true;
    } else if (v.data == "qr code") {
        set_destination(1,1,1, 0);
        moving = true;
    } else if (v.data == "land") {

        land();
    }
}

void qr_cb(const std_msgs::String::ConstPtr& codes){
    qr = *codes;
}


int main(int argc, char** argv)
{
	qr.data = "null";
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    init_publisher_subscriber(nh);
    ros::Subscriber voiceRecognition = nh.subscribe<std_msgs::String>("Android", 10, voice_cb);
    ros::Subscriber QR = nh.subscribe<std_msgs::String>("CV", 10, qr_cb);

    wait4start();

    initialize_local_frame();

    wait4connect();

    ros::spinOnce();

    while(!moving)
    {
        ros::spinOnce();
    }

    float tollorance = .2;
    if (local_pos_pub) {

        for (int i = 10000; ros::ok() && i > 0; --i) {
            if(check_waypoint_reached(tollorance)){
                break;
            }
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            if (i == 1) {
                ROS_INFO("Failed to reach destination. Stepping to next task.");
            }
        }
        ROS_INFO("Done moving foreward.");
    }
    //Waiting for QR recognition
    float offset = .1;
    for(int i = 10000; qr.data == "null" && ros::ok() && i > 0; --i){
        if(check_waypoint_reached(tollorance)){
            offset *= -1;
            set_destination(1 + offset, 1, 1, 0);
        }
        ros::spinOnce();
        ros::Duration(0.02).sleep();
        if (i == 1) {
            ROS_INFO("Failed to reach destination. Stepping to next task.");
        }
    }
    ROS_INFO("Done moving foreward.");
    set_destination(0,0,1, 0);
    if (local_pos_pub) {

            for (int i = 10000; ros::ok() && i > 0; --i) {
                if(check_waypoint_reached(tollorance)){
                    break;
                }
                ros::spinOnce();
                ros::Duration(0.5).sleep();
                if (i == 1) {
                    ROS_INFO("Failed to reach destination. Stepping to next task.");
                }
            }
            ROS_INFO("Done moving foreward.");
    }
    return 0;
}

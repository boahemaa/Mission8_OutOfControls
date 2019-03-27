#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/UInt16.h"
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

std::vector<float> c;
std::vector<UInt8> dxlist;
std::vector<UInt16> plist;

// set orientation of the drone (drone should always be level)
void voice_cb(const std_msgs::String::ConstPtr& voice)
{
    std_msgs::String v = *voice;
    //boost::algorithm::to_lower(v);
    if (v.data == "takeoff") {
            takeoff(1);
            //moving = true;

    } else if (v.data == "qr code") {
    	c[0] = 0;
    	c[1] = 5;
    	c[2] = 1;
        set_destination(c[0],c[1],c[2], 0);
        moving = true;
    } else if (v.data == "land") {

        land();
    }
}

void qr_cb(const std_msgs::String::ConstPtr& codes){

    qr = *codes;
}

void point_cb(const std_msgs::UInt16::ConstPtr& numPoints){
	std_msgs::UInt16 p = *numPoints;
    if(plist.size() < 2){
        plist.push_back(p);
    }
    else{
        std_msgs::UInt16 tol = 50;
        std_msgs::UInt16 p_predicted = dxlist[1]*(plist[1] - plist[0])/dxlist[0] - plist[1];
        if(p > p_predicted-tol && p < p_predicted+tol){
            dxlist[0] = dxlist[1];
            dxlist[1] = 1;
            plist[0] = plist[1];
            plist[1] = p;
        }
        else{
            p = (3*p_predicted)/5 + (2*p)/5;
            if(p > p_predicted-tol && p < p_predicted+tol){
                dxlist[0] = dxlist[1];
                dxlist[1] = 1;
                plist[0] = plist[1];
                plist[1] = p;
            }
            else{
                dxlist[1]++;
            }
        }
    }
}

float deltaZ(int p){
	return .0000000005*[](float x){return x * x * x;}((.2*(p-6400)));
}


int main(int argc, char** argv)
{
	qr.data = "null";
    dxlist[0] = 1;
    dxlist[1] = 1;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    init_publisher_subscriber(nh);
    ros::Subscriber voiceRecognition = nh.subscribe<std_msgs::String>("Android", 10, voice_cb);
    ros::Subscriber QR = nh.subscribe<std_msgs::String>("CV", 10, qr_cb);
    ros::Subscriber points = nh.subscribe<std_msgs::UInt16>("Points", 5, point_cb);

    wait4start();

    initialize_local_frame();

    wait4connect();

    ros::spinOnce();

    while(!moving)
    {
        ros::spinOnce();
    }

    float tollorance = .2;
    //if (local_pos_pub) {

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
        ROS_INFO("Done moving forward.");
    //}
    //Waiting for QR recognition
    float r = .2;
    float t = 0;
    for(int i = 10000; qr.data == "null" && ros::ok() && i > 0; --i){
        if(check_waypoint_reached(tollorance)){
            set_destination(c[0] + r*cos(t), c[1] + r*sin(t), c[2] + deltaZ(plist[1]), 0);
            //ROS_INFO("Setting destination QR");
        }
        ros::spinOnce();
        ros::Duration(0.02).sleep();
        if (i == 1) {
            ROS_INFO("Failed to reach destination. Stepping to next task.");
        }
        t+=.1;
    }
    ROS_INFO("Got QR Code.");
    set_destination(0,0,1, 0);
    if (local_pos_pub) {

            for (int i = 10000; ros::ok() && i > 0; --i) {
                if(check_waypoint_reached(tollorance)){
                    moving = false;
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
    while(!moving)
    {
        ros::spinOnce();
    }
    return 0;
}

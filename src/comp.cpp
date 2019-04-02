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
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped pose;
std_msgs::Float64 current_heading;
float GYM_OFFSET;
std_msgs::String qr;
bool moving = false;
std_msgs::String msg;
vector<float> c;
std::vector<UInt16> plist;
std_msgs::UInt16 p;

void voice_cb(const std_msgs::String::ConstPtr& voice)
{
    std_msgs::String v = *voice;
    msg = v;
}

void qr_cb(const std_msgs::String::ConstPtr& codes){

    qr = *codes;
}

void point_cb(const std_msgs::UInt16::ConstPtr& numPoints){
	std_msgs::UInt16 points = *numPoints;
    if(plist.size() == 5){
        std_msgs::UInt32 sum = 0;
        for(int i = 0; i < 5; i++){
            sum+=plist[i];
            plist.pop_back();
        }
        p = sum/5;
    }
    plist.push_back(points);

}

//to be changed: function will accept a column of a 2D vector
void flyTo(float x, float y, float z){
	set_destination(x,y,z, 0);
	float tol = .2;
	//to be added: while loop, time-based timeout, obstacle avoidance
        for (int i = 10000; ros::ok() && i > 0; --i) {
            if(check_waypoint_reached(tol)){
                break;
            }
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            if (i == 1) {
                ROS_INFO("Failed to reach destination. Stepping to next task.");
            }
        }
        ROS_INFO("Done moving forward.");
}

//to be changed: function will accept a column of a 2D vector
void QR(float x, float y, float z){
	float tol = .2;
	float r = .2;
    float t = 0;
    //to be added: while loop, time-based timeout, obstacle avoidance
    for(int i = 10000; qr.data == "null" && ros::ok() && i > 0; --i){
        if(check_waypoint_reached(tol) && z + deltaZ(p) >= .5 && z + deltaZ(p) <= 1){
            set_destination(x + r*cos(t), y + r*sin(t), z + deltaZ(p), 0);
        }
        ros::spinOnce();
        ros::Duration(0.02).sleep();
        if (i == 1) {
            ROS_INFO("Failed to reach destination. Stepping to next task.");
        }
        t+=.1;
    }
    ROS_INFO("Got QR Code.");
}

float deltaZ(int n){
	return .0000000005*[](float x){return x * x * x;}((.2*(n-6400)));
}


int main(int argc, char** argv)
{
	qr.data = "null";
	msg = "nothin";
    ros::init(argc, argv, "outtaControls");
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

    while(ros::ok()){
    	ros::spinOnce();
    	//add obstacle avoidance here too 
    	switch(msg){
    		case "takeoff": takeoff(1)
    						break;
    		case "qr code": c[0] = 0; //This will be later refactored
					    	c[1] = 5; //to store the locations of 
					    	c[2] = 1; //all 4 qr codes
					        flyTo(c[0],c[1], c[2]);
					        QR(c[0],c[1], c[2]);
					        break;
			case "land": land()
						 break();
    	}
    	msg.data = "nuthin";
    }
    
    return 0;
}
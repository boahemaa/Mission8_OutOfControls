#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Range.h"
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
#include <ros/time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "control_functions.hpp"
#include <errno.h>
#include <string.h>

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
std::vector<std_msgs::UInt16> plist;
std_msgs::UInt16 cv_points;
vector<float> simSonar;
int up;


//to be changed: function will accept a column of a 2D vector
void flyTo(float x, float y, float z){
	set_destination(x,y,z, 0);
	float tol = .2;
	ros::Time start = ros::Time::now();
	while(ros::ok() && !(check_waypoint_reached(tol)) && (ros::Time::now().toSec() - start.toSec() < 60)){
		if(msg.data == "stop"){
			break;
		}
		if(!(avoid())){
			set_destination(x,y,z,0);
			ros::Duration(.5).sleep();
		}
		ros::spinOnce();
		ros::Duration(0.3).sleep();
	}

    ROS_INFO("Done moving to (%f, %f, %f).", x, y, z);

}

/*
tracking_cb 
inputs: ros array which contains xzy vector in drone's reference frame 
outputs: nothing n/a
postcondition: the drone is moving towards the new destion calculated
*/
void tracking_cb(const std_msgs::Float32::ConstPtr& tracking){
	//input: 
	information = *tracking;
	vector location;
	location.push_back(information.data[0].data);
	location.push_back(information.data[1].data);
	location.push_back(information.data[2].data);
	set_destination_local(location);


}

int main(int argc, char** argv)
{

	
    ros::init(argc, argv, "outtaControls");
    ros::NodeHandle nh;
    ros::Time runStart = ros::Time::now();

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    init_publisher_subscriber(nh);
    


    ros::Subscriber points = nh.subscribe<std_msgs::String>("Points", 5, tracking_cb);


    wait4connect();
    cout << "connected" << endl;
    wait4start();
    cout << "started" << endl;
    initialize_local_frame();
    cout << "local frame" << endl;

    ros::spinOnce();


    return 0;
}

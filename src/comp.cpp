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
vector<float> sonars{3};
int up;
int drone_mode; //0 = on the ground, 1 = in air, 2 = avoiding


void voice_cb(const std_msgs::String::ConstPtr& voice)
{
    std_msgs::String v = *voice;
    msg = v;
}

void qr_cb(const std_msgs::String::ConstPtr& codes){

    qr = *codes;
}

void point_cb(const std_msgs::UInt16::ConstPtr& numPoints){
    std_msgs::UInt16 nPoints = *numPoints;
    int max = 0;
    int min = 0;
    //dropping highest and lowest point
    if(plist.size() == 5){
    	for(int i = 1; i < 5; i++){
    		if(plist[max].data < plist[i].data){
    			max = i;
    		}
    		else if(plist[min].data > plist[i].data){
    			min = i;
    		}
    	}
    	plist[max].data = 0;
    	plist[min].data = 0;
        std_msgs::UInt16 sum;
        sum.data = 0;
        for(int i = 0; i < 5; i++){
            sum.data+=plist[i].data;
            plist.pop_back();
        }
        cv_points.data = sum.data/3;
    }
    plist.push_back(nPoints);
}

int avoid(){
	//Dirction order: 0- north, 1- east, 2- south, 3- west
	vector<int> avoid_obs; //whether we need to avoid in a certain direction or not
	vector<float> proportion; //proportion (how close is the object we need to avoid);
	int sum = 0;
	float tol = 1.5;
	float move_mag = 3.0;//this is how much you want the drone to move
	float h = 0.0;
	for(int i = 0; i < sonars.size(); i++){
		if(sonars[i] < tol){
			ROS_INFO("Direction: %d , obs distance: %f", i , sonars[i]);
			avoid_obs.push_back(1);
			proportion.push_back(tol - sonars[i]);
			sum++;
		}
		else{
			avoid_obs.push_back(0);
			proportion.push_back(0);
		}
	}
	if(sum == 0){
		drone_mode = 1; //1 = in air
	}
	// //if surrounded in all 4 directions or in 2 opposite directions
	// if(sum == 4 || (avoid_obs[3]-avoid_obs[1] == 0 && avoid_obs[2]-avoid_obs[0] != 0) || (avoid_obs[2]-avoid_obs[0] == 0 && avoid_obs[3]-avoid_obs[1] != 0)){
	// 	if(current_pose_g.pose.pose.position.z >= 1.5){
	// 		h = -.5;
	// 	}
	// 	else{
	// 		h = .5;
	// 	}
	// }

	//Need to update postition accourdingly 
	drone_mode = 2; //2 = avoiding // might be worth putting in a function that leaves this as avoiding until checkpoint reached
	set_destination(current_pose_g.pose.pose.position.x + move_mag*(proportion[1]+proportion[3])*((avoid_obs[3]-avoid_obs[1])*cos(current_heading_g) + (avoid_obs[2]-avoid_obs[0])*sin(current_heading_g)),
		current_pose_g.pose.pose.position.y + move_mag*(proportion[0]+proportion[2])*(-1*(avoid_obs[3]-avoid_obs[1])*sin(current_heading_g) + (avoid_obs[2]-avoid_obs[0])*cos(current_heading_g)),
		current_pose_g.pose.pose.position.z + h, 0);
	
	ROS_INFO("SKKKKKRRRRRTTTT SKKKKKRRRRRTTTT\n");

}

void avoid_n_cb(const sensor_msgs::Range::ConstPtr& direction){
	sonars[0] = direction->range;
}

void avoid_e_cb(const sensor_msgs::Range::ConstPtr& direction){
	sonars[1] = direction->range;
}
void avoid_s_cb(const sensor_msgs::Range::ConstPtr& direction){
	sonars[2] = direction->range;
}

void avoid_w_cb(const sensor_msgs::Range::ConstPtr& direction){
	//sensor_msgs::Range dist = *direction;
	sonars[3] = direction->range;
}


float deltaZ(const std_msgs::UInt16 n){
	return .0000000005*[](float x){return x * x * x;}((.2*(n.data-6400)));
}

//to be changed: function will accept a column of a 2D vector
void flyTo(float x, float y, float z){
	set_destination(x,y,z, 0);
	float tol = .2;
	ros::Time start = ros::Time::now();
	while(ros::ok() && !(check_waypoint_reached(tol)) && (ros::Time::now().toSec() - start.toSec() < 60)){
		if(msg.data == "stop"){
			break;
		}
		// if(!(avoid())){  //replace with the drone_mode function
		// 	set_destination(x,y,z,0);
		// 	ros::Duration(.5).sleep();
		// }
		ros::spinOnce();
		ros::Duration(0.3).sleep();
	}

    ROS_INFO("Done moving to (%f, %f, %f).", x, y, z);

}

//to be changed: function will accept a column of a 2D vector
void QRcode(float x, float y, float z){
	float tol = .2;
	float r = .1;
    float t = 0;

   ros::Time start = ros::Time::now();
   while(ros::ok() && qr.data == "null" && (ros::Time::now().toSec() - start.toSec() < 60)){
   	if(msg.data == "stop"){
		break;
	}
	if(z + deltaZ(cv_points) >= .5 && z + deltaZ(cv_points) <= 1){
		//idk if it matters but it might be faster to just change a global variable every time insteasd of rerunning avoid
		if(drone_mode == 1){
			ros::Duration(.2).sleep();
			set_destination(x + r*cos(t), y + r*sin(t), z + deltaZ(cv_points), 0);
			t+=.1;
		}
        }
        else{
        	if(drone_mode == 1){
			ros::Duration(.5).sleep();
			set_destination(x,y,z,0);
		}
        }
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Got QR Code.");
}
 


int main(int argc, char** argv)
{

	qr.data = "null";
	msg.data = "nothin";
	up = 0;
	drone_mode = 0; //0 = on the ground, 1 = in air, 2 = avoiding

	for(int i = 0; i < 4; i++){
		float a;
		a = 10.0;
		sonars.push_back(a);
	}
	bool drone1 = true;
    ros::init(argc, argv, "outtaControls");
    ros::NodeHandle nh;
    ros::Time runStart = ros::Time::now();

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    init_publisher_subscriber(nh);
    ros::Subscriber voiceRecognition = nh.subscribe<std_msgs::String>("Android", 10, voice_cb);
    ros::Subscriber QR = nh.subscribe<std_msgs::String>("CV", 10, qr_cb);
    ros::Subscriber points = nh.subscribe<std_msgs::UInt16>("Points", 5, point_cb);
    ros::Subscriber n_sonar = nh.subscribe<sensor_msgs::Range>("drone1/sensor/sonar/front", 12, avoid_n_cb);
    ros::Subscriber e_sonar = nh.subscribe<sensor_msgs::Range>("drone1/sensor/sonar/right", 12, avoid_e_cb);
	ros::Subscriber s_sonar = nh.subscribe<sensor_msgs::Range>("drone1/sensor/sonar/back", 12, avoid_s_cb);
	ros::Subscriber w_sonar = nh.subscribe<sensor_msgs::Range>("drone1/sensor/sonar/left", 12, avoid_w_cb);
    
    wait4connect();
    cout << "connected" << endl;
    wait4start();
    cout << "started" << endl;
    initialize_local_frame();
    cout << "local frame" << endl;

	takeoff(1);
	sleep(10);
	up = 1; //1 = in air
	drone_mode = 1; 

    ros::spinOnce();

    while(ros::ok()){
		
    	ros::spinOnce();
    	if(drone_mode != 0){
    		avoid();
    	}
    	if(msg.data == "takeoff"){
    		takeoff(1);
    		drone_mode = 1;
		ros::Duration(5.0).sleep();
    	}
    	else if(msg.data == "qr"){
    		float posQR1[3];
    		posQR1[1] = 26;
    		posQR1[2] = 2;
			if(drone1){
				posQR1[0] = 6.5;
			}
			else{
				posQR1[0] = 8.5;
			}
			flyTo(posQR1[0],posQR1[1],posQR1[2]);
			QRcode(posQR1[0],posQR1[1],posQR1[2]);

			//2nd set of qr codes
			float posQR2[3];
			posQR2[1] = 26;
			posQR2[2] = 2;
			if(drone1){
				posQR2[0] = 5;
			}
			else{
				posQR2[0] = 10;
			}
			flyTo(posQR2[0],posQR2[1],posQR2[2]);
			QRcode(posQR2[0],posQR2[1],posQR2[2]);
		}
		else if(msg.data == "heal"){
			flyTo(0,0,1);
		}
		else if(msg.data == "left"){
			flyTo(current_pose_g.pose.pose.position.x-.5,current_pose_g.pose.pose.position.y,current_pose_g.pose.pose.position.z);
		}
		else if(msg.data == "right"){
			flyTo(current_pose_g.pose.pose.position.x+.5,current_pose_g.pose.pose.position.y,current_pose_g.pose.pose.position.z);
		}
		else if(msg.data == "forward"){
			flyTo(current_pose_g.pose.pose.position.x,current_pose_g.pose.pose.position.y+.5,current_pose_g.pose.pose.position.z);
		}
		else if(msg.data == "backward"){
			flyTo(current_pose_g.pose.pose.position.x,current_pose_g.pose.pose.position.y-.5,current_pose_g.pose.pose.position.z);
		}
		else if(msg.data == "up"){
			flyTo(current_pose_g.pose.pose.position.x,current_pose_g.pose.pose.position.y,current_pose_g.pose.pose.position.z+.5);
		}
		else if(msg.data == "down"){
			flyTo(current_pose_g.pose.pose.position.x,current_pose_g.pose.pose.position.y,current_pose_g.pose.pose.position.z-.5);
		}
		else if(msg.data == "land"){//} || (ros::Time::now().toSec() - runStart.toSec() < 480)){
			flyTo(0,0,1);
			land();
    	}
    	msg.data = "nuthin";
    	ros::Duration(.3).sleep();
    }

    return 0;
}

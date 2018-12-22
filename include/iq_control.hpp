#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <geometry_msgs/Pose2D.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/LaserScan.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <vector>
#include <ros/duration.h>
#include <fstream>
#include <sstream>

using namespace std;

//Set global variables
mavros_msgs::State current_state;
nav_msgs::Odometry current_pose;
geometry_msgs::PoseStamped waypoint;
float GYM_OFFSET;
float current_heading;
geometry_msgs::Pose correctionVector;
float CORRECTIONHEADING = 0;



struct localWaypoint{
	float x;
	float y;
	float z;
	float psi;
};
std::vector<localWaypoint> waypointList;
std::vector<localWaypoint> correctionList;

void enu_2_gym(nav_msgs::Odometry current_pose_enu)
{
  float x = current_pose_enu.pose.pose.position.x;
  float y = current_pose_enu.pose.pose.position.y;
  float z = current_pose_enu.pose.pose.position.z;
  float deg2rad = (M_PI/180);
  float X = x*cos(GYM_OFFSET*deg2rad) - y*sin(GYM_OFFSET*deg2rad);
  float Y = x*sin(GYM_OFFSET*deg2rad) + y*cos(GYM_OFFSET*deg2rad);
  float Z = z;
  //ROS_INFO("Local position %f %f %f",X, Y, Z);
  // current_pose.pose.pose.position.x = X;
  // current_pose.pose.pose.position.y = Y;
  // current_pose.pose.pose.position.z = Z;


}

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

//get current position of drone
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose = *msg;
  enu_2_gym(current_pose);
  float q0 = current_pose.pose.pose.orientation.w;
  float q1 = current_pose.pose.pose.orientation.x;
  float q2 = current_pose.pose.pose.orientation.y;
  float q3 = current_pose.pose.pose.orientation.z;
  float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  //ROS_INFO("Current Heading %f ENU", psi*(180/M_PI));
  //Heading is in ENU
  current_heading = psi*(180/M_PI) - GYM_OFFSET;
  //ROS_INFO("Current Heading %f origin", current_heading);
  //ROS_INFO("x: %f y: %f z: %f", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
}
//set orientation of the drone (drone should always be level) 
// Heading input should match the NED coordinate system
void setHeading(float heading)
{
  heading = heading + CORRECTIONHEADING + GYM_OFFSET;
  float yaw = heading*(M_PI/180);
  float pitch = 0;
  float roll = 0;

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  float qw = cy * cr * cp + sy * sr * sp;
  float qx = cy * sr * cp - sy * cr * sp;
  float qy = cy * cr * sp + sy * sr * cp;
  float qz = sy * cr * cp - cy * sr * sp;

  waypoint.pose.orientation.w = qw;
  waypoint.pose.orientation.x = qx;
  waypoint.pose.orientation.y = qy;
  waypoint.pose.orientation.z = qz;
}
// set position to fly to in the gym frame
void setDestination(float x, float y, float z)
{

	//transform map to local
	float deg2rad = (M_PI/180);
	float Xlocal = x*cos((CORRECTIONHEADING + GYM_OFFSET - 90)*deg2rad) - y*sin((CORRECTIONHEADING + GYM_OFFSET - 90)*deg2rad);
	float Ylocal = x*sin((CORRECTIONHEADING + GYM_OFFSET - 90)*deg2rad) + y*cos((CORRECTIONHEADING + GYM_OFFSET - 90)*deg2rad);
	float Zlocal = z;

	x = Xlocal + correctionVector.position.x;
	y = Ylocal + correctionVector.position.y;
	z = Zlocal + correctionVector.position.z;
	ROS_INFO("Destination set to x: %f y: %f z: %f origin frame", x, y, z);
	
	// float X = x*cos(-GYM_OFFSET*deg2rad) - y*sin(-(GYM_OFFSET)*deg2rad);
	// float Y = x*sin(-GYM_OFFSET*deg2rad) + y*cos(-(GYM_OFFSET)*deg2rad);
	// float Z = z;
	//ROS_INFO("Destination set to x: %f y: %f z: %f ENU frame", X, Y, Z);
	waypoint.pose.position.x = x;
	waypoint.pose.position.y = y;
	waypoint.pose.position.z = z;
	
}
void loadWaypoints(string filename)
{

	ifstream in(filename.c_str());
	float x;
    float y;
    float z;
    float psi;
	
	string line;
	ROS_INFO("Loading waypoints");
	localWaypoint nextWayPoint;
	while( in >> x >> y >> z >> psi)
	{
		nextWayPoint.x = x;
		nextWayPoint.y = y;
		nextWayPoint.z = z;
		nextWayPoint.psi = psi;

		waypointList.push_back(nextWayPoint);

		cout << x << " " << y << " " << z << " " << psi << endl;
	}
}

void local_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	geometry_msgs::PoseWithCovarianceStamped localPose;
	localPose = *msg;
	localWaypoint potentialCorrectionVector;
	potentialCorrectionVector.x = current_pose.pose.pose.position.x - localPose.pose.pose.position.x;
	potentialCorrectionVector.y = current_pose.pose.pose.position.y - localPose.pose.pose.position.y;
	potentialCorrectionVector.z = current_pose.pose.pose.position.z - localPose.pose.pose.position.z;
	
	float q0 = localPose.pose.pose.orientation.w;
	float q1 = localPose.pose.pose.orientation.x;
	float q2 = localPose.pose.pose.orientation.y;
	float q3 = localPose.pose.pose.orientation.z;
	float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );

  	float map_heading = current_heading - psi*(180/M_PI);
  	potentialCorrectionVector.psi = map_heading;
	correctionList.push_back(potentialCorrectionVector);
	ROS_INFO("Current heading %f", current_heading);
	ROS_INFO("Current map heading %f", psi*(180/M_PI));
	ROS_INFO("potential correction heading %f", map_heading);
	//ROS_INFO("Correction Vector x %f y %f z %f " , potentialCorrectionVector.pose.position.x, potentialCorrectionVector.pose.position.y, potentialCorrectionVector.pose.position.z);
	//ROS_INFO("LOCAL POSITION RECIEVED %f %f %f", localPose.pose.position.x, localPose.pose.position.y, localPose.pose.position.z);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "offb_node");
	ros::NodeHandle controlnode;

	ros::Rate rate(5.0);

	ros::Publisher local_pos_pub = controlnode.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::Subscriber currentPos = controlnode.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, pose_cb);
	ros::Subscriber state_sub = controlnode.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber localization = controlnode.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/rtabmap/localization_pose", 1, local_cb);
	ros::ServiceClient arming_client = controlnode.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient land_client = controlnode.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
	ros::ServiceClient set_mode_client = controlnode.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	ros::ServiceClient takeoff_client = controlnode.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

	correctionVector.position.x = 0; 
	correctionVector.position.y = 0;
	correctionVector.position.z = 0;  


	string filename;
	if (!controlnode.hasParam("/training/filename"))
	{
		ROS_INFO("No param named 'filename'");
	}else{
		string path;
		string file;
		ros::param::get("/training/filename", file);
		ros::param::get("/training/path", path);
		filename = path + file;
		cout << filename << endl;
		ROS_INFO("File name loaded %s ", filename.c_str());
	}
	loadWaypoints(filename);
	string inspectionFile;
	if (!controlnode.hasParam("/training/inspection"))
	{
		ROS_INFO("No param named 'inspection'");
	}else{
		string path;
		string file;
		ros::param::get("/training/inspection", file);
		ros::param::get("/training/path", path);
		inspectionFile = path + file;
		cout << filename << endl;
		ROS_INFO("File name loaded %s ", filename.c_str());
	}
	

  	// wait for FCU connection
	while (ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("Connected to FCU");


	ROS_INFO("INITIALIZING ROS");
	//set the orientation of the local reference frame
	GYM_OFFSET = 0;
	for (int i = 1; i <= 30; ++i) {
		ros::spinOnce();
		ros::Duration(0.1).sleep();

		float q0 = current_pose.pose.pose.orientation.w;
		float q1 = current_pose.pose.pose.orientation.x;
		float q2 = current_pose.pose.pose.orientation.y;
		float q3 = current_pose.pose.pose.orientation.z;
		float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) ); // yaw

		GYM_OFFSET += psi*(180/M_PI);
		// ROS_INFO("current heading%d: %f", i, GYM_OFFSET/i);
	}
	GYM_OFFSET /= 30;
	ROS_INFO("the X' axis is facing: %f", GYM_OFFSET);
	cout << GYM_OFFSET << "\n" << endl;
	

	setDestination(0,0,1.5);
	setHeading(0);
	for(int i=0; i<100; i++)
	{
		local_pos_pub.publish(waypoint);
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	

	// arming
	mavros_msgs::CommandBool arm_request;
	arm_request.request.value = true;
	while (!current_state.armed && !arm_request.response.success)
	{
		ros::Duration(.1).sleep();
		arming_client.call(arm_request);
		local_pos_pub.publish(waypoint);
	}
	ROS_INFO("ARM sent %d", arm_request.response.success);

	

	// //NEED WAY TO START PROGRAM FROM GCS
	// // wait for mode to be set to OFFBOARD
 //    ros::Time last_request_offboard = ros::Time::now();
 //    mavros_msgs::SetMode ModeMsg;
	// ModeMsg.request.base_mode = 0;
 //    ModeMsg.request.custom_mode = "OFFBOARD";
	// while (ros::ok() && current_state.mode != "OFFBOARD")
	// {
	// 	ros::spinOnce();
	// 	rate.sleep();
	// 	local_pos_pub.publish(waypoint);

	// 	if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request_offboard > ros::Duration(1.0)))
	// 	{
 //            if( set_mode_client.call(ModeMsg) && ModeMsg.response.mode_sent)
 //            {
 //                ROS_INFO("Enabling offboard...");
 //            }
 //            last_request_offboard = ros::Time::now();
 //        }
	// }
	// ROS_INFO("Mode set to OFFBOARD");

	
	cout << "First waypoint  " << waypointList[0].x << " " << waypointList[0].y << " " << waypointList[0].z << " " << waypointList[0].psi << endl; 

	int wayPointNum = 0;
	float deltaX;
	float deltaY;
	float deltaZ;
	float dMag;
	float tollorance = .3;
	int inspectionStart = 0;
	
	while(ros::ok())
	{
		deltaX = abs(waypoint.pose.position.x - current_pose.pose.pose.position.x);
        deltaY = abs(waypoint.pose.position.y - current_pose.pose.pose.position.y);
        deltaZ = abs(waypoint.pose.position.z - current_pose.pose.pose.position.z);
        //cout << " dx " << deltaX << " dy " << deltaY << " dz " << deltaZ << endl;
        dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
        //cout << dMag << endl;

        if( dMag < tollorance)
		{
			if(wayPointNum < waypointList.size())
			{
				setDestination(waypointList[wayPointNum].x,waypointList[wayPointNum].y,waypointList[wayPointNum].z);
				setHeading(waypointList[wayPointNum].psi);
				wayPointNum++;
			}else{
				setDestination(0,0,1.5);
				setHeading(0);
			}

		}

		if(correctionList.size() == 7 )
		{
			ROS_INFO("Starting Inspection");
			while(!waypointList.empty())
			{
				waypointList.pop_back();
			}
			//make sure drone doesn't fly into plane while flying to first waypoint of inspection process
			localWaypoint nextWayPoint;

			nextWayPoint.x = waypointList[wayPointNum-1].x;
			nextWayPoint.y = waypointList[wayPointNum-1].y;
			nextWayPoint.z = 10;
			nextWayPoint.psi = current_heading;
			waypointList.push_back(nextWayPoint);

			nextWayPoint.x = 0;
			nextWayPoint.y = 0;
			nextWayPoint.z = 10;
			nextWayPoint.psi = current_heading;
			waypointList.push_back(nextWayPoint);
			loadWaypoints(inspectionFile);
			wayPointNum = 0;
			inspectionStart = 1;



			float sumx = 0;
			float sumy = 0;
			float sumz = 0;
			float q0, q1, q2, q3, psi;
			float sumPsiCos = 0;
			float sumPsiSin = 0;
			for(int i=0; i < correctionList.size(); i++ )
			{
				sumx = correctionList[i].x + sumx;
				sumy = correctionList[i].y + sumy;
				sumz = correctionList[i].z + sumz;

				//averaging circular numbers is fun! https://en.wikipedia.org/wiki/Mean_of_circular_quantities
				sumPsiCos = cos(correctionList[i].psi*(M_PI/180)) + sumPsiCos;
				sumPsiSin = sin(correctionList[i].psi*(M_PI/180)) + sumPsiSin;
				
			} 
			CORRECTIONHEADING = atan2(sumPsiSin,sumPsiCos)*(180/M_PI);
			ROS_INFO("Correction is %f %f %f ", sumx/correctionList.size(), sumy/correctionList.size(), sumz/correctionList.size() );
			ROS_INFO("Correction Heading is %f", CORRECTIONHEADING );
			correctionVector.position.x = sumx/correctionList.size(); 
			correctionVector.position.y = sumy/correctionList.size();
			correctionVector.position.z = sumz/correctionList.size(); 
			
		}
		ros::spinOnce();
		local_pos_pub.publish(waypoint);
		rate.sleep();
		//ROS_INFO("Correction is %f %f %f ", correctionVector.position.x, correctionVector.position.y, correctionVector.position.z );
	}
	return 0;
}


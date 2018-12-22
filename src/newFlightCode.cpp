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
#include <cmath>
#include <math.h>
#include <vector>
#include <ros/duration.h>
#include <fstream>
#include <sstream>
#include <control_functions.hpp>

using namespace std;

struct localWaypoint{
	float x;
	float y;
	float z;
	float psi;
};
std::vector<localWaypoint> waypointList;
std::vector<localWaypoint> correctionList;


//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
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

  	float map_heading = current_heading_g - psi*(180/M_PI);
  	potentialCorrectionVector.psi = map_heading;
	correctionList.push_back(potentialCorrectionVector);
	ROS_INFO("Current heading %f", current_heading_g);
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



  	// wait for FCU connection
	wait4connect();

	wait4start();

	initialize_local_frame();


	takeoff(arming_client, takeoff_client, local_pos_pub);


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
			nextWayPoint.psi = current_heading_g;
			waypointList.push_back(nextWayPoint);

			nextWayPoint.x = 0;
			nextWayPoint.y = 0;
			nextWayPoint.z = 10;
			nextWayPoint.psi = current_heading_g;
			waypointList.push_back(nextWayPoint);
			//loadWaypoints(inspectionFile);
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
			correction_heading_g = atan2(sumPsiSin,sumPsiCos)*(180/M_PI);
			ROS_INFO("Correction is %f %f %f ", sumx/correctionList.size(), sumy/correctionList.size(), sumz/correctionList.size() );
			ROS_INFO("Correction Heading is %f", correction_heading_g );
			correction_vector_g.position.x = sumx/correctionList.size(); 
			correction_vector_g.position.y = sumy/correctionList.size();
			correction_vector_g.position.z = sumz/correctionList.size(); 
			
		}
		ros::spinOnce();
		local_pos_pub.publish(waypoint);
		rate.sleep();
		//ROS_INFO("Correction is %f %f %f ", correction_vector_g.position.x, correction_vector_g.position.y, correction_vector_g.position.z );
	}
	return 0;
}


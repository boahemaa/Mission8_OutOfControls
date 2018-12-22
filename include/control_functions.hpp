#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <math.h>

mavros_msgs::State current_state;
nav_msgs::Odometry current_pose;
geometry_msgs::Pose correction_vector_g;
geometry_msgs::PoseStamped waypoint;

float current_heading_g;
float local_offset_g;
float correction_heading_g = 0;

// correction_vector_g.position.x = 0; 
// correction_vector_g.position.y = 0;
// correction_vector_g.position.z = 0;  

void enu_2_local(nav_msgs::Odometry current_pose_enu)
{
  float x = current_pose_enu.pose.pose.position.x;
  float y = current_pose_enu.pose.pose.position.y;
  float z = current_pose_enu.pose.pose.position.z;
  float deg2rad = (M_PI/180);
  float X = x*cos(local_offset_g*deg2rad) - y*sin(local_offset_g*deg2rad);
  float Y = x*sin(local_offset_g*deg2rad) + y*cos(local_offset_g*deg2rad);
  float Z = z;
  //ROS_INFO("Local position %f %f %f",X, Y, Z);
}
//get current position of drone
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose = *msg;
  enu_2_local(current_pose);
  float q0 = current_pose.pose.pose.orientation.w;
  float q1 = current_pose.pose.pose.orientation.x;
  float q2 = current_pose.pose.pose.orientation.y;
  float q3 = current_pose.pose.pose.orientation.z;
  float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  //ROS_INFO("Current Heading %f ENU", psi*(180/M_PI));
  //Heading is in ENU
  current_heading_g = psi*(180/M_PI) - local_offset_g;
  //ROS_INFO("Current Heading %f origin", current_heading_g);
  //ROS_INFO("x: %f y: %f z: %f", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
}
// set position to fly to in the gym frame
void setDestination(float x, float y, float z)
{

	//transform map to local
	float deg2rad = (M_PI/180);
	float Xlocal = x*cos((correction_heading_g + local_offset_g - 90)*deg2rad) - y*sin((correction_heading_g + local_offset_g - 90)*deg2rad);
	float Ylocal = x*sin((correction_heading_g + local_offset_g - 90)*deg2rad) + y*cos((correction_heading_g + local_offset_g - 90)*deg2rad);
	float Zlocal = z;

	x = Xlocal + correction_vector_g.position.x;
	y = Ylocal + correction_vector_g.position.y;
	z = Zlocal + correction_vector_g.position.z;
	ROS_INFO("Destination set to x: %f y: %f z: %f origin frame", x, y, z);
	
	// float X = x*cos(-local_offset_g*deg2rad) - y*sin(-(local_offset_g)*deg2rad);
	// float Y = x*sin(-local_offset_g*deg2rad) + y*cos(-(local_offset_g)*deg2rad);
	// float Z = z;
	//ROS_INFO("Destination set to x: %f y: %f z: %f ENU frame", X, Y, Z);
	waypoint.pose.position.x = x;
	waypoint.pose.position.y = y;
	waypoint.pose.position.z = z;
	
}
//set orientation of the drone (drone should always be level) 
// Heading input should match the ENU coordinate system
void setHeading(float heading)
{
  heading = heading + correction_heading_g + local_offset_g;
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
int wait4connect()
{
	ROS_INFO("Waiting for FCU connection");
	// wait for FCU connection
	while (ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	if(current_state.connected)
	{
		ROS_INFO("Connected to FCU");	
		return 0;
	}else{
		ROS_INFO("Error connecting to drone");
		return -1;	
	}
	
	
}
int wait4start()
{
	ROS_INFO("Waiting for user to set mode to GUIDED");
	while(ros::ok() && current_state.mode != "GUIDED")
	{
	    ros::spinOnce();
	    ros::Duration(0.01).sleep();
  	}
  	if(current_state.mode == "GUIDED")
	{
		ROS_INFO("Mode set to GUIDED. Mission starting");
		return 0;
	}else{
		ROS_INFO("Error starting mission!!");
		return -1;	
	}
}
int initialize_local_frame()
{
	//set the orientation of the local reference frame
	ROS_INFO("Initializing local coordinate system");
	local_offset_g = 0;
	for (int i = 1; i <= 30; ++i) {
		ros::spinOnce();
		ros::Duration(0.1).sleep();

		float q0 = current_pose.pose.pose.orientation.w;
		float q1 = current_pose.pose.pose.orientation.x;
		float q2 = current_pose.pose.pose.orientation.y;
		float q3 = current_pose.pose.pose.orientation.z;
		float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) ); // yaw

		local_offset_g += psi*(180/M_PI);
		// ROS_INFO("current heading%d: %f", i, local_offset_g/i);
	}
	local_offset_g /= 30;
	ROS_INFO("Coordinate offset set");
	ROS_INFO("the X' axis is facing: %f", local_offset_g);
	return 0;
}
int takeoff(ros::ServiceClient arming_client, ros::ServiceClient takeoff_client, ros::Publisher local_pos_pub)
{
	//intitialize first waypoint of mission
	setDestination(0,0,1.5);
	setHeading(0);
	for(int i=0; i<100; i++)
	{
		local_pos_pub.publish(waypoint);
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	// arming
	ROS_INFO("Arming drone");
	mavros_msgs::CommandBool arm_request;
	arm_request.request.value = true;
	while (!current_state.armed && !arm_request.response.success)
	{
		ros::Duration(.1).sleep();
		arming_client.call(arm_request);
		local_pos_pub.publish(waypoint);
	}
	if(arm_request.response.success)
	{
		ROS_INFO("Arming Successful");	
	}else{
		ROS_INFO("Arming failed with %d", arm_request.response.success);
		return -1;	
	}

	//request takeoff
	
	mavros_msgs::CommandTOL srv_takeoff;
	srv_takeoff.request.altitude = 1.5;
	if(takeoff_client.call(srv_takeoff)){
		ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
	}else{
		ROS_ERROR("Failed Takeoff");
		return -2;
	}
	sleep(5);
	return 0; 
}

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <gazebo_msgs/ModelStates.h> 
#include <geometry_msgs/Pose.h>
//#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

geometry_msgs::PoseStamped currentDroneState; 
geometry_msgs::PoseStamped orb_estimate;

// use ideal odometry from gazebo
void model_cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  gazebo_msgs::ModelStates current_states = *msg;
  int irisArrPos = 999;

  //search for the drone
  for (int i=0; i< current_states.name.size(); i++)
  {
  	if(current_states.name[i] == "iris")
  	{
  		irisArrPos = i;
  		break;
  	}
  	
  }
  if (irisArrPos == 999)
  {
  	std::cout << "iris is not in world" << std::endl;
  }else{
  	//assign drone pose to pose stamped
  	currentDroneState.pose = current_states.pose[irisArrPos];
  	currentDroneState.header.stamp = ros::Time::now();
    std::cout <<  currentDroneState << std::endl;
  }
  

  //std::cout << current_states << std::endl;
}
nav_msgs::Odometry current_poseEKF;
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_poseEKF = *msg;
  // ROS_INFO("x: %f y: %f z: %f", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
}
void orb_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  orb_estimate = *msg;
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "position_sender");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::TransformStamped>("/mavros/fake_gps/mocap/tf", 10);    
	ros::Publisher pubStamped = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);    
	ros::Subscriber currentHeading = n.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, model_cb);
  ros::Subscriber currentPos = n.subscribe<nav_msgs::Odometry>("mavros/global_position/local", 10, pose_cb);
  ros::Subscriber orb_sub = n.subscribe<geometry_msgs::PoseStamped>("/Stereo/CamPoseENUFrame", 10, orb_cb);
	ros::Rate rate(20.0);

  //tf::TransformBroadcaster brVision;
  // tf::TransformBroadcaster brEKF;
  // tf::Transform Tmap2droneVison;
  // tf::Transform Tmap2droneEKF;
  

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  geometry_msgs::PoseStamped currentDroneState_fcu;

	while(ros::ok())
	{
    //vison pose
    // Tmap2droneVison.setOrigin( tf::Vector3(currentDroneState.pose.position.x, currentDroneState.pose.position.y, currentDroneState.pose.position.z) );
    // tf::Quaternion q;
    // q.setRPY(0, 0, 90);
    // Tmap2droneVison.setRotation(tf::Quaternion(0,0,0,1));
    //Tmap2droneVison.setRotation(tf::Quaternion(currentDroneState.pose.orientation.x, currentDroneState.pose.orientation.y, currentDroneState.pose.orientation.z, currentDroneState.pose.orientation.w));
    //brVision.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.707, 0, 0, 0.707), tf::Vector3(currentDroneState.pose.position.x, currentDroneState.pose.position.y, currentDroneState.pose.position.z) ,ros::Time::now(),"map", "droneVsion")));
    //brVision.sendTransform(tf::StampedTransform(Tmap2droneVison, ros::Time::now(), "map", "droneVsion"));
		
    tf2::Quaternion q_fcu, q_local_heading_offset , q_new;
    q_local_heading_offset.setRPY( 0, 0, 1.57 );  //

    tf2::convert(currentDroneState.pose.orientation , q_fcu);

    q_new = q_fcu*q_local_heading_offset;  // Calculate the new orientation
    q_new.normalize();


    float deg2rad = (M_PI/180);
    float X_fcu = currentDroneState.pose.position.x*cos((90)*deg2rad) - currentDroneState.pose.position.y*sin((90)*deg2rad);
    float Y_fcu = currentDroneState.pose.position.x*sin((90)*deg2rad) + currentDroneState.pose.position.y*cos((90)*deg2rad);
    float Z_fcu = currentDroneState.pose.position.z;


    //brVision.sendTransform(tf::StampedTransform(tf::Transform(q_new, tf::Vector3(X_fcu, Y_fcu, Z_fcu)),ros::Time::now(),"map", "droneVsion"));

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "droneVsion";
    transformStamped.transform.translation.x = X_fcu;
    transformStamped.transform.translation.y = Y_fcu;
    transformStamped.transform.translation.z = Z_fcu;
    
    transformStamped.transform.rotation.x = q_new.x();
    transformStamped.transform.rotation.y = q_new.y();
    transformStamped.transform.rotation.z = q_new.z();
    transformStamped.transform.rotation.w = q_new.w();

    br.sendTransform(transformStamped);


    currentDroneState_fcu.header.stamp = ros::Time::now();
    currentDroneState_fcu.header.frame_id = "droneVsion";
    currentDroneState_fcu.pose.position.x = X_fcu;
    currentDroneState_fcu.pose.position.y = Y_fcu;
    currentDroneState_fcu.pose.position.z = Z_fcu;
    currentDroneState_fcu.pose.orientation.x = q_new.x();
    currentDroneState_fcu.pose.orientation.y = q_new.y();
    currentDroneState_fcu.pose.orientation.z = q_new.z();
    currentDroneState_fcu.pose.orientation.w = q_new.w();

    //EKF pose
    // Tmap2droneVison.setOrigin( tf::Vector3(current_poseEKF.pose.pose.position.x, current_poseEKF.pose.pose.position.y, current_poseEKF.pose.pose.position.z) );
    // tf::Quaternion qEKF;
    // qEKF.setRPY(0, 0, 0);
    // Tmap2droneVison.setRotation(tf::Quaternion(current_poseEKF.pose.pose.orientation.x, current_poseEKF.pose.pose.orientation.y, current_poseEKF.pose.pose.orientation.z, current_poseEKF.pose.pose.orientation.w));
    // brEKF.sendTransform(tf::StampedTransform(Tmap2droneEKF, ros::Time::now(), "map", "droneEKF"));

    pubStamped.publish(currentDroneState_fcu);
		ros::spinOnce();
		rate.sleep();
	}	
}
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <gazebo_msgs/ModelStates.h> 
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::PoseStamped currentDroneState; 

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
  }
  

  //std::cout << current_states << std::endl;
}
nav_msgs::Odometry current_poseEKF;
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_poseEKF = *msg;
  // ROS_INFO("x: %f y: %f z: %f", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "position_sender");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::TransformStamped>("/mavros/fake_gps/mocap/tf", 10);    
	ros::Publisher pubStamped = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);    
	ros::Subscriber currentHeading = n.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, model_cb);
  ros::Subscriber currentPos = n.subscribe<nav_msgs::Odometry>("mavros/global_position/local", 10, pose_cb);
	ros::Rate rate(20.0);

  tf::TransformBroadcaster brVision;
  tf::TransformBroadcaster brEKF;
  tf::Transform Tmap2droneVison;
  tf::Transform Tmap2droneEKF;
  
  

	while(ros::ok())
	{
    //vison pose
    Tmap2droneVison.setOrigin( tf::Vector3(currentDroneState.pose.position.x, currentDroneState.pose.position.y, currentDroneState.pose.position.z) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    Tmap2droneVison.setRotation(tf::Quaternion(currentDroneState.pose.orientation.x, currentDroneState.pose.orientation.y, currentDroneState.pose.orientation.z, currentDroneState.pose.orientation.w));
    brVision.sendTransform(tf::StampedTransform(Tmap2droneVison, ros::Time::now(), "map", "droneVsion"));
		
    //EKF pose
    Tmap2droneVison.setOrigin( tf::Vector3(current_poseEKF.pose.pose.position.x, current_poseEKF.pose.pose.position.y, current_poseEKF.pose.pose.position.z) );
    tf::Quaternion qEKF;
    qEKF.setRPY(0, 0, 0);
    Tmap2droneVison.setRotation(tf::Quaternion(current_poseEKF.pose.pose.orientation.x, current_poseEKF.pose.pose.orientation.y, current_poseEKF.pose.pose.orientation.z, current_poseEKF.pose.pose.orientation.w));
    brEKF.sendTransform(tf::StampedTransform(Tmap2droneEKF, ros::Time::now(), "map", "droneEKF"));

    //pubStamped.publish(currentDroneState);
		ros::spinOnce();
		rate.sleep();
	}	
}
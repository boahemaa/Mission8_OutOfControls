#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>

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
  

  std::cout << current_states << std::endl;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "position_sender");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::TransformStamped>("/mavros/fake_gps/mocap/tf", 10);    
	ros::Publisher pubStamped = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);    
	ros::Subscriber currentHeading = n.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, model_cb);
	ros::Rate rate(20.0);

	while(ros::ok())
	{
		pubStamped.publish(currentDroneState);
		ros::spinOnce();
		rate.sleep();
	}	
}
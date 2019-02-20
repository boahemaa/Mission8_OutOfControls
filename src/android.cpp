#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char** argv){

	ros::init(argc, argv, "offb_node");
	ros::NodeHandle nh;

	ros::Publisher sendMsg = nh.advertise<std_msgs::String>("Android topic", 10);

	std_msgs::String a;
	a.data = "takeoff";

	sendMsg.publish(a);
	ros::spinOnce();
	ros::Rate(10);
	while(ros::ok()){
		ros::Duration(.1).sleep();
	}
}
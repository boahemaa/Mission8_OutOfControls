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
//Going to create a node that outputs the sonars to match the same as the sim so testing stuff works
//


vector<float> getSonars(){
	vector<float> dist;
	//opening i2c channel
	const char *filename = "/dev/i2c-0"; //
	int file_i2c;
	if ((file_i2c = open(filename, O_RDWR)) < 0) { //rdwr = read write permissions
       	ROS_INFO("Failed to open the i2c bus"); //all i2c info just needs to opended once?
	}
	int addr = 0x70;
	for(int i = addr; i<=0x73; i++){
	    if (ioctl(file_i2c, I2C_SLAVE, i) < 0) { //opening communication (inout output control)
	    	ROS_INFO("Failed to acquire bus access and/or talk to slave.\n");
			cout << strerror(errno) << endl;
		}
		unsigned char buffer[2]; //
		buffer[0] = 0x51;
		//writing to sensor to make it take a reading
		if (write(file_i2c, buffer, 1) != 1) {
          		ROS_INFO("Failed to write to the i2c bus.\n");
		}
		buffer[0] = 0xe1;
		//reading in the data from the sensor
		int readRes = read(file_i2c, buffer, 2);
		while (readRes != 2) {
			buffer[0] = 0xe1;
			readRes = read(file_i2c, buffer, 2);
			ros::Duration(.05).sleep();
        }
		long val = buffer[1];
		val = (((val >> 8) & 0xff) | (val & 0xff));
		//if sonars are close to each other, add more sleep duration between readings
		dist.push_back(val/100.f); //converting ditance from cm to m and long to float
		ros::Duration(.3).sleep(); //optamize this
	}
	return dist;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "Get_Sonars");
    ros::NodeHandle nh;


   	vector<float> sonarsgot;
    ros::Publisher n_sonar_pub = nh.advertise<sensor_msgs::Range>("drone1/sensor/sonar/front", 12);
	ros::Publisher e_sonar_pub = nh.advertise<sensor_msgs::Range>("drone1/sensor/sonar/right", 12);
	ros::Publisher s_sonar_pub = nh.advertise<sensor_msgs::Range>("drone1/sensor/sonar/back", 12);
	ros::Publisher w_sonar_pub = nh.advertise<sensor_msgs::Range>("drone1/sensor/sonar/left", 12);
	sensor_msgs::Range n_sonar_msg;
	sensor_msgs::Range e_sonar_msg;
	sensor_msgs::Range s_sonar_msg;
	sensor_msgs::Range w_sonar_msg;

	ros::Rate rate(20.0);

	while (ros::ok()){
    	ros::spinOnce();
    	vector<float> sonarsgot = getSonars();

		n_sonar_msg.range = sonarsgot[0];
		e_sonar_msg.range = sonarsgot[1];
		s_sonar_msg.range = sonarsgot[2];
		w_sonar_msg.range = sonarsgot[3];

    	n_sonar_pub.publish(n_sonar_msg);
    	e_sonar_pub.publish(n_sonar_msg);
		s_sonar_pub.publish(n_sonar_msg);
		w_sonar_pub.publish(n_sonar_msg);
    	rate.sleep();
  	}
}
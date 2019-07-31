#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/String.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <geometry_msgs/Pose2D.h>
#include <mavros_msgs/CommandTOL.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <ros/duration.h>

using namespace std;

//Set global variables
mavros_msgs::State current_state;
nav_msgs::Odometry current_pose;
geometry_msgs::PoseStamped waypoint;

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}
void setDestination(float x, float y, float z)
{
  float X = x; 
  float Y = y;
  float Z = z;
  waypoint.pose.position.x = X;
  waypoint.pose.position.y = Y;
  waypoint.pose.position.z = Z;
  cout << "Destination set to x: " << X << " y: " << Y << " z: " << Z << endl;
}

//initialize ROS
void init_ros()
{
  ROS_INFO("INITIALIZING ROS");
  for(int i=0; i<100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle controlnode;

  ros::Rate rate(20.0);
  ros::Publisher local_pos_pub = controlnode.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Subscriber state_sub = controlnode.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::ServiceClient arming_client = controlnode.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = controlnode.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  ros::ServiceClient takeoff_client = controlnode.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

  // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Connected to FCU");
    // wait for mode to be set to guided
  while (current_state.mode != "GUIDED")
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Mode set to GUIDED");

  // arming
  mavros_msgs::CommandBool arm_request;
  arm_request.request.value = true;
  while (!current_state.armed && !arm_request.response.success)
  {
    ros::Duration(.1).sleep();
    arming_client.call(arm_request);
  }
  ROS_INFO("ARM sent %d", arm_request.response.success);

  //request takeoff
  mavros_msgs::CommandTOL takeoff_request;
  takeoff_request.request.altitude = 1.5;

  while (!takeoff_request.response.success)
  {
    ros::Duration(.1).sleep();
    takeoff_client.call(takeoff_request);
  }
  ROS_INFO("Takeoff initialized");
  sleep(10);

  setDestination(0,0,1.5);

  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();

    float dist;
    //opening i2c channel
    const char *filename = "/dev/i2c-0";
    int file_i2c;
    if ((file_i2c = open(filename, O_RDWR)) < 0) {
          printf("Failed to open the i2c bus");
    }
    if (ioctl(file_i2c, I2C_SLAVE, 0x70) < 0) {
      printf("Failed to acquire bus access and/or talk to slave.\n");
      cout << strerror(errno) << endl;
    }
    unsigned char buffer[2];
    buffer[0] = 0x51;
    //writing to sensor to make it take a reading
    if (write(file_i2c, buffer, 1) != 1) {
              printf("Failed to write to the i2c bus.\n");
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
    dist = (val/100.f); //converting ditance from cm to m and long to float

ROS_INFO("Sonar: %f", dist);

    if(dist < 1 && dist > .2)
    {
      ROS_INFO("Obstacle sighted @%f", dist);
      ROS_INFO("-------------------SKRTT SKRTT-------------------");

      setDestination(0, -2, 1.5);
}
    rate.sleep();

    local_pos_pub.publish(waypoint);
	
  }
  return 0;
}

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
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
#include <time.h>
#include <cmath>
#include <math.h>
#include <ros/duration.h>

using namespace std;

//Set global variables
mavros_msgs::State current_state;
nav_msgs::Odometry current_pose;
geometry_msgs::PoseStamped waypoint;

std_msgs::String MODE;
float GYM_OFFSET;
float current_heading;
bool currentlyAvoiding = false;

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}
//get current position of drone
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose = *msg;
  float q0 = current_pose.pose.pose.orientation.w;
  float q1 = current_pose.pose.pose.orientation.x;
  float q2 = current_pose.pose.pose.orientation.y;
  float q3 = current_pose.pose.pose.orientation.z;
  float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  current_heading = -psi*(180/M_PI) + 90;
  ROS_INFO("Current Heading %f ", current_heading);
  // ROS_INFO("x: %f y: %f z: %f", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
}
void mode_cb(const std_msgs::String::ConstPtr& msg)
{
  MODE = *msg;
}
//set orientation of the drone (drone should always be level)
void setHeading(float heading)
{
  heading = -heading + 90 - GYM_OFFSET;
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
void setHeading_cb(const std_msgs::Float64::ConstPtr& msg)
{
  std_msgs::Float64 set_heading = *msg;
  setHeading(set_heading.data);
  // ROS_INFO("current heading: %f", current_heading.data);
}
// set position to fly to in the gym frame
void setDestination(float x, float y, float z)
{
  float deg2rad = (M_PI/180);
  float X = x*cos(-GYM_OFFSET*deg2rad) - y*sin(-GYM_OFFSET*deg2rad);
  float Y = x*sin(-GYM_OFFSET*deg2rad) + y*cos(-GYM_OFFSET*deg2rad);
  float Z = z;
  waypoint.pose.position.x = X;
  waypoint.pose.position.y = Y;
  waypoint.pose.position.z = Z;
  // ROS_INFO("Destination set to x: %f y: %f z: %f", X, Y, Z);
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

  ros::Rate rate(5.0);
  ros::Publisher gym_offset_pub = controlnode.advertise<std_msgs::Float64>("gymOffset", 1);
  ros::Publisher local_pos_pub = controlnode.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Subscriber currentPos = controlnode.subscribe<nav_msgs::Odometry>("mavros/global_position/local", 10, pose_cb);
  ros::Subscriber heading_pub = controlnode.subscribe<std_msgs::Float64>("setHeading", 1, setHeading_cb);
  ros::Subscriber mode_sub = controlnode.subscribe<std_msgs::String>("mode", 10, mode_cb);
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

  //set the orientation of the gym
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
  GYM_OFFSET /= -30;
  GYM_OFFSET += 90;
  ROS_INFO("the N' axis is facing: %f", GYM_OFFSET);
  cout << GYM_OFFSET << "\n" << endl;
  std_msgs::Float64 gymOffset;
  gymOffset.data = GYM_OFFSET;
  gym_offset_pub.publish(gymOffset);

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
  setHeading(0);

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

    if(dist < 1.5 && dist > .4)
    {
      ROS_INFO("Obstacle sighted @%f", dist);
      ROS_INFO("-------------------SKRTT SKRTT-------------------");

      ROS_INFO("goto x: %f, y: %f", 0, -2);
      setDestination(0, -2, 1.5);
      currentlyAvoiding = true;

    }
    rate.sleep();

    local_pos_pub.publish(waypoint);
    gym_offset_pub.publish(gymOffset);
  }
  return 0;
}

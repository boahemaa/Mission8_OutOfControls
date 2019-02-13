#include "std_msgs/Float64.h"
#include <boost/algorithm/string.hpp>
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
#include <ros/ros.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <vector>

using namespace std;

// Set global variables
mavros_msgs::State current_state;
// nav_msgs::Odometry current_pose;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped pose;
std_msgs::Float64 current_heading;
float GYM_OFFSET;
vector<std_msgs::Uint32> qr;
bool qr_codes = false;

// get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
    bool connected = current_state.connected;
    bool armed = current_state.armed;
}
// get current position of drone
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
    ROS_INFO("x: %f y: %f z: %f", current_pose.pose.position.x,
        current_pose.pose.position.y, current_pose.pose.position.z);
    // ROS_INFO("y: %f", current_pose.pose.position.y);
    // ROS_INFO("z: %f", current_pose.pose.position.z);
}
// void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
// {
//  current_pose = *msg;
//  ROS_INFO("x: %f y: %f z: %f", current_pose.pose.pose.position.x,
//  current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
// }
// get compass heading
void heading_cb(const std_msgs::Float64::ConstPtr& msg)
{
    current_heading = *msg;
    // ROS_INFO("current heading: %f", current_heading.data);
}
// set orientation of the drone (drone should always be level)
void voice_cb(const std_msgs::String::ConstPtr& voice)
{
    std_msgs::string v = *voice;
    boost::algorithm::to_lower(v);
    if (v.data == "takeoff") {

    } else if (v.data == "qr code") {
        flyOver()
    } else if (v.data == "land") {

    } else if (v.data == "realign")
    {
    	realign();
    }
}
void qr_cb(const vector<std_msgs::Uint32>::ConstPtr& codes){
	qr = *codes;
	qr_codes = true;
}

void realign()
{
	setDestination(1+(float)rand()/RAND_MAX,1+(float)rand()/RAND_MAX,1);
}

void flyOver()
{
    setHeading(0);
    setDestination(1,1,1);
    float tollorance = .2;
    if (local_pos_pub) {

        for (int i = 10000; ros::ok() && i > 0; --i) {

            local_pos_pub.publish(pose);
            float deltaX = abs(pose.pose.position.x - current_pose.pose.position.x);
            float deltaY = abs(pose.pose.position.y - current_pose.pose.position.y);
            float deltaZ = abs(pose.pose.position.z - current_pose.pose.position.z);
            float dMag = sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2));
            cout << dMag << endl;
            if (dMag < tollorance) {
                break;
            }
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            if (i == 1) {
                ROS_INFO("Failed to reach destination. Stepping to next task.");
            }
        }
        ROS_INFO("Done moving foreward.");
    }
        //WAIT UNTIL WE RECOGNIZE QR CODE
    	//Realign stuff goes here
    while()

}
void setHeading(float heading)
{
    heading = -heading + 90 - GYM_OFFSET;
    float yaw = heading * (M_PI / 180);
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

    pose.pose.orientation.w = qw;
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;
}
// set position to fly to in the gym frame
void setDestination(float x, float y, float z)
{
    float deg2rad = (M_PI / 180);
    float X = x * cos(-GYM_OFFSET * deg2rad) - y * sin(-GYM_OFFSET * deg2rad);
    float Y = x * sin(-GYM_OFFSET * deg2rad) + y * cos(-GYM_OFFSET * deg2rad);
    float Z = z;
    pose.pose.position.x = X;
    pose.pose.position.y = Y;
    pose.pose.position.z = Z;
    ROS_INFO("Destination set to x: %f y: %f z %f", X, Y, Z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher set_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/global_position/pose", 10, pose_cb);
    ros::Subscriber currentHeading = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, heading_cb);
    ros::Subscriber voiceRecognition = nh.subscribe<std_msgs::String>("Android topic", 10, voice_cb);
    ros::Subscriber QR = nh.subscribe<vector<std_msgs::Uint32>>("CV Node", 10, )

    // allow the subscribers to initialize
    ROS_INFO("INITILIZING...");
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    while (current_state.mode != "GUIDED") {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    // set the orientation of the gym
    GYM_OFFSET = 0;
    for (int i = 1; i <= 30; ++i) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        GYM_OFFSET += current_heading.data;
        ROS_INFO("current heading%d: %f", i, GYM_OFFSET / i);
    }
    GYM_OFFSET /= 30;
    ROS_INFO("the N' axis is facing: %f", GYM_OFFSET);
    cout << GYM_OFFSET << "\n"
         << endl;

    // wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // arming
    ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mavros_msgs::CommandBool srv_arm_i;
    srv_arm_i.request.value = true;
    if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success)
        ROS_INFO("ARM sent %d", srv_arm_i.response.success);
    else {
        ROS_ERROR("Failed arming");
        return -1;
    }

    return 0;
}
#include <control_functions.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/Mavlink.h>
//include API 
#include <pid_controller.h>

using namespace pidcontroller;

PIDController::PIDController()
{};

void PIDController::setup_linvel_pid(double p_gain, double i_gain, double d_gain, double i_max, double i_min, const ros::NodeHandle &node){
	// PID values
	std::array<double, 3> linvel_pid = { {p_gain, i_gain, d_gain} };

	// Min/max bounds for the integral windup
	double linvel_imin = i_min;
	double linvel_imax = i_max;

#ifdef CONTROL_TOOLBOX_PRE_1_14
	pid_linvel_x.initPid(linvel_pid[0], linvel_pid[1], linvel_pid[2], linvel_imax, linvel_imin, node);
	pid_linvel_y.initPid(linvel_pid[0], linvel_pid[1], linvel_pid[2], linvel_imax, linvel_imin, node);
	pid_linvel_z.initPid(linvel_pid[0], linvel_pid[1], linvel_pid[2], linvel_imax, linvel_imin, node);
#else
	pid_linvel_x.initPid(linvel_pid[0], linvel_pid[1], linvel_pid[2], linvel_imax, linvel_imin, false, node);
	pid_linvel_y.initPid(linvel_pid[0], linvel_pid[1], linvel_pid[2], linvel_imax, linvel_imin, false, node);
	pid_linvel_z.initPid(linvel_pid[0], linvel_pid[1], linvel_pid[2], linvel_imax, linvel_imin, false, node);
#endif
}

void PIDController::setup_yawrate_pid(double p_gain, double i_gain, double d_gain, double i_max, double i_min, const ros::NodeHandle &node){
	// PID values
	std::array<double, 3> yawrate_pid = { {p_gain, i_gain, d_gain} };

	// Min/max bounds for the integral windup
	double yawrate_imin = i_min;
	double yawrate_imax = i_max;

#ifdef CONTROL_TOOLBOX_PRE_1_14
	pid_yaw_rate.initPid(yawrate_pid[0], yawrate_pid[1], yawrate_pid[2], yawrate_imax, yawrate_imin, node);
#else
	pid_yaw_rate.initPid(yawrate_pid[0], yawrate_pid[1], yawrate_pid[2], yawrate_imax, yawrate_imin, false, node);
#endif
}

Eigen::Vector3d PIDController::compute_linvel_effort(Eigen::Vector3d goal, Eigen::Vector3d current, ros::Time last_time){
	double lin_vel_x = pid_linvel_x.computeCommand(goal.x() - current.x(), ros::Time::now() - last_time);
	double lin_vel_y = pid_linvel_y.computeCommand(goal.y() - current.y(), ros::Time::now() - last_time);
	double lin_vel_z = pid_linvel_z.computeCommand(goal.z() - current.z(), ros::Time::now() - last_time);

	return Eigen::Vector3d(lin_vel_x, lin_vel_y, lin_vel_z);
}

double PIDController::compute_yawrate_effort(double goal, double current, ros::Time last_time){
	double yaw_rate = pid_yaw_rate.computeCommand(goal - current, ros::Time::now() - last_time);

	return yaw_rate;
}


int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "offb_node");
	ros::NodeHandle controlnode;
	aircraft tar_drone;
	//initialize control publisher/subscribers
	tar_drone.init_publisher_subscriber(controlnode);
	// ros::Publisher vel_pub = controlnode.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
	// ros::Publisher mavlink_pub = controlnode.advertise<mavros_msgs::Mavlink>("/mavlink/to", 10);

	// mavros_msgs::Mavlink do_change_speed;

	// PIDController pid;
	// pid.setup_linvel_pid(.5, .2, .01, 1, 0, controlnode);


	// //specify some waypoints 
	// std::vector<control_api_waypoint> waypointList;
	// control_api_waypoint nextWayPoint;
	// nextWayPoint.x = 0;
	// nextWayPoint.y = 0;
	// nextWayPoint.z = 3;
	// nextWayPoint.psi = 0;
	// waypointList.push_back(nextWayPoint);
	// nextWayPoint.x = 10;
	// nextWayPoint.y = 0;
	// nextWayPoint.z = 3;
	// nextWayPoint.psi = -90;
	// waypointList.push_back(nextWayPoint);
	// nextWayPoint.x = 10;
	// nextWayPoint.y = 10;
	// nextWayPoint.z = 3;
	// nextWayPoint.psi = 0;
	// waypointList.push_back(nextWayPoint);
	// nextWayPoint.x = 0;
	// nextWayPoint.y = 10;
	// nextWayPoint.z = 3;
	// nextWayPoint.psi = 90;
	// waypointList.push_back(nextWayPoint);
	// nextWayPoint.x = 0;
	// nextWayPoint.y = 0;
	// nextWayPoint.z = 3;
	// nextWayPoint.psi = 180;
	// waypointList.push_back(nextWayPoint);
	// nextWayPoint.x = 0;
	// nextWayPoint.y = 0;
	// nextWayPoint.z = 3;
	// nextWayPoint.psi = 0;
	// waypointList.push_back(nextWayPoint);

 //  	// wait for FCU connection
	// wait4connect();

	// //wait for used to switch to mode GUIDED
	// wait4start();

	// //create local reference frame 
	// initialize_local_frame();

	// //request takeoff
	// takeoff(3);

	// land();
	// //specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	// ros::Rate rate(2.0);
	// int counter = 0;
	// ros::Time last_time = ros::Time::now();
	// Eigen::Vector3d current;
	

	// Eigen::Vector3d goal; 
	// goal.x() = waypointList[counter].x;
	// goal.y() = waypointList[counter].y;
	// goal.z() = waypointList[counter].z;
	// while(ros::ok())
	// {


	// 	//Eigen::Vector3d PIDController::compute_linvel_effort(Eigen::Vector3d goal, Eigen::Vector3d current, last_time)

	// 	ros::spinOnce();
	// 	rate.sleep();
	// 	mavlink_pub.publish(do_change_speed);
	// 	if(check_waypoint_reached() == 1)
	// 	{
	// 		if (counter < waypointList.size())
	// 		{
	// 			set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
	// 			counter++;	
	// 		}else{
	// 			//land after all waypoints are reached
	// 			land();
	// 		}	
	// 	}	
		
	// }
	return 0;
}


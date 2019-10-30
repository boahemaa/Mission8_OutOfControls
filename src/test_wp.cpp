#include <ros/ros.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandHome.h>
#include <std_msgs/String.h>
#include <cstdlib>
#include <mavros_msgs/Waypoint.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "srv_waypoint");
  ros::NodeHandle p;
  ros::NodeHandle n;
  ros::NodeHandle l;

  ros::ServiceClient wp_clear_client = p.serviceClient<mavros_msgs::WaypointClear>("waypoint_clear_client");
  ros::ServiceClient wp_srv_client = n.serviceClient<mavros_msgs::WaypointPush>("waypoint_push_client");
  ros::ServiceClient set_home_client = l.serviceClient<mavros_msgs::CommandHome>("command_home_client");

  mavros_msgs::WaypointPush wp_push_srv;
  mavros_msgs::WaypointClear wp_clear_srv;
  mavros_msgs::CommandHome set_home_srv;


  mavros_msgs::Waypoint wp_msg;

  wp_msg.frame = 0; // mavros_msgs::Waypoint::FRAME_GLOBAL;
  wp_msg.command = 16;
  wp_msg.is_current = false;
  wp_msg.autocontinue = false;
  wp_msg.param1 = 0;
  wp_msg.param2 = 0;
  wp_msg.param3 = 0;
  wp_msg.param4 = 0;
  wp_msg.x_lat = 40.0;
  wp_msg.y_long = 30.0;
  wp_msg.z_alt = 5.0;

  wp_clear_srv.request = {};

  wp_push_srv.request.start_index = 0;
  wp_push_srv.request.waypoints.push_back(wp_msg);

  set_home_srv.request.current_gps = false;
  set_home_srv.request.latitude = 11;
  set_home_srv.request.longitude = 12;
  set_home_srv.request.altitude = 1;

  if (set_home_client.call(set_home_srv))
{
    ROS_INFO("Home was set to new value ");
}
else
{
    ROS_ERROR("Home position couldn't been changed");
}


if (wp_clear_client.call(wp_clear_srv))
{
    ROS_INFO("Waypoint list was cleared");
}
else
{
    ROS_ERROR("Waypoint list couldn't been cleared");
}

  if (wp_srv_client.call(wp_push_srv))
  {
    ROS_INFO("Success:%d", (bool)wp_push_srv.response.success);
  }
  else
  {
    ROS_ERROR("Waypoint couldn't been sent");
    ROS_INFO("Success:%d", (bool)wp_push_srv.response.success);
  }

   ros::spinOnce();

  return 0;
}

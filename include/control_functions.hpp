 #include <mavros_msgs/CommandTOL.h>
 #include <mavros_msgs/State.h>
 #include <nav_msgs/Odometry.h>
 #include <geometry_msgs/Pose.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <cmath>
 #include <math.h>
 #include <ros/ros.h>
 #include <std_msgs/Float64.h>
 #include <std_msgs/String.h>
 #include <mavros_msgs/CommandBool.h>
 #include <mavros_msgs/SetMode.h>
 #include <mavros_msgs/PositionTarget.h>
 #include <unistd.h>
 #include <vector>
 #include <ros/duration.h>
 #include <iostream>


 mavros_msgs::State current_state_g;
 nav_msgs::Odometry current_pose_g;
 geometry_msgs::Pose correction_vector_g;
 geometry_msgs::PoseStamped waypoint_g;

 float current_heading_g;
 float local_offset_g;
 float correction_heading_g = 0;



 ros::Publisher local_pos_pub;
 ros::Subscriber currentPos;
 ros::Subscriber state_sub;
 ros::ServiceClient arming_client;
 ros::ServiceClient land_client;
 ros::ServiceClient set_mode_client;
 ros::ServiceClient takeoff_client;

 struct control_api_waypoint{
     float x;
     float y;
     float z;
     float psi;
 };

 //get armed state
 void state_cb(const mavros_msgs::State::ConstPtr& msg)
 {
   current_state_g = *msg;
 }
 void enu_2_local(nav_msgs::Odometry current_pose_enu)
 {
   float x = current_pose_enu.pose.pose.position.x;
   float y = current_pose_enu.pose.pose.position.y;
   float z = current_pose_enu.pose.pose.position.z;
   float deg2rad = (M_PI/180);
   float X = x*cos(local_offset_g*deg2rad) - y*sin(local_offset_g*deg2rad);
   float Y = x*sin(local_offset_g*deg2rad) + y*cos(local_offset_g*deg2rad);
   float Z = z;
   //ROS_INFO("Local position %f %f %f",X, Y, Z);
 }
 //get current position of drone
 void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
 {
   current_pose_g = *msg;
   enu_2_local(current_pose_g);
   float q0 = current_pose_g.pose.pose.orientation.w;
   float q1 = current_pose_g.pose.pose.orientation.x;
   float q2 = current_pose_g.pose.pose.orientation.y;
   float q3 = current_pose_g.pose.pose.orientation.z;
   float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
   //ROS_INFO("Current Heading %f ENU", psi*(180/M_PI));
   //Heading is in ENU
   current_heading_g = psi*(180/M_PI) - local_offset_g;
   //ROS_INFO("Current Heading %f origin", current_heading_g);
   //ROS_INFO("x: %f y: %f z: %f", current_pose_g.pose.pose.position.x, current_pose_g.pose.pose.position.y, current_pose_g.pose.pose.position.z);
 }
 //set orientation of the drone (drone should always be level)
 // Heading input should match the ENU coordinate system
 void set_heading(float heading)
 {
   heading = heading + correction_heading_g + local_offset_g;
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

   waypoint_g.pose.orientation.w = qw;
   waypoint_g.pose.orientation.x = qx;
   waypoint_g.pose.orientation.y = qy;
   waypoint_g.pose.orientation.z = qz;
 }
 // set position to fly to in the local frame
 void set_destination(float x, float y, float z, float psi)
 {
     set_heading(psi);
     //transform map to local
     float deg2rad = (M_PI/180);
     float Xlocal = x*cos((correction_heading_g + local_offset_g - 90)*deg2rad) - y*sin((correction_heading_g + local_offset_g - 90)*deg2rad);
     float Ylocal = x*sin((correction_heading_g + local_offset_g - 90)*deg2rad) + y*cos((correction_heading_g + local_offset_g - 90)*deg2rad);
     float Zlocal = z;

     x = Xlocal + correction_vector_g.position.x;
     y = Ylocal + correction_vector_g.position.y;
     z = Zlocal + correction_vector_g.position.z;
     ROS_INFO("Destination set to x: %f y: %f z: %f origin frame", x, y, z);

     waypoint_g.pose.position.x = x;
     waypoint_g.pose.position.y = y;
     waypoint_g.pose.position.z = z;

 }
 int wait4connect()
 {
     ROS_INFO("Waiting for FCU connection");
     // wait for FCU connection
     while (ros::ok() && !current_state_g.connected)
     {
         ros::spinOnce();
         ros::Duration(0.01).sleep();
     }
     if(current_state_g.connected)
     {
         ROS_INFO("Connected to FCU");
         return 0;
     }else{
         ROS_INFO("Error connecting to drone");
         return -1;
     }


 }
 int wait4start()
 {
     ROS_INFO("Waiting for user to set mode to GUIDED");
     while(ros::ok() && current_state_g.mode != "GUIDED")
     {
         ros::spinOnce();
         ros::Duration(0.01).sleep();
     }
     if(current_state_g.mode == "GUIDED")
     {
         ROS_INFO("Mode set to GUIDED. Mission starting");
         return 0;
     }else{
         ROS_INFO("Error starting mission!!");
         return -1;
     }
 }
 int initialize_local_frame()
 {
     //set the orientation of the local reference frame
     ROS_INFO("Initializing local coordinate system");
     local_offset_g = 0;
     for (int i = 1; i <= 30; i++) {
         ros::spinOnce();
         ros::Duration(0.1).sleep();

         float q0 = current_pose_g.pose.pose.orientation.w;
         float q1 = current_pose_g.pose.pose.orientation.x;
         float q2 = current_pose_g.pose.pose.orientation.y;
         float q3 = current_pose_g.pose.pose.orientation.z;
         float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) ); // yaw

         local_offset_g += psi*(180/M_PI);
         // ROS_INFO("current heading%d: %f", i, local_offset_g/i);
     }
     local_offset_g /= 30;
     ROS_INFO("Coordinate offset set");
     ROS_INFO("the X' axis is facing: %f", local_offset_g);
     return 0;
 }
 int takeoff(float takeoff_alt)
 {
     //intitialize first waypoint of mission
     set_destination(0,0,takeoff_alt,0);
     for(int i=0; i<100; i++)
     {
         local_pos_pub.publish(waypoint_g);
         ros::spinOnce();
         ros::Duration(0.01).sleep();
     }
     // arming
     ROS_INFO("Arming drone");
     mavros_msgs::CommandBool arm_request;
     arm_request.request.value = true;
     while (!current_state_g.armed && !arm_request.response.success)
     {
         ros::Duration(.1).sleep(); //pinoutpinout
         arming_client.call(arm_request);
         local_pos_pub.publish(waypoint_g);
     }
     if(arm_request.response.success)
     {
         ROS_INFO("Arming Successful");
     }else{
         ROS_INFO("Arming failed with %d", arm_request.response.success);
         return -1;
     }

     //request takeoff

     mavros_msgs::CommandTOL srv_takeoff;
     srv_takeoff.request.altitude = takeoff_alt;
     if(takeoff_client.call(srv_takeoff)){
         ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
     }else{
         ROS_ERROR("Failed Takeoff");
         return -2;
     }
     sleep(5);
     return 0;
 }
 int check_waypoint_reached()
 {
     local_pos_pub.publish(waypoint_g);

     float tollorance = .3;
     float deltaX = abs(waypoint_g.pose.position.x - current_pose_g.pose.pose.position.x);
     float deltaY = abs(waypoint_g.pose.position.y - current_pose_g.pose.pose.position.y);
     float deltaZ = abs(waypoint_g.pose.position.z - current_pose_g.pose.pose.position.z);
     float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );

     if( dMag < tollorance)
     {
         return 1;
     }else{
         return 0;
     }
 }
 int land()
 {
   mavros_msgs::CommandTOL srv_land;
   if(land_client.call(srv_land) && srv_land.response.success)
   {
     ROS_INFO("land sent %d", srv_land.response.success);
     return 0;
   }else{
     ROS_ERROR("Landing failed");
     return -1;
   }
 }
 int init_publisher_subscriber(ros::NodeHandle controlnode)
 {
     local_pos_pub = controlnode.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
     currentPos = controlnode.subscribe<nav_msgs::Odometry>("mavros/global_position/local", 10, pose_cb);
     state_sub = controlnode.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
     arming_client = controlnode.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
     land_client = controlnode.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
     set_mode_client = controlnode.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
     takeoff_client = controlnode.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

 }

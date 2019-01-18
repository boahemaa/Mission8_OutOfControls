# Control API Usage 

/ref control_functions

The Texas Aerial Robotics control API is designed to simplify the process of writing intelligent drone applications. The API is built off of mavros and mavlink. These two packages are middle men between the arducopter flight code on the pixhawk and our code on the jetson. 

## List of functions. 

### set_destination(float x, float y, float z, float psi)
This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. 

#### returns
```
n/a
```
### set_heading(float psi)
This function is used to specify the drone’s heading in the local reference frame. Psi is counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. 

#### returns
```
n/a
```
### wait4connect()
Wait for connect is a function that will hold the program until communication with the FCU is established.

#### returns
```
0 - connected to fcu 
-1 - failed to connect to drone
```
### wait4start()
Wait for strat will hold the program until the user signals the FCU to enther mode guided. This is typically done from a switch on the safety pilot’s remote or from the ground control station. 

#### returns
```
0 - mission started
-1 - failed to start mission
```
### initialize_local_frame()
This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to.

#### returns
0 - frame initialized 

### takeoff(float height)
The takeoff function will arm the drone and put the drone in a hover above the initial position. 

#### returns
```
0 - nominal takeoff 
-1 - failed to arm 
-2 - failed to takeoff
```
### check_waypoint_reached()
This function returns an int of 1 or 0. THis function can be used to check when to request the next waypoint in the mission. 

#### returns
```
1 - waypoint reached
0 - waypoint not reached
```
### land()
this function changes the mode of the drone to land

#### returns
```
1 - mode change successful
0 - mode change not successful
```
### init_publisher_subscriber(ros::NodeHandle nh)
This function is called at the beginning of a program and will start of the communication links to the FCU. The function requires the program's ros nodehandle as an input 

#### returns
``` 
n/a
```
## List of Structures

### control_api_waypoint 
```
float x
float y
float z
float psi
```

## Example Code

src/controlAPIExample.cpp 

```
#include <control_functions.hpp>
//include API 

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "offb_node");
	ros::NodeHandle controlnode;
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(controlnode);

	//specify some waypoints 
	std::vector<control_api_waypoint> waypointList;
	control_api_waypoint nextWayPoint;
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = -90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 5;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 5;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 180;
	waypointList.push_back(nextWayPoint);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(3);

	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		if(check_waypoint_reached() == 1)
		{
			if (counter < waypointList.size())
			{
				set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
				counter++;	
			}else{
				//land after all waypoints are reached
				land();
			}	
		}	
		
	}
	return 0;
}

```
### run example code
```
roslaunch mission8_sim droneOnly.launch
# New Terminal
~/catkin_ws/src/Mission8_OutOfControls/scripts/startsim.sh
# New Terminal
roslaunch out_of_controls apm.launch
# New Terminal 
roslaunch out_of_controls contolAPIExample.launch
```
NOTE** you can tile gnome terminals by pressing `ctrl + shift + t`


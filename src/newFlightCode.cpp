#include <iostream>
#include <control_functions.hpp>

using namespace std;

struct localWaypoint{
	float x;
	float y;
	float z;
	float psi;
};
std::vector<localWaypoint> waypointList;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "offb_node");
	ros::NodeHandle controlnode;
	
	init_publisher_subscriber(controlnode);

	ros::Rate rate(5.0);

	//specify some waypoints 
	localWaypoint nextWayPoint;
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 5;
	nextWayPoint.y = 5;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 5;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);



  	// wait for FCU connection
	wait4connect();

	wait4start();

	initialize_local_frame();

	takeoff(3);


	setDestination(0,0,3);
	setHeading(0);
	
	int counter = 0;
	while(ros::ok())
	{
		ros::spinOnce();
		 rate.sleep();
		if(check_waypoint_reached() == 1)
		{
			if (counter < waypointList.size())
			{
				setDestination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z);
				setHeading(waypointList[counter].psi);
				counter++;	
			}
			

		}
		std::cout << "counter " << counter << endl;	
		
	}
	return 0;
}


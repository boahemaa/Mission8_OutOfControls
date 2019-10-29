#!/bin/bash

roslaunch mission8_sim droneOnly.launch &
~/catkin_ws/src/Mission8_OutOfControls/scripts/startsim.sh 

#sleep 10

#roslaunch out_of_controls apm.launch &

#sleep 10

#roslaunch out_of_controls contolAPIExample.launch

##FUNKTIONIERT NICHT, KA WARUM
##Ausfuehren mit:
##~/catkin_ws/src/Mission8_OutOfControls/scripts/waypoint_example.sh 



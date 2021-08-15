#!/bin/bash

gnome-terminal -- roslaunch interiit21 interiit_world1.launch &&
sleep 10 &&
gnome-terminal -- sim_vehicle.py -v ArduCopter -f gazebo-iris --console &&
sleep 10 &&
gnome-terminal -- python follow.py &&
sleep 20 &&
gnome-terminal -- python rgb.py &&
sleep 10 &&
gnome-terminal -- python algo.py 




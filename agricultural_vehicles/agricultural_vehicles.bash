#!/bin/bash
# Program:
#       This program shows start agricultural_vehicles
# History:
# 2020/11/12	kevin	First release

# PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
# export PATH
# echo -e "Hello World! \a \n"
# exit 0

echo -e "\033[32m
##############################
# ros agricultural_vehicles  #
# by Kevin Chiu 2020         #
##############################
\033[0m"
cd ~/ros/agricultural_vehicles/
source ~/ros/agricultural_vehicles/devel/setup.bash

gnome-terminal --tab -t "roscore" -- bash -ic "roscore"
sleep 0.2
gnome-terminal --tab -t "joystick" -- bash -ic "rosrun JoyStick joystick /dev/input/js0"
sleep 0.2
gnome-terminal --tab -t "magnetic_rail" -- bash -ic "rosrun magnetic_rail mr_position_py.py /dev/agricultural_magnetic_rail 115200"
sleep 0.2
gnome-terminal --tab -t "GPS_" -- bash -ic "rosrun GPS_pkg GPS_ /dev/ttyUSB1 38400"
sleep 0.2

gnome-terminal --tab -t "IMU_" -- bash -ic "rosrun GPS_pkg IMU_ /dev/ttyACM0 115200"
# gnome-terminal --tab -t "IMU_" -- bash -ic "rosrun GPS_pkg kevin_vmu931_rev_py.py /dev/ttyACM0 115200"
sleep 0.2

gnome-terminal --tab -t "rviz" -- bash -ic "rosrun rviz rviz -d ~/ros/agricultural_vehicles/src/agricultural_vehicles_rviz.rviz"
sleep 0.2

gnome-terminal --tab -t "move_robot" -- bash -ic "rosrun move_robot move_robot /dev/agricultural_arduino_nano 115200"
# gnome-terminal --tab -t "move_robot" -- bash -ic "rosrun move_robot move_robot /dev/kevin_arduino_nano 115200"
# gnome-terminal --tab -t "move_robot" -- bash -ic "rosrun move_robot move_robot /dev/ttyUSB0 115200"
sleep 0.2

exec bash
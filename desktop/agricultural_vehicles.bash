#!/bin/bash
# Program:
#       This program shows start agricultural_vehicles
# History:
# 2020/11/12	kevin	First release

# PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
# export PATH
# echo -e "Hello World! \a \n"
# exit 0

gnome-terminal --tab -- bash -ic "roscore"
sleep 0.2
gnome-terminal --tab -- bash -ic "rosrun JoyStick joystick /dev/input/js0"
sleep 0.2
gnome-terminal --tab -- bash -ic "rosrun magnetic_rail mr_position_py.py /dev/agricultural_magnetic_rail 115200"
sleep 0.2
gnome-terminal --tab -- bash -ic "rosrun GPS_pkg GPS_ /dev/ttyUSB2 38400"
sleep 0.2
gnome-terminal --tab -- bash -ic "rosrun GPS_pkg IMU_ /dev/ttyACM0 115200"
sleep 0.2
gnome-terminal --tab -- bash -ic "rosrun rviz rviz -d /home/user/Desktop/Farm_AGV_MPC_0929_ws/src/agricultural_vehicles_rviz.rviz"
sleep 0.2
gnome-terminal --tab -- bash -ic "rosrun move_robot move_robot /dev/agricultural_arduino_nano 115200"
sleep 0.2
# gnome-terminal --tab -- bash -ic "rosrun move_robot move_robot /dev/kevin_arduino_nano 115200"
# sleep 0.2

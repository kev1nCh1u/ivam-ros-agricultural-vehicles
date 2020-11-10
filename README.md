# agricultural_vehicles

## start command
    cd ~/ros/agricultural_vehicles/Farm_AGV_MPC_0929_ws
    
    roscore
    
    . devel/setup.bash
    
    rosrun JoyStick joystick /dev/input/js0
    
    rosrun microsoft_rfid rfid_py.py /dev/agricultural_rfid 38400
    
    rosrun magnetic_rail mr_position_py.py /dev/agricultural_magnetic_rail 115200
    
    rosrun move_robot move_robot /dev/agricultural_arduino_nano 115200

    rosrun kevin_tcp tcp_server_py.py
    
    rosrun rviz rviz -d /home/user/Desktop/Farm_AGV_MPC_0929_ws/src/agricultural_vehicles_rviz.rviz

    rosrun GPS_pkg GPS_ /dev/ttyUSB2 38400

    rosrun GPS_pkg IMU_ /dev/ttyACM0 115200

## 啟動步驟
- open rviz
- load map 搖桿back
- 拉 heading -> 2d pose estimate -> 隨便拉
- 按 y -> send mission
- 按 LB 進 auto

## debug
    rostopic echo /mr_msg

    rostopic echo /Send_Pose

    rostopic pub -r 5 Send_Pose geometry_msgs/PoseStamped 

## button
- back -> load map
- Y -> send mission
- LB -> auto/mannal
- A -> joystick mode
- RT -> 油門
- LT -> 煞車


## usb 固定
    lsusb
    udevadm info --attribute-walk --name=/dev/ttyUSB0

    sudo vim /etc/udev/rules.d/usb.rules

    sudo service udev reload
    sudo service udev restart

    cd /dev/
    ls

    KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0777", SYMLINK+="agricultural_arduino_nano"
    KERNELS=="1-1.1:1.0", SYMLINK+="agricultural_magnetic_rail"
    KERNELS=="1-1.2:1.0", SYMLINK+="agricultural_rfid"

    nano Bus 001 Device 008: ID 0403:6001 Future Technology Devices International, Ltd FT232 USB-Serial (UART) I
    mag KERNELS=="1-1.1:1.0"
    rfid KERNELS=="1-1.2:1.0"

## 問題
- 無
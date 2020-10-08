# agricultural_vehicles

## start command

    roscore
    rosrun microsoft_rfid rfid_py.py /dev/ttyUSB1 115200
    rosrun magnetic_rail mr_position_py.py /dev/ttyUSB1 115200
    rosrun move_robot move_robot /dev/ttyUSB1 115200
    rosrun rviz rviz

## 啟動步驟
- open rviz
- load map
- 拉 heading -> 2d pose estimate -> 隨便拉
- 按 y -> send mission
- 按 LB 進 auto

## debug
    rostopic echo /mr_msg

## button
- back -> load map
- Y -> send mission
- LB -> auto/mannal
- A -> joystick mode
- RT -> 油門
- LT -> 煞車

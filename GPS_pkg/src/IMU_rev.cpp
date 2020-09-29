#include <ros/ros.h>
#include "ros/package.h"
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Point.h"
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <string>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <bitset>
#include <vector>
#include <boost/thread.hpp>
#include <time.h>
#include <fstream>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <stdio.h>
#include <fstream>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string.h>
#include <iomanip>



struct SERIAL{
    struct termios  oldtio;
    int fd,res,done;
    bool serial_ok;
};


SERIAL mySerial;

//Set parameter
double IMU_Angle = 0;
std_msgs::Float64 Send_IMU_Angle;

int setBaudrate(int baudrate)
{
    if(baudrate == 50)
        return B50;
    else if(baudrate == 75)
        return B75;
    else if(baudrate == 110)
        return B110;
    else if(baudrate == 134)
        return B134;
    else if(baudrate == 150)
        return B150;
    else if(baudrate == 200)
        return B200;
    else if(baudrate == 300)
        return B300;
    else if(baudrate == 600)
        return B600;
    else if(baudrate == 1200)
        return B1200;
    else if(baudrate == 1800)
        return B1800;
    else if(baudrate == 2400)
        return B2400;
    else if(baudrate == 4800)
        return B4800;
    else if(baudrate == 9600)
        return B9600;
    else if(baudrate == 19200)
        return B19200;
    else if(baudrate == 38400)
        return B38400;
    else if(baudrate == 115200)
        return B115200;
    else
        return B0;
}
int SerialConnect(SERIAL &serial, char *port_name, int speed)
{
    serial.serial_ok = false;

    serial.fd = open(port_name, O_RDWR | O_NOCTTY | O_SYNC);
    if(serial.fd < 0){

        return -1;
    }

    if(tcgetattr(serial.fd, &(serial.oldtio)) < 0){

        return -1;
    }

    cfsetospeed(&serial.oldtio, (speed_t)speed);
    cfsetispeed(&serial.oldtio, (speed_t)speed);

    serial.oldtio.c_cflag |= (CLOCAL | CREAD);
    serial.oldtio.c_cflag &= ~CSIZE;
    serial.oldtio.c_cflag |= CS8;
    serial.oldtio.c_cflag &= ~PARENB;
    serial.oldtio.c_cflag &= ~CSTOPB;
    serial.oldtio.c_cflag &= ~CRTSCTS;


    serial.oldtio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    serial.oldtio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    serial.oldtio.c_oflag &= ~OPOST;

    serial.oldtio.c_cc[VMIN] = 1;
    serial.oldtio.c_cc[VTIME] = 1;

    if(tcsetattr(serial.fd, TCSANOW, &serial.oldtio) != 0){
        std::cout<<"Error from tcgetattr"<<std::endl;
        return -1;

    }

    serial.serial_ok = true;

    std::cout<<"Serial Opened"<<std::endl;

    return 0;
}

/*void Set_IMU_to_zero()
{
    unsigned char array[8];

    array[0] = 0x01;
    array[1] = 0x06;
    array[2] = 0x00;
    array[3] = 0x10;
    array[4] = 0x00;
    array[5] = 0xFF;
    array[6] = 0xC8;
    array[7] = 0x01;

    int nByte = 0;
    if(mySerial.serial_ok)
       nByte = write(mySerial.fd,array,8);


}*/

/*void SendPackage()
{

    unsigned char command[8];

    command[0] = 0x01;
    command[1] = 0x04;
    command[2] = 0x00;
    command[3] = 0x02;
    command[4] = 0x00;
    command[5] = 0x16;
    command[6] = 0x04;
    command[7] = 0x07;

    int nByte = 0;
    if(mySerial.serial_ok)
       nByte = write(mySerial.fd,command,8);
   //std::cout<< "nByte  " << nByte << std::endl;
   std::cout<<"=================Send================"<<std::endl;
   for(int i = 0; i < 8; i++)
   {
    //std::cout<<"=================Send================"<<std::endl;
    printf("%d\n",command[i]);
   }
}*/

void RevProcess(double receive_period)
{
        int Receive_Package_Size = 256;

        //Declear Parameters
        ros::Rate r_receive(1.0 / receive_period);
        std::vector<char> rev_buf;
        //std::fstream file;  //for write txt test
        //file.open("IMU.txt" , std::fstream::out);  //for write txt test
        static int i = 0; 

        //Publisher
        ros::NodeHandle imu;
        ros::Publisher chatter_pub = imu.advertise<std_msgs::Float64>("Send_IMU", 1000);

        while(1){
            //std::cout<<"========================================"<<std::endl;
            if(mySerial.serial_ok == true){
                char buff[Receive_Package_Size];
                int readByte = 0;
                readByte = read(mySerial.fd,buff,4);
                if(readByte > 0)
                {
                    //std::cout<<"=================Recv================"<<std::endl;
                    for(int j = 0; j < readByte ; j++)
                    {
                      rev_buf.push_back(buff[j]);
                      //printf("%d\n",rev_buf[j]);  
                    }
                    if(rev_buf[0] != 'A')
                    {
                        rev_buf.clear();
                    }     
                }
                if(rev_buf[0] == 65)
                {
                    if(rev_buf[3] == 66)
                    {
                        //std::cout<<"======================================"<<std::endl;
                        double recv_yaw_h = rev_buf[1];
                        double recv_yaw_l = rev_buf[2];
                        IMU_Angle = ((recv_yaw_h * 256 + recv_yaw_l)-10000)/100;
                        Send_IMU_Angle.data =  IMU_Angle;
                        std::cout<<"IMU Angle:  "<< IMU_Angle<<std::endl;
                        //file << "IMU Angle: " << IMU_Angle << std::endl;;   //for write txt test
                        i = 0;
                        rev_buf.clear();
                    }
                    else
                    {
                        rev_buf.clear();
                    } 
                }

            }
            chatter_pub.publish(Send_IMU_Angle);
            r_receive.sleep();


        }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "IMU");
    ros::Time::init();

    if(argc < 3){

        ROS_INFO("usage: [<Device name>] [<Baud rate>]");

        return 0;

    }

    char **argv_buf;
    int Baudrate = std::atoi(argv[2]);
    char *dev_name = argv[1];

    int baudrate = setBaudrate(Baudrate);
     if(SerialConnect(mySerial, dev_name, (speed_t)baudrate) < 0){


         ROS_INFO("Serial failed");
     }

    boost::thread* receive_thread_;
    //Set_IMU_to_zero();
    receive_thread_ = new boost::thread(boost::bind(RevProcess, 0.01));

    ros::spin();
    return 0;
}

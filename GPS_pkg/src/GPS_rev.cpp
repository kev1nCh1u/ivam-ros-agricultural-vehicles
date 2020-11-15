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

//Set Parameter
double Affine_lat = 0;
double Affine_lon = 0;
double Affine_lat_m = 0;
double Affine_lon_m = 0;
double Recv_IMU_Angle = 0;
double IMU_Angle = 0;
double scale = 111000; // in meter
std_msgs::Float64 Send_lat;
std_msgs::Float64 Send_lon;
geometry_msgs::PoseStamped EV_Pose_msg; 


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

void IMUCallback(const std_msgs::Float64::ConstPtr& Send_IMU_Angle)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  //ROS_INFO("I heard: [%f]", Send_IMU_Angle->data);
  EV_Pose_msg.pose.position.z  = Send_IMU_Angle->data;

}

void normalize(std::vector<Eigen::Vector3f> &point)
{
    Eigen::MatrixXf p;

    std::vector<float> l_buffer;

    int size = point[0].size();
    float l = 0;
    Eigen::Matrix<float , 3, 3> T;

     p = Eigen::MatrixXf::Constant(size, 1, 0);

    for (int i = 0; i < point.size(); i++)
    {
        p = p + point[i];

    }

    p = p / point.size();
    for (int i = 0; i < point.size(); i++)
    {
        l = sqrt(pow(p(0) - point[i].x(), 2) + pow(p(1) - point[i].y(), 2));
        l_buffer.push_back(l);
    }
    std::stable_sort(l_buffer.begin(),l_buffer.end());
    l = 5.0 / l_buffer.back();

    /*l = l / point.size();
    l = sqrt(2) / l;*/
    std::cout<< l <<std::endl;


    Eigen::MatrixXf s;
     s=Eigen::MatrixXf::Identity(size, size);
    Eigen::MatrixXf m;
    m = Eigen::MatrixXf::Identity(size, size);

    s = s * l;
    s(size - 1, size - 1) = 1;


    for (int i = 0; i < size-1; i++)
    {
        m(i, size - 1) = -p(i);
    }


    T = s * m;

    std::cout<< s <<std::endl;
    std::cout<< m <<std::endl;
    std::cout<< T <<std::endl;

    for (int i = 0; i < point.size(); i++)
    {
         Eigen::Vector3f po = T * point[i];
         point[i] = po;

    }
}

void RevProcess(double receive_period)
{
        int Receive_Package_Size = 256;

        //Declear Parameters
        ros::Rate r_receive(1.0 / receive_period);
        std::vector<char> rev_buf;
        std::vector<std::string> gps_data_buf;
        std::fstream file;
        file.open("src/GPS_path.txt" , std::fstream::app);
        double Display_lat = 0;
        double Display_lon = 0;
        double Center_Lat = 2500.67057;
        double Center_Lon = 12132.38304;

        //Publisher
        ros::NodeHandle gps;
        ros::Publisher chatter_pub = gps.advertise<geometry_msgs::PoseStamped>("Send_Pose", 1000);

        static int i = 0; 

        while(1){
            // EV_Pose_msg.pose.position.x = 0.0185; //fake data
            // EV_Pose_msg.pose.position.y = -1.14526e-09;  //fake data
            // EV_Pose_msg.pose.position.z = 0.0;  //fake data
            // std::cout<<"========================================"<<std::endl;
            if(mySerial.serial_ok == true){
                char buff[Receive_Package_Size];
                int readByte = 0;
                readByte = read(mySerial.fd,buff,1);
                if(readByte > 0)
                {
                    rev_buf.push_back(buff[0]);
                    i++;
                    if(rev_buf[0] != '$')
                    {
                        rev_buf.clear();
                        i = 0;
                        std::cout<<"=========LOSS encoder========="<<std::endl;
                        //file << "GGGGGGG" << "\n";
                        //std::cout<<"========================================"<<std::endl;
                    }     
                }

                if(rev_buf[0] == '$' && rev_buf[1] == 'G' && rev_buf[2] == 'N' && rev_buf[3] == 'R' && rev_buf[4] == 'M' && rev_buf[5] == 'C' )
                {
                    if(buff[0] == '\n')
                    {
                        std::string data(rev_buf.begin(),rev_buf.end());
                        std::cout<<"data:"<< data<<std::endl;
                        //std::stringstream g_data_buf = g_data_buf.str(data);

                        //split method 1
                        std::string str2 = data;
                       while (str2.find(",") != std::string::npos)
                        {
                            int found = str2.find(",");
                            gps_data_buf.push_back(str2.substr(0, found));
                            str2 = str2.substr(found + 1);
                        }
                        gps_data_buf.push_back(str2);//GNGLL: gps_data_buf[1] is lat and gps_data_buf[3] is lon , GNRMC:gps_data_buf[3] is lat and gps_data_buf[5] is lon
                        // std::cout << gps_data_buf[3] << " " << gps_data_buf[5] << std::endl;
                        Affine_lat = strtold(gps_data_buf[3].data(),NULL) - Center_Lat; 
                        Affine_lon = strtold(gps_data_buf[5].data(),NULL) - Center_Lon;
                        Affine_lat_m = (Affine_lat/60) * scale;
                        Affine_lon_m = (Affine_lon/60) * scale;
                        //Display_lat = ((strtold(gps_data_buf[3].data(),NULL) - 2500) / 60) + 25;
                        //Display_lon = ((strtold(gps_data_buf[5].data(),NULL) - 12100) / 60) + 121;
                        // EV_Pose_msg.pose.position.x = Affine_lon_m;
                        // EV_Pose_msg.pose.position.y = Affine_lat_m;
                        EV_Pose_msg.pose.position.x = -Affine_lat_m;
                        EV_Pose_msg.pose.position.y = Affine_lon_m;
                        //std::cout<<"lat:  "<< Display_lat << "  lon: "<< Display_lon <<std::endl;
                        // std::cout<<"lat:  "<< -Affine_lat_m << "  lon: "<< Affine_lon_m <<std::endl;
                        file << -Affine_lat_m << " " << Affine_lon_m << std::endl;
                        //file << "aaaa" << "\n";
                        //std::cout<<"========================================"<<std::endl;
                        i = 0;
                        gps_data_buf.clear();
                        rev_buf.clear();
                    } 
                }
                else if(buff[0] == '\n')
                {
                    i = 0;
                    rev_buf.clear();
                }
            }

            ROS_INFO("Lat: %f ,Lon: %f,IMU: %f", EV_Pose_msg.pose.position.x, EV_Pose_msg.pose.position.y, EV_Pose_msg.pose.position.z);
            chatter_pub.publish(EV_Pose_msg);
            r_receive.sleep();


        }



}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "GPS");
    ros::Time::init();

    ros::NodeHandle imu;

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
    receive_thread_ = new boost::thread(boost::bind(RevProcess, 0.000001));

    //Subcriber
    ros::Subscriber sub = imu.subscribe("Send_IMU", 1000, IMUCallback);
    

    ros::spin();

    return 0;


}

#include <vector>
#include <math.h>
#include <stdio.h>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <move_robot/Battery.h>

#define ChinaMotor 0
#define Boling_onewheel 0
#define IVAM_EV 1
#define IVAM_Car_vlvr 0
#define IVAM_Car_vw 1
#define Boling_smallgray 2



class sendrev
{
public:
    sendrev();
    void Setmode(int input);

    void Package_AllDir_encoder(int FL, int FLA,int BL,int BLA,int FR ,int FRA,int BR,int BRA,std::vector<unsigned char> &return_cmd);
    void RevProcess_AllDir_encoder(std::vector<unsigned char> &rev_buf,float &Rev_FR_rpm,float &Rev_FL_rpm,float &Rev_RR_rpm,float &Rev_RL_rpm,float &Rev_FR_deg,float &Rev_FL_deg,float &Rev_RR_deg,float &Rev_RL_deg);
    void Package_OneWheel_encoder(double cmd_v, double cmd_w,int DriveMode,int ManualMode,int BrakeSign,std::vector<unsigned char> &return_cmd);
    void RevProcess_IVAM_EV_encoder(std::vector<unsigned char> &rev_buf, int &RPM_L,int &RPM_R,int &Delta_now, float &V_cal, float &a_cal);
    void Package_Diff_encoder(double cmd_vl, double cmd_vr,std::vector<unsigned char> &return_cmd);
    void Package_Boling_smallgray(double &cmd_vl, double &cmd_vr,std::vector<unsigned char> &return_cmd);
    void RevProcess_two_wheel_encoder(std::vector<unsigned char> &rev_buf,float &Rev_V, float &Rev_W);

private:
    void Package_Boling_onewheel(double cmd_v, double cmd_w,std::vector<unsigned char> &return_cmd);
    void Package_IVAM_EV_Auto(float cmd_v, float cmd_w,int DriveMode,int ManualMode,int BrakeSign,std::vector<unsigned char> &return_cmd);
    void Package_ChinaMotor(int &FL, int &FLA,int &BL,int &BLA,int &FR ,int &FRA,int &BR,int &BRA,std::vector<unsigned char> &return_cmd );
    void RevProcess_ChinaMotor(std::vector<unsigned char> &rev_buf,float &Rev_FR_rpm,float &Rev_FL_rpm,float &Rev_RR_rpm,float &Rev_RL_rpm,float &Rev_FR_deg,float &Rev_FL_deg,float &Rev_RR_deg,float &Rev_RL_deg);
    void RevProcess_IVAM_EV(std::vector<unsigned char> &rev_buf,int &RPM_L,int &RPM_R,int &Delta_now, float &V_cal, float &a_cal);
    void Package_IVAM_Car_vlvr(double &cmd_vl, double &cmd_vr,std::vector<unsigned char> &return_cmd);
    void Package_IVAM_Car_vw(double &cmd_vl, double &cmd_vr,std::vector<unsigned char> &return_cmd);
    void RevProcess_IVAM_Car_vw(std::vector<unsigned char> &rev_buf,float &Rev_V, float &Rev_W);


private:
    int mode;
    ros::NodeHandle nodeHandle_;
    ros::Publisher Battery_Publisher_;

};
sendrev::sendrev()
{
        mode = -1 ;
        Battery_Publisher_ = nodeHandle_.advertise<move_robot::Battery>("Battery",10);
}
void sendrev::Setmode(int input)
{
        mode = input;
}
void sendrev::Package_AllDir_encoder(int FL, int FLA,int BL,int BLA,int FR ,int FRA,int BR,int BRA,std::vector<unsigned char> &return_cmd)
{
    switch(mode){
            case ChinaMotor:
                 Package_ChinaMotor( FL,  FLA, BL, BLA, FR , FRA, BR, BRA, return_cmd );
            break;

            default:
            break;
    }

}

void sendrev::Package_Diff_encoder(double cmd_vl, double cmd_vr,std::vector<unsigned char> &return_cmd)
{
    switch(mode){
            case IVAM_Car_vlvr:
                 Package_IVAM_Car_vlvr(cmd_vl, cmd_vr, return_cmd);
            break;

            case IVAM_Car_vw:
                 Package_IVAM_Car_vw(cmd_vl, cmd_vr, return_cmd);
            break;

            case Boling_smallgray:
                 Package_Boling_smallgray(cmd_vl, cmd_vr, return_cmd);
            break;

            default:
            break;
    }
}

void sendrev::Package_OneWheel_encoder(double cmd_v, double cmd_w,int DriveMode,int ManualMode,int BrakeSign,std::vector<unsigned char> &return_cmd)
{
    switch(mode){
            case Boling_onewheel:
                 Package_Boling_onewheel(cmd_v, cmd_w, return_cmd);
            break;

            case IVAM_EV:
                 Package_IVAM_EV_Auto(cmd_v,cmd_w, DriveMode, ManualMode, BrakeSign, return_cmd);
            default:
            break;
    }
}

void sendrev::RevProcess_IVAM_Car_vw(std::vector<unsigned char> &rev_buf,float &Rev_V, float &Rev_W)
{
    if(rev_buf[0] == 'E' && rev_buf[1] == 'C'  && rev_buf[10] == 'E')
    {
        int HighByte_integer_v = rev_buf[2];
        int LowByte_integer_v = rev_buf[3];
        int HighByte_float_v = rev_buf[4];
        int LowByte_float_v = rev_buf[5];

        int HighByte_integer_w = rev_buf[6];
        int LowByte_integer_w = rev_buf[7];
        int HighByte_float_w = rev_buf[8];
        int LowByte_float_w = rev_buf[9];

        int integer_v = HighByte_integer_v * 256 + LowByte_integer_v;
        int float_v = HighByte_float_v * 256 + LowByte_float_v;
        int integer_w = HighByte_integer_w * 256 + LowByte_integer_w;
        int float_w = HighByte_float_w * 256 + LowByte_float_w;

        float V = integer_v + float_v * 0.001;
        float W = integer_w + float_w * 0.001;

        Rev_V = V - 20;
        Rev_W = W - 20;
    }
//std::cout<<"Rev_V = "<<Rev_V<<" "<<" Rev_W =  "<<Rev_W<<std::endl;

}

void sendrev::Package_IVAM_EV_Auto(float cmd_v, float cmd_w,int DriveMode,int ManualMode,int BrakeSign, std::vector<unsigned char> &return_cmd)
{
    //std::cout<<"cmd_v: "<< cmd_v << " cmd_w: " << cmd_w <<std::endl;
    std::vector<unsigned char> command;
    //cmd_w = -cmd_w;

    if(cmd_w > 50)cmd_w = 50; // kevin limit
    else if(cmd_w < -50)cmd_w = -50;
    if(cmd_v > 400)cmd_v = 400;
    else if(cmd_v <= 0)cmd_v = 0;

    int steering_data = int(cmd_w * 3.0 *100.0) + 9000;


    int HighByte_integer_v = (int(cmd_v)) / 256; // kevin
    int LowByte_integer_v  = (int(cmd_v)) % 256;
    //int HighByte_float_v   = float_v/256;
    //int LowByte_float_v    = float_v%256;

    int HighByte_integer_w = steering_data / 256;
    int LowByte_integer_w  = steering_data % 256;
    //int HighByte_float_w   = float_w/256;
    //int LowByte_float_w    = float_w%256;

    if(DriveMode == 1)  //Autonomous Mode
    {
      command.push_back(255);
      command.push_back(DriveMode);
      command.push_back(ManualMode);
      command.push_back(BrakeSign);
      command.push_back(0);
      command.push_back(HighByte_integer_w);
      command.push_back(LowByte_integer_w);
      //command.push_back(HighByte_float_v);
      //command.push_back(LowByte_float_v);
      command.push_back(HighByte_integer_v);
      command.push_back(LowByte_integer_v);
      //command.push_back(HighByte_float_w);
      //command.push_back(LowByte_float_w);
      command.push_back(254);
    }

    else if(DriveMode == 0) //Manual Mode
    {
    command.push_back(255);
    command.push_back(DriveMode);
    command.push_back(ManualMode);
    command.push_back(BrakeSign);
    command.push_back(0);
    command.push_back(0);
    command.push_back(0);
    //command.push_back(HighByte_float_v);
    //command.push_back(LowByte_float_v);
    command.push_back(0);
    command.push_back(0);
    //command.push_back(HighByte_float_w);
    //command.push_back(LowByte_float_w);
    command.push_back(254);
    }

    return_cmd = command;

   for(int i = 0; i < 10; i++)
   {
    printf("%d ",command[i]); // kevin 讓它變一行
   }
   printf("\n"); // kevin 讓它變一行

   std::cout<<"=================Send================"<<std::endl; // print info

}

void sendrev::Package_Boling_onewheel(double cmd_v, double cmd_w, std::vector<unsigned char> &return_cmd)
{
    // std::cout<<"cmd_v: "<<cmd_v<<std::endl;
    // std::cout<<"cmd_w: "<<cmd_w<<std::endl;
    double send_v = cmd_v + 20.0;
    double send_w = cmd_w + 20.0;

    std::vector<unsigned char> command;

    int integer_v = (int(send_v));
    int float_v = ( int((send_v - double(integer_v))*1000 ));
    int integer_w = (int(send_w));
    int float_w = ( int((send_w - double(integer_w))*1000 ));

    int HighByte_integer_v = integer_v/256;
    int LowByte_integer_v  = integer_v%256;
    int HighByte_float_v   = float_v/256;
    int LowByte_float_v    = float_v%256;

    int HighByte_integer_w = integer_w/256;
    int LowByte_integer_w  = integer_w%256;
    int HighByte_float_w   = float_w/256;
    int LowByte_float_w    = float_w%256;

    command.push_back('S');
    command.push_back('T');
    command.push_back(HighByte_integer_v);
    command.push_back(LowByte_integer_v);
    command.push_back(HighByte_float_v);
    command.push_back(LowByte_float_v);
    command.push_back(HighByte_integer_w);
    command.push_back(LowByte_integer_w);
    command.push_back(HighByte_float_w);
    command.push_back(LowByte_float_w);
    command.push_back('E');
    command.push_back('N');
    command.push_back('D');

    return_cmd = command;


}

void sendrev::Package_IVAM_Car_vlvr(double &cmd_vl, double &cmd_vr,std::vector<unsigned char> &return_cmd)
{
    double send_vl = cmd_vl + 20.0;
    double send_vr = cmd_vr + 20.0;

    std::vector<unsigned char> command;
    //std::cout<<"send_vl: "<<send_vl<<std::endl;
    //std::cout<<"send_vr: "<<send_vr<<std::endl;



    int integer_vl = (int(send_vl));
    int float_vl = ( int((send_vl - double(integer_vl))*1000 ));
    int integer_vr = (int(send_vr));
    int float_vr = ( int((send_vr - double(integer_vr))*1000 ));


    int HighByte_integer_vl = integer_vl/256;
    int LowByte_integer_vl  = integer_vl%256;
    int HighByte_float_vl   = float_vl/256;
    int LowByte_float_vl    = float_vl%256;

    int HighByte_integer_vr = integer_vr/256;
    int LowByte_integer_vr  = integer_vr%256;
    int HighByte_float_vr   = float_vr/256;
    int LowByte_float_vr    = float_vr%256;



    command.push_back('S');
    command.push_back('T');
    command.push_back(HighByte_integer_vl);
    command.push_back(LowByte_integer_vl);
    command.push_back(HighByte_float_vl);
    command.push_back(LowByte_float_vl);
    command.push_back(HighByte_integer_vr);
    command.push_back(LowByte_integer_vr);
    command.push_back(HighByte_float_vr);
    command.push_back(LowByte_float_vr);
    command.push_back('E');
    command.push_back('N');
    command.push_back('D');



    return_cmd = command;


}

void sendrev::Package_IVAM_Car_vw(double &cmd_v, double &cmd_w,std::vector<unsigned char> &return_cmd)
{

    // std::cout<<"cmd_v: "<<cmd_v<<std::endl;
    // std::cout<<"cmd_w: "<<cmd_w<<std::endl;
    double send_v = cmd_v + 20.0;
    double send_w = cmd_w + 20.0;

    std::vector<unsigned char> command;
    // std::cout<<"send_v: "<<send_v<<std::endl;
    // std::cout<<"send_w: "<<send_w<<std::endl;



    int integer_v = (int(send_v));
    int float_v = ( int((send_v - double(integer_v))*1000 ));
    int integer_w = (int(send_w));
    int float_w = ( int((send_w - double(integer_w))*1000 ));


    int HighByte_integer_v = integer_v/256;
    int LowByte_integer_v  = integer_v%256;
    int HighByte_float_v   = float_v/256;
    int LowByte_float_v    = float_v%256;

    int HighByte_integer_w = integer_w/256;
    int LowByte_integer_w  = integer_w%256;
    int HighByte_float_w   = float_w/256;
    int LowByte_float_w    = float_w%256;


    command.push_back('S');
    command.push_back('T');
    command.push_back(HighByte_integer_v);
    command.push_back(LowByte_integer_v);
    command.push_back(HighByte_float_v);
    command.push_back(LowByte_float_v);
    command.push_back(HighByte_integer_w);
    command.push_back(LowByte_integer_w);
    command.push_back(HighByte_float_w);
    command.push_back(LowByte_float_w);
    command.push_back('E');
    command.push_back('N');
    command.push_back('D');



    return_cmd = command;
}

void sendrev::Package_ChinaMotor(int &FL, int &FLA,int &BL,int &BLA,int &FR ,int &FRA,int &BR,int &BRA,std::vector<unsigned char> &return_cmd )
{
    static unsigned char counter = 0;

    //int nByte = 0;

    //unsigned char command[30];
    std::vector<unsigned char> command;

    unsigned char FLHrpm,FLLrpm, FLHdegree,FLLdegree,FRHrpm,FRFLLrpm, FRHdegree,FRLdegree
            ,BLHrpm,BLLrpm, BLHdegree,BLLdegree,BRHrpm,BRFLLrpm, BRHdegree,BRLdegree;



    if(FL < 0)
    {
        FLHrpm=(unsigned char)((FL&0xFF00)>>8);
        FLLrpm=(unsigned char)(FL&0x00FF);
    }
    else
    {
        FLHrpm=(unsigned char)(FL/256);
        FLLrpm=(unsigned char)(FL%256);
    }
    if(FLA < 0)
    {
        FLHdegree=(unsigned char)((FLA&0xFF00)>>8);
        FLLdegree=(unsigned char)(FLA&0x00FF);
    }
    else
    {
        FLHdegree=(unsigned char)(FLA/256);
        FLLdegree=(unsigned char)(FLA%256);
    }
    if(BL < 0)
    {
        BLHrpm=(unsigned char)((BL&0xFF00)>>8);
        BLLrpm=(unsigned char)(BL&0x00FF);
    }else
    {
        BLHrpm=(unsigned char)(BL/256);
        BLLrpm=(unsigned char)(BL%256);
    }
    if(BLA < 0)
    {
        BLHdegree=(unsigned char)((BLA&0xFF00)>>8);
        BLLdegree=(unsigned char)(BLA&0x00FF);
    }
    else
    {
        BLHdegree=(unsigned char)(BLA/256);
        BLLdegree=(unsigned char)(BLA%256);
    }
    if(FR < 0)
    {
        FRHrpm=(unsigned char)((FR&0xFF00)>>8);
        FRFLLrpm=(unsigned char)(FR&0x00FF);
    }
    else
    {
        FRHrpm=(unsigned char)(FR/256);
        FRFLLrpm=(unsigned char)(FR%256);
    }
    if(FRA < 0)
    {
        FRHdegree=(unsigned char)((FRA&0xFF00)>>8);
        FRLdegree=(unsigned char)(FRA&0x00FF);
    }
    else
    {
        FRHdegree=(unsigned char)(FRA/256);
        FRLdegree=(unsigned char)(FRA%256);
    }
    if(BR < 0)
    {
        BRHrpm=(unsigned char)((BR&0xFF00)>>8);
        BRFLLrpm=(unsigned char)(BR&0x00FF);
    }else
    {
        BRHrpm=(unsigned char)(BR/256);
        BRFLLrpm=(unsigned char)(BR%256);
    }
    if(BRA < 0)
    {
        BRHdegree=(unsigned char)((BRA&0xFF00)>>8);
        BRLdegree=(unsigned char)(BRA&0x00FF);
    }
    else
    {
        BRHdegree=(unsigned char)(BRA/256);
        BRLdegree=(unsigned char)(BRA%256);
    }
    //std::cout<<"FL  "<< FL<<std::endl;
    command.push_back(0x0a);

    command.push_back(FRFLLrpm);
    command.push_back(FRHrpm);

    command.push_back(FRLdegree);
    command.push_back(FRHdegree);

    command.push_back(FLLrpm);
    command.push_back(FLHrpm);

    command.push_back(FLLdegree);
    command.push_back(FLHdegree);

    command.push_back(BLLrpm);
    command.push_back(BLHrpm);

    command.push_back(BLLdegree);
    command.push_back(BLHdegree);

    command.push_back(BRFLLrpm);
    command.push_back(BRHrpm);

    command.push_back(BRLdegree);
    command.push_back(BRHdegree);

    command.push_back(0x00);
    command.push_back(0x00);

    command.push_back(0x00);
    command.push_back(0x00);

    command.push_back(0x00);
    command.push_back(0x00);

    command.push_back(0x00);
    command.push_back(0x00);

    command.push_back(0x00);
    command.push_back(0x00);

    command.push_back(counter);
    command.push_back(0x00);

    command.push_back(0x0d);


    for(int i=1; i<=27; i++)
        command[28] += command[i];


    counter += 1;
    if(counter > 255) counter = 0;

    return_cmd = command;

}

void sendrev::RevProcess_two_wheel_encoder(std::vector<unsigned char> &rev_buf,float &Rev_V, float &Rev_W)
{
    switch(mode)
    {
        case IVAM_Car_vw:
            RevProcess_IVAM_Car_vw(rev_buf, Rev_V, Rev_W);
            break;
        default:
            break;
    }
}

void sendrev::RevProcess_AllDir_encoder(std::vector<unsigned char> &rev_buf,float &Rev_FR_rpm,float &Rev_FL_rpm,float &Rev_RR_rpm,float &Rev_RL_rpm,float &Rev_FR_deg,float &Rev_FL_deg,float &Rev_RR_deg,float &Rev_RL_deg)
{
			switch(mode){
            case ChinaMotor:
                 RevProcess_ChinaMotor(rev_buf, Rev_FR_rpm, Rev_FL_rpm, Rev_RR_rpm, Rev_RL_rpm, Rev_FR_deg, Rev_FL_deg, Rev_RR_deg, Rev_RL_deg);
            break;
            default:
            break;
    }
}

void sendrev::RevProcess_IVAM_EV_encoder(std::vector<unsigned char> &rev_buf, int &RPM_L,int &RPM_R,int &Delta_now, float &V_cal, float &a_cal)
{
            switch(mode)
            {
                case IVAM_EV:
                    RevProcess_IVAM_EV(rev_buf, RPM_L, RPM_R, Delta_now, V_cal, a_cal);
                break;
                default:
                break;
            }
}

void sendrev::RevProcess_IVAM_EV(std::vector<unsigned char> &rev_buf,int &RPM_L,int &RPM_R,int &Delta_now, float &V_cal, float &a_cal)
{

    //for(int i =0;i<rev_buf.size();i++)
    //{
    //     printf("%x \n",rev_buf[i]);
    //}

    if(rev_buf[0] == 100 && rev_buf[7] == 101)
    {
        static float V_last = 0.0;
        unsigned char rpm_lh = rev_buf[1];
        unsigned char rpm_ll = rev_buf[2];
        unsigned char rpm_rh = rev_buf[3];
        unsigned char rpm_rl = rev_buf[4];
        unsigned char delta_now_h = rev_buf[5];
        unsigned char delta_now_l = rev_buf[6];

        int DELTA_NOW = (delta_now_h * 255) + delta_now_l;
        DELTA_NOW = DELTA_NOW - 9000;


        if(DELTA_NOW >= 28)
        {
            DELTA_NOW = 28;
        }
        else if(DELTA_NOW <= -28)
        {
            DELTA_NOW = -28;
        }

        Delta_now = DELTA_NOW;
        //Delta_now = 0;//for testing


        RPM_L = (rpm_lh * 255) + rpm_ll;
        RPM_R = (rpm_rh * 255) + rpm_rl;
        float RPM_avg;

        RPM_avg = (RPM_L + RPM_R) / 2;

        //RPM_avg = 0;



        V_cal = (RPM_avg * M_PI * 0.55)/(60 * 10); //10 ----> Reduction Ratio

        a_cal = (V_cal - V_last) * 10; //send packet every 0.1sec *10 ->  = dt

        V_last = V_cal;


        //std::cout<<"V_cal: "<< V_cal <<" , a_cal: "<< a_cal <<" , Delta_now: "<< Delta_now <<std::endl;
        //std::cout<<"V_last: "<< V_last << " , V_cal: "<< V_cal<<" , a_cal: "<< a_cal << std::endl;
        rev_buf.clear();

    }
}

void sendrev::Package_Boling_smallgray(double &cmd_vl, double &cmd_vr,std::vector<unsigned char> &return_cmd)
{


    double send_vl = -1*cmd_vl + 20.0;
    double send_vr = -1*cmd_vr + 20.0;

    std::vector<unsigned char> command;

    std::cout<<"send_vl: "<<send_vl<<std::endl; // print info
    std::cout<<"send_vr: "<<send_vr<<std::endl;



    int integer_vl = (int(send_vl));
    int float_vl = ( int((send_vl - double(integer_vl))*1000 ));
    int integer_vr = (int(send_vr));
    int float_vr = ( int((send_vr - double(integer_vr))*1000 ));


    int HighByte_integer_vl = integer_vl/128;
    int LowByte_integer_vl  = integer_vl%128;
    int HighByte_float_vl   = float_vl/128;
    int LowByte_float_vl    = float_vl%128;

    int HighByte_integer_vr = integer_vr/128;
    int LowByte_integer_vr  = integer_vr%128;
    int HighByte_float_vr   = float_vr/128;
    int LowByte_float_vr    = float_vr%128;


    command.push_back('S');
    command.push_back('A');
    command.push_back(HighByte_integer_vr);
    command.push_back(LowByte_integer_vr);
    command.push_back(HighByte_float_vr);
    command.push_back(LowByte_float_vr);
    command.push_back(HighByte_integer_vl);
    command.push_back(LowByte_integer_vl);
    command.push_back(HighByte_float_vl);
    command.push_back(LowByte_float_vl);
    command.push_back('E');
    command.push_back('N');
    command.push_back('D');



    return_cmd = command;



}

void sendrev::RevProcess_ChinaMotor(std::vector<unsigned char> &rev_buf,float &Rev_FR_rpm,float &Rev_FL_rpm,float &Rev_RR_rpm,float &Rev_RL_rpm,float &Rev_FR_deg,float &Rev_FL_deg,float &Rev_RR_deg,float &Rev_RL_deg)
{


			// for(int i =0;i<rev_buf.size();i++)
			// {

			// 	printf("%x \n",rev_buf[i]);
			// }
					if(rev_buf[0] == 0x0A && rev_buf[26] ==0x00  && rev_buf[29] == 0x0D){


						unsigned char FR_RPM_L = rev_buf[1] + 0x00;
						unsigned char FR_RPM_H = rev_buf[2] + 0x00;

						unsigned char FR_DEG_L = rev_buf[3] + 0x00;
						unsigned char FR_DEG_H = rev_buf[4] + 0x00;

						unsigned char FL_RPM_L = rev_buf[5] + 0x00;
						unsigned char FL_RPM_H = rev_buf[6] + 0x00;

						unsigned char FL_DEG_L = rev_buf[7] + 0x00;
						unsigned char FL_DEG_H = rev_buf[8] + 0x00;

						unsigned char RL_RPM_L = rev_buf[9] + 0x00;
						unsigned char RL_RPM_H = rev_buf[10] + 0x00;

						unsigned char RL_DEG_L = rev_buf[11] + 0x00;
						unsigned char RL_DEG_H = rev_buf[12] + 0x00;

						unsigned char RR_RPM_L = rev_buf[13] + 0x00;
						unsigned char RR_RPM_H = rev_buf[14] + 0x00;

						unsigned char RR_DEG_L = rev_buf[15] + 0x00;
						unsigned char RR_DEG_H = rev_buf[16] + 0x00;

						unsigned char check_byte = rev_buf[28] + 0x00;


						unsigned char sum_of_byte = 0;
						for(int j=1; j <= 27; j++){
							unsigned char byte_buf = rev_buf[j] + 0x00;
							sum_of_byte += byte_buf;
						}

						sum_of_byte = sum_of_byte%256;


						float FR_RPM = float(int(FR_RPM_H)*256 + int(FR_RPM_L));
						float FL_RPM = float(int(FL_RPM_H)*256 + int(FL_RPM_L));
						float RR_RPM = float(int(RR_RPM_H)*256 + int(RR_RPM_L));
						float RL_RPM = float(int(RL_RPM_H)*256 + int(RL_RPM_L));

						float FR_DEG = float(int(FR_DEG_H)*256 + int(FR_DEG_L));
						float FL_DEG = float(int(FL_DEG_H)*256 + int(FL_DEG_L));
						float RR_DEG = float(int(RR_DEG_H)*256 + int(RR_DEG_L));
						float RL_DEG = float(int(RL_DEG_H)*256 + int(RL_DEG_L));

						if(FR_RPM >= 32768.0)  FR_RPM = FR_RPM - 65536;
						if(FL_RPM >= 32768.0)  FL_RPM = FL_RPM - 65536;
						if(RR_RPM >= 32768.0)  RR_RPM = RR_RPM - 65536;
						if(RL_RPM >= 32768.0)  RL_RPM = RL_RPM - 65536;


						if(FR_DEG >= 32768.0)  FR_DEG = FR_DEG - 65536.0;
						if(FL_DEG >= 32768.0)    FL_DEG = FL_DEG - 65536.0;
						if(RR_DEG >= 32768.0)    RR_DEG = RR_DEG - 65536.0;
						if(RL_DEG >= 32768.0)    RL_DEG = RL_DEG - 65536.0;


						FR_RPM = FR_RPM/10;
						FL_RPM = FL_RPM/10;
						RR_RPM = RR_RPM/10;
						RL_RPM = RL_RPM/10;


						FR_DEG = FR_DEG/10;
						FL_DEG = FL_DEG/10;
						RR_DEG = RR_DEG/10;
						RL_DEG = RL_DEG/10;

                        //std::cout<<"FR_DEG: "<<FR_DEG<<std::endl;



						if(sum_of_byte == check_byte){

							Rev_FR_rpm = FR_RPM;
							Rev_FL_rpm = FL_RPM;
							Rev_RR_rpm = RR_RPM;
							Rev_RL_rpm = RL_RPM;



							Rev_FR_deg = FR_DEG;
							Rev_FL_deg = FL_DEG;
							Rev_RR_deg = RR_DEG;
							Rev_RL_deg = RL_DEG;

							// std::cout<<"========================================="<<std::endl;
							// std::cout<<"FR_RPM: "<<FR_RPM<<std::endl;
							// std::cout<<"FL_RPM: "<<FL_RPM<<std::endl;
							// std::cout<<"RR_RPM: "<<RR_RPM<<std::endl;
							// std::cout<<"RL_RPM: "<<RL_RPM<<std::endl;
							// std::cout<<"-----------------------------------------"<<std::endl;
							//std::cout<<"FR_DEG: "<<FR_DEG * M_PI /180.0<<std::endl;
							// std::cout<<"FL_DEG: "<<FL_DEG * M_PI /180.0<<std::endl;
							// std::cout<<"RR_DEG: "<<RR_DEG<<std::endl;
							// std::cout<<"RL_DEG: "<<RL_DEG<<std::endl;
							// std::cout<<"-----------------------------------------"<<std::endl;
							// std::cout<<"check_byte: "<<check_byte<<std::endl;
							// std::cout<<"sum_of_byte: "<<sum_of_byte<<std::endl;
						}
						rev_buf.clear();
                        //std::cout<<"========rev_buf.size()======== "<<rev_buf.size()<<std::endl;
					}
                    else if(rev_buf[0] == 0x0A && rev_buf[26] == 0x01  && rev_buf[29] == 0x0D){

                        unsigned char Voltage1_buf = rev_buf[1] + 0x00;
						unsigned char Voltage2_buf = rev_buf[2] + 0x00;
						unsigned char Voltage3_buf = rev_buf[3] + 0x00;
						unsigned char Voltage4_buf = rev_buf[4] + 0x00;

						unsigned char Current1_buf = rev_buf[5] + 0x00;
						unsigned char Current2_buf = rev_buf[6] + 0x00;
						unsigned char Current3_buf = rev_buf[7] + 0x00;
						unsigned char Current4_buf = rev_buf[8] + 0x00;

						unsigned char RelativeSOC1_buf = rev_buf[9] + 0x00;
						unsigned char RelativeSOC2_buf = rev_buf[10] + 0x00;
						unsigned char RelativeSOC3_buf = rev_buf[11] + 0x00;
						unsigned char RelativeSOC4_buf = rev_buf[12] + 0x00;

						unsigned char AbsoluteSOC1_buf = rev_buf[13] + 0x00;
						unsigned char AbsoluteSOC2_buf = rev_buf[14] + 0x00;
						unsigned char AbsoluteSOC3_buf = rev_buf[15] + 0x00;
						unsigned char AbsoluteSOC4_buf = rev_buf[16] + 0x00;

                        unsigned char Temp1_buf = rev_buf[17] + 0x00;
                        unsigned char Temp2_buf = rev_buf[18] + 0x00;
                        unsigned char Temp3_buf = rev_buf[19] + 0x00;
                        unsigned char Temp4_buf = rev_buf[20] + 0x00;


						unsigned char check_byte = rev_buf[28] + 0x00;


						unsigned char sum_of_byte = 0;
						for(int j=1; j <= 27; j++){
							unsigned char byte_buf = rev_buf[j] + 0x00;
							sum_of_byte += byte_buf;
						}

						sum_of_byte = sum_of_byte%256;


						if(sum_of_byte == check_byte){

                            float Voltage1 = float(Voltage1_buf % 256);
                            float Voltage2 = float(Voltage2_buf % 256);
                            float Voltage3 = float(Voltage3_buf % 256);
                            float Voltage4 = float(Voltage4_buf % 256);

                            float Current1 = float(Current1_buf % 256 );
                            float Current2 = float(Current2_buf % 256 );
                            float Current3 = float(Current3_buf % 256 );
                            float Current4 = float(Current4_buf % 256 );

                            float RelativeSOC1 = float(RelativeSOC1_buf % 256);
                            float RelativeSOC2 = float(RelativeSOC2_buf % 256);
                            float RelativeSOC3 = float(RelativeSOC3_buf % 256);
                            float RelativeSOC4 = float(RelativeSOC4_buf % 256);

                            float AbsoluteSOC1 = float(AbsoluteSOC1_buf % 256);
                            float AbsoluteSOC2 = float(AbsoluteSOC2_buf % 256);
                            float AbsoluteSOC3 = float(AbsoluteSOC3_buf % 256);
                            float AbsoluteSOC4 = float(AbsoluteSOC4_buf % 256);

                            float Temp1 = float(Temp1_buf % 256);
                            float Temp2 = float(Temp2_buf % 256);
                            float Temp3 = float(Temp3_buf % 256);
                            float Temp4 = float(Temp4_buf % 256);

                            //有負的話
                            if(Current1 >128){Current1 = Current1 - 256;}
                            if(Current2 >128){Current2 = Current2 - 256;}
                            if(Current3 >128){Current3 = Current3 - 256;}
                            if(Current4 >128){Current4 = Current4 - 256;}



                            if(Temp1 >128){Temp1 = Temp1 - 256;}
                            if(Temp2 >128){Temp2 = Temp2 - 256;}
                            if(Temp3 >128){Temp3 = Temp3 - 256;}
                            if(Temp4 >128){Temp4 = Temp4 - 256;}


                            float Voltage = (Voltage1 + Voltage2 + Voltage3 + Voltage4)/10.0;
                            float Current = (Current1 + Current2 + Current3 + Current4)/4.0;

                            move_robot::Battery msg;
                            msg.Voltage = Voltage;
                            msg.Current = Current;
                            msg.RelativeSOC1 = RelativeSOC1;
                            msg.RelativeSOC2 = RelativeSOC2;
                            msg.RelativeSOC3 = RelativeSOC3;
                            msg.RelativeSOC4 = RelativeSOC4;
                            msg.AbsoluteSOC1 = AbsoluteSOC1;
                            msg.AbsoluteSOC2 = AbsoluteSOC2;
                            msg.AbsoluteSOC3 = AbsoluteSOC3;
                            msg.AbsoluteSOC4 = AbsoluteSOC4;
                            msg.Temp1 = Temp1;
                            msg.Temp2 = Temp2;
                            msg.Temp3 = Temp3;
                            msg.Temp4 = Temp4;

                            Battery_Publisher_.publish(msg);



                            //電壓 除10倍才是原值

                            // std::cout<<"========================================="<<std::endl;
                            // std::cout<<"Voltage1: "<<Voltage1 /10.0<<std::endl;
                            // std::cout<<"Voltage2: "<<Voltage2 /10.0<<std::endl;
                            // std::cout<<"Voltage3: "<<Voltage3 /10.0<<std::endl;
                            // std::cout<<"Voltage4: "<<Voltage4 /10.0<<std::endl;

                            // std::cout<<"Current1: "<< Current1<<std::endl;
                            // std::cout<<"Current2: "<< Current2<<std::endl;
                            // std::cout<<"Current3: "<< Current3<<std::endl;
                            // std::cout<<"Current4: "<< Current4<<std::endl;

                            // std::cout<<"RelativeSOC1: "<<RelativeSOC1<<std::endl;
                            // std::cout<<"RelativeSOC2: "<<RelativeSOC2<<std::endl;
                            // std::cout<<"RelativeSOC3: "<<RelativeSOC3<<std::endl;
                            // std::cout<<"RelativeSOC4: "<<RelativeSOC4<<std::endl;

                            // std::cout<<"AbsoluteSOC1: "<<AbsoluteSOC1<<std::endl;
                            // std::cout<<"AbsoluteSOC2: "<<AbsoluteSOC2<<std::endl;
                            // std::cout<<"AbsoluteSOC3: "<<AbsoluteSOC3<<std::endl;
                            // std::cout<<"AbsoluteSOC4: "<<AbsoluteSOC4<<std::endl;

                            // std::cout<<"Temp1: "<<Temp1<<std::endl;
                            // std::cout<<"Temp2: "<<Temp2<<std::endl;
                            // std::cout<<"Temp3: "<<Temp3<<std::endl;
                            // std::cout<<"Temp4: "<<Temp4<<std::endl;


						}
						rev_buf.clear();

                    }



}

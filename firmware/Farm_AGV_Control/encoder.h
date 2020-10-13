#include <math.h>
#include <TimerOne.h>


///----------pin--------------
#define Pin_CLK1 2
#define Pin_DATA1 4

///-----------Offset-----------
#define offset_theta 238

///-----------frequency--------
#define cycle_time 20000

///-----------SSI bits---------
#define frameSize 16


float getAngle(bool EncoderData[frameSize - 2]);
float AngleCorrection(float data, float offset);
float getAngle_ori(bool EncoderData[frameSize - 2]);
float getPosition();

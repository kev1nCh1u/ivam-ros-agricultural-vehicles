#include "encoder.h"
#include "Arduino.h"
bool EncoderData[frameSize];
bool NewEncoderData[frameSize - 2];
float Theta = 0;

float getAngle(bool EncoderData[frameSize - 2])
{
  float Data = 0.0;

  Data =   EncoderData[12]          + EncoderData[11] *  2.0   + EncoderData[10] *   4.0 + EncoderData[9] *   8.0 + EncoderData[8] *  16.0
           + EncoderData[7]  * 32.0   + EncoderData[6]  * 64.0   + EncoderData[5]  * 128.0 + EncoderData[4] * 256.0 + EncoderData[3] * 512.0
           + EncoderData[2]  * 1024.0 + EncoderData[1]  * 2048.0;

  Data = float(Data * 180.0 / 4096.0);
  Data = Data + 180.0 * EncoderData[0];

  return  Data;
}

float getAngle_ori(bool EncoderData[frameSize - 2]) {
  float angle = 0;
  for (int j = 1; j < frameSize - 3; j++) {
    angle = angle + pow(2, (12 - j)) * EncoderData[j];
  }

  angle = angle + 4096 * EncoderData[0];
  return  angle;
}

float AngleCorrection(float data, float offset)
{
  float angle = data + (360 - offset); // to create "0/360 degree" to be initial position

  if (angle >= 360) angle = data - offset;

  float correct_angle = angle + 180;   // to create "180 degree" to be initial position

  if (correct_angle >= 360) correct_angle = angle - 180;

  return correct_angle;
}

float getPosition()
{

  for (int j = 0; j < frameSize; j++)       
  {
    EncoderData[j] = 0;
  }
  delayMicroseconds(1);
  
  for (int j = 0; j < frameSize; j++)
  {
    digitalWrite(Pin_CLK1, LOW);
    delayMicroseconds(1);  //delayMicroseconds(3) fits a 10us tictac
    EncoderData[j] = digitalRead(Pin_DATA1);
    delayMicroseconds(1);
    digitalWrite(Pin_CLK1, HIGH);
    delayMicroseconds(1);
  }

  for (int j = 1; j < frameSize - 1; j++)
  {
    NewEncoderData[j - 1] = EncoderData[j];
  }
  Theta = AngleCorrection(getAngle(NewEncoderData), offset_theta);
  delayMicroseconds(1);

  return Theta;
 
}

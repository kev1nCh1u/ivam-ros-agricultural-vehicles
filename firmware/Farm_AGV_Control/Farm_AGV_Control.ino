 #include <math.h>
#include <TimerOne.h>
#include "encoder.h"

//Throttle and Steering Define
#define Accelerator 6
#define Speed_Out 5
#define Steering_PWM_R 11
#define Steering_PWM_L 3

//Brake
int D4 = 12;
int D5 = 10;
int D6 = 7;
int D7 = 8;
int lock = 0;

//power
double Power_Speed = 0.0;
double Brake_Power = 0.0;
int Power_Output = 0;

//Steering
double steering_angle = 0.0;

//data buf
int Man_Acc = 0;
int Man_Out = 0;

//Serial Receive Data
byte Data[150];
int Data_point = 0;
int DriveMode = 0;
int Manual_mode = 1;
int Brake_Sign = 0;
bool Brake_limit_flag_h = false;
bool Brake_limit_flag_r = false;
int Brake_release_cnt = 0;

//Throttle PID Parameter
bool Auto_Init = true;
int Auto_Init_Count = 0;
double kp_throttle = 3.5;
double ki_throttle = 0.0;
double kd_throttle = 0.1;
double Error = 0.0, Pre_Error = 0.0, Constant = 0.0, Output = 0.0;

//function
void PID_Parameter_Init(void);
void Count_InterruptR(void);
void Count_InterruptL(void);
void PID(void);
void Mode_Change(void);
void Show_Car_Information(void);
void Brake_Control(void);

//PID constants (PID參數)
float kp = 1.0; // First
float ki = 0.0000005; // Third
float kd = 0.06; //Second

float angle;
unsigned long currentTime, previousTime;
float elapsedTime;
float error;
float lastError;
float input, output, setPoint;
float cumError, rateError;
bool verse;

float tmp = 0;

void setup() {

  //--------Encoder Pin Setup------------
  pinMode(Pin_CLK1, OUTPUT);
  pinMode(Pin_DATA1, INPUT);


  //------------Timer1 Setup-------------
  Serial.begin(115200); //115200
  delay(100);
  Timer1.initialize(20000);
  Timer1.attachInterrupt(Mode_Change);

  ///----------PWM(timer2) setup----------

  pinMode(3, OUTPUT); //PWM PIN
  pinMode(11, OUTPUT);
  TCCR2B = TCCR2B & 0xF8 | 2;

  ///----------PID setup-----------------
  setPoint = 53; //以180度為正前方
  //角度offset在encoder.h的offset_theta

  //-------------------------------------------------------------

  ///----------Throuttle Setup-----------
  pinMode(Accelerator, INPUT);
  pinMode(Speed_Out, OUTPUT);

  //-------------Brake Setup----------------
  pinMode(D4, INPUT_PULLUP);
  pinMode(D5, INPUT_PULLUP);

  //正反轉
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);

  digitalWrite(D6, HIGH);
  digitalWrite(D7, HIGH);

  PID_Parameter_Init();
}

void loop() {
  
  int K1_state = digitalRead(D5);//pin 10
  int K2_state = digitalRead(D4);//pin 12

  if (K1_state == 0)//hold brake limit
  {
    Serial.println("K1_LOW");
    digitalWrite(D6, HIGH);
    lock = 0;
    Brake_limit_flag_h = true;
    Brake_limit_flag_r == false;
  }

  if (K2_state == 0)//release brake limit
  {
    Serial.println("K2_LOW");
    digitalWrite(D7, HIGH);
    lock = 0;
    Brake_limit_flag_r = true;
    Brake_limit_flag_h == false;
  }
}

void Brake_Control() {
  if (Brake_Sign == 1 && lock == 0 && Brake_limit_flag_h == false && Brake_limit_flag_r == false) //brake
  {
    digitalWrite(D6, LOW);
    digitalWrite(D7, HIGH);

    lock = 1;
  }
  else if (Brake_Sign == 0 && lock == 1 && Brake_limit_flag_h == false && Brake_limit_flag_r == false) //release
  {
    digitalWrite(D7, LOW);
    digitalWrite(D6, HIGH);

    lock = 0;
  }
  else if (Brake_Sign == 1 && lock == 0 && Brake_limit_flag_h == true && Brake_limit_flag_r == false)//hit hold limit
  {
    digitalWrite(D7, HIGH);
    digitalWrite(D6, HIGH);

  }
  else if (Brake_Sign == 0 && lock == 0 && Brake_limit_flag_h == true && Brake_limit_flag_r == false)//release while hit hold limit
  {
    digitalWrite(D7, LOW);
    digitalWrite(D6, HIGH);

    Brake_limit_flag_h = false;
    lock = 1;
  }
  else if (Brake_Sign == 0 && lock == 0 && Brake_limit_flag_r == true && Brake_limit_flag_h == false)//hit release limit
  {
    digitalWrite(D6, HIGH);
    digitalWrite(D7, HIGH);
  }
  else if (Brake_Sign == 1 && lock == 0 && Brake_limit_flag_r == true && Brake_limit_flag_h == false)//release while hit release limit
  {
    digitalWrite(D6, LOW);
    digitalWrite(D7, HIGH);

    Brake_limit_flag_r = false;
    lock = 0;
  }

}

void Mode_Change()
{
  if (DriveMode == 0) {
    Man_Acc = analogRead(Accelerator);
    Man_Out = map(Man_Acc, 245, 1023, 0, 255);
    analogWrite(Speed_Out, Power_Output);
    computePID();
  }
  else if (DriveMode == 1) {
    if (angle != setPoint && angle < setPoint) { // what is rotate_now ?? lennart
      computePID();
      //setPoint++;
    } else if (angle != setPoint && angle > setPoint) {
      computePID();
      //setPoint--;
    }
  }
  if (Power_Output > 255)Power_Output = 255;
  else if (Power_Output < 0)Power_Output = 0;

  analogWrite(Speed_Out, Power_Output);
  //  Man_Acc = analogRead(Accelerator);Man_Out = map(Man_Acc, 245, 1023, 0, 255);analogWrite(Speed_Out, Man_Out);
}
void serialEvent()
{
  Data[Data_point] =  Serial.read();
  if (Data_point >= 149) Data_point = 0;
  if (Data_point >= 9) { //  Lennart: from 8 to 10
    if ((Data[Data_point] == 254) && (Data[Data_point - 9] == 255)) {
      DriveMode = Data[Data_point - 8];//模式(0:手動,1:自動) Lenna
      Manual_mode = Data[Data_point - 7];//檔位(0:R,1:N,2:D) Le
      Brake_Sign = Data[Data_point - 6];
      if (DriveMode == 1) {//自動模式 需要解前輪轉向以及後輪速度
        int rotate_data_h = Data[Data_point - 4];
        int rotate_data_l = Data[Data_point - 3];
        setPoint = ((rotate_data_h * 255 + rotate_data_l) - 9000) / 100.0 / 3.0; //+:右轉,-:左轉
        setPoint = setPoint + 53;
        int sp = (Data[Data_point - 2] * 255 + Data[Data_point - 1]);//rps
        Power_Output = map(sp, 0, 600, 0, 150);
      }
      else if (DriveMode == 1 && Brake_Sign == 1)
      {
        int rotate_data_h = Data[Data_point - 4];
        int rotate_data_l = Data[Data_point - 3];
        setPoint = ((rotate_data_h * 255 + rotate_data_l) - 9000) / 100.0 / 3.0; //+:右轉,-:左轉
        setPoint = setPoint + 53; 
        Power_Output = 0;
        

      }
      else { //手動模式
        PID_Parameter_Init();
        steering_angle = 0.0;
        setPoint = 53.0;
        //Power_Speed = 0;
      }
      Brake_Control();
      Data_point = 0;
    }
  }
  Data_point++;
}

void PID_Parameter_Init() {
  Auto_Init = true;
  Error = 0.0; Pre_Error = 0.0;
  Output = 0.0; Power_Output = 0;
  Constant = 0.0;
}

void computePID() {

  float angle = getPosition();  //inp 0~360
  Serial.print("angle:");
  Serial.println(angle);
  currentTime = millis();
  elapsedTime = (float)(currentTime - previousTime);        //compute time elapsed from previous computation

  error = abs(setPoint - angle);                 // determine error
  cumError += error * elapsedTime;               // compute integral
  rateError = (error - lastError) / elapsedTime; // compute derivative
  lastError = error;                             // remember current error
  previousTime = currentTime;                    // remember current time

  float out = kp * error + ki * cumError + kd * rateError;  //PID output
  Serial.print("out :");
  Serial.println(out);  
  steering_angle = map(out, 0, 31, 0, 245); //馬達速度5~40 //out*10;

  if(steering_angle >= 245)
  {
    steering_angle = 245;
  }
  Serial.print("steering_angle :");
  Serial.println(steering_angle);
  //右極限74.8-左極限11.8 = 63.0
  //07/10 左減右 78-10 =68.8
  //07/23 左15/右75/中間53
  if (angle < 15)
  {
    steering_angle = 0;
  }
  if (angle > 75)  {
    steering_angle = 0;
  }

  if (angle < setPoint) { //Turn Right
    analogWrite(Steering_PWM_R, steering_angle);
    analogWrite(Steering_PWM_L, 0);

    //Serial.print("L_Out: ");
  }
  if (angle > setPoint) { //Turn Left
    analogWrite(Steering_PWM_R, 0);
    analogWrite(Steering_PWM_L, steering_angle);

    //Serial.print("R_Out: ");
  }

  //  Serial.println(steering_angle);



}

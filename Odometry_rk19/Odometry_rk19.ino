/*********************************************  Header Files ************************************************************************************************/
#include <DueTimer.h>
#include <DuePWM.h>

/*********************************************************************************************************************************************/
/********************************************* Serial Print ************************************************************************************************/
#define printXY 0
#define printLineAngleSpeed 0
#define printCircleAngleSpeed 0
#define printPIDOutput 0
#define TimedCircle

/*********************************************************************************************************************************************/
/*********************************************  Functions and variables definiton ************************************************************************************************/

#define TimerEncoder Timer1
#define EncoderTime 0.1 
#define Omega_Timer 1/EncoderTime
#define GearRatio 1.33
#define maxWheelRPM 300
#define maxMotRPM 468
#define maxPWM 255
#define pi 3.141592
#define root3by2 0.86602
#define onebyroot2 0.7071
#define DegreeToRadian(x) x*0.0174532
#define RadianToDegree(x) x*57.295779
#define PWM_FREQ1 2500
DuePWM pwm(PWM_FREQ1, 3000);

#define Angle1 90   //1.57079  //90       //90+0
#define Angle2 210  //3.66519  //210      //90+120
#define Angle3 330  //5.75958  //330      //90+240

float temp[4];
float output[4];
/*********************************************************************************************************************************************/
/*********************************************   Encoders ************************************************************************************************/
class Encoder
{
  public:
  int channel1;
  int channel2;
  int ppr;
  volatile long long int Count;
  volatile long long int prevCount;
  int rpm;

  void initEncoder(){
    pinMode(channel1,INPUT);
    pinMode(channel2,INPUT);
   }
};

  Encoder encoder1={41,43,135,0,0,0} , *pEncoder1=&encoder1;
  Encoder encoder2={45,47,135,0,0,0} , *pEncoder2=&encoder2;
  Encoder encoder3={49,51,135,0,0,0} , *pEncoder3=&encoder3;
  
  Encoder *pEncoder[3]={&encoder1,&encoder2,&encoder3};
  
  Encoder xencoder={/*52,50*/18,15,2000,0,0,0};
  Encoder *pEncoderX=&xencoder;
  
  Encoder yencoder={/*48,46*/14,5,2000,0,0,0};
  Encoder *pEncoderY=&yencoder;
  
  void returnCount1();
  void returnCount2();
  void returnCount3();
 
  void returnCountX();
  void returnCountY();
 
  
 /*********************************************************************************************************************************************/
 /***************************************************      Motors          ******************************************************************************************/
class Motor{
  public:
  int direction1;
  int pwmPin;
  int direction2;
  void initMotor(){
    pinMode(direction1,OUTPUT);
    pinMode(pwmPin,OUTPUT);
    pinMode(direction2,OUTPUT);
  }
  void driveMotor(float op,float maxvalue);
};

  Motor motor1={29,8,23};
  Motor *pMotor1=&motor1;

  Motor motor2={25,7,27};
  Motor *pMotor2=&motor2;

  Motor motor3={39,6,37};
  Motor *pMotor3=&motor3;
  
  Motor *pMotor[3]={&motor1,&motor2,&motor3};
   
 /*********************************************************************************************************************************************/
 /******************************************************       PID          ***************************************************************************************/
class PID{
   float Kp;
   float Kd;
   float Ki;
   float maxControl;
   float minControl;
  public:
   float required;
   float prevRequired;
   float error;
   float prevError;
   float derivativeError;
   float integralError;
   void initPID(float kp,float kd,float ki,float req,float minV,float maxV);
   float pidControl(float actual);
  
};
 
  PID PIDMotor1;
  PID *pPIDMotor1=&PIDMotor1;

  PID PIDMotor2;
  PID *pPIDMotor2=&PIDMotor2;

  PID PIDMotor3;
  PID *pPIDMotor3=&PIDMotor3;

  PID *pPIDMotor[3]={&PIDMotor1,&PIDMotor2,&PIDMotor3};
 
/*********************************************************************************************************************************************/
/*********************************************************** Wheels **********************************************************************************/
class Wheel{
  public:
  float translationRPM; 
  float angularRPM;
  float Radius; 
  int angle; 
  float maxRPM; 
  float rpm;
  int prevRPM;
  float distance;
  float prevDistance;
};

 float CastorWheelXradius=0.05;
 float CastorWheelYradius=0.05;
 
 Wheel Wheel1 = {0.0, 0.0, 0.1, Angle1, maxWheelRPM, 0, 0, 0.0, 0.0};
 
 Wheel Wheel2 = {0.0, 0.0, 0.1, Angle2, maxWheelRPM, 0, 0, 0.0, 0.0};
 
 Wheel Wheel3 = {0.0, 0.0, 0.1, Angle3, maxWheelRPM, 0, 0, 0.0, 0.0};

 Wheel *pWheel[3]={&Wheel1,&Wheel2,&Wheel3};
 
 /*********************************************************************************************************************************************/
/*********************************************************** Autonomous Bot **********************************************************************************/
class Auto_Bot{
  public:
  float X_pos;
  float Y_pos;
  float Angle;
  float X_vel;
  float Y_vel;
  float vel;
  Auto_Bot(){
             X_pos = 0;
             Y_pos = 0;
             Angle = 0;
             X_vel = 0;
             Y_vel = 0;
             vel = 0;
            }
};

  Auto_Bot ThreeWheelDrive; 
  Auto_Bot *pBot=&ThreeWheelDrive;

  PID pidX;
  PID *ppidX = &pidX;
  PID pidY;
  PID *ppidY = &pidY;
  PID pid_theta;
  PID *ppid_theta = &pid_theta;

/*********************************************************************************************************************************************/
float Circle_theta=0;

void setup() {
  Serial.begin(9600);
 
  for(int i=0;i<3;++i)
  pEncoder[i]->initEncoder();
  
  attachInterrupt(pEncoder1->channel1,returnCount1,RISING);   
  attachInterrupt(pEncoder2->channel1,returnCount2,RISING);
  attachInterrupt(pEncoder3->channel1,returnCount3,RISING);

  for(int i=0;i<3;++i)
  pMotor[i]->initMotor();

  pwm.setFreq1(PWM_FREQ1);
 
  for(int i=0;i<3;++i)
  pwm.pinFreq1(pMotor[i]->pwmPin);
  
  pPIDMotor1->initPID(0.3,0.01,0.118,0,-maxMotRPM,maxMotRPM);
  pPIDMotor2->initPID(0.3,0.01,0.118,0,-maxMotRPM,maxMotRPM);		
  pPIDMotor3->initPID(0.3,0.01,0.118,0,-maxMotRPM,maxMotRPM);
  
  pEncoderX->initEncoder();
  attachInterrupt(pEncoderX->channel1,returnCountX,RISING);

  pEncoderY->initEncoder();
  attachInterrupt(pEncoderY->channel1,returnCountY,RISING);
  
  TimerEncoder.attachInterrupt(timerHandler);
  TimerEncoder.start( 1000000 * EncoderTime );
  
}


void loop() {

}

void timerHandler()
{
  getactualRPM(pEncoder);
  setOutput(pEncoder, pPIDMotor, pMotor,maxMotRPM/2);
  #ifdef TimedCircle
  CircleWithTime(3.3, Omega_Timer, millis());  
  #endif
}

void returnCount1()
  {
    if(digitalRead(pEncoder1->channel2))
    pEncoder1->Count++;
    else 
    pEncoder1->Count--;
  }

void returnCount2()
  {
    if(digitalRead(pEncoder2->channel2))
    pEncoder2->Count++;
    else 
    pEncoder2->Count--;
  }

void returnCount3()
  {
    if(digitalRead(pEncoder3->channel2))
    pEncoder3->Count--;
    else 
    pEncoder3->Count++;
  }

  void returnCountX()
  {
    if(digitalRead(pEncoderX->channel2))
    pEncoderX->Count++;
    else 
    pEncoderX->Count--;
  }
  
  void returnCountY()
  {
    if(digitalRead(pEncoderY->channel2))
    pEncoderY->Count--;
    else 
    pEncoderY->Count++;
  }


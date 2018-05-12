/*********************************************  Header Files ************************************************************************************************/
#include <DueTimer.h>
#include <DuePWM.h>

/*********************************************************************************************************************************************/
/*********************************************  Functions and variables definiton ************************************************************************************************/

#define TimerEncoder Timer1
#define EncoderTime 0.1 
#define EncoderGearRatio 0.89
#define MAXRPM 400
#define maxPWM 255
#define PWM_FREQ1 2500
#define pi 3.141592
#define root3by2 0.86602
#define onebyroot2 0.7071
#define DegreeToRadian(x) x*0.0174532
#define RadianToDegree(x) x*57.295779

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
  long int Count;
  long long int prevCount;
  int rpm;

  void initEncoder(){
    pinMode(channel1,INPUT);
    pinMode(channel2,INPUT);
   }
};

  Encoder encoder1={41,43,135,0,0,0};
  Encoder *pEncoder1=&encoder1;

  Encoder encoder2={45,47,135,0,0,0};
  Encoder *pEncoder2=&encoder2;

  Encoder encoder3={49,51,135,0,0,0};
  Encoder *pEncoder3=&encoder3;

  Encoder xencoder={52,50,2000,0,0,0};
  Encoder *pEncoderX=&xencoder;
  
  Encoder yencoder={48,46,2000,0,0,0};
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

  Motor motor1={29,3,23};
  Motor *pMotor1=&motor1;

  Motor motor2={25,4,27};
  Motor *pMotor2=&motor2;

  Motor motor3={39,5,37};
  Motor *pMotor3=&motor3;

  
 /*********************************************************************************************************************************************/
 /******************************************************       PID          ***************************************************************************************/
class PID{
  public:
  float Kp;
  float Kd;
  float Ki;
  float required;
  float prevRequired;
  float maxControl;
  float minControl;
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
 
 Wheel Wheel1 = {0.0, 0.0, 0.1, Angle1, MAXRPM, 0, 0, 0.0, 0.0};
 
 Wheel Wheel2 = {0.0, 0.0, 0.1, Angle2, MAXRPM, 0, 0, 0.0, 0.0};
 
 Wheel Wheel3 = {0.0, 0.0, 0.1, Angle3, MAXRPM, 0, 0, 0.0, 0.0};

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
};
Auto_Bot ThreeWheelDrive={0,0,0,0,0};
Auto_Bot *pBot=&ThreeWheelDrive;

/*********************************************************************************************************************************************/
float Circle_theta=0;

void setup() {
  Serial.begin(9600);
 
  pEncoder1->initEncoder();
  pEncoder2->initEncoder();
  pEncoder3->initEncoder();

  attachInterrupt(pEncoder1->channel1,returnCount1,RISING);
  attachInterrupt(pEncoder2->channel1,returnCount2,RISING);
  attachInterrupt(pEncoder3->channel1,returnCount3,RISING);

  pMotor1->initMotor();
  pMotor2->initMotor();
  pMotor3->initMotor();
  
  pPIDMotor1->initPID(0.3,0.01,0.118,0,-MAXRPM,MAXRPM);
  pPIDMotor2->initPID(0.3,0.01,0.118,0,-MAXRPM,MAXRPM);
  pPIDMotor3->initPID(0.3,0.01,0.118,0,-MAXRPM,MAXRPM);

  pEncoderX->initEncoder();
  attachInterrupt(pEncoderX->channel1,returnCountX,RISING);

  pEncoderY->initEncoder();
  attachInterrupt(pEncoderY->channel1,returnCountY,RISING);
  
  TimerEncoder.attachInterrupt(timerHandler);
  TimerEncoder.start( 1000000 * EncoderTime );
  
}


void loop() {
 getBotPosition();
 Goto_XYSigmoid( 0,0,4,4,150 );
 //CircleLogic1(1.335,200);
 calculateRPM(0,pBot->Angle,pBot->vel);
}

void timerHandler()
{
  pEncoder1->rpm=((pEncoder1->Count - pEncoder1->prevCount) * 60.0)/(EncoderTime * EncoderGearRatio * pEncoder1->ppr);
  pEncoder2->rpm=((pEncoder2->Count - pEncoder2->prevCount) * 60.0)/(EncoderTime * EncoderGearRatio * pEncoder2->ppr);
  pEncoder3->rpm=((pEncoder3->Count - pEncoder3->prevCount) * 60.0)/(EncoderTime * EncoderGearRatio * pEncoder3->ppr);
  pEncoder1->prevCount=pEncoder1->Count;
  pEncoder2->prevCount=pEncoder2->Count;
  pEncoder3->prevCount=pEncoder3->Count;

  if(pPIDMotor1->required*pPIDMotor1->prevRequired>0)
  {
    temp[0]=pPIDMotor1->pidControl((pEncoder1->rpm));
    output[0]=temp[0];
  }
  else
  {
    pPIDMotor1->prevRequired=pPIDMotor1->required;
    output[0]=0;
  }
  if(pPIDMotor2->required*pPIDMotor2->prevRequired>0)
  {
    temp[1]=pPIDMotor2->pidControl((pEncoder2->rpm));
    output[1]=temp[1];
  }
  else
  {
    pPIDMotor2->prevRequired=pPIDMotor2->required;
    output[1]=0;
  }
  if(pPIDMotor3->required*pPIDMotor3->prevRequired>0)
  {
    temp[2]=pPIDMotor3->pidControl((pEncoder3->rpm));
    output[2]=temp[2];
  }
  else
  {
    pPIDMotor3->prevRequired=pPIDMotor3->required;
    output[2]=0;
  }

  if(pPIDMotor1->required==0) output[0]=0;
  if(pPIDMotor2->required==0) output[1]=0;
  if(pPIDMotor3->required==0) output[2]=0;


  pMotor1->driveMotor(output[0],MAXRPM/2);
  pMotor2->driveMotor(output[1],MAXRPM/2);
  pMotor3->driveMotor(output[2],MAXRPM/2);
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



/*********************************************  Header Files ************************************************************************************************/
#include <DueTimer.h>
#include <DuePWM.h>

/*********************************************************************************************************************************************/
/*********************************************  Functions and variables definiton ************************************************************************************************/
//#ifndef FourWheelDrive
//#define FourWheelDrive
//#endif

#ifndef PositionEncoders
#define PositionEncoders
#endif
  
#define TimerEncoder Timer1
#define EncoderTime 0.1 
#define EncoderGearRatio 0.75
#define MAXRPM 470
#define maxPWM 255
#define pi 3.141592
#define root3by2 0.86602
#define onebyroot2 0.7071
#define DegreeToRadian(x) x*0.0174532
#define RadianToDegree(x) x*57.295779

#ifdef FourWheelDrive   //Degree   
#define Angle1 0.78539  //45       //45+0
#define Angle2 2.13562  //135      //45+90
#define Angle3 3.92699  //225      //45+180
#define Angle4 5.49778  //315      //45+270

#else
#define Angle1 1.57079  //90       //90+0
#define Angle2 3.66519  //210      //90+120
#define Angle3 5.75958  //330      //90+240
#endif

/*********************************************************************************************************************************************/
/*********************************************   Encoders ************************************************************************************************/
class Encoder
{
  public:
  int channel1;
  int channel2;
  int ppr;
  long long int Count;
  long long int prevCount;
  int rpm;

  void initEncoder(){
    pinMode(channel1,INPUT);
    pinMode(channel2,INPUT);
   }
};

  Encoder encoder1={41,43,200,0,0,0};
  Encoder *pEncoder1=&encoder1;

  Encoder encoder2={45,47,200,0,0,0};
  Encoder *pEncoder2=&encoder2;

  Encoder encoder3={49,51,200,0,0,0};
  Encoder *pEncoder3=&encoder3;

  #ifdef FourWheelDrive
  Encoder encoder4={2,3,200,0,0,0};
  Encoder *pEncoder4=&encoder4;
  #endif

  #ifdef PositionEncoders
  Encoder xencoder={52,50,2048,0,0,0};
  Encoder *pEncoderX=&xencoder;
  
  Encoder yencoder={48,46,2048,0,0,0};
  Encoder *pEncoderY=&yencoder;
  #endif
  
  void returnCount1();
  void returnCount2();
  void returnCount3();
 
  #ifdef FourWheelDrive
  void returnCount4();
  #endif

  #ifdef PositionEncoders
  void returnCountX();
  void returnCountY();
  #endif
 
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

  #ifdef FourWheelDrive
  Motor motor4={1,2,3};
  Motor *pMotor4=&motor4;
  #endif
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
  void initPID(float kp,float kd,float ki,float req,float min,float max);
  float pidControl(float actual);
  
};
 
  
  
  PID PIDMotor1;
  PID *pPIDMotor1=&PIDMotor1;

  PID PIDMotor2;
  PID *pPIDMotor2=&PIDMotor2;

  PID PIDMotor3;
  PID *pPIDMotor3=&PIDMotor3;

  #ifdef FourWheelDrive
  PID PIDMotor4;
  PID *pPIDMotor4=&PIDMotor4;
  #endif
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
};

#ifdef PositionEncoders
 float CastorWheelXradius=0.05;
 float CastorWheelYradius=0.05;
#endif
 
 Wheel Wheel1 = {0.0, 0.0, 0.1, Angle1, MAXRPM, 0, 0, 0.0};
 
 Wheel Wheel2 = {0.0, 0.0, 0.1, Angle2, MAXRPM, 0, 0, 0.0};
 
 Wheel Wheel3 = {0.0, 0.0, 0.1, Angle3, MAXRPM, 0, 0, 0.0};

 #ifdef FourWheelDrive
 Wheel Wheel4 = {0, 0, Angle4, MAXRPM, 0, 0};
 Wheel *pWheel[4]={&Wheel1,&Wheel2,&Wheel3,&Wheel4};
 #else
 Wheel *pWheel[3]={&Wheel1,&Wheel2,&Wheel3};
 #endif

 
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

#ifdef FourWheelDrive
Auto_Bot FourWheelDrive;
Auto_Bot *pBot=&FourWheelDrive;
#else
Auto_Bot ThreeWheelDrive;
Auto_Bot *pBot=&ThreeWheelDrive;
#endif

/*********************************************************************************************************************************************/

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
  
  pPIDMotor1->initPID(0.65,0.1,0.1,0,-MAXRPM,MAXRPM);
  pPIDMotor2->initPID(0.65,0.0,0,0,0,255);
  pPIDMotor3->initPID(0.65,0.0,0,0,0,255);

  #ifdef FourWheelDrive
  pEncoder4->initEncoder();
  attachInterrupt(pEncoder4->channel1,returnCount4,RISING);
  pPIDMotor3->initPID(0,0,0,0,0,100);
  #endif

  #ifdef PositionEncoders
  pEncoderX->initEncoder();
  attachInterrupt(pEncoderX->channel1,returnCountX,RISING);
  pEncoderY->initEncoder();
  attachInterrupt(pEncoderY->channel1,returnCountY,RISING);
  #endif
  
  TimerEncoder.attachInterrupt(timerHandler);
  TimerEncoder.start( 1000000 * EncoderTime );
  
}


void loop() {
  
  pPIDMotor1->required=100;
  pPIDMotor1->prevRequired=100;
  pPIDMotor2->required=100;
  pPIDMotor2->prevRequired=100;
  pPIDMotor3->required=100;
  pPIDMotor3->prevRequired=100;
  
  //getIndidualDistances();
  getBotPosition();
  //getAngleofMotion();
  //requiredVelocity1( 0, 0, 2, 2, 1);  //(x1,y1,x2,y2,1 m/s)
  //calculateRPM(0,pBot->Angle,pBot->vel);
  //WheelIntoMotors();
  /*
  Serial.println("Distance1    "+String(pWheel[0]->distance)+"     Distance2   "+String(pWheel[1]->distance)+"    Distance3   "+String(pWheel[2]->distance));
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
  */
  //Serial.println("DistanceX        "+String(pBot->X_pos)+"           DistanceY         "+String(pBot->Y_pos));
  //Serial.println(" ");
  //Serial.println(" ");
  //Serial.println(" ");
  //Serial.println("RPMX    "+String(pEncoderX->Count)+"     RPMY   "+String(pEncoderY->Count));//+"    RPM3   "+String(pEncoder3->rpm));
  //Serial.println(" ");
  //Serial.println(" ");
  //Serial.println(" ");  
    Serial.println(" WheelA   "+String(pEncoder1->rpm)+" WheelB   "+String(pEncoder2->rpm)+" WheelC   "+String(pEncoder3->rpm));
    Serial.println("pPIDMotor1->pidControl(abs(pEncoder1->rpm))"+String(pPIDMotor1->pidControl(abs(pEncoder1->rpm))));
   // Serial.println(" WheelB   "+String(pEncoder2->rpm));
   // Serial.println(" WheelC   "+String(pEncoder3->rpm));
}

void timerHandler()
{
  pEncoder1->rpm=((pEncoder1->Count - pEncoder1->prevCount) * 60.0)/(EncoderTime * EncoderGearRatio * pEncoder1->ppr);
  pEncoder2->rpm=((pEncoder2->Count - pEncoder2->prevCount) * 60.0)/(EncoderTime * EncoderGearRatio * pEncoder2->ppr);
  pEncoder3->rpm=((pEncoder3->Count - pEncoder3->prevCount) * 60.0)/(EncoderTime * EncoderGearRatio * pEncoder3->ppr);
  pEncoder1->prevCount=pEncoder1->Count;
  pEncoder2->prevCount=pEncoder2->Count;
  pEncoder3->prevCount=pEncoder3->Count;
  #ifdef FourWheelDrive
  pEncoder4->rpm=((pEncoder4->Count - pEncoder4->prevCount) * 60.0)/(EncoderTime * EncoderGearRatio * pEncoder4->ppr);
  pEncoder4->prevCount=pEncoder4->Count;
  #endif
  float temp[4];
  float output[4];
  if(pPIDMotor1->required*pPIDMotor1->prevRequired>0)
  {
    temp[0]=pPIDMotor1->pidControl(abs(pEncoder1->rpm));
    output[0]=temp[0];
  }
  else
  {
    pPIDMotor1->prevRequired=pPIDMotor1->required;
    output[0]=0;
  }
  if(pPIDMotor2->required*pPIDMotor2->prevRequired>0)
  {
    temp[1]=pPIDMotor2->pidControl(abs(pEncoder2->rpm));
    output[1]=temp[1];
  }
  else
  {
    pPIDMotor2->prevRequired=pPIDMotor2->required;
    output[1]=0;
  }
  if(pPIDMotor3->required*pPIDMotor3->prevRequired>0)
  {
    temp[2]=pPIDMotor3->pidControl(abs(pEncoder3->rpm));
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

#ifdef FourWheelDrive
  if(pPIDMotor4->required*pPIDMotor4->prevRequired>0)
  {
    temp[3]=pPIDMotor4->pidControl(pEncoder4->rpm);
    output[3]=temp[3];
  }
  else
  {
    pPIDMotor4->prevRequired=pPIDMotor4->required;
    output[3]=0;
  }
   if(pPIDMotor4->required==0) output[3]=0;
#endif  

  pMotor1->driveMotor(output[0],maxPWM);
  pMotor2->driveMotor(output[1],maxPWM);
  pMotor3->driveMotor(output[2],maxPWM);
  #ifdef FourWheelDrive
  pMotor4->driveMotor(output[3],100);
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
    pEncoder3->Count++;
    else 
    pEncoder3->Count--;
  }

#ifdef FourWheelDrive
void returnCount4()
  {
    if(digitalRead(pEncoder4->channel2))
    pEncoder4->Count++;
    else 
    pEncoder4->Count--;
  }
#endif

#ifdef PositionEncoders
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
    pEncoderY->Count++;
    else 
    pEncoderY->Count--;
  }
#endif

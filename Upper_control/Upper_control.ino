#include <EasyTransfer.h>

#define printXY 1

EasyTransfer ET;
struct DATASTRUCT{
  int16_t rpm[4];
}; 
DATASTRUCT mydata;

#define UpperLowerSerial Serial3
#define pi 3.141592 
#define Omega_Timer 2*pi/EncoderTime
#define maxWheelRPM 300
#define maxMotRPM 468
#define maxPWM 255
#define onebyroot2 0.7071
#define DegreeToRadian(x) x*0.0174532
#define RadianToDegree(x) x*57.295779
#define Angle1 45   //1.57079  //90       //90+0
#define Angle2 135  //3.66519  //210      //90+120
#define Angle3 225  //5.75958  //330      //90+240
#define Angle4 315  //
#define RadiusOmniDrive 0.35  //metre
#define RadiusOmniWheel 0.075 //metre
#define RadiusXYWheel 0.029 //metre
#define VelocityToRPM(x) x*60/(2*pi*RadiusOmniWheel)
/*********************************************************************************************************************************************/
/*********************************************************** Wheels **********************************************************************************/

class Wheel{
  public:
  float translationSpeed; 
  float angularSpeed;
  int angle; 
  float maxRPM; 
  float Speed;
  float rpm;
  int prevRPM;

};
 
 Wheel Wheel1 = {0.0, 0.0, Angle4, maxWheelRPM, 0, 0, 0 };
 
 Wheel Wheel2 = {0.0, 0.0, Angle3, maxWheelRPM, 0, 0, 0 };
 
 Wheel Wheel3 = {0.0, 0.0, Angle2, maxWheelRPM, 0, 0, 0 };

 Wheel Wheel4 = {0.0, 0.0, Angle1, maxWheelRPM, 0, 0, 0 };
 
 Wheel *pWheel[4]={&Wheel1,&Wheel2,&Wheel3,&Wheel4};

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

  Encoder xencoder={18,19,1000,0,0,0};  
  Encoder *pEncoderX=&xencoder;
  
  Encoder yencoder={20,49,1000,0,0,0};   //1000 ppr
  Encoder *pEncoderY=&yencoder;
  
  void returnCountX();
  void returnCountY();
  
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

  Auto_Bot FourWheelDrive; 
  Auto_Bot *pBot=&FourWheelDrive;
  
////////////////////////////////////////////////
float Circle_theta=0;                        
float sine_X = 0;                            
////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  UpperLowerSerial.begin(9600);
  
  ET.begin(details(mydata), &UpperLowerSerial);
  
  for(int k =0;k<4;k++)
  pWheel[k]->rpm=0;
  
  pEncoderX->initEncoder();
  attachInterrupt(digitalPinToInterrupt(pEncoderX->channel1),returnCountX,RISING);
  
  pEncoderY->initEncoder();
  attachInterrupt(digitalPinToInterrupt(pEncoderY->channel1),returnCountY,RISING);
  
  TransmitURPM();
  interrupts();
  
}

void loop() {
   getBotPosition();
   Goto_XYSigmoid(0.0,0.0,3.0,0.0,5);  //speed in m/s
   TransmitURPM();
}

void returnCountX()
{
    if(digitalRead(pEncoderX->channel2))
     pEncoderX->Count--;
    else 
     pEncoderX->Count++;
}
  
void returnCountY()
{
    if(digitalRead(pEncoderY->channel2))
     pEncoderY->Count++;
    else 
     pEncoderY->Count--;
}


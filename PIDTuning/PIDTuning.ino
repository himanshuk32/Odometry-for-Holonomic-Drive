#include <EasyTransfer.h>

#define PIDTuning 1

const int StopButton = 53;
const int Forward = 23;
const int Backward = 5;
const int Left = 2;
const int Right = 3;

EasyTransfer ETpid, ETrpm;

struct RPM_data{
  float rpm[4];
  }rpm_data;

struct PID_data{
  float Kp[4];
  float Ki[4];
  float Kd[4];
  }pid_data;
 
void setup() {
  Serial.begin(38400);
  Serial2.begin(38400);
  Serial.println("Enter pid values");
  ETrpm.begin(details(rpm_data), &Serial2);
  ETpid.begin(details(pid_data), &Serial2);

   pinMode(StopButton,INPUT_PULLUP);
   pinMode(Forward,INPUT_PULLUP);
   pinMode(Backward,INPUT_PULLUP);
   pinMode(Left,INPUT_PULLUP);
   pinMode(Right,INPUT_PULLUP);
   
    //WHEEL 1
    pid_data.Kp[0]=2.5;
    pid_data.Ki[0]=0;
    pid_data.Kd[0]=0;

    //WHEEL 2
    pid_data.Kp[1]=8.0;     //8
    pid_data.Ki[1]=0.01;  //0.01
    pid_data.Kd[1]=0;

    //WHEEL 3
    pid_data.Kp[2]=2.5;
    pid_data.Ki[2]=0;
    pid_data.Kd[2]=0;

    //WHEEL 4
    pid_data.Kp[3]=8.0;
    pid_data.Ki[3]=0.01;
    pid_data.Kd[3]=0;

      

}

void loop(){

  if(PIDTuning)
    ETpid.sendData();
  
  if(ETrpm.receiveData()>0)
    {
//      for(int i=0;i<4;++i)
//      {
//        Serial.print(i+1);
//        Serial.println(" RPM " +String(rpm_data.rpm[i]));
         Serial.println(rpm_data.rpm[2]);
    //  }
    }
      
  CommandUsingBT();
}

void CommandUsingBT()
{
  if(!digitalRead(StopButton))
    Serial2.write('s');    
  if(!digitalRead(Forward))
    Serial2.write('f');
  if(!digitalRead(Backward))
    Serial2.write('b');
  if(!digitalRead(Left))
    Serial2.write('l');
  if(!digitalRead(Right))
    Serial2.write('r');     
}


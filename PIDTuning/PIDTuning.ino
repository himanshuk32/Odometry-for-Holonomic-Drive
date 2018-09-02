#include <EasyTransfer.h>

//#ifndef PIDTuning 
#define PIDTuning 0
//#endif

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
   
    //WHEEL 1
    for(int i=0;i<4;++i)
    {pid_data.Kp[i]=0.9;
    pid_data.Ki[i]=0;
    pid_data.Kd[i]=0;}
//
//    //WHEEL 2
//    pid_data.Kp[1]=0.5;
//    pid_data.Ki[1]=0;
//    pid_data.Kd[1]=0;
//
//    //WHEEL 3
//    pid_data.Kp[2]=0.5;
//    pid_data.Ki[2]=0;
//    pid_data.Kd[2]=0;
//
//    //WHEEL 4
//    pid_data.Kp[3]=0.5;
//    pid_data.Ki[3]=0;
//    pid_data.Kd[3]=0;

      

}

void loop(){

  if(PIDTuning)
    ETpid.sendData();
  
  if(ETrpm.receiveData()>0){
   // for(int i=0;i<4;i++){
     // Serial.print(i+1);
      Serial.println((rpm_data.rpm[0]));
      }
  StopusingBT();
}

void StopusingBT()
{
  if(Serial.available())
  {
    char stopButton = Serial.read();
    if(stopButton == 's')
    Serial2.write(stopButton);    
  }
}


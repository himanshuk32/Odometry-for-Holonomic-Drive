
void getUpperData(){
  if(ET.receiveData()){
    //Serial.println("Receiving");
    digitalWrite(13,HIGH);
    pPIDMotor[3]->required = mydata.rpm[0];
    pPIDMotor[0]->required = mydata.rpm[1];
    pPIDMotor[1]->required = mydata.rpm[2];
    pPIDMotor[2]->required = mydata.rpm[3];
    
    pPIDMotor[0]->prevRequired = pPIDMotor[0]->required;
    pPIDMotor[1]->prevRequired = pPIDMotor[1]->required;
    pPIDMotor[2]->prevRequired = pPIDMotor[2]->required;
    pPIDMotor[3]->prevRequired = pPIDMotor[3]->required;
    
    digitalWrite(13,LOW);
  }  
}

void RPMtoBT()
{
  for(int i=0;i<4;++i)
   {
    int temp = (int)pEncoder[i]->rpm;
    rpm_data.rpm[i] = temp;
   }
  ETrpm.sendData();
}

void PIDfromBT()
{
 if(ETpid.receiveData()>0)
  {
    for(int i=0;i<4;++i)
    {
      Serial.print(i+1);
      Serial.println(" Kp "+String(pid_data.Kp[i]));
      Serial.println(" ");
      pPIDMotor[i]->Kp = pid_data.Kp[i];
      pPIDMotor[i]->Kd = pid_data.Kd[i];
      pPIDMotor[i]->Ki = pid_data.Ki[i];
      pPIDMotor[i]->integralError = 0;
    }
  }

}

void StopUsingBT()
{
  if(BTSerial.available())
  {
    char stopButton = BTSerial.read();
    if(stopButton == 's')
    for(int i=0;i<4;++i)
    {
      pMotor[i]->driveMotorPID(0,maxMotRPM);
      pPIDMotor[i]->required = 0;
    }
    Serial.println("Stopped");
  }
}


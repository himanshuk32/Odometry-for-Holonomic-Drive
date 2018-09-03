
void getUpperData(){
  if(ET.receiveData()){
    Serial.println("Receiving");
    digitalWrite(13,HIGH);
    pPIDMotor[3]->required = mydata.rpm[0];
    pPIDMotor[0]->required = mydata.rpm[1];
    pPIDMotor[1]->required = mydata.rpm[2];
    pPIDMotor[2]->required = mydata.rpm[3];
    
    pPIDMotor[0]->prevRequired = pPIDMotor[0]->required;
    pPIDMotor[1]->prevRequired = pPIDMotor[1]->required;
    pPIDMotor[2]->prevRequired = pPIDMotor[2]->required;
    pPIDMotor[3]->prevRequired = pPIDMotor[3]->required;

    Serial.println(pPIDMotor[2]->required);
    digitalWrite(13,LOW);
  }  
}

#ifdef TunePID
void RPMtoBT()
{
  float temp;
  for(int i=0;i<4;++i)
   {
    if(i!=2)
    temp = (float)pEncoder[i]->rpm * pEncoder[i]->gearRatio;
    else
    temp = (float)pEncoder[i]->rpm * pEncoder[i]->gearRatio /pEncoder[0]->gearRatio;
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
      //Serial.println(" Kp "+String(pid_data.Kp[i]));
      //Serial.println(" ");
      pPIDMotor[i]->Kp = pid_data.Kp[i];
      pPIDMotor[i]->Kd = pid_data.Kd[i];
      pPIDMotor[i]->Ki = pid_data.Ki[i];
     }
  }

}

void DirUsingBT(int rpm)
{
  if(BTSerial.available())
  {
    char received = BTSerial.read();
    if(received == 's')
    for(int i=0;i<4;++i)
     pPIDMotor[i]->required = 0;
  
    if(received == 'f')
  {
    pPIDMotor[0]->required = -rpm;
    pPIDMotor[1]->required = rpm;
    pPIDMotor[2]->required = rpm;
    pPIDMotor[3]->required = -rpm;
  }
    
  if(received == 'b')
  {
    pPIDMotor[0]->required = rpm;
    pPIDMotor[1]->required = -rpm;
    pPIDMotor[2]->required = -rpm;
    pPIDMotor[3]->required = rpm;
  }
  if(received == 'l')
  {
    pPIDMotor[0]->required = rpm;
    pPIDMotor[1]->required = rpm;
    pPIDMotor[2]->required = -rpm;
    pPIDMotor[3]->required = -rpm;
  }
    
  if(received == 'r')
  {
    pPIDMotor[0]->required = -rpm;
    pPIDMotor[1]->required = -rpm;
    pPIDMotor[2]->required = rpm;
    pPIDMotor[3]->required = rpm;
  }  
  for(int i=0;i<4;++i)
  pPIDMotor[i]->integralError = 0;
  
  }
}

void NoCommunication()
{
  if(ETpid.receiveData()<=0)//&&!BTSerial.available())
  {
      debouncing++;
      if(debouncing > 10000)
    {
      for(int i=0;i<4;++i)
      {
        pPIDMotor[i]->required = 0;
        pPIDMotor[i]->integralError = 0;
      }
    debouncing = 0;
   }
 }
}

#endif

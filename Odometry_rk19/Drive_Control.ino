 void Motor::driveMotor(float op,float maxval)
{     
  if(op>maxval)
  op=maxval;
  if(op<-maxval)
  op=-maxval;
  if(op>0)
  {
    digitalWrite(direction1,HIGH);
    digitalWrite(direction2,LOW);
  }
  if(op<0)
  {
    op*=-1;
    digitalWrite(direction2,HIGH);
    digitalWrite(direction1,LOW);
  }
  if(op==0)
  {
    digitalWrite(direction2,HIGH);
    digitalWrite(direction1,HIGH);
  }
  op=(op*maxPWM)/maxval;
  
  analogWrite(pwmPin,(int)op);
}

void getIndidualDistances()
{
      
   pWheel[0]->distance=EncoderGearRatio*pEncoder1->Count*2*pi*pWheel[0]->Radius/pEncoder1->ppr;
   pWheel[1]->distance=EncoderGearRatio*pEncoder2->Count*2*pi*pWheel[1]->Radius/pEncoder2->ppr;
   pWheel[2]->distance=EncoderGearRatio*pEncoder3->Count*2*pi*pWheel[2]->Radius/pEncoder3->ppr;
 
}
void getBotPosition()
{
  float constX=1.20;
  float constY=1.16;
   pBot->X_pos = pEncoderX->Count*2*pi*CastorWheelXradius/pEncoderX->ppr;
   pBot->Y_pos = pEncoderY->Count*2*pi*CastorWheelYradius/pEncoderY->ppr;
   pBot->X_pos*=constX;
   pBot->Y_pos*=constY;
   Serial.println("X :  "+String(pBot->X_pos)+"     Y:  "+String(pBot->Y_pos));
}


void calculateRPM( float Omega , float angle , float Vtranslational )
{
  int r=1;
  for(int i=0;i<3;++i)
  {
    pWheel[i]->translationRPM = (3*Vtranslational/2)*sin(DegreeToRadian(pWheel[i]->angle)-angle);
    pWheel[i]->angularRPM = Omega*r;
    pWheel[i]->rpm = pWheel[i]->translationRPM + pWheel[i]->angularRPM;
  }
  ScaleWheels(MAXRPM);
  MotorRequiredPID();
}

void MotorRequiredPID()
{
  pPIDMotor1->prevRequired = pPIDMotor1->required;
  pPIDMotor2->prevRequired = pPIDMotor2->required;
  pPIDMotor3->prevRequired = pPIDMotor3->required;
  pPIDMotor1->required = pWheel[0]->rpm;
  pPIDMotor2->required = pWheel[1]->rpm;
  pPIDMotor3->required = pWheel[2]->rpm;
  
}


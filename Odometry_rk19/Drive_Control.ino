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

  if(printPIDOutput)
  Serial.println(" PID Output "+String(op));
  op=(op*maxPWM)/maxMotRPM;
  
  analogWrite(pwmPin,(int)op);
}

void getIndidualDistances()
{
   for(int i=0;i<3;++i)
   pWheel[i]->distance=GearRatio*pEncoder[i]->Count*2*pi*pWheel[i]->Radius/pEncoder[i]->ppr;
}
void getBotPosition()
{
  float constX=1.20;
  float constY=1.16;
   pBot->X_pos = pEncoderX->Count*2*pi*CastorWheelXradius/pEncoderX->ppr;
   pBot->Y_pos = pEncoderY->Count*2*pi*CastorWheelYradius/pEncoderY->ppr;
   pBot->X_pos*=constX;
   pBot->Y_pos*=constY;

   if(printXY)
   Serial.println("X :  "+String(pBot->X_pos)+"     Y:  "+String(pBot->Y_pos));
}


void calculateRPM( float Omega , float angle , float Vtranslational )
{
  int r=1;
  for(int i=0;i<3;++i)
  {
    pWheel[i]->translationRPM = Vtranslational*sin(DegreeToRadian(pWheel[i]->angle)-angle);
    pWheel[i]->angularRPM = Omega*r;
    pWheel[i]->rpm = pWheel[i]->translationRPM + pWheel[i]->angularRPM;
  }
  ScaleWheels(maxWheelRPM);
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


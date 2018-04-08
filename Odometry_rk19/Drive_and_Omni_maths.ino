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
  //op=(op*255)/maxval;
  analogWrite(pwmPin,128);//(int)op);
}

void getIndidualDistances()
{
   pWheel[0]->distance=EncoderGearRatio*pEncoder1->Count*2*pi*pWheel[0]->Radius/pEncoder1->ppr;
   pWheel[1]->distance=EncoderGearRatio*pEncoder2->Count*2*pi*pWheel[1]->Radius/pEncoder2->ppr;
   pWheel[2]->distance=EncoderGearRatio*pEncoder3->Count*2*pi*pWheel[2]->Radius/pEncoder3->ppr;
   #ifdef FourWheelDrive
   pWheel[3]->distance=EncoderGearRatio*pEncoder4->Count*2*pi*pWheel[3]->Radius/pEncoder4->ppr;
   #endif
}
void getBotPosition()
{
  #ifdef PositionEncoders
  
   pBot->X_pos = pEncoderX->Count*2*pi*CastorWheelXradius/pEncoderX->ppr;
   pBot->Y_pos = pEncoderY->Count*2*pi*CastorWheelYradius/pEncoderY->ppr;
 
  #else
  
    #ifdef FourWheelDrive
  
    pBot->X_pos = (pWheel[0]->distance+pWheel[3]->distance-pWheel[2]->distance-pWheel[1]->distance)*onebyroot2;
    pBot->Y_pos = (pWheel[0]->distance+pWheel[1]->distance-pWheel[2]->distance-pWheel[3]->distance)*onebyroot2;
  
    #else
  
    pBot->X_pos = abs(pWheel[0]->distance)-(abs(pWheel[1]->distance)+ abs(pWheel[2]->distance))*0.5;
    pBot->Y_pos = (abs(pWheel[1]->distance)-abs(pWheel[2]->distance))*root3by2;
  
    #endif

  #endif
}
void getAngleofMotion()
{
  pBot->Angle=angle(0,pBot->X_vel,0,pBot->Y_vel);
}

void requiredVelocity1(float x1,float y1,float x2,float y2,float V0)
{
  float  r = dist ( x1, pBot->X_pos, y1, pBot->Y_pos);
  float r0 = dist ( x1, x2, y1, y2);
  float r1 = dist ( 0, x1, 0, y1);
  float r2 = dist ( 0, x2, 0, y2);
  if(r<=r0/2)
  {
    {
      pBot->X_vel=(4*V0)*(pBot->X_pos-x1)/(x2-x1);
      pBot->Y_vel=(4*V0)*(pBot->Y_pos-y1)/(y2-y1);
    }
  }
  else
  {
    pBot->X_vel=(4*V0)*(pBot->X_pos-x2)/(x1-x2);
    pBot->Y_vel=(4*V0)*(pBot->Y_pos-y2)/(y1-y2);
  }
  pBot->vel = sqrt(pBot->X_vel*pBot->X_vel+pBot->Y_vel*pBot->Y_vel);
  
  //pBot->Requiredangle = atan2(y2,x2)+acos((r2*r2-r1*r1-r0*r0)/(2*r0*r1));
}

void requiredVelocity2( float x1,float y1,float x2,float y2,float V0)
{
  float  r = dist ( x1, pBot->X_pos, y1, pBot->Y_pos);
  float r0 = dist ( x1, x2, y1, y2);
  long int timer=millis();
  int accelerationTime=1000;
  //if(millis()-timer < accelerationTime && r<=r0/2)
  {
    
  }
}

void calculateRPM(float Omega,int angle,float Vtranslational)
{
  int n;
  int r=1;                                  //YET TO THINK ABOUT THIS   since WHeel->Radius exist

  #ifdef FourWheelDrive
  n=4;
  #else
  n=3;
  #endif
  
  for(int i=0;i<n;++i)
  {
    pWheel[i]->prevRPM = pWheel[i]->rpm;
    pWheel[i]->translationRPM = Vtranslational*sin(pWheel[i]->angle-DegreeToRadian(angle));
    pWheel[i]->angularRPM = Omega*r;
    pWheel[i]->rpm = pWheel[i]->translationRPM + pWheel[i]->angularRPM;
  }
  ScaleWheels(MAXRPM);
}

void WheelIntoMotors()
{
  pPIDMotor1->required=pWheel[0]->rpm;
  pPIDMotor2->required=pWheel[1]->rpm;
  pPIDMotor3->required=pWheel[2]->rpm;
  #ifdef FourWheelDrive
  pPIDMotor4->required=pWheel[3]->rpm;
  #endif
}


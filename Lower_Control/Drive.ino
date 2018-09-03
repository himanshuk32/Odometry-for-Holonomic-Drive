 void Motor::driveMotorPID(float op, int maxval)
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

  op=(op*maxPWM)/maxMotRPM;

 // analogWrite(pwmPin,(int)op);
 pwm.pinDuty(pwmPin,(int)op);
}

void driveMotorReq(float op,int maxval,PID *pPIDMotor, Motor *pMotor)
{
  if(op>maxval)
  op=maxval;
  if(op<-maxval)
  op=-maxval;
  if(pPIDMotor->required > 0)
  {
    digitalWrite(pMotor->direction1,HIGH);
    digitalWrite(pMotor->direction2,LOW);
  }
  if(pPIDMotor->required < 0)
  {
    op*=-1;
    digitalWrite(pMotor->direction2,HIGH);
    digitalWrite(pMotor->direction1,LOW);
  }
  if(pPIDMotor->required == 0)
  {
    digitalWrite(pMotor->direction2,HIGH);
    digitalWrite(pMotor->direction1,HIGH);
  }

  op=(op*maxPWM)/maxMotRPM;

  pwm.pinDuty(pMotor->pwmPin,(int)op);
 // analogWrite(pMotor->pwmPin,(int)op);
}


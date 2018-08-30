 void Motor::driveMotor(float op, float maxval)
{ 
  
//  if(printPIDOutput)
  //Serial.println(" PID Output "+String(op));
      
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

  analogWrite(pwmPin,(int)op);
//  pwm.pinDuty(pwmPin,(int)op);
}


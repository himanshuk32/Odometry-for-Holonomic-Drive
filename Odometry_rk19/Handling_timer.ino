void getactualRPM(Encoder **encoder)
{
  int n=3;
  for(int i=0;i<n;++i)
  encoder[i]->rpm=((encoder[i]->Count - encoder[i]->prevCount) * 60.0)/(EncoderTime * GearRatio * encoder[i]->ppr);
}

void setOutput(Encoder **encoder, PID **PIDmotor, Motor **motor, int rpmBound)
{
  int n=3;
  for(int i=0;i<n;++i)
  {
    if(PIDmotor[i]->required * PIDmotor[i]->prevRequired>0)
    {
      output[i]=PIDmotor[i]->pidControl((encoder[i]->rpm));
    }
    else
    {
     PIDmotor[i]->prevRequired=PIDmotor[i]->required;
     output[i]=0;
    }
  }

  for(int i=0;i<n;++i)
  if(PIDmotor[i]->required==0) 
  output[i]=0;

  for(int i=0;i<n;++i)
  motor[i]->driveMotor(output[i],rpmBound);

  for(int i=0;i<n;++i)
  encoder[i]->prevCount=encoder[i]->Count;
}


void PID::initPID(float kp,float kd,float ki,float req,float minV,float maxV)
  {
    Kp=kp;
    Kd=kd;
    Ki=ki;
    required=req;
    maxControl=maxV;
    minControl=minV;
    error=0;
    prevError=1;
    derivativeError=0;
    integralError=0;
  }

float PID::pidControl(float actual)
  {
    error=required-actual;
    derivativeError=error-prevError;
    integralError=integralError+error;//+prevError;
    prevError=error;
    float Output=Kp*error+Kd*derivativeError+Ki*integralError;
    if(Output>maxControl)
    Output=maxControl;
    if(Output<minControl)
    Output=minControl;
    return Output;
  }


void Goto_XYHard( float x1,float y1,float x2,float y2,float V0)
{
  float tolerance = 0.05;
  float angleReqOP = angle ( pBot->X_pos, pBot->Y_pos, x2, y2);
  pBot->Angle = angleReqOP;
  
  float  r = dist ( x1, y1 , pBot->X_pos, pBot->Y_pos );
  float r0 = dist ( x1, y1, x2, y2 );

  pBot->vel = V0;

  if( r > r0 +tolerance )
  {
     pBot->vel = - 2*V0/3;
     pBot->Angle  = pi + angleReqOP;
  }

  if( r > r0 - tolerance && r < r0 + tolerance )
  pBot->vel = 0;

  calculateSpeed(0 ,pBot->Angle ,pBot->vel);
}

void Goto_XYSigmoid( float x1,float y1,float x2,float y2,float V0)
{
  float angleReqOP = angle ( pBot->X_pos, pBot->Y_pos, x2, y2);
  pBot->Angle = angleReqOP;
  
  float  r = dist ( x1, y1 , pBot->X_pos, pBot->Y_pos );
  float r0 = dist ( x1, y1, x2, y2 );

  pBot->vel = V0 * trapezoid_sigmoid (r , r0); 
  calculateSpeed(0 ,pBot->Angle ,pBot->vel);
}

void Goto_XYTimed(float x1, float y1, float x2, float y2, float V0, long long int currentTime)
{
  float t = currentTime/1000;
  float angleReqOP = angle ( pBot->X_pos, pBot->Y_pos, x2, y2);
  pBot->Angle = angleReqOP;
  
  float  r = dist ( x1, y1 , pBot->X_pos, pBot->Y_pos );
  float r0 = dist ( x1, y1, x2, y2 );

  //DO WE NEED????????????????????????????????????????????????????????????????
}

void TraceCircle_discrete(float radius, float V0)
{
  Goto_XYHard ( radius*cos( Circle_theta ) , radius + radius*sin( Circle_theta ), radius*cos( Circle_theta + 0.01 ) , radius + radius*sin( Circle_theta + 0.01 ) , V0 ); 
  Circle_theta+= 0.01 ;
}

void TraceCircle_pos(float radius, float V0)
{
  float theta = - angle( pBot->X_pos , pBot->Y_pos - radius , 0, 0 );
  calculateSpeed(0, theta , V0);
}

void TraceCircle_time(float radius, int omega, long long int currentTime)
{
   float t = currentTime/1000;
   float v = omega*radius;
   float angle = floatModulo(omega*t-pi/2,2*pi);
   calculateSpeed(0 ,angle ,v);
}

void CircularArc(float radius, int t_omega, int ArcAngle, long long int currentTime)
{
  float t = currentTime/1000;
  float theta = DegreeToRadian(ArcAngle);
  float r = dist(0, 0, pBot->X_pos, pBot->Y_pos);    //Gotta change the initial coordinates
  float current_theta = 2 * asin( r / (2*radius));
  float v = t_omega * radius * trapezoid_sigmoid(current_theta, theta);
  float angleBot = floatModulo(t_omega*t-pi/2,2*pi);
  calculateSpeed(0 ,angleBot ,v);
}

void TraceSine_discrete(float amplitude, float frequency, float V0)  //not gonna be exact
{
  float delta_x = frequency;
  Goto_XYHard ( sine_X , amplitude* sin(sine_X), sine_X + delta_x , amplitude* sin(sine_X + delta_x), V0 ); 
  sine_X += delta_x;
}

void TraceSine_pos(float amplitude,float V)  
{

   float angle = atan2(amplitude*cos(pBot->X_pos),1);
   calculateSpeed(0 ,angle ,V);
}

void TraceSine_time(float amplitude, float omega, long long int currentTime)     //Use of yield() in delay!!!!! delay() doesn't block anything
{
   float t = currentTime/1000;
   float v = omega * sqrt((amplitude*cos(omega*currentTime))*(amplitude*cos(omega*currentTime))+1);
   float angle = atan2(amplitude*cos(omega*currentTime),1);
   calculateSpeed(0 ,angle ,v);
}  

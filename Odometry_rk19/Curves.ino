void Goto_XYLogic1( float x1,float y1,float x2,float y2,float V0)
{
  float angleReqOP = angle ( pBot->X_pos, pBot->Y_pos, x2, y2);
  pBot->Angle = angleReqOP;
  
  float  r = dist ( x1, y1 , pBot->X_pos, pBot->Y_pos );
  float r0 = dist ( x1, y1, x2, y2 );

  pBot->vel = V0;

  if( r>r0 )
  {
     pBot->vel = - 2*V0/3;
     pBot->Angle  = pi + angleReqOP;
  }

  if( r==r0 )
  pBot->vel = 0;
  
  if(printLineAngleSpeed)
  Serial.println(" Angle "+ String(RadianToDegree(pBot->Angle))+ " Speed "+String(pBot->vel));
}

void Goto_XYSigmoid( float x1,float y1,float x2,float y2,float V0)
{
  float angleReqOP = angle ( pBot->X_pos, pBot->Y_pos, x2, y2);
  pBot->Angle = angleReqOP;
  
  float  r = dist ( x1, y1 , pBot->X_pos, pBot->Y_pos );
  float r0 = dist ( x1, y1, x2, y2 );

  pBot->vel = V0 * trapezoid_sigmoid (r , r0); 

  if(printLineAngleSpeed)
  Serial.println(" Angle "+ String(RadianToDegree(pBot->Angle))+ " Speed "+String(pBot->vel));
}


void CircleLogic1(float radius, float V0)
{
  Goto_XYLogic1 ( radius*cos( Circle_theta ) , radius + radius*sin( Circle_theta ), radius*cos( Circle_theta + 0.01 ) , radius + radius*sin( Circle_theta + 0.01 ) , V0 ); 
  Circle_theta+= 0.01 ;

  if(printCircleAngleSpeed)
  Serial.println(" Angle "+ String(RadianToDegree(pBot->Angle))+ " Speed "+String(pBot->vel));

}

void CircleLogic2(float radius, float V0)
{
  float theta = - angle( pBot->X_pos , pBot->Y_pos - radius , 0, 0 );
  calculateRPM ( 0, theta , V0 );

  if(printCircleAngleSpeed)
  Serial.println(" Angle "+ String(RadianToDegree(pBot->Angle))+ " Speed "+String(pBot->vel)); 
}

void CircleWithTime(float radius, int t_omega, long long int currentTime)
{
   float t = currentTime/1000;
   float v = t_omega*radius;
   float angle = floatModulo(t_omega*t-pi/2,2*pi);
   calculateRPM(0 ,angle ,v);
}

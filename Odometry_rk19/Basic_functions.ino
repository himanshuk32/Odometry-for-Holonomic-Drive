float dist(float x1,float y1,float x2,float y2)
{
  return sqrt((x1-x2)*(x1-x2)+(y1-y1)*(y1-y2));
}

float angle(float x1,float y1,float x2,float y2)
{
  return atan2((y2-y1),(x2-x1));
}

void ScaleWheels(float maximum)
{
  int n;
  #ifdef FourWheelDrive
  n=4;
  #else
  n=3;
  #endif
  
  float maxR=pWheel[0]->rpm;
  for(int i=0;i<n;++i)
  {
    if(pWheel[i]->rpm>maxR)
    maxR=pWheel[i]->rpm;
  }
  if(maxR>maximum)
  for(int i=0;i<n;++i)
  {
    pWheel[i]->rpm*=maximum/maxR;
  }
}


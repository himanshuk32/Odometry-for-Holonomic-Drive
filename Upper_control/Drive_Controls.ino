void calculateSpeed(float Omega , float angle , float Vtranslational )     //Omega rad/s      Vtranslational  m/s
{
  for(int i=0;i<4;++i)
  {
    pWheel[i]->translationSpeed = Vtranslational * cos( DegreeToRadian(pWheel[i]->angle) - angle );
    pWheel[i]->angularSpeed = Omega * RadiusOmniDrive;
    pWheel[i]->Speed = pWheel[i]->translationSpeed + pWheel[i]->angularSpeed;
    pWheel[i]->rpm = VelocityToRPM(pWheel[i]->Speed); 
  }

  ScaleWheels(maxWheelRPM);
}



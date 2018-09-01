void calculateSpeed(float Omega , float angle , float Vtranslational )     //Omega rad/s      Vtranslational  m/s
{
  for(int i=0;i<4;++i)
  {
    pWheel[i]->translationSpeed = Vtranslational * cos( DegreeToRadian(pWheel[i]->angle) - angle );
    pWheel[i]->angularSpeed = Omega * RadiusOmniDrive;
    pWheel[i]->Speed = pWheel[i]->translationSpeed + pWheel[i]->angularSpeed;
    pWheel[i]->rpm = VelocityToRPM(pWheel[i]->Speed); 
  }

  ScaleWheels(maxMotRPM);
}

void getBotPosition()
{
  float constX = 1.0;
  float constY = 1.0;
  pBot->X_pos = (pEncoderX->Count * 2 * pi * RadiusXYWheel / pEncoderX->ppr);
  pBot->Y_pos = (pEncoderY->Count * 2 * pi * RadiusXYWheel / pEncoderY->ppr);
  pBot->X_pos *= constX;
  pBot->Y_pos *= constY;
  if(printXY)
 { 
   Serial.println("X:   "+String(pBot->X_pos));
   Serial.println("Y:   "+String(pBot->Y_pos));
 }
}


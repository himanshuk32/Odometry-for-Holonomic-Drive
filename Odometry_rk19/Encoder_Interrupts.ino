void returnCount1()
  {
    if(digitalRead(pEncoder1->channel2))
    pEncoder1->Count++;
    else 
    pEncoder1->Count--;
  }

void returnCount2()
  {
    if(digitalRead(pEncoder2->channel2))
    pEncoder2->Count++;
    else 
    pEncoder2->Count--;
  }

void returnCount3()
  {
    if(digitalRead(pEncoder3->channel2))
    pEncoder3->Count--;
    else 
    pEncoder3->Count++;
  }

  void returnCountX()
  {
    if(digitalRead(pEncoderX->channel2))
    pEncoderX->Count++;
    else 
    pEncoderX->Count--;
  }
  
  void returnCountY()
  {
    if(digitalRead(pEncoderY->channel2))
    pEncoderY->Count--;
    else 
    pEncoderY->Count++;
  }


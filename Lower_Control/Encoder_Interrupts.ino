
void returnCount1()
  {
    if(digitalRead(pEncoder1->channel2))
    pEncoder1->Count--;
    else 
    pEncoder1->Count++;
  }

void returnCount2()
  {
    if(digitalRead(pEncoder2->channel2))
      pEncoder2->Count--;
    else 
      pEncoder2->Count++;
  }

void returnCount3()
  {
    if(digitalRead(pEncoder3->channel2))
    pEncoder3->Count++;
    else 
    pEncoder3->Count--;
  }

void returnCount4()
  {
    if(digitalRead(pEncoder4->channel2))
    pEncoder4->Count++;
    else 
    pEncoder4->Count--;
  }


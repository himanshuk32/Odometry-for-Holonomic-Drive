
void getUpperData(){
  if(ET.receiveData()){
    digitalWrite(13,HIGH);
    pPIDMotor[0]->required = mydata.rpm[0];
    pPIDMotor[1]->required = mydata.rpm[1];
    pPIDMotor[2]->required = mydata.rpm[2];
    pPIDMotor[3]->required = mydata.rpm[3];
    
    pPIDMotor[0]->prevRequired = pPIDMotor[0]->required;
    pPIDMotor[1]->prevRequired = pPIDMotor[1]->required;
    pPIDMotor[2]->prevRequired = pPIDMotor[2]->required;
    pPIDMotor[3]->prevRequired = pPIDMotor[3]->required;
    digitalWrite(13,LOW);
  }  
}


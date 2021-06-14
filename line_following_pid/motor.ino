
const int motor2Speed = 120; 
const int motor1Speed = 120; 

const int adj = 1;
float adjTurn = 8;

void changeMotorSpeed() {
  //Change motor speed of both motors accordingly
  //Serial.print(" OUTPUT : ");Serial.print(output);
  motor1newSpeed = motor1Speed + output;
  motor2newSpeed = motor2Speed - output;
  
  //Constrain the new speed of motors to be between the range 0-255
  constrain(motor2newSpeed, 0, 255);
  constrain(motor1newSpeed, 0, 255);

  //Serial.print("left speed: "); Serial.print(motor2newSpeed);
  //Serial.print(" right speed: "); Serial.println(motor1newSpeed);
  
  //Set new speed, and run motors in forward direction
  ledcWrite(M2_ENABLE_CHANNEL, motor2newSpeed);
  ledcWrite(M1_ENABLE_CHANNEL, motor1newSpeed);
  if (error>1){
    // Serial.println("EMPHKAAAAA     11111");
    // ledcWrite(M2_ENABLE_CHANNEL, motor2newSpeed+50);
    
    digitalWrite(motor1Forward, LOW);
    digitalWrite(motor1Backward, HIGH);
    digitalWrite(motor2Forward, HIGH);
    digitalWrite(motor2Backward, LOW);
    // delay(100);
  }
  else if (error<-1){
    // Serial.println("EMPHKAAAAA     2222");
    // ledcWrite(M1_ENABLE_CHANNEL, motor1newSpeed+50);

    digitalWrite(motor1Forward, HIGH);
    digitalWrite(motor1Backward, LOW);
    digitalWrite(motor2Forward, LOW);
    digitalWrite(motor2Backward, HIGH);
    // delay(100);

  }else{
    digitalWrite(motor1Forward, HIGH);
    digitalWrite(motor1Backward, LOW);
    digitalWrite(motor2Forward, HIGH);
    digitalWrite(motor2Backward, LOW);
  }
  
  
}

void motorStop() {
  ledcWrite(M1_ENABLE_CHANNEL, 0);
  ledcWrite(M2_ENABLE_CHANNEL, 0);
  vTaskDelay(200/portTICK_PERIOD_MS);
}

void motorTurn(int direction, int degrees)
{
  ledcWrite(M1_ENABLE_CHANNEL, 200);
  ledcWrite(M2_ENABLE_CHANNEL, 200);

  digitalWrite(motor1Forward, LOW);
  digitalWrite(motor1Backward, HIGH);  
  digitalWrite(motor2Forward, HIGH);
  digitalWrite(motor2Backward, LOW);
  readIRSensors();
  calculateError();
  // delay (round(adjTurn*degrees+1));
  // motorStop();
}

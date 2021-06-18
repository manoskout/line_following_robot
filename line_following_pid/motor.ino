const int motor2Speed = 180; 
const int motor1Speed = 180; 

const int adj = 1;
float adjTurn = 8;

void changeMotorSpeed() {
  //Change motor speed of both motors accordingly
//   Serial.print(" OUTPUT : ");Serial.print(output);
  int motor1newSpeed = motor1Speed + output;
  int motor2newSpeed = motor2Speed - output;
  
  //Constrain the new speed of motors to be between the range 0-255
  constrain(motor2newSpeed, 0, 255);
  constrain(motor1newSpeed, 0, 255);

  Serial.print("left speed: "); Serial.print(motor2newSpeed);
  Serial.print(" right speed: "); Serial.println(motor1newSpeed);
  
  //Set new speed, and run motors in forward direction
  ledcWrite(M2_ENABLE_CHANNEL, motor2newSpeed);
  ledcWrite(M1_ENABLE_CHANNEL, motor1newSpeed);
  if (error>=1){
    // ledcWrite(M2_ENABLE_CHANNEL, motor2newSpeed+50);
    
    digitalWrite(motor1Forward, LOW);
    digitalWrite(motor1Backward, HIGH);
    digitalWrite(motor2Forward, HIGH);
    digitalWrite(motor2Backward, LOW);
    // Inevitable gap between the sensors, thats why the delay
//    vTaskDelay(100/portTICK_PERIOD_MS);

  }
  else if (error<=-1){
//     Serial.println("EMPHKAAAAA     2222");    
    digitalWrite(motor1Forward, HIGH);
    digitalWrite(motor1Backward, LOW);
    digitalWrite(motor2Forward, LOW);
    digitalWrite(motor2Backward, HIGH);

  }else{
      digitalWrite(motor1Forward, HIGH);
      digitalWrite(motor1Backward, LOW);
      digitalWrite(motor2Forward, HIGH);
      digitalWrite(motor2Backward, LOW);
  }  
}

void motorStop() {
  vTaskDelay(100/portTICK_PERIOD_MS);
  ledcWrite(M1_ENABLE_CHANNEL, 0);
  ledcWrite(M2_ENABLE_CHANNEL, 0);
  
}

void motorTurn(int direction, int degrees)
{
  Serial.println("TURN");
//  vTaskDelay(100/portTICK_PERIOD_MS);
  ledcWrite(M1_ENABLE_CHANNEL, MAX_SPEED);
  ledcWrite(M2_ENABLE_CHANNEL, MAX_SPEED);
  if (direction==TURN_LEFT){
    digitalWrite(motor1Forward, LOW);
    digitalWrite(motor1Backward, HIGH);  
    digitalWrite(motor2Forward, HIGH);
    digitalWrite(motor2Backward, LOW);
  }else if(direction==TURN_RIGHT){
    digitalWrite(motor1Forward, HIGH);
    digitalWrite(motor1Backward, LOW);  
    digitalWrite(motor2Forward, LOW);
    digitalWrite(motor2Backward, HIGH);
  
  }
  readIRSensors();
  calculateError();
  vTaskDelay(round(adjTurn*degrees+1)/portTICK_PERIOD_MS);
  // motorStop();
}

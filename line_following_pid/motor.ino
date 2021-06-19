const int motor2Speed = 180; 
const int motor1Speed = 180; 

const int adj = 1;
float adjTurn = 5;

void changeMotorSpeed() {
  //Change motor speed of both motors accordingly
  int motor1newSpeed = motor1Speed + output * 20;
  int motor2newSpeed = motor2Speed - output * 20;
  
  //Constrain the new speed of motors to be between the range 0-255
  if (motor2newSpeed < 0) {
    motor2newSpeed = 0;
  } else if (motor2newSpeed > 255) {
    motor2newSpeed = 255;
  }
  
  if (motor1newSpeed < 0) {
    motor1newSpeed = 0;
  } else if (motor1newSpeed > 255) {
    motor1newSpeed = 255;
  }

  // Set the other's side speed to 0 in order to turn
  if (motor1newSpeed > motor2newSpeed) {
    motor2newSpeed = 0;
  }
  else if (motor1newSpeed < motor2newSpeed) {
    motor1newSpeed = 0;
  }
  // When there is not turn set the speed to max
  else if (motor1newSpeed == motor2newSpeed) {
    motor1newSpeed = MAX_SPEED;
    motor2newSpeed = MAX_SPEED;
  }

  //Set new speed, and run motors in forward direction
  ledcWrite(M2_ENABLE_CHANNEL, motor2newSpeed);
  ledcWrite(M1_ENABLE_CHANNEL, motor1newSpeed);

  if (error >= 1) {
    digitalWrite(motor1Forward, LOW);
    digitalWrite(motor1Backward, HIGH);
    digitalWrite(motor2Forward, HIGH);
    digitalWrite(motor2Backward, LOW);
  } 
  else if (error <= -1) {
    digitalWrite(motor1Forward, HIGH);
    digitalWrite(motor1Backward, LOW);
    digitalWrite(motor2Forward, LOW);
    digitalWrite(motor2Backward, HIGH);
  }
  else {
    digitalWrite(motor1Forward, HIGH);
    digitalWrite(motor1Backward, LOW);
    digitalWrite(motor2Forward, HIGH);
    digitalWrite(motor2Backward, LOW); 
  }
  
  Serial.print("left speed: "); Serial.print(motor2newSpeed);
  Serial.print(" right speed: "); Serial.print(motor1newSpeed);
  Serial.print(" error: "); Serial.println(error);
}

void motorStop() {
  vTaskDelay(100/portTICK_PERIOD_MS);
  ledcWrite(M1_ENABLE_CHANNEL, 0);
  ledcWrite(M2_ENABLE_CHANNEL, 0); 
}

void motorTurn(int direction, int degrees)
{
  vTaskDelay(200/portTICK_PERIOD_MS);
  ledcWrite(M1_ENABLE_CHANNEL, MAX_SPEED);
  ledcWrite(M2_ENABLE_CHANNEL, MAX_SPEED);
  
  if (direction == TURN_LEFT){
    Serial.println("TURN LEFT");
    digitalWrite(motor1Forward, LOW);
    digitalWrite(motor1Backward, HIGH);  
    digitalWrite(motor2Forward, HIGH);
    digitalWrite(motor2Backward, LOW);
  }
  else if(direction == TURN_RIGHT){
    Serial.println("TURN RIGHT");
    digitalWrite(motor1Forward, HIGH);
    digitalWrite(motor1Backward, LOW);  
    digitalWrite(motor2Forward, LOW);
    digitalWrite(motor2Backward, HIGH);
  }
//  readIRSensors();
//  calculateError();
  vTaskDelay(round(adjTurn*degrees+1)/portTICK_PERIOD_MS);
}

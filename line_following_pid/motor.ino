const int motor2Speed = 180; 
const int motor1Speed = 180; 

float adjTurn = 1;

void changeMotorSpeed() {
  //Change motor speed of both motors accordingly
  int motor1newSpeed = motor1Speed + PIDvalue;
  int motor2newSpeed = motor2Speed - PIDvalue;
  
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

  // When there is a sharp turn move the wheels opposite way
  if (error >= 5) {
    motor1newSpeed = MAX_SPEED;
    motor2newSpeed = MAX_SPEED;
    
    digitalWrite(motor1Forward, HIGH);
    digitalWrite(motor1Backward, LOW);
    digitalWrite(motor2Forward, LOW);
    digitalWrite(motor2Backward, HIGH);

    motorTurn(RIGHT, 90);
  } 
  else if (error <= -5) {
    motor1newSpeed = MAX_SPEED;
    motor2newSpeed = MAX_SPEED;
    
    digitalWrite(motor1Forward, LOW);
    digitalWrite(motor1Backward, HIGH);
    digitalWrite(motor2Forward, HIGH);
    digitalWrite(motor2Backward, LOW);
    
    motorTurn(LEFT, 90);
  }
  // When there is a light turn move the wheels in the same way
  else {
    digitalWrite(motor1Forward, HIGH);
    digitalWrite(motor1Backward, LOW);
    digitalWrite(motor2Forward, HIGH);
    digitalWrite(motor2Backward, LOW); 

    // When there is not any turn set the speed to MAX_SPEED
    if (motor1newSpeed == motor2newSpeed) {
      motor1newSpeed = MAX_SPEED;
      motor2newSpeed = MAX_SPEED;
    }
  }
  
  //Set new speed, and run motors in forward direction
  ledcWrite(M2_ENABLE_CHANNEL, motor2newSpeed);
  ledcWrite(M1_ENABLE_CHANNEL, motor1newSpeed);

  vTaskDelay(1/portTICK_PERIOD_MS);
  
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
  vTaskDelay(150/portTICK_PERIOD_MS);
    
  if (direction == LEFT){
    digitalWrite(motor1Forward, LOW);
    digitalWrite(motor1Backward, HIGH);  
    digitalWrite(motor2Forward, HIGH);
    digitalWrite(motor2Backward, LOW);
  }
  else if(direction == RIGHT){
    digitalWrite(motor1Forward, HIGH);
    digitalWrite(motor1Backward, LOW);  
    digitalWrite(motor2Forward, LOW);
    digitalWrite(motor2Backward, HIGH);
  }

  vTaskDelay(round(adjTurn*degrees+1)/portTICK_PERIOD_MS);
}

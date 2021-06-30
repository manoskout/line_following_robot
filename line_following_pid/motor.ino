

float adjTurn = 0.5;
void changeMotorSpeed() {
  /*
   * Change motor speed of both motors accordingly
   */
  
  motor1newSpeed = motor1Speed + PIDvalue;
  motor2newSpeed = motor2Speed - PIDvalue;
  
  //Constrain the new speed of motors to be between the range 0-255
  if (motor2newSpeed < 0) {
    motor2newSpeed = 0;
  } else if (motor2newSpeed > MAX_SPEED) {
    motor2newSpeed = MAX_SPEED;
  }
  if (motor1newSpeed < 0) {
    motor1newSpeed = 0;
  } else if (motor1newSpeed > MAX_SPEED) {
    motor1newSpeed = MAX_SPEED;
  }  

  // When there is a sharp turn move the wheels opposite way
  if (error >= 5) {
    motor1newSpeed = MAX_SPEED;
    motor2newSpeed = MAX_SPEED;

    motorTurn(RIGHT, 90);
  } 
  else if (error <= -5) {
    motor1newSpeed = MAX_SPEED;
    motor2newSpeed = MAX_SPEED;
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
      motor1newSpeed = BASE_SPEED;
      motor2newSpeed = BASE_SPEED;
    }
  }
  
  //Set new speed, and run motors in forward direction
  ledcWrite(M2_ENABLE_CHANNEL, motor2newSpeed);
  ledcWrite(M1_ENABLE_CHANNEL, motor1newSpeed);
}

void motorStop() {
  motor1newSpeed = 0;
  motor2newSpeed = 0;
  ledcWrite(M1_ENABLE_CHANNEL, motor1newSpeed);
  ledcWrite(M2_ENABLE_CHANNEL, motor2newSpeed); 
}

void motorTurn(int direction, int degrees)
{
//  vTaskDelay(50/portTICK_PERIOD_MS);  
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
  vTaskResume(PIDTask_handle);

}

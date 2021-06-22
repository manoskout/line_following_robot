float pTerm, iTerm, dTerm;
float error=0;
int previousError=0;
// GAIN CONTSTANTS
float kp = 40;
float ki = 0;
float kd = 30; 
float PIDvalue;
int integral, derivative;
int irReadings[5];

void readIRSensors() {
  //Read the IR sensors and put the readings in irReadings array
  for (int pin = 0; pin < 5; pin++) {
    int pinNum = irSensors[pin];
    irReadings[pin] = digitalRead(pinNum);
  }
}


void calculateError() {
  
  //Determine an error based on the readings
  if ((irReadings[0] == 0) && (irReadings[1] == 0) && (irReadings[2] == 0) && (irReadings[3] == 0) && (irReadings[4] == 0)) {
    error = 0; mode = NO_LINE;
  } 
  else if ((irReadings[0] == 1) && (irReadings[1] == 1) && (irReadings[2] == 1) && (irReadings[3] == 1) && (irReadings[4] == 1)) {
    error = 0; mode = STOPPED;
  }
  else if ((irReadings[0] == 0) && (irReadings[1] == 0) && (irReadings[2] == 0) && (irReadings[3] == 0) && (irReadings[4] == 1)) {
    error = 4; mode = FOLLOWING_LINE;
  } 
  else if ((irReadings[0] == 0) && (irReadings[1] == 0) && (irReadings[2] == 0) && (irReadings[3] == 1) && (irReadings[4] == 1)) {
    error = 3; mode = FOLLOWING_LINE;
  } 
  else if ((irReadings[0] == 0) && (irReadings[1] == 0) && (irReadings[2] == 0) && (irReadings[3] == 1) && (irReadings[4] == 0)) {
    error = 2; mode = FOLLOWING_LINE;
  } 
  else if ((irReadings[0] == 0) && (irReadings[1] == 0) && (irReadings[2] == 1) && (irReadings[3] == 1) && (irReadings[4] == 0)) {
    error = 1; mode = FOLLOWING_LINE;
  } 
  else if ((irReadings[0] == 0) && (irReadings[1] == 0) && (irReadings[2] == 1) && (irReadings[3] == 0) && (irReadings[4] == 0)) {
    error = 0; mode = FOLLOWING_LINE;
  } 
  else if ((irReadings[0] == 0) && (irReadings[1] == 1) && (irReadings[2] == 1) && (irReadings[3] == 0) && (irReadings[4] == 0)) {
    error = -1; mode = FOLLOWING_LINE;
  } 
  else if ((irReadings[0] == 0) && (irReadings[1] == 1) && (irReadings[2] == 0) && (irReadings[3] == 0) && (irReadings[4] == 0)) {
    error = -2; mode = FOLLOWING_LINE;
  } 
  else if ((irReadings[0] == 1) && (irReadings[1] == 1) && (irReadings[2] == 0) && (irReadings[3] == 0) && (irReadings[4] == 0)) {
    error = -3; mode = FOLLOWING_LINE;
  } 
  else if ((irReadings[0] == 1) && (irReadings[1] == 0) && (irReadings[2] == 0) && (irReadings[3] == 0) && (irReadings[4] == 0)) {
    error = -4; mode = FOLLOWING_LINE;
  }
  else if (((irReadings[0] == 1) && (irReadings[1] == 1) && (irReadings[2] == 1) && (irReadings[3] == 0) && (irReadings[4] == 0)) || ((irReadings[0] == 1) && (irReadings[1] == 1) && (irReadings[2] == 1) && (irReadings[3] == 1) && (irReadings[4] == 0))) {
    error = -5; mode = FOLLOWING_LINE;
  }
  else if (((irReadings[0] == 0) && (irReadings[1] == 0) && (irReadings[2] == 1) && (irReadings[3] == 1) && (irReadings[4] == 1)) || ((irReadings[0] == 0) && (irReadings[1] == 1) && (irReadings[2] == 1) && (irReadings[3] == 1) && (irReadings[4] == 1))) {
    error = 5; mode = FOLLOWING_LINE;
  } 
  else {
    error = 0; mode = STOPPED;
  }
}

void calculatePID() {
  pTerm = kp * error;
  integral += error;
  iTerm = ki * integral;
  derivative = error - previousError;
  dTerm = kd * derivative;
  PIDvalue = pTerm + iTerm + dTerm;
  previousError = error;
}

//void calculatePID()
//{
//  pTerm = error;
//  iTerm = iTerm + error;
//  dTerm = error-previousError;
//  PIDvalue = (kp*pTerm) + (ki*iTerm) + (kd*dTerm);
//  previousError = error;
//}

void printIRSensors(){
  /*
  Print the sensors' PIDvalue
  Just for debug
  */
  Serial.print(" Sensors readings: ");
  for (int i=0; i<5; i++){
    Serial.print(irReadings[i]); Serial.print(" ");
  }
  // Serial.print("Error: "); Serial.print(error);
  Serial.println(" ");
  // vTaskDelay(400/portTICK_PERIOD_MS);
  
}

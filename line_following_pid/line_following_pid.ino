#define PWM_FREQUENCY   1000
#define PWM_RESOLUTION  8

#define M1A_PWM_CHANNEL 0
#define M1B_PWM_CHANNEL 1
#define M1_ENABLE_CHANNEL 2
#define M2A_PWM_CHANNEL 3
#define M2B_PWM_CHANNEL 4
#define M2_ENABLE_CHANNEL 5

#define STOPPED 0 
#define FOLLOWING_LINE 1 
#define NO_LINE 2

#define RIGHT 1
#define LEFT -1

#define MAX_SPEED 150

const int irSensors[] = {13, 4, 5, 15, 18}; //IR sensor pins

const int motor1Forward = 27;
const int motor1Backward = 14;
const int motor1pwmPin = 19;
const int motor2Forward = 25;
const int motor2Backward = 26;
const int motor2pwmPin = 23;

const int motor2Speed = 120; 
const int motor1Speed = 120; 

const int adj = 1;
float adjTurn = 8;

int mode = FOLLOWING_LINE;

float pTerm, iTerm, dTerm;
int error;
int previousError;
float kp = 11; //11
float ki = 0;
float kd = 11; //11
float output;
int integral, derivative;
int irReadings[5];
int motor1newSpeed;
int motor2newSpeed;

void setup() {
  // Declare all Motor Channels
  ledcSetup(M1A_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(M1B_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(M1_ENABLE_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(M2A_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(M2B_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(M2_ENABLE_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);

  // Attach Motor Pins to Channels  
  ledcAttachPin(motor1pwmPin, M1_ENABLE_CHANNEL);
  ledcAttachPin(motor2pwmPin, M2_ENABLE_CHANNEL);

  pinMode(motor1Forward, OUTPUT);
  pinMode(motor1Backward, OUTPUT);
  pinMode(motor2Forward, OUTPUT);
  pinMode(motor2Backward, OUTPUT);
  pinMode(motor1pwmPin,OUTPUT);
  pinMode(motor2pwmPin,OUTPUT);
  
  // Begin Serial Monitor
  Serial.begin(115200);
  Serial.println("ESP32 PD Line Following Robot");

  delay(500);
}

void readIRSensors() {
  //Read the IR sensors and put the readings in irReadings array
  for (int pin = 0; pin < 5; pin++) {
    int pinNum = irSensors[pin];
    irReadings[pin] = digitalRead(pinNum);
  }
  delay(100);

}


void calculateError() {
  
  //Determine an error based on the readings
  if ((irReadings[0] == 0) && (irReadings[1] == 0) && (irReadings[2] == 0) && (irReadings[3] == 0) && (irReadings[4] == 1)) {
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
  
  if ((irReadings[0] == 1) && (irReadings[1] == 0) && (irReadings[2] == 0) && (irReadings[3] == 0) && (irReadings[4] == 0)) {
    error = -4; mode = FOLLOWING_LINE;
  } 
  else if ((irReadings[0] == 0) && (irReadings[1] == 0) && (irReadings[2] == 0) && (irReadings[3] == 0) && (irReadings[4] == 0)) {
    error = 0; mode = NO_LINE;
  } 
  else if ((irReadings[0] == 1) && (irReadings[1] == 1) && (irReadings[2] == 1) && (irReadings[3] == 1) && (irReadings[4] == 1)) {
    error = 0; mode = STOPPED;
  }
  // Serial.print(" Error: "); Serial.print(error);

//  Serial.println(mode);
}

void pidCalculations() {
  pTerm = kp * error;
  integral += error;
  iTerm = ki * integral;
  derivative = error - previousError;
  dTerm = kd * derivative;
  output = pTerm + iTerm + dTerm;
  previousError = error;
}

void changeMotorSpeed() {
  //Change motor speed of both motors accordingly
  Serial.print(" OUTPUT : ");Serial.print(output);
  motor1newSpeed = motor1Speed + output;
  motor2newSpeed = motor2Speed - output;
  
  //Constrain the new speed of motors to be between the range 0-255
  constrain(motor2newSpeed, 0, 255);
  constrain(motor1newSpeed, 0, 255);

  Serial.print("left speed: "); Serial.print(motor2newSpeed);
  Serial.print(" right speed: "); Serial.println(motor1newSpeed);
  
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
  delay(200);
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

void printIRSensors(){
  /*
  Print the sensors' output
  Just for debug
  */
  Serial.print(" Sensors readings: ");
  for (int i=0; i<5; i++){
    Serial.print(irReadings[i]); Serial.print(" ");
  }
  // Serial.print("Error: "); Serial.print(error);
  Serial.print(" ");
  
}

void loop() {
  //Put all of our functions here
  readIRSensors();
  calculateError();
  // Serial.print(" MODE : ");Serial.print(mode);
  // printIRSensors();
  if (mode == STOPPED){
    // Serial.println("-- STOPPED -- ");
    motorStop();
  }else if (mode==NO_LINE){
    // Serial.println("-- NO_LINE -- ");
    // motorStop();
    delay(400);
    motorTurn(RIGHT,180);
  }else{
    // Serial.println("-- FOLLOWING_LINE -- ");
    pidCalculations();
    changeMotorSpeed();
  }
  // delay(500);
  // switch(mode) {
  //   case STOPPED:
  //     Serial.println("STOPPED");
  //     motorStop();
  //     break;
  //   case NO_LINE:
  //     // Serial.println("TURN");
  //     motorStop();
  //     // motorTurn(LEFT, 180);
  //     break;
  //   case FOLLOWING_LINE:
  //     // Serial.println("FOLLOWING_LINE");
  //     pidCalculations();
  //     changeMotorSpeed();
  // }
}

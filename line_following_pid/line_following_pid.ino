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
#define BASE_SPEED 120
#define MAX_SPEED 180

const int irSensors[] = {18, 15, 5, 4, 13}; //IR sensor pins

const int motor1Forward = 27;
const int motor1Backward = 14;
const int motor1pwmPin = 19;
const int motor2Forward = 25;
const int motor2Backward = 26;
const int motor2pwmPin = 23;
const int motor2Speed = 120; 
const int motor1Speed = 120; 
int motor2newSpeed,motor1newSpeed;
int mode = FOLLOWING_LINE;


float pTerm, iTerm, dTerm;
int error=0;
int previousError=0;
// GAIN CONTSTANTS
float kp = 20;
float ki = 0;
float kd = 60; 
float PIDvalue;
int integral, derivative;
int irReadings[5];

TaskHandle_t PIDTask_handle = NULL;


void PIDtask(void * parameters){
  for(;;){
    if (error==5 || error ==-5){
          vTaskSuspend(NULL);
    }
    readIRSensors();
    calculateError();
    // task_wdt: Task watchdog got triggered. The following tasks did not reset the watchdog in time:
    vTaskDelay(5/portTICK_PERIOD_MS);
  }
}

void motorTask(void * parameters){
  for(;;){
    if (mode == STOPPED){
      // Serial.print("-- STOPPED -- ");
      motorStop();
    }else if (mode==NO_LINE){
      // Serial.print("-- NO_LINE -- ");
      motorStop();
      // motorTurn(LEFT, 180);
    }
    else{
      // Serial.print("-- FOLLOWING_LINE -- ");
      calculatePID();
      changeMotorSpeed();
    }
    vTaskDelay(5/portTICK_PERIOD_MS);
  }
}
void debugTask(void * parameters){
  for(;;){
    if(error==-5){
      Serial.print(motor2newSpeed);
      Serial.print(", "); 
      Serial.print("-"); 
      Serial.print(motor1newSpeed);
      Serial.print(", "); 
      Serial.print(error);
      Serial.println();
    }else if (error ==5){
      Serial.print("-"); 
      Serial.print(motor2newSpeed); 
      Serial.print(", ");
      Serial.print(motor1newSpeed);
      Serial.print(", ");
      Serial.print(error);
      Serial.println();
    }else if (error != 0) {
      Serial.print(motor2newSpeed);
      Serial.print(", "); 
      Serial.print(motor1newSpeed);
      Serial.print(", "); 
      Serial.print(error);
      Serial.println();
    }
    
    vTaskDelay(5/portTICK_PERIOD_MS);
  }
}
void setup() {
  // Begin Serial Monitor
  Serial.begin(115200);
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
  
  Serial.println("ESP32 PD Line Following Robot");
  xTaskCreate(
    PIDtask, // function name
    "PID task",
    8000, // Stack size
    NULL, // Task parameter
    1, // Task priority
    &PIDTask_handle // Task handler
  );
  xTaskCreate(
    motorTask, // function name
    "motor task",
    8000, // Stack size
    NULL, // Task parameter
    1, // Task priority
    NULL
  );
  xTaskCreate(
    debugTask, // function name
    "Debug task",
    2000, // Stack size
    NULL, // Task parameter
    1, // Task priority
    NULL
  );
  vTaskDelay(500/portTICK_PERIOD_MS);

}




void loop() {
}

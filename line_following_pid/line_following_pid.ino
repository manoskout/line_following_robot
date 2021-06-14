#include <PubSubClient.h>
#include <WiFi.h>
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


int mode = FOLLOWING_LINE;
// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;
// 1883 is the listener port for the Broker
PubSubClient client(mqtt_server, 1885, wifiClient); 



int motor1newSpeed;
int motor2newSpeed;

void PIDtask(void * parameters){
  for(;;){
    String test="connected";
    vTaskDelay(500/portTICK_PERIOD_MS);
    if (client.publish(test_topic, test.c_str())) {
    Serial.println("Message sent!");
  }
  }
}

void motorTask(void * parameter){
  for(;;){
    readIRSensors();
    calculateError();
    if (mode == STOPPED){
    // Serial.println("-- STOPPED -- ");
    motorStop();
    }else if (mode==NO_LINE){
      // Serial.println("-- NO_LINE -- ");
      // motorStop();
      vTaskDelay(400/portTICK_PERIOD_MS);
      motorTurn(RIGHT,180);
    }else{
      // Serial.println("-- FOLLOWING_LINE -- ");
      pidCalculations();
      changeMotorSpeed();
    }
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

  // Initialize wifi
  if(init_wifi()) {Serial.println("WIFI--OK--");}
  else {Serial.println("WIFI--OK--");}
  
  connect_mqtt();
  Serial.println("ESP32 PD Line Following Robot");
  xTaskCreate(
    PIDtask, // function name
    "PID task",
    8000, // Stack size
    NULL, // Task parameter
    1, // Task priority
    NULL
  );
  xTaskCreate(
    motorTask, // function name
    "motor task",
    8000, // Stack size
    NULL, // Task parameter
    1, // Task priority
    NULL
  );
  vTaskDelay(500/portTICK_PERIOD_MS);

}




void loop() {
  // //Put all of our functions here
  // readIRSensors();
  // calculateError();
  // if (mode == STOPPED){
  //   // Serial.println("-- STOPPED -- ");
  //   motorStop();
  // }else if (mode==NO_LINE){
  //   // Serial.println("-- NO_LINE -- ");
  //   // motorStop();
  //   delay(400);
  //   motorTurn(RIGHT,180);
  // }else{
  //   // Serial.println("-- FOLLOWING_LINE -- ");
  //   pidCalculations();
  //   changeMotorSpeed();
  // }
}

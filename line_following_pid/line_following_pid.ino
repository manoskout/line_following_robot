// #include <PubSubClient.h>
// #include <WiFi.h>
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

#define TURN_RIGHT 3
#define TURN_LEFT -3

#define MAX_SPEED 200

const int irSensors[] = {13, 4, 5, 15, 18}; //IR sensor pins

const int motor1Forward = 27;
const int motor1Backward = 14;
const int motor1pwmPin = 19;
const int motor2Forward = 25;
const int motor2Backward = 26;
const int motor2pwmPin = 23;

int mode = FOLLOWING_LINE;
// // For WIFI
// const char* ssid = "Koutoulakis";
// const char* password = "2810751032";
// bool internet_connected = false;

// // For MQTT
// const char* mqtt_server = "192.168.1.99";  // IP of the MQTT broker
// int mqtt_port=1885;
// const char* car_topic = "car/car";
// const char* imu_topic = "car/imu";
// const char* test_topic = "car/test";
// const char* mqtt_username = "manos"; // MQTT username
// const char* mqtt_password = "tp4002"; // MQTT password
// const char* clientID = "manos"; // MQTT client ID
// // Initialise the WiFi and MQTT Client objects
// WiFiClient espClient;
// // 1883 is the listener port for the Broker
// PubSubClient client(espClient); 

void PIDtask(void * parameters){
  for(;;){
    // if (!client.connected()) {
      // reconnect();
    // }
    // String test="connected";
    // vTaskDelay(500/portTICK_PERIOD_MS);
    // if (client.publish(test_topic, test.c_str())) {
    // Serial.println("Message sent!");
  // }
    readIRSensors();
//    printIRSensors();
    calculateError();
    // task_wdt: Task watchdog got triggered. The following tasks did not reset the watchdog in time:
    vTaskDelay(10/portTICK_PERIOD_MS);

  }
}

void motorTask(void * parameter){
  for(;;){
    
    // Serial.println("mode: "+ String(mode));
    if (mode == STOPPED){
      // Serial.print("-- STOPPED -- ");
      motorStop();
    }else if (mode==NO_LINE){
      // Serial.print("-- NO_LINE -- ");
      motorStop();
    }else if (mode == TURN_RIGHT){
      motorTurn(TURN_RIGHT,90);
    }else if (mode == TURN_LEFT){
      motorTurn(TURN_LEFT,90);
    }else{
      // Serial.print("-- FOLLOWING_LINE -- ");
      pidCalculations();
      changeMotorSpeed();
    }
    vTaskDelay(10/portTICK_PERIOD_MS);
    // Serial.println();
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

  // // Initialize wifi
  // if(init_wifi()) {Serial.println("WIFI--OK--");}
  // else {Serial.println("WIFI--OK--");}
  
  
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
}

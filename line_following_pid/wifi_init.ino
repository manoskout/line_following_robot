/ For WIFI
const char* ssid = "Koutoulakis";
const char* password = "2810751032";
bool internet_connected = false;

// For MQTT
const char* mqtt_server = "192.168.1.99";  // IP of the MQTT broker
const char* car_topic = "car/car";
const char* imu_topic = "car/imu";
const char* test_topic = "car/test";
const char* mqtt_username = "manos"; // MQTT username
const char* mqtt_password = "tp4002"; // MQTT password
const char* clientID = "car"; // MQTT client ID
// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;
// 1883 is the listener port for the Broker
PubSubClient client(mqtt_server, 1885, wifiClient); 



bool init_wifi(){
  int connAttempts = 0;
  Serial.println("\r\nConnecting to: " + String(ssid));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
    if (connAttempts > 10) return false;
    connAttempts++;
  }
  
  return true;
}


void connect_mqtt(){
  // Connect to MQTT Broker
  // client.connect returns a boolean value to let us know if the connection was successful.
  // If the connection is failing, make sure you are using the correct MQTT Username and Password (Setup Earlier in the Instructable)
  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }
}

// bool init_wifi(){
//   int connAttempts = 0;
//   Serial.println("\r\nConnecting to: " + String(ssid));
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED ) {
//     delay(500);
//     Serial.print(".");
//     if (connAttempts > 10) return false;
//     connAttempts++;
//   }
  
//   return true;
// }

// void reconnect(){
//   // Loop until we're reconnected
//   while (!client.connected()){
//     Serial.println("Attempting MQTT connection...");
//     if (client.connect(clientID)){
//       // TODO-> the subscribes need to be changed !!!
//       Serial.println("Connected");
//       client.subscribe(car_topic);
//       client.subscribe(imu_topic);
//       client.subscribe(test_topic);

//     }
//     else{
//       Serial.print(client.state());
//       Serial.println("Failed - Try again in 5 seconds");
//       delay(5000);
//     }
//   }
// }

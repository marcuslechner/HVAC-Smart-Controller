/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp8266-nodemcu-mqtt-publish-dht11-dht22-arduino/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>

#define WIFI_SSID "TrapHouse" //Wifi SSID
#define WIFI_PASSWORD "thesimpsons" //Wifi Password

#define MQTT_HOST IPAddress(192, 168, 0, 34)//MQTT Host IP
#define MQTT_PORT 1883 //MQTT Host Port

#define MQTT_PUB_TEMP "esp/dht/temperature"
#define MQTT_PUB_PRESS "esp/dht/pressure"
#define MQTT_PUB_HUM "esp/dht/humidity"

//intialize objects
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
Adafruit_BME280 bme;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

//initialize variables
const int filter = 30; //filter size for oving average
unsigned long previousMillis = 0;   // Stores last time temperature was published
const long intervalPub = 2000;        // intervalPub at which to publish sensor readings
int i = 0;
//data array
float temp[100];// = 0.00;
float humid[100];//  = 0.00;
float pressure[100];// = 0.00;
//sum of data array
float tempsum = 0.00;
float humidsum = 0.00;
float pressuresum = 0.00;



void setup() 
{
  Serial.begin(115200); //initialize serial monitor for debugging
  bme.begin(0x76); //I2C address 

  //Wifi Handler initialization
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  //MQTT initialization
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials("mlechner", "Herrner1");
  connectToWifi();

  //Fill data variables with 100 data values
  for(i = 0; i < 99; i++)
  {
    temp[i] = bme.readTemperature();
    humid[i] = bme.readHumidity();
    pressure[i] = bme.readPressure();
  }
}

void loop() 
{
  unsigned long currentMillis = millis(); //for timer

  if (currentMillis - previousMillis >= intervalPub) //has if been intervalPub milliseconds since last publication
  {
    previousMillis = currentMillis;  // Save the last time a new reading was published

    pressure[filter-1] = bme.readPressure(); //read data into the last index of the array
    pressuresum = 0;//reset the average
    for(i = 0; i < filter-1; i++) //shift values down the array and delete the first index value
    {
      pressure[i] = pressure[i + 1];
    }
    for(i = 0; i < filter; i++) //sum values
    {
      pressuresum = pressuresum + pressure[i];
    }
    pressuresum = pressuresum/filter;


    temp[filter - 1] = bme.readTemperature(); //read data into the last index of the array
    tempsum = 0; //reset the average
    for(i = 0; i < filter - 1; i++) //shift values down the array and delete the first index value
    {
      temp[i] = temp[i + 1];
    }
    for(i = 0; i < filter; i++)  //sum values
    {
      tempsum = tempsum + temp[i];
    }
    tempsum = tempsum/filter;

    humid[filter - 1] = bme.readHumidity(); //read data into the last index of the array
    humidsum = 0; //reset the average
    for(i = 0; i < filter - 1; i++) //shift values down the array and delete the first index value
    {
      humid[i] = humid[i + 1];
    }
    for(i = 0; i < filter; i++)  //sum values
    {
      humidsum = humidsum + humid[i];
    }
    humidsum = humidsum/filter;
    
    // Publish an MQTT message containing Temperature Readings
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(tempsum).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP, packetIdPub1);
    Serial.printf("Message: %.2f \n", tempsum);

    // Publish an MQTT message containing Humidity Readings
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(humidsum).c_str());                             
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
    Serial.printf("Message: %.2f \n", humidsum);

    // Publish an MQTT message containing Pressure Readings
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_PRESS, 1, true, String(pressuresum).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_PRESS, packetIdPub2);
    Serial.printf("Message: %.2f \n", pressuresum);
  }
}

void connectToWifi()
{
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) 
{
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt(); 
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) 
{
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void connectToMqtt() 
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) 
{
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) 
{
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttPublish(uint16_t packetId) 
{
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

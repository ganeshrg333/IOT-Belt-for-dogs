#include <UbidotsESPMQTT.h>

/****************************************
  Include Libraries
****************************************/
#include "UbidotsESPMQTT.h"
#include <OneWire.h>
#include <DallasTemperature.h>

/****************************************
  Define Constants and Instances
****************************************/
#define TOKEN "BBFF-MOKqBFugZOwNb0qigyN9NTji6iKQ5k" // Your Ubidots TOKEN
#define WIFINAME "T" // Your SSID
#define WIFIPASS "sahijwani" // Your Wifi Pass
#define MQTTCLIENTNAME "Tanish" // Your MQTT Client Name, it must be unique so we recommend to choose a random ASCCI name
#define Pin D3 
int temp;

OneWire ourWire(Pin);
DallasTemperature sensors(&ourWire);
Ubidots client(TOKEN, MQTTCLIENTNAME);

/****************************************
  Auxiliar Functions
****************************************/
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

/****************************************
  Main Functions
****************************************/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  client.wifiConnection(WIFINAME, WIFIPASS);
  sensors.begin();
  client.begin(callback);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    client.reconnect();
  }
  sensors.requestTemperatures();
  //Prepare the sensor for reading
  temp = sensors.getTempCByIndex(0);
  Serial.print(sensors.getTempCByIndex(0));
  //Read and print the temperature in Celsius
  client.add("temperature", temp);
  //Insert your variable Labels and the value to be sent
  delay(1000);
  Serial.println("Grados Centigrados");
  client.ubidotsPublish("True");
  client.loop();
}

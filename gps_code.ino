/*************************************************************************************************
 * This Example sends harcoded data to Ubidots and serves as example for users that have devices
 * based on ESP8266 chips
 *
 * This example is given AS IT IS without any warranty
 *
 * Made by Jose García @https://github.com/jotathebest/ , 
 * adapted from the original WiFiClient ESP8266 example
 *************************************************************************************************/

/****************************************
 * Include Libraries
 ****************************************/

#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
//#include <ESP8266WiFiMulti.h>


static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
#ifndef WIFICLIENTMULTI_H_
#define WIFICLIENTMULTI_H_

#include "ESP8266WiFi.h"
#include <vector>

#ifdef DEBUG_ESP_WIFI
#ifdef DEBUG_ESP_PORT
#define DEBUG_WIFI_MULTI(fmt, ...) DEBUG_ESP_PORT.printf_P( (PGM_P)PSTR(fmt), ##__VA_ARGS__ )
#endif
#endif

#ifndef DEBUG_WIFI_MULTI
#define DEBUG_WIFI_MULTI(...) do { (void)0; } while (0)
#endif

struct WifiAPEntry {
    char * ssid;
    char * passphrase;
};

typedef std::vector<WifiAPEntry> WifiAPlist;

class ESP8266WiFiMulti {
    public:
        ESP8266WiFiMulti();
        ~ESP8266WiFiMulti();

        bool addAP(const char* ssid, const char *passphrase = NULL);
        bool existsAP(const char* ssid, const char *passphrase = NULL);

        wl_status_t run(void);

        void cleanAPlist(void);

    private:
        WifiAPlist APlist;
        bool APlistAdd(const char* ssid, const char *passphrase = NULL);
        bool APlistExists(const char* ssid, const char *passphrase = NULL);
        void APlistClean(void);

};

#endif /* WIFICLIENTMULTI_H_ */
#include <stdio.h>


/****************************************
 * Define Constants
 ****************************************/

#define WIFISSID "T" // Put your WifiSSID here
#define PASSWORD "sahijwani" // Put your wifi password here
#define TOKEN "BBFF-MOKqBFugZOwNb0qigyN9NTji6iKQ5k" // Put your Ubidots' TOKEN
#define VARIABLE_LABEL "position" // Assing the variable label
#define DEVICE_LABEL "BELT" // Assig the device label
#define MQTT_CLIENT_NAME "...." // MQTT client Name, put a Random ASCII

char mqttBroker[] = "industrial.api.ubidots.com";
char payload[700];
char topic[150];

// Space to store values to send
char str_val[6];
char str_lat[6];
char str_lng[6];


/****************************************
 * Initializate constructors for objects
 ****************************************/

ESP8266WiFiMulti WiFiMulti;
WiFiClient ubidots;
PubSubClient client(ubidots);


/****************************************
 * Auxiliar Functions
 ****************************************/
 
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
  Serial.print((char)payload[i]);
  }
  Serial.println();
} 

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
  Serial.println("Attempting MQTT connection...");

  // Attempt to connect
  if (client.connect(MQTT_CLIENT_NAME, TOKEN,"")) {
    Serial.println("connected");
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 2 seconds");
    // Wait 2 seconds before retrying
    delay(2000);
  }
  }
}


/****************************************
 * Main Functions
 ****************************************/

void setup() {
  Serial.begin(115200);
  pinMode(A0, INPUT);
  WiFiMulti.addAP(WIFISSID, PASSWORD);
  Serial.println();
  Serial.println();
  Serial.print("Wait for WiFi... ");

  while(WiFiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqttBroker, 1883);
  client.setCallback(callback);
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }

  // Values to send
  float belt_position = random(0, 9);
  float lat = gps.location.lat();
  float lng = gps.location.lng();

  /* 4 is mininum width, 2 is precision; float value is copied onto str_temp*/
  dtostrf(belt_position, 4, 2, str_val);
  dtostrf(lat, 4, 2, str_lat);
  dtostrf(lng, 4, 2, str_lng);

  sprintf(topic, "%s", ""); // Cleans the topic content
  sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);

  sprintf(payload, "%s", ""); // Cleans the payload content
  sprintf(payload, "{\"%s\":", VARIABLE_LABEL); // Adds the variable label   
  sprintf(payload, "%s {\"value\": %s", payload, str_val); // Adds the value
  sprintf(payload, "%s, \"context\":{\"lat\": %s, \"lng\": %s}", payload, str_lat, str_lng); // Adds coordinates
  sprintf(payload, "%s } }", payload); // Closes the dictionary brackets

  client.publish(topic, payload);
  client.loop();
  delay(1000);
}

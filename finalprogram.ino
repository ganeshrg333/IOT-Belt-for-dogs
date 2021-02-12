/****************************************
  Include Libraries
****************************************/
#include "Arduino.h"
#include "MPU6050.h"
#include "Wire.h"
#include "I2Cdev.h"
#include <UbidotsESPMQTT.h>
#include "UbidotsESPMQTT.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>
#include <stdio.h>
/****************************************
  Code for ESP8266WiFiMulti.h
****************************************/
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
/****************************************
  Define Constants and Instances
****************************************/
#define TOKEN "BBFF-MOKqBFugZOwNb0qigyN9NTji6iKQ5k" // Your Ubidots TOKEN
#define WIFINAME "T" // Your SSID
#define WIFIPASS "sahijwani" // Your Wifi Pass
#define MQTTCLIENTNAME "Tanish" // Your MQTT Client Name, it must be unique so we recommend to choose a random ASCCI name
#define VARIABLE_LABEL "position" // Assing the variable label
#define DEVICE_LABEL "BELT" // Assig the device label
#define Pin D3 
#define pulsePin A0
int temp;

static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
OneWire ourWire(Pin);
DallasTemperature sensors(&ourWire);
Ubidots client(TOKEN,MQTTCLIENTNAME);

// Global variables and defines
int16_t mpu6050Ax, mpu6050Ay, mpu6050Az;
int16_t mpu6050Gx, mpu6050Gy, mpu6050Gz;
// object initialization
MPU6050 mpu6050;


// define vars for testing menu
const int timeout = 10000;       //define timeout of 10 sec
char menuOption = 0;
long time0;

int rate[10];                    
unsigned long sampleCounter = 0; 
unsigned long lastBeatTime = 0;  
unsigned long lastTime = 0, N;
int BPM = 0;
int IBI = 0;
int P = 512;
int T = 512;
int thresh = 512;  
int amp = 100;                   
int Signal;
boolean Pulse = false;
boolean firstBeat = true;          
boolean secondBeat = true;
boolean QS = false;    

char mqttBroker[] = "industrial.api.ubidots.com";
char payload[700];
char topic[150];

// Space to store values to send
char str_val[6];
char str_lat[6];
char str_lng[6];

/****************************************
  Auxiliar Functions
****************************************/
void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

/*void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
  Serial.println("Attempting MQTT connection...");

  // Attempt to connect
  if (client.connect(MQTTCLIENTNAME,TOKEN,"")) {
    Serial.println("connected");
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 2 seconds");
    // Wait 2 seconds before retrying
    delay(2000);
  }
  }
}*/


/****************************************
  Main Functions
****************************************/
void setup() 
{
  Serial.begin(9600);
  client.wifiConnection(WIFINAME, WIFIPASS);
  sensors.begin();
  client.begin(callback);
  Wire.begin();
  mpu6050.initialize();
  
 // WiFiMulti.addAP(WIFINAME, WIFIPASS);
 // client.setServer(mqttBroker, 1883);
  //client.setCallback(callback)
}

void loop()
{ 
// put your main code here, to run repeatedly:
  if (!client.connected()) 
  {
    client.reconnect();
  }
  
  sensors.requestTemperatures();
  //Prepare the sensor for reading
  temp = sensors.getTempCByIndex(0);
  Serial.print(sensors.getTempCByIndex(0));
  
    mpu6050.getMotion6(&mpu6050Ax, &mpu6050Ay, &mpu6050Az, &mpu6050Gx, &mpu6050Gy, &mpu6050Gz); 
  //read accelerometer and gyroscope raw data in three axes
    double mpu6050Temp = ((double)mpu6050.getTemperature() + 12412.0) / 340.0;
    Serial.print("a/g-\t");
    Serial.print(mpu6050Ax); Serial.print("\t");
    Serial.print(mpu6050Ay); Serial.print("\t");
    Serial.print(mpu6050Az); Serial.print("\t");
    Serial.print(mpu6050Gx); Serial.print("\t");
    Serial.print(mpu6050Gy); Serial.print("\t");
    Serial.print(mpu6050Gz); Serial.print("\t");
    Serial.print(F("Temp- "));   
    Serial.println(mpu6050Temp);
	
	if (QS == true) 
   {
   Serial.println("BPM: "+ String(BPM));
   QS = false;
   } else if (millis() >= (lastTime + 2))
   {
     readPulse(); 
     lastTime = millis();
    // client.add("pulse",BPM);
    // delay(1000);
	}
	 
   //Read and print 
  client.add("temperature", temp);
  client.add("motion",mpu6050Az);
  client.add("pulse",BPM);
  //Insert your variable Labels and the value to be sent
  delay(1000);
  client.ubidotsPublish("IoTBelt");
  client.loop();
  
  
  float belt_position = random(0, 9);
  float lat =gps.location.lat();
  float lng= gps.location.lng();

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

  client.ubidotsPublish(payload);
  client.loop();
  delay(1000);
}

/****************************************
  Functions call for Pulse sensor
****************************************/


void readPulse()
 {

  Signal = analogRead(pulsePin);              
  sampleCounter += 2;                           
  int N = sampleCounter - lastBeatTime;   

  detectSetHighLow();

  if (N > 250) 
  {  
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI / 5) * 3) )
      pulseDetected();
  }

  if (Signal < thresh && Pulse == true)
   {  
    Pulse = false;
    amp = P - T;
     T = thresh;
  }

  if (N > 2500) 
  {
    thresh = 512;
    P = 512;
    T = 512;
    lastBeatTime = sampleCounter;
    firstBeat = true;            
    secondBeat = true;           
  }

}

void detectSetHighLow()
 {

  if (Signal < thresh && N > (IBI / 5) * 3)
	  {
    if (Signal < T)
	   {                       
        T = Signal;                         
       }
      }

  if (Signal > thresh && Signal > P) 
  {    
    P = Signal;                           
  }                                       

}

void pulseDetected() 
{
  Pulse = true;                           
  IBI = sampleCounter - lastBeatTime;     
  lastBeatTime = sampleCounter;           

  if (firstBeat) 
  {                       
    firstBeat = false;                 
    return;                            
  }
  if (secondBeat)
	  {                    
        secondBeat = false;                
        for (int i = 0; i <= 9; i++)
		{   
         rate[i] = IBI;
        }
  }

  word runningTotal = 0;                   

  for (int i = 0; i <= 8; i++)
	{          
    rate[i] = rate[i + 1];            
    runningTotal += rate[i];          
    }

  rate[9] = IBI;                      
  runningTotal += rate[9];            
  runningTotal /= 10;                 
  BPM = 60000 / runningTotal;         
  QS = true;
                            
}

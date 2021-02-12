#define pulsePin A0
#include <ESP8266WiFi.h>

#include <UbidotsESPMQTT.h>

/****************************************
  Include Libraries
****************************************/
#include "UbidotsESPMQTT.h"
//#include <OneWire.h>
//#include <DallasTemperature.h>

/****************************************
  Define Constants and Instances
****************************************/
#define TOKEN "BBFF-MOKqBFugZOwNb0qigyN9NTji6iKQ5k" // Your Ubidots TOKEN
#define WIFINAME "T" // Your SSID
#define WIFIPASS "sahijwani" // Your Wifi Pass
#define MQTTCLIENTNAME "Tanish" // Your MQTT Client Name, it must be unique so we recommend to choose a random ASCCI name
Ubidots client(TOKEN, MQTTCLIENTNAME);

// Pin Definitions
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}


//const char* ssid = "qwerty"; 
//const char* password = "12345678";

//WiFiClient client;

//const int channelID = 605045;
//String writeAPIKey = "LRO5YB9WB6JM4JOS"; 
//const char* server = "api.thingspeak.com";
//const int postingInterval = 10 * 1000; 

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

void setup() {
Serial.begin(115200);
  client.wifiConnection(WIFINAME, WIFIPASS);
  //sensors.begin();
 client.begin(callback);

  
  /*Serial.begin(9600);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);*/
  //}
}

void loop() {
if (!client.connected()) {
    client.reconnect();
  }



  if (QS == true) {
   Serial.println("BPM: "+ String(BPM));
   QS = false;
   } else if (millis() >= (lastTime + 2)) {
     readPulse(); 
     lastTime = millis();

client.add("pulse",BPM);
  //Insert your variable Labels and the value to be sent
  delay(1000);
  Serial.println("Grados Centigrados");
  client.ubidotsPublish("control");
  client.loop();

     
   }     
}



void readPulse() {

  Signal = analogRead(pulsePin);              
  sampleCounter += 2;                           
  int N = sampleCounter - lastBeatTime;   

  detectSetHighLow();

  if (N > 250) {  
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI / 5) * 3) )
      pulseDetected();
  }

  if (Signal < thresh && Pulse == true) {  
    Pulse = false;
    amp = P - T;
    thresh = amp / 2 + T;  
    P = thresh;
    T = thresh;
  }

  if (N > 2500) {
    thresh = 512;
    P = 512;
    T = 512;
    lastBeatTime = sampleCounter;
    firstBeat = true;            
    secondBeat = true;           
  }

}

void detectSetHighLow() {

  if (Signal < thresh && N > (IBI / 5) * 3) {
    if (Signal < T) {                       
      T = Signal;                         
    }
  }

  if (Signal > thresh && Signal > P) {    
    P = Signal;                           
  }                                       

}

void pulseDetected() {
  Pulse = true;                           
  IBI = sampleCounter - lastBeatTime;     
  lastBeatTime = sampleCounter;           

  if (firstBeat) {                       
    firstBeat = false;                 
    return;                            
  }
  if (secondBeat) {                    
    secondBeat = false;                
    for (int i = 0; i <= 9; i++) {   
      rate[i] = IBI;
    }
  }

  word runningTotal = 0;                   

  for (int i = 0; i <= 8; i++) {          
    rate[i] = rate[i + 1];            
    runningTotal += rate[i];          
  }

  rate[9] = IBI;                      
  runningTotal += rate[9];            
  runningTotal /= 10;                 
  BPM = 60000 / runningTotal;         
  QS = true;
  
  /*if (client.connect(server, 80)) {
    
    String body = "field1=";
           body += BPM;

    client.println("POST /update HTTP/1.1");
    client.println("Host: api.thingspeak.com");
    client.println("User-Agent: ESP8266 (nothans)/1.0");
    client.println("Connection: close");
    client.println("X-THINGSPEAKAPIKEY: " + writeAPIKey);
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.println("Content-Length: " + String(body.length()));
    client.println("");
    client.print(body);

  }
  client.stop();
  delay(postingInterval);*/                              
}

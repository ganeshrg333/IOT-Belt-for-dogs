//#include <PubSubClient.h>

#include "Arduino.h"
#include "MPU6050.h"
#include "Wire.h"
#include "I2Cdev.h"
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




// Global variables and defines
int16_t mpu6050Ax, mpu6050Ay, mpu6050Az;
int16_t mpu6050Gx, mpu6050Gy, mpu6050Gz;
// object initialization
MPU6050 mpu6050;


// define vars for testing menu
//const int timeout = 10000;       //define timeout of 10 sec
//char menuOption = 0;
//long time0;

// Setup the essentials for your circuit to work. It runs first every time your circuit is powered with electricity.
void setup() 
{
  Serial.begin(9600);
  client.wifiConnection(WIFINAME, WIFIPASS);
  //sensors.begin();
 client.begin(callback);
    // Setup Serial which is useful for debugging
    // Use the Serial Monitor to view printed messages
    //Serial.begin(9600);
    //while (!Serial) ; // wait for serial port to connect. Needed for native USB
    Serial.println("start");
    
    Wire.begin();
    mpu6050.initialize();
    //menuOption = menu();
    
}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop() 
{if (!client.connected()) {
    client.reconnect();
  }

  //sensors.requestMotion();
    
    
    //if(menuOption == '1') {
    // SparkFun MPU-6050 - Accelerometer and Gyro - Test Code
    mpu6050.getMotion6(&mpu6050Ax, &mpu6050Ay, &mpu6050Az, &mpu6050Gx, &mpu6050Gy, &mpu6050Gz);   //read accelerometer and gyroscope raw data in three axes
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
    delay(100);
client.add("motion123",mpu6050Ay);
  //Insert your variable Labels and the value to be sent
  delay(1000);
  Serial.println("Grados Centigrados");
  client.ubidotsPublish("control");
  client.loop();
    }
    
    //if (millis() - time0 > timeout)
    //{/
       // menuOption = menu();
    //}
    




// Menu function for selecting the components to be tested
// Follow serial monitor for instrcutions
/*char menu()
{

    //Serial.println(F("\nWhich component would you like to test?"));
    //Serial.println(F("(1) SparkFun MPU-6050 - Accelerometer and Gyro"));
///Serial.println(F("(menu) send anything else or press on board reset button\n"));
    while (!Serial.available());

    // Read data from serial monitor if received
    while (Serial.available()) 
    {
        char c = Serial.read();
        if (isAlphaNumeric(c)) 
        {   
            
            if(c == '1') 
          Serial.println(F("Now Testing SparkFun MPU-6050 - Accelerometer and Gyro"));
            else
            {
                Serial.println(F("illegal input!"));
                return 0;
            }
            time0 = millis();
            return c;
        }
    }
}
*/

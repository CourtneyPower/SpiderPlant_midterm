/*
 * Project Plant Watering System - Spider Plants (2024)
 * Author: Courtney Power
 * Date: Feb 2024
 * Measure Soil Moisture levels and determine if plants needs water, water plant
 * Measure ambient environmental factors (temp, press, humidity, dust, air quality)
 * Include a Adafruit Button to manually water plant
 * Include a Zapier-SMS interface
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "IoTClassroom_CNM.h"
#include "IoTTimer.h"
#include "Adafruit_BME280.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Air_Quality_Sensor.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentials.h"

const int OLED_RESET = -1;
const int MOTORPIN = D9;
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_BME280 roomSensor;
int moistureSensor = A5;
AirQualitySensor aqSensor(A2);
const int DUSTSENSOR = D6;


SYSTEM_MODE(AUTOMATIC);
const int READ_DUSTSENSOR = 30000;
int moistureReading, currentTime, lastSecond;
float humidRH, tempF, pressInHg;
//float tempC, pressPA,;
int lastInterval, duration, airQuality;
int lowPulseOccupancy = 0;
int last_lpo = 0;
float ratio = 0;
float concentration = 0;
bool status;
String DateTime, TimeOnly;
IoTTimer checkSensors;

void MQTT_connect();
bool MQTT_ping();
void getDustSensorReadings();
void calcRoomVals(float *humid, float *temp, float *press);
/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
//Adafruit_MQTT_Subscribe subButtonFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttononoff"); 
Adafruit_MQTT_Publish pubDustSensorFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dustsensor");
Adafruit_MQTT_Publish pubAirQualityFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airquality");

// Run the application and system concurrently in separate threads
// SYSTEM_THREAD(ENABLED);
// SerialLogHandler logHandler(LOG_LEVEL_INFO);

// setup() runs once, when the device is first turned on
void setup()
{
  // Put initialization like pinMode and begin functions here
  pinMode(A5, INPUT); // MouistureSensor is an INPUT
  pinMode (MOTORPIN, OUTPUT);
 digitalWrite(MOTORPIN, LOW);
  WiFi.connect();
  while (WiFi.connecting())
  {
    Serial.printf(",");
  }
  // Particle.connect();
  Time.zone(-7);
  Particle.syncTime();
  Serial.begin(9600);
  waitFor(Serial.isConnected, 10000);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
  status = roomSensor.begin(0x76);
    if (status == false) {
    Serial.printf ("BME280/TempSensor at address 0x%02X failed to start ", 0x76);
    }
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.display();
  delay(2000);
  display.clearDisplay();
  display.display();
}

// loop() runs over and over again, as quickly as it can execute.
void loop()
{
MQTT_connect();
MQTT_ping();
 //digitalWrite(MOTORPIN, LOW);
//Air Quality
airQuality = aqSensor.slope();
//Dust Concentration
duration = pulseIn(DUSTSENSOR, LOW);
lowPulseOccupancy = lowPulseOccupancy + duration;
if ((millis() - lastInterval) > READ_DUSTSENSOR) { //replace with IoTTimer
  getDustSensorReadings();
      if(mqtt.Update()) {
      pubDustSensorFeed.publish(concentration);
      pubAirQualityFeed.publish(airQuality);
      Serial.printf("Publishing %i air quality, %02f concentration \n",airQuality, concentration); 
      } 
  lowPulseOccupancy = 0;
  lastInterval = millis();
}
//BME280 
calcRoomVals(&humidRH, &tempF, &pressInHg);
// tempC = roomSensor.readTemperature();
// pressPA = roomSensor.readPressure();
// humidRH = roomSensor.readHumidity();
// tempF = map(tempC,0.0,100.0,32.0,212.0);
// pressInHg = (pressPA/3386.39);
//soil moisture reading
moistureReading = analogRead(moistureSensor);

Serial.printf("temp %02f, pressure %02f, humidity %02f\n", tempF, pressInHg, humidRH);
Serial.printf("moisture %i\n", moistureReading);
//OLED
display.clearDisplay();
 display.setCursor(0, 0);
display.printf("Moisture\n");
display.printf("%i\n", moistureReading);
display.printf("%s\n",TimeOnly.c_str());
display.display();
}

//additional functions
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}

void getDustSensorReadings() {
  if (lowPulseOccupancy == 0) {
    lowPulseOccupancy = last_lpo;
  }
  else {
    last_lpo = lowPulseOccupancy;
  }
  ratio = lowPulseOccupancy / (READ_DUSTSENSOR * 10.0);
  concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio,2) + 520 * ratio + 0.62;
// Serial.printf("LPO: %i\n", lowPulseOccupancy);
// Serial.printf ("Ratio: %02f\n", ratio);
// Serial.printf("Concentration: %02f cps/L\n", concentration);
}

void calcRoomVals(float *humid, float *temp, float *press) {
 float tempC, pressPA;
 tempC = roomSensor.readTemperature();
 pressPA = roomSensor.readPressure();
*humid = roomSensor.readHumidity();
*temp = map(tempC,0.0,100.0,32.0,212.0);
*press = (pressPA/3386.39);
}
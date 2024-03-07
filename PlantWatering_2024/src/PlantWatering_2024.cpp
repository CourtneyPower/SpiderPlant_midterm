/*
 * Project Plant Watering System - Spider Plants (2024)
 * Author: Courtney Power
 * Date: Feb/Mar 2024
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
const int SOILSENSOR = A2;
AirQualitySensor aqSensor(A0);
const int DUSTSENSOR = D6;


SYSTEM_MODE(AUTOMATIC);
int moistureReading, moistureVal, currentTime, lastSecond, subButtonState;
float humidRH, tempF, pressInHg;
int lastInterval, duration, airQuality;
int lowPulseOccupancy = 0;
int last_lpo = 0;
float ratio = 0;
float concentration = 0;
bool status;
String DateTime, TimeOnly;
IoTTimer publishTimer, checkPlantTimer;

void MQTT_connect();
bool MQTT_ping();
void calcRoomVals(float *humid, float *temp, float *press);
void waterPlant();
void publishValues();
void getConc();
/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
Adafruit_MQTT_Subscribe subButtonFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttononoff");
Adafruit_MQTT_Subscribe subEmailFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/email"); 
Adafruit_MQTT_Publish pubDustSensorFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dustsensor");
Adafruit_MQTT_Publish pubAirQualityFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airquality");
Adafruit_MQTT_Publish pubRoomTempFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/roomtemp");
Adafruit_MQTT_Publish pubRoomPressureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/roompressure");
Adafruit_MQTT_Publish pubRoomHumidityFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/roomhumidity");
Adafruit_MQTT_Publish pubSoilMoistureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soilmoisture");
// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

void setup()
{
  // Put initialization like pinMode and begin functions here
  pinMode(DUSTSENSOR, INPUT);
  pinMode (MOTORPIN, OUTPUT);
    digitalWrite(MOTORPIN, LOW);
  WiFi.connect();
  while (WiFi.connecting())
  {
    Serial.printf(",");
  }
  mqtt.subscribe(&subButtonFeed);
  mqtt.subscribe(&subEmailFeed);
  // Particle.connect();
  Time.zone(-7);
  Particle.syncTime();
  Serial.begin(9600);
  waitFor(Serial.isConnected, 15000);
  new Thread("concTread", getConc);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
  status = roomSensor.begin(0x76);
    if (status == false) {
    Serial.printf ("BME280/TempSensor at address 0x%02X failed to start ", 0x76);
    }
if (aqSensor.init()) {
  Serial.printf("Air Quality Sensor ready.\n");
} else {
  Serial.printf("Air Quality Sensor ERROR!\n");
}
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.display();
  delay(2000);
  display.clearDisplay();
  display.display();
publishTimer.startTimer(60000);//1 minute timer
checkPlantTimer.startTimer(60000); //1 minute timer
  moistureVal = 1100;
pinMode(SOILSENSOR, INPUT); 
}

void loop()
{
MQTT_connect();
MQTT_ping();
DateTime = Time.timeStr();
TimeOnly = DateTime.substring(11, 19);
//Air Quality
airQuality = aqSensor.slope(); //3 is good, 0 is danger

//BME280 function call
calcRoomVals(&humidRH, &tempF, &pressInHg);

Adafruit_MQTT_Subscribe *subscription;
while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &subButtonFeed) {
      subButtonState = atoi((char *)subButtonFeed.lastread);
    }
    if (subscription == &subEmailFeed) {
      digitalWrite(D7, HIGH);
      Serial.printf("You've got mail!\n");
      delay(500);
      digitalWrite(D7, LOW);
    }
}

if (checkPlantTimer.isTimerReady()){
  moistureReading = analogRead(A5);
  Serial.printf("moisture read %i\n", moistureReading);
  moistureVal = moistureReading;
  checkPlantTimer.startTimer(60000); //1min timer
}
 

if (publishTimer.isTimerReady()) { //once per minute
publishValues();
//OLED
display.clearDisplay();
display.setCursor(0, 0);
display.printf("%s\n",TimeOnly.c_str());
//display.printf("Soil %i\n", moistureReading);
display.printf("Temp %0.1f\n", tempF);
display.printf("PressHg %0.1f\n", pressInHg);
display.printf("Humid %0.1f\n", humidRH);
display.display();
delay(2000);
display.clearDisplay();
display.setCursor(0,0); //reset display b/c only allows 4 lines of text
display.printf("%s\n",TimeOnly.c_str());
display.printf("Dust %0.1f\n", concentration);
display.printf("AQ %i\n", airQuality);
display.display();
delay(2000);
display.clearDisplay();
display.display();
//troubleshooting prints
Serial.printf("temp %0.2f, pressure %0.2f, humidity %0.2f\n", tempF, pressInHg, humidRH);
Serial.printf("moisture reading %i\n", moistureReading);
Serial.printf("moisture value %i\n", moistureVal);
Serial.printf("Publishing %i air quality, %0.2f concentration \n",airQuality, concentration);
Serial.printf("Air Quality %i \n", airQuality);
Serial.printf("ButtonState %i \n", subButtonState);

publishTimer.startTimer(60000); // restart 1minute timer
}

if ((moistureVal >= 4096) || (subButtonState == 1)){  //preventing pump from constantly triggering, reset to ~1500 when fixed
    waterPlant();
    moistureVal = 1200;
} 

}
//end loop
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

void calcRoomVals(float *humid, float *temp, float *press) {
 float tempC, pressPA;
 tempC = roomSensor.readTemperature();
 pressPA = roomSensor.readPressure();
*humid = roomSensor.readHumidity();
*temp = map(tempC,0.0,100.0,32.0,212.0);
*press = (pressPA/3386.39);
}

void waterPlant() {
  digitalWrite(MOTORPIN,HIGH);
  delay(500); //0.5 seconds
  digitalWrite(MOTORPIN, LOW);
}

void publishValues() {
  if(mqtt.Update()) {
      pubDustSensorFeed.publish(concentration);
      pubAirQualityFeed.publish(airQuality);
      pubRoomTempFeed.publish(tempF);
      pubRoomPressureFeed.publish(pressInHg);
      pubRoomHumidityFeed.publish(humidRH);
     pubSoilMoistureFeed.publish(moistureReading);
      }
}

void getConc() {
  const int sampleTime = 30000;
  unsigned int duration, startTime;
  startTime = 0;
  lowPulseOccupancy = 0;
  while (true) {
    duration = pulseIn(DUSTSENSOR, LOW);
    lowPulseOccupancy = lowPulseOccupancy+duration;
    if ((millis()-startTime) > sampleTime) {
      ratio = lowPulseOccupancy/ (sampleTime * 10.0);
      concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62;
      startTime = millis();
      lowPulseOccupancy = 0;
    }
  }
}
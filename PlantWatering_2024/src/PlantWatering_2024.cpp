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
#include "Adafruit_SSD1306.h"
#include "Air_Quality_Sensor.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
//SYSTEM_THREAD(ENABLED);
//SerialLogHandler logHandler(LOG_LEVEL_INFO);

// setup() runs once, when the device is first turned on
void setup() {
  // Put initialization like pinMode and begin functions here
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // The core of your code will likely live here.

}

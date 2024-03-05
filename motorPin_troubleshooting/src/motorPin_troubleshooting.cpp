/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

const int MOTORPIN = D9;

// setup() runs once, when the device is first turned on
void setup() {
  // Put initialization like pinMode and begin functions here
pinMode (MOTORPIN, OUTPUT);
digitalWrite(MOTORPIN, LOW);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // The core of your code will likely live here.
delay(5000);
digitalWrite(MOTORPIN, HIGH);
Serial.printf("MotorPin is HIGH\n");
delay(10000);
digitalWrite(MOTORPIN, LOW);
Serial.printf("MotorPin is LOW\n");
delay(5000);
  // Example: Publish event to cloud every 10 seconds. Uncomment the next 3 lines to try it!
  // Log.info("Sending Hello World to the cloud!");
  // Particle.publish("Hello world!");
  // delay( 10 * 1000 ); // milliseconds and blocking - see docs for more info!
}

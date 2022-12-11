#include "SparkFun_Si7021_Breakout_Library.h"
#include <Wire.h>

#define MINTEMP 70
#define MAXTEMP 90
#define SECOND 1000
#define MINUTE 60000

// Pin assignments
const int power = A3;
const int GND = A2;
const int relay = 3;

// Variable for readings
float humidity = 0;
float tempf = 0;

// Heater state
bool heat = false;

// Create Instance SI7021 sensor
Weather sensor;

void setup()
{
    Serial.begin(9600);   // open serial over USB at 9600 baud

    pinMode(power, OUTPUT);
    pinMode(GND, OUTPUT);

    digitalWrite(power, HIGH);
    digitalWrite(GND, LOW);

    //Initialize the I2C sensors and ping them
    sensor.begin();

}

void loop()
{
    // Read temp and humidity
    getWeather();
    printInfo();

    // Heating logic
    if (tempf < MINTEMP) {
      heat = true;
      digitalWrite(relay, HIGH);
    } else if (tempf > MAXTEMP) {
      heat = false;
      digitalWrite(relay, LOW);
    }

    // Send info to website
    // *Code Here*

    // Delay one second while heater is on, else one minute
    if (heat) {
      delay(SECOND);
    } else {
      delay(MINUTE);
    }
}
//---------------------------------------------------------------
void getWeather()
{
  // Measure relative humidity
  humidity = sensor.getRH();

  // Measure temperature
  tempf = sensor.getTempF();
  // Temperature is measured every time RH is requested.
  // It is faster, therefore, to read it from previous RH
  // measurement with getTemp() instead with readTemp()
}
//---------------------------------------------------------------
void printInfo()
{
//This function prints the weather data out to the default Serial Port

  Serial.print("Temp:");
  Serial.print(tempf);
  Serial.print("F, ");

  Serial.print("Humidity:");
  Serial.print(humidity);
  Serial.println("%");
}

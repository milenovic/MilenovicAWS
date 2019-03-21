#include <SD.h> //Load SD card library
#include <SPI.h> //Load SPI Library (SD card)
#include <Wire.h> // Load Wire library for talking over I2C (BME280)
#include <TimeLib.h> // Time library
#include <DS1302.h> //Library for Real Time Clock module DS1302
#include <Adafruit_BME280.h>  // import the BME280 Sensor Library of Adafruit

Adafruit_BME280 bme; // create I2C sensor object called bme
DS1302 rtc(8, 7, 6); // Set RTC pins (digital):  RST, DAT, CLK

//#define SEALEVELPRESSURE_HPA (994.00) // sea level pressure for altitude calculation, not important for weather station
int sdok = 0; //sd card was okay?
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete  //for incoming serial
unsigned long NEWtime; // Variable for holding NEWRTC time reading

float temperature;  // Variable for holding temp in C
float pressure; // Variable for holding pressure reading
float humidity; // Variable for holding humidity reading
unsigned long UNIXtime; // Variable for holding RTC time reading

//wind direction parameters
int xPin = A0;    // select the 2SA-10 X pin
int yPin = A1;    // select the 2SA-10 Y pin
float xValue; // hold the value x
float yValue; // hold the value y
float atan2value; //atan2 function result (converted to -180 to 180 degrees)
int vaneSamples = 10; //number of samples to average out
float windVane;   //angle in 0-360 degrees

//wind speed parameters
volatile int rpmcount = 0;  //see http://arduino.cc/en/Reference/Volatile
int speedSampleTime = 5000; //5s wind sampling
//float circumference = 0.439822972; // anemometer circumference in meters, not used anymore, cal. equation instead
float windSpeed = 0; //in m/s
unsigned long lastmillis = 0;

File mySensorData; //Data object you will write your sesnor data to

void setup() {
  Serial.begin(9600);          //Initialize serial comm
  inputString.reserve(200);
  pinMode(5, OUTPUT);          // set pin 5 for MAX485 activation
  digitalWrite(5, HIGH);       // set high to activate Tx mode of MAX485
  delay(1000); //wait a sec...

  if (!bme.begin()) { //initialize BME280 sensor
    Serial.println("BME280err");
    while (1);
  }

  // weather monitoring
  //Weather Station Scenario according to https://github.com/adafruit/Adafruit_BME280_Library/blob/master/examples/advancedsettings/advancedsettings.ino
  //forced mode, 1x temperature / 1x humidity / 1x pressure oversampling, filter off
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );
  // suggested rate is 1/60Hz (1m)
  
  //  if (RTC.haltRTC()) {
  //    Serial.println("DS1302 stopped!?");
  //  }
  //  if (RTC.writeEN()) {
  //    Serial.print("RTC writtable!?");
  //    Serial.println();
  //  }

  if (!SD.begin(4)) {
    Serial.println("SDerr");
    return;
  }

  UNIXtime = printTime(); //get time
  if (UNIXtime < 1535368000){
    Serial.println("CLKerr");
  }

  attachInterrupt(0, rpm_wind, FALLING);//interrupt zero (0) is on pin two(2), wind speed hall sensor
}

void loop() {
  if (millis() - lastmillis >= speedSampleTime) {
    detachInterrupt(0);   //Disable wind hall interrupt when calculating
    windSpeed = 5.62584e-5 * pow(rpmcount,3) - 8.51293e-3 * pow(rpmcount,2) +  1.36162e+0 * rpmcount + 1.69869e-1; //Convert clicks to km/h
    UNIXtime = printTime(); //get time
    bme.takeForcedMeasurement(); //needed in forced mode!
    temperature = bme.readTemperature(); // Read Temperature
    pressure = bme.readPressure() / 100.0F; // Read Pressure
    humidity = bme.readHumidity(); // Read Humidity
    windVane = windAngle(); // Read Wind Direction

    mySensorData = SD.open("WDat2.csv", FILE_WRITE);
    if (mySensorData) {
      sdok = 1;

      //writing to microSD card starts
      mySensorData.print(UNIXtime);         //write UNIX time
      mySensorData.print(",");
      mySensorData.print(temperature);      //write temperature
      mySensorData.print(",");
      mySensorData.print(pressure);         //write pressure
      mySensorData.print(",");
      mySensorData.print(humidity);         //write humidity
      mySensorData.print(",");
      mySensorData.print(windSpeed);        //write wind speed
      mySensorData.print(",");
      mySensorData.println(windVane);       //write wind angle and end the line (println)
      mySensorData.close();                 //close the file
    } else //if WDAT.csv is open
    { //sd card error!?
      sdok = 0;
    }
    //Send to openHAB
    Serial.print("W1:");
    Serial.print(UNIXtime);
    Serial.print(",");
    Serial.print(temperature);
    Serial.print(",");
    Serial.print(humidity); 
    Serial.print(",");
    Serial.print(pressure);
    Serial.print(",");
    Serial.print(windVane);
    Serial.print(",");
    Serial.print(windSpeed);
    Serial.print(",");
    Serial.println(sdok);
  
    delay(55000); //Wind sampling is speedSampleTime = 5000ms. Additional time between readings is defined here, 60000 (1m) is recommended for accurate BME280 measurement.
    rpmcount = 0; // Restart the RPM counter
    lastmillis = millis(); // Uptade lasmillis, wind speed rpm counting starts!
    attachInterrupt(0, rpm_wind, FALLING); //enable rpm interrupt
  } //if millis - lastMillis
} //loop()

unsigned long printTime() { // Get the current time and date from the chip.
  Time t = rtc.time();
  static tmElements_t tm; // this is fine static is ok here but not necessary
  tm.Year = t.yr - 1970;  // this would be year 2013
  tm.Month = t.mon;
  tm.Day = t.date;
  tm.Hour = t.hr;
  tm.Minute = t.min;
  tm.Second = t.sec;
  //Serial.println(makeTime(tm));
  return makeTime(tm);
}

float windAngle() { //samples 2SA-10 and calculates 0-360 angle
  float average = 0.0;
  for (int i = 0; i < vaneSamples; i++) {
    xValue = ((analogRead(xPin) * (5.0 / 1023.0)) - 2.5) / 2.5;
    yValue = ((analogRead(yPin) * (5.0 / 1023.0)) - 2.5) / 2.5;
    atan2value = (atan2(yValue, xValue) * 4068) / 71;
    if (atan2value < 0.0) {
      atan2value = atan2value + 360.0;
    }
    average = average + atan2value;
    delay(10);
  }
  return (average / vaneSamples);
}

void rpm_wind() { // this code will be executed every time the interrupt 0 (pin2) gets low.*/
  rpmcount++;
}


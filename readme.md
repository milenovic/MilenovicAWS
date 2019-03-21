# Intro

MilenovicAWS is an automatic weather station project which aims to provide low cost but high reliability and accuracy weather monitoring and logging system. Sensors are connected to Arduino Uno board which collects sensor data and sends over RS485 link to Raspberry Pi running openHAB.

Currently implemented sensors:
+ Temperature + Humidity + Pressure (BME280)
+ Wind speed (using Hall switch TLE4945L)
+ Wind direction (using 2-Axis Hall Sensor 2SA-10)

## Schematics
See Schematic_MilenovicAWS.svg image.
![schematics](https://github.com/milenovic/MilenovicAWS/blob/master/Schematic_MilenovicAWS.svg)

## Notes

For some BME280 variants, Adafruit_BME280.h in Adafruit_BME280_Library needs to be modified from:
```C++
#define BME280_ADDRESS                (0x77)
```
to 
```C++
#define BME280_ADDRESS                (0x76)
```

## Future
I am working on adding air quality and CO2 sensors, as well as rain gauge.

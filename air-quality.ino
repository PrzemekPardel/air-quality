 /*!
 * @file wheater-station.ino
 * Sensors:
 *  - SEN0177 PM2.5 laser dust sensor
 *   -- Measuring pm diameter: 0.3-1.0, 1.0-2.5, 2.5-10(um)
 *   -- Measuring pm range：0~999 ug/m3
 *   -- Response time: ≤10 s
 *   -- Operating temperature range: -20 ~ 50C
 *   -- Operating humidity range: 0 ~ 99% RH
 *  - DHT22 SEN0137 digital temperature and humidity sensor
 *   -- Temperature range:-40-80℃ resolution0.1℃ error <±0.5℃
 *   -- Humidity range:0-100%RH resolution0.1%RH error±2%RH
 * @license     The MIT License (MIT)
 * @author przemyslaw.pardel@gmail.com
 * References: 
 *  - https://wiki.dfrobot.com/PM2.5_laser_dust_sensor_SKU_SEN0177
 */
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "DHTesp.h"
#define LENG 31   //0x42 + 31 bytes equal to 32 bytes
unsigned char buf[LENG];

double PM1Value=0;          //define PM1.0 value of the air detector module
double PM2_5Value=0;         //define PM2.5 value of the air detector module
double PM10Value=0;         //define PM10 value of the air detector module

DHTesp dht;
int DHTPIN = 7; // temperature and humifdity sensor digital pin

// The RX pin on the sensor connects to pin 11 on the Arduino
 //The TX pin on the sensor connects to pin 10 on the Arduino
SoftwareSerial PMSerial(10, 11); // RX, TX

void setup()
{
  Serial.begin(115200);
  PMSerial.begin(9600);
  PMSerial.setTimeout(1500);

  //setup temperature and humifdity sensor
  dht.setup(DHTPIN, DHTesp::DHT22);
  
  Serial.println("Starting Loop...");
}

void loop()
{
  if(PMSerial.find(0x42)){
    PMSerial.readBytes(buf,LENG);

    if(buf[0] == 0x4d){
      if(checkValue(buf,LENG)){
        PM1Value = transmitPM1(buf); //count PM1.0 value of the air detector module
        PM2_5Value = transmitPM2_5(buf);//count PM2.5 value of the air detector module
        PM10Value = transmitPM10(buf); //count PM10 value of the air detector module
      }
    }
  }

  Serial.println("Air polution:");

  Serial.print("PM  1.0: ");
  Serial.print(PM1Value);
  Serial.println(" ug/m3");

  Serial.print("PM  2.5: ");
  Serial.print(PM2_5Value);
  Serial.println(" ug/m3");

  Serial.print("PM 10.0: ");
  Serial.print(PM10Value);
  Serial.println(" ug/m3");
  
  // Technical Assistance Document for the Reporting of Daily Air Quality
  // https://www.airnow.gov/sites/default/files/2020-05/aqi-technical-assistance-document-sept2018.pdf

  if (PM2_5Value <= 12 && PM10Value <= 54) Serial.println("Air Quality Index (AQI): Good (Up to 50)");
  else if (PM2_5Value <= 35.4 && PM10Value <= 154) Serial.println("Air Quality Index (AQI): Moderate (51 - 100)");
  else if (PM2_5Value <= 55.4 && PM10Value <= 254) Serial.println("Air Quality Index (AQI): Unhealthy for Sensitive Groups (101 - 150)");
  else if (PM2_5Value <= 150.4 && PM10Value <= 354) Serial.println("Air Quality Index (AQI): Unhealthy (151 - 200)");
  else if (PM2_5Value <= 250.4 && PM10Value <= 424) Serial.println("Air Quality Index (AQI): Very Unhealthy (201 - 300)");
  else Serial.println("Air Quality Index (AQI): Hazardous (301 - 500)");

  TempAndHumidity measurement = dht.getTempAndHumidity();

  Serial.print("Temperature: ");
  Serial.println(measurement.temperature);

  Serial.print("Humidity: ");
  Serial.println(measurement.humidity);

  Serial.println();
  delay(1000);
}

char checkValue(unsigned char *thebuf, char leng)
{
  char receiveflag=0;
  int receiveSum=0;

  for(int i=0; i<(leng-2); i++){
    receiveSum=receiveSum+thebuf[i];
  }
  receiveSum=receiveSum + 0x42;

  if(receiveSum == ((thebuf[leng-2]<<8)+thebuf[leng-1]))  //check the serial data
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  
  return receiveflag;
}

//transmit PM Value to PC
double transmitPM1(unsigned char *thebuf)
{
  double PM01Val = ((thebuf[3]<<8) + thebuf[4]); //count PM1.0 value of the air detector module
  return PM01Val;
}

//transmit PM Value to PC
double transmitPM2_5(unsigned char *thebuf)
{
  double PM2_5Val = ((thebuf[5]<<8) + thebuf[6]);//count PM2.5 value of the air detector module
  return PM2_5Val;
  }

//transmit PM Value to PC
double transmitPM10(unsigned char *thebuf)
{
  double PM10Val = ((thebuf[7]<<8) + thebuf[8]); //count PM10 value of the air detector module
  return PM10Val;
}

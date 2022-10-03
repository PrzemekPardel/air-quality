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
 *  - Fermion: 1.54" 240x240 IPS TFT LCD Display with MicroSD Card
 *   -- Operating Voltage: 3.3V~5V
 *   -- IPS Angle of View: 80/80/80/80
 *   -- Color Depth: 16-bit (RGB565)
 *   --Pixels: 240 × 240
 *   -- Connection Port: SPI
 *   -- Driver Chip: ST7789
 *   -- Brightness: 250 (Typ) cd/m2
 *   -- Full-screen Power Consumption: about 17mA(3.3V) 17mA(5V)(Typ)
 *   -- Operating Temperature: -30℃~+70℃
 *   -- Display Area: 27.72×27.72 mm
 *   -- Weight: 19g
 * @license     The MIT License (MIT)
 * @author przemyslaw.pardel@gmail.com
 * References: 
 *  - https://wiki.dfrobot.com/PM2.5_laser_dust_sensor_SKU_SEN0177
 */
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "DHTesp.h"
#include "DFRobot_GDL.h"
#include "Icon.h"

/*M0*/
#if defined ARDUINO_SAM_ZERO
#define TFT_DC  7
#define TFT_CS  5
#define TFT_RST 6
/*ESP32 and ESP8266*/
#elif defined(ESP32) || defined(ESP8266)
#define TFT_DC  D2
#define TFT_CS  D6
#define TFT_RST D3
/*AVR series mainboard*/
#else
#define TFT_DC  2
#define TFT_CS  3
#define TFT_RST 4
#endif

DFRobot_ST7789_240x240_HW_SPI screen(/*dc=*/TFT_DC,/*cs=*/TFT_CS,/*rst=*/TFT_RST);
DHTesp dht;
// The RX pin on the sensor connects to pin 11 on the Arduino
 //The TX pin on the sensor connects to pin 10 on the Arduino
SoftwareSerial PMSerial(7, 8); // RX, TX

double PM1Value = 0;          //define PM1.0 value of the air detector module
double PM2_5Value = 0;         //define PM2.5 value of the air detector module
double PM10Value = 0;         //define PM10 value of the air detector module
double oldPM1Value = 9;          //define old PM1.0 value of the air detector module
double oldPM2_5Value = 9;         //define old PM2.5 value of the air detector module
double oldPM10Value = 9;         //define old PM10 value of the air detector module

String AQI = "0";
double oldHumidity = 0;
double oldTemperature = 0;

void setup()
{ 
  //PIN to PM sensor
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH); //turn on PM sensor
  PMSerial.begin(9600);
  PMSerial.setTimeout(1500);
  //Setup serial port
  Serial.begin(115200);
  //Setup screen
  screen.begin();
  loadStaticDesign();
  // digitalWrite(5, LOW); 
  //setup temperature and humifdity sensor
  dht.setup(6, DHTesp::DHT22); // 6 - temperature and humifdity sensor digital pin
}

void loadStaticDesign(){
  // Background
  screen.fillScreen(COLOR_RGB565_BLACK);
  // LOGO
  screen.fillCircle(/*x0=*/34, /*y0=*/34, /*r=*/28, /*color=*/COLOR_RGB565_ORANGE);
  screen.drawXBitmap(/*x=*/10,/*y=*/10,/*bitmap gImage_Bitmap=*/logo48 ,/*w=*/48,/*h=*/48,COLOR_RGB565_WHITE);
  printText(0, 68, COLOR_RGB565_WHITE, 0, " Photometer");
  printText(0, 80, COLOR_RGB565_WHITE, 0, "  PRO (TM)");
  //PM2.5
  printText(100, 0, COLOR_RGB565_WHITE, 2, "PM2.5");
  //printText(100, 30, COLOR_RGB565_WHITE, 5, "0");
  printText(180, 70, COLOR_RGB565_WHITE, 2, "ug/m3");
  //Other
  printText(0, 105, COLOR_RGB565_WHITE, 2, "PM1 :");
  //printText(60, 105, COLOR_RGB565_WHITE, 2, "0");
  printText(115, 105, COLOR_RGB565_WHITE, 1, "ug/m3");
  printText(0, 130, COLOR_RGB565_WHITE, 2, "PM10:");
  //printText(60, 130, COLOR_RGB565_WHITE, 2, "0");
  printText(115, 130, COLOR_RGB565_WHITE, 1, "ug/m3");
  printText(0, 155, COLOR_RGB565_WHITE, 2, "Tmp.:");
  //printText(60, 155, COLOR_RGB565_WHITE, 2, "0.0");
  printText(120, 155, COLOR_RGB565_WHITE, 1, "oC");
  printText(0, 180, COLOR_RGB565_WHITE, 2, "Humidity");
  //printText(0, 200, COLOR_RGB565_WHITE, 2, "0.0%");
  //Air Quality Index
  printText(150, 105, COLOR_RGB565_WHITE, 1, "Air Quality");
  printText(150, 115, COLOR_RGB565_WHITE, 1, "Index");
  //printText(170, 130, COLOR_RGB565_RED, 6, "0");
  printText(130, 180, COLOR_RGB565_RED, 2, "Hazardous");
  printText(130, 200, COLOR_RGB565_RED, 1, "AQI: 301-500");
  printText(130, 210, COLOR_RGB565_RED, 1, "Extremely poor");
  //Version
  printText(0, 230, COLOR_RGB565_WHITE, 1, "V.1.01a (C) Photometer PRO 2016-22");
}

void printText(byte  cursorX, byte  cursorY, int color, byte  textSize, String text){
  //Set text size
  screen.setTextSize(textSize);
  //Set the coordinate position
  screen.setCursor(cursorX, cursorY);
  //Set the text color; this is a changeable value
  screen.setTextColor(color);
  //Output text
  screen.println(text);
}

void loop()
{
  digitalWrite(5, HIGH); //turn on PM sensor
  #define LENG 31   //0x42 + 31 bytes equal to 32 bytes
  unsigned char buf[LENG];
  // Read data from sensors
  if(PMSerial.find(0x42)){
    PMSerial.readBytes(buf,LENG);

    if(buf[0] == 0x4d){
      if(checkValue(buf,LENG)){
        PM1Value=transmitPM01(buf); //count PM1.0 value of the air detector module
        PM2_5Value=transmitPM2_5(buf);//count PM2.5 value of the air detector module
        PM10Value=transmitPM10(buf); //count PM10 value of the air detector module 
      }
    }
  }

  static unsigned long OledTimer = millis();
  if (millis() - OledTimer >= 2000)
  {
    OledTimer = millis();
    // Refresh data on LCD screen
    refreshData();
  }
}

void refreshData(){
  //!!!Refresh only those values which changes!!! This avoid screen blinking !!!
  String strval = "";
  //PM 2.5
  if (oldPM2_5Value != PM2_5Value){
    screen.fillRect(100,30,120,35, COLOR_RGB565_BLACK);
    strval = String(PM2_5Value,0);
    oldPM2_5Value = PM2_5Value;
    printText(100, 30, COLOR_RGB565_WHITE, 5, strval);
  }
  Serial.println("PM2.5: " + strval);
  //Other
  //PM1
  if (oldPM1Value != PM1Value){
    screen.fillRect(60,105,55,20, COLOR_RGB565_BLACK);
    strval = String(PM1Value,0);
    oldPM1Value = PM1Value;
    printText(60, 105, COLOR_RGB565_WHITE, 2, strval);
  }
  Serial.println("PM1.0: " + strval);
  //PM10
  if (oldPM10Value != PM10Value){
    screen.fillRect(60,130,55,20, COLOR_RGB565_BLACK);
    strval = String(PM10Value,0);
    oldPM10Value = PM10Value;
    printText(60, 130, COLOR_RGB565_WHITE, 2, strval);
  }
  Serial.println("PM10: " + strval);
  //Temperature and humidity
  TempAndHumidity measurement = dht.getTempAndHumidity();
  if (oldTemperature != measurement.temperature){
    screen.fillRect(60,155,58,20, COLOR_RGB565_BLACK);
    strval = String(measurement.temperature,1);
    oldTemperature = measurement.temperature;
    printText(60, 155, COLOR_RGB565_WHITE, 2, strval);
  }
  Serial.println("Temperature: " + strval);
  if (oldHumidity != measurement.humidity){
    screen.fillRect(0,200,78,20, COLOR_RGB565_BLACK);
    strval = String(measurement.humidity,1) + "%";  
    oldHumidity = measurement.humidity;
    printText(0, 200, COLOR_RGB565_WHITE, 2, strval);
  }
  Serial.println("Humidity: " + strval);
  //Air Quality Index
  // Technical Assistance Document for the Reporting of Daily Air Quality
  // https://www.airnow.gov/sites/default/files/2020-05/aqi-technical-assistance-document-sept2018.pdf
  
  unsigned int AQI_COLOR;
  if (PM2_5Value <= 12 && PM10Value <= 54) {
    if (AQI != "5") {
      clearAQI();
      AQI = "5";//Serial.println("Air Quality Index (AQI): Good (Up to 50)");
      AQI_COLOR = COLOR_RGB565_GREEN;
      printText(130, 180, COLOR_RGB565_GREEN, 2, "Good");
      printText(130, 200, COLOR_RGB565_GREEN, 1, "AQI: up to 50");
      printText(130, 210, COLOR_RGB565_GREEN, 1, "");
    }
  }
  else if (PM2_5Value <= 35.4 && PM10Value <= 154) {
    if (AQI != "4") {
      clearAQI();
      AQI = "4";//Serial.println("Air Quality Index (AQI): Moderate (51 - 100)");
      AQI_COLOR = COLOR_RGB565_YELLOW;    
      printText(130, 180, COLOR_RGB565_YELLOW, 2, "Moderate");
      printText(130, 200, COLOR_RGB565_YELLOW, 1, "AQI: 51-100");
      printText(130, 210, COLOR_RGB565_YELLOW, 1, "Fair");
    }
  }
  else if (PM2_5Value <= 55.4 && PM10Value <= 254) {
    if (AQI != "3") {
      clearAQI();
      AQI = "3";//Serial.println("Air Quality Index (AQI): Unhealthy for Sensitive Groups (101 - 150)");
      AQI_COLOR = COLOR_RGB565_ORANGE;
      printText(130, 180, COLOR_RGB565_ORANGE, 2, "Unhealthy");
      printText(130, 200, COLOR_RGB565_ORANGE, 1, "AQI: 101-150");
      printText(130, 210, COLOR_RGB565_ORANGE, 1, "Moderate");
    }
  }
  else if (PM2_5Value <= 150.4 && PM10Value <= 354) {
    if (AQI != "2") {
      clearAQI();
      AQI = "2";//Serial.println("Air Quality Index (AQI): Unhealthy (151 - 200)");
      //printText(170, 130, COLOR_RGB565_ORANGE, 6, AQI);
      AQI_COLOR = COLOR_RGB565_ORANGE;    
      printText(130, 180, COLOR_RGB565_ORANGE, 2, "Unhealthy");
      printText(130, 200, COLOR_RGB565_ORANGE, 1, "AQI: 151-200");
      printText(130, 210, COLOR_RGB565_ORANGE, 1, "Poor");
    }
  }
  else if (PM2_5Value <= 250.4 && PM10Value <= 424) {
    if (AQI != "1") {
      clearAQI();
      AQI = "1";// Serial.println("Air Quality Index (AQI): Very Unhealthy (201 - 300)");
      //printText(170, 130, COLOR_RGB565_RED, 6, AQI);
      AQI_COLOR = COLOR_RGB565_RED;
      printText(130, 180, COLOR_RGB565_RED, 2, "Danger");
      printText(130, 200, COLOR_RGB565_RED, 1, "AQI: 201-300");
      printText(130, 210, COLOR_RGB565_RED, 1, "Very poor");
    }
  }
  else {
    if (AQI != "0") {
      clearAQI();
      AQI = "0"; //Serial.println("Air Quality Index (AQI): Hazardous (301 - 500)");
      AQI_COLOR = COLOR_RGB565_RED;
      printText(130, 180, COLOR_RGB565_RED, 2, "Hazardous");
      printText(130, 200, COLOR_RGB565_RED, 1, "AQI: 301-500");
      printText(130, 210, COLOR_RGB565_RED, 1, "Extremely poor");
    }
  }
  printText(170, 130, AQI_COLOR, 6, AQI);
  Serial.println("AQI: " + AQI);
}

void clearAQI(){
  screen.fillRect(160,130,52,58, COLOR_RGB565_BLACK);
  screen.fillRect(130,180,120,16, COLOR_RGB565_BLACK);
  screen.fillRect(130,200,120,8, COLOR_RGB565_BLACK);
  screen.fillRect(130,210,120,8, COLOR_RGB565_BLACK);
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

int transmitPM01(unsigned char *thebuf)
{
  int PM01Val;
  PM01Val=((thebuf[3]<<8) + thebuf[4]); //count PM1.0 value of the air detector module
  return PM01Val;
}

//transmit PM Value to PC
int transmitPM2_5(unsigned char *thebuf)
{
  int PM2_5Val;
  PM2_5Val=((thebuf[5]<<8) + thebuf[6]);//count PM2.5 value of the air detector module
  return PM2_5Val;
  }

//transmit PM Value to PC
int transmitPM10(unsigned char *thebuf)
{
  int PM10Val;
  PM10Val=((thebuf[7]<<8) + thebuf[8]); //count PM10 value of the air detector module
  return PM10Val;
}

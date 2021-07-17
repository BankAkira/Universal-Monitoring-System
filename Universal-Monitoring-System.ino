/*
 * Created Date: Friday, May 31st 2019, 2:12:24 pm
 * Author: akira
 * 
 * Copyright (c) 2019
 */


#include <Wire.h>
#include <SPI.h>
#include "StopWatch.h"

//------------Config---------------------------
String serverIP = "";
String serverPort = "";
String siteName = "UMS Test1";
//set 1 to online and 0 to offline 
#define setMode 1
//set interval time for record to sd card in seconds
int SDTime = 60;
//set multiplier for light sensor
double light1Mutiplier = 1;
double light2Multiplier = -23.1;
//set multiplier for rain sensor
double rainMuliplier = 0.254;
//set I2C LCD address 
#define lcd_address 0x3F


//---------- BEM280 temp humid and pressure sensor ---------------------------------
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;

//------------SHT31 temp humid sensor -----------------------------------------------
#include "Adafruit_SHT31.h"
#define sht31_1_addr 0x44
#define sht31_2_addr 0x45

Adafruit_SHT31 sht31_1 = Adafruit_SHT31();
Adafruit_SHT31 sht31_2 = Adafruit_SHT31();


//------------ UART K30 FR -------------------------------------------------------------
byte readCO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25}; //Command packet to read Co2
byte response_1[] = {0, 0, 0, 0, 0, 0, 0};                   //create an array to store the response
byte response_2[] = {0, 0, 0, 0, 0, 0, 0};                   //create an array to store the response

//multiplier for value. default is 1. set to 3 for K-30 3% and 10 for K-33 ICB
int valMultiplier = 1;

//-----Co2 UART function --------
void sendRequest_1(byte packet[]) {
  while (!Serial1.available()) //keep sending request until we start to get a response
  {
    Serial1.write(readCO2, 7);
    delay(50);
  }

  int timeout = 0;                //set a timeoute counter
  while (Serial1.available() < 7) //Wait to get a 7 byte response
  {
    timeout++;
    if (timeout > 10) //if it takes to long there was probably an error
    {
      while (Serial1.available()) //flush whatever we have
        Serial1.read();

      break; //exit and try again
    }
    delay(50);
  }

  for (int i = 0; i < 7; i++)
  {
    response_1[i] = Serial1.read();
  }
}

void sendRequest_2(byte packet[]) {
  while (!Serial2.available()) //keep sending request until we start to get a response
  {
    Serial2.write(readCO2, 7);
    delay(50);
  }

  int timeout = 0;                //set a timeoute counter
  while (Serial2.available() < 7) //Wait to get a 7 byte response
  {
    timeout++;
    if (timeout > 10) //if it takes to long there was probably an error
    {
      while (Serial2.available()) //flush whatever we have
        Serial2.read();

      break; //exit and try again
    }
    delay(50);
  }

  for (int i = 0; i < 7; i++)
  {
    response_2[i] = Serial2.read();
  }
}

unsigned long getValue(byte packet[]) {
  int high = packet[3]; //high byte for value is 4th byte in packet in the packet
  int low = packet[4];  //low byte for value is 5th byte in the packet
  Serial.print("High : ");
  Serial.print(high);
  Serial.print(" Low : ");
  Serial.println(low);

  unsigned long val = high * 256 + low; //Combine high byte and low byte with this formula to get value
  return val * valMultiplier;
}


//--------- DS18B20 Sensor ---------------------------------------------------------------------
#include "OneWire.h"
#include "DallasTemperature.h"

OneWire ds18b20_pin1(34);
OneWire ds18b20_pin2(36);
OneWire ds18b20_pin3(38);

DallasTemperature ds18b20_1(&ds18b20_pin1);
DallasTemperature ds18b20_2(&ds18b20_pin2);
DallasTemperature ds18b20_3(&ds18b20_pin3);

void init_ds18b20() {
  ds18b20_1.begin();
  ds18b20_2.begin();
  ds18b20_3.begin();
}

double soilTemp1, soilTemp2, soilTemp3;

void getSoilTemp() {
  ds18b20_1.requestTemperatures();
  ds18b20_2.requestTemperatures();
  ds18b20_3.requestTemperatures();
  soilTemp1 = ds18b20_1.getTempCByIndex(0);
  soilTemp2 = ds18b20_2.getTempCByIndex(0);
  soilTemp3 = ds18b20_3.getTempCByIndex(0);
}

//------- Quantum sensor and Pyranometer sensor  ---------------------------------------------------------------------
#include <Adafruit_ADS1015.h>

Adafruit_ADS1015 ads1015;
// Quantum data in µmol m-2 s-1
double getLight1Data() {
  double tmp = ads1015.readADC_Differential_0_1();
  return tmp * light1Mutiplier;
}

// Pyranometer data in W m-2
double getLight2Data() {
  double temp = ads1015.readADC_Differential_2_3();
  return temp * light2Multiplier;
}


//--------------- Soil Humidity ------------------------------------------------------
int soilHumidity1, soilHumidity2, soilHumidity3;

void getSoilHumidity() {
  soilHumidity1 = analogRead(A13);
  soilHumidity2 = analogRead(A14);
  soilHumidity3 = analogRead(A15);
}

void init_SoilHumdity() {
  pinMode(40, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(44, OUTPUT);
}

void powerOnSoilHumidity() {
  digitalWrite(40, HIGH);
  digitalWrite(42, HIGH);
  digitalWrite(44, HIGH);

}

void powerOffSoilHumidity() {
  digitalWrite(40,LOW);
  digitalWrite(42,LOW);
  digitalWrite(44,LOW);
}


//------ Rain counter ----------------------------------------------------------------------------------------
const byte rainInterruptPin = 3;
int rainCnt = 0;
void rainCount() {
  rainCnt++;
}


//-------- Wind Speed and Direction ----------------------------------------------------------------------
#include "TimerOne.h"
#include <math.h>

#define windSersorPin 2
#define windVanePin A7
#define vaneOffset 0

int vaneValue; //raw analog val from wind vane
int direction; //translate to 0-360 direction
int calWindDirection; // connverted val with offset applied
int lastValue;

volatile bool isSampleRequired;  // this is set true every 2.5s. Get wind speed
volatile unsigned int timerCount; // used to determine 2.5sec timer count
volatile unsigned long rotations; // cup rotation counter used in interrupt routine
volatile unsigned long contactBounceTime; // Timer to avoid contact bounce in interrupt routine 

float windSpeed; // speed m/s 

//isr routine for timer interrupt
void isr_timer() {
  timerCount++;
  if(timerCount == 6) {
    isSampleRequired = true;
    timerCount = 0;
  }
}

// interrupt calls to inncrement tho rotation count
void isr_rotation () {
  if((millis() - contactBounceTime) > 15) {
    rotations++;
    contactBounceTime = millis();
  }
}

void init_wind() {
  lastValue = 0;
  isSampleRequired = false;
  timerCount = 0;
  rotations = 0;  //  set rotations to 0 ready for calculations

  pinMode(windSersorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(windSersorPin), isr_rotation, FALLING);

  // setup timer interrupt
  Timer1.initialize(500000); //timer interrupt every 2.5 sec 
  Timer1.attachInterrupt(isr_timer);
}

void getWindSpeed() {
  if(abs(calWindDirection - lastValue) > 5) {
    lastValue = calWindDirection;
  }
  if (isSampleRequired) {
    windSpeed = rotations * 0.4023;
    rotations = 0; // reset count for next sample
    isSampleRequired = false;
  }
}


// Get Wind Direction
void getWindDirection() {
  vaneValue = analogRead(windVanePin);
  direction = map(vaneValue, 0, 1023, 0, 360);
  calWindDirection = direction + vaneOffset;

  if (calWindDirection > 360)
    calWindDirection = calWindDirection - 360;

  if (calWindDirection < 0)
    calWindDirection = calWindDirection + 360;
}


//-------- DC Volt meter ----------------------------------------------------------------------
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
float volt1, volt2, volt3, volt4, volt5;

void getVoltData() {
  volt1 = fmap(analogRead(A8), 0, 1023, 0.0, 25.0);
  volt2 = fmap(analogRead(A9), 0, 1023, 0.0, 25.0);
  volt3 = fmap(analogRead(A10), 0, 1023, 0.0, 25.0);
  volt4 = fmap(analogRead(A11), 0, 1023, 0.0, 25.0);
  volt5 = fmap(analogRead(A12), 0, 1023, 0.0, 25.0);
}


//------------- AIS Nb-IOT --------------------------------------------------------------
#include "AIS_NB_BC95.h"

String apnName = "devkit.nb";
String token = "";

AIS_NB_BC95 AISnb;

String payload = "";
String rssi = "";

// initialize NBIOT
void initAISNbIOT() {
  AISnb.debug = true;
  AISnb.setupDevice(serverPort);
  String ip1 = AISnb.getDeviceIP();
  delay(1000);
  pingRESP pingR = AISnb.pingIP(serverIP);
  delay(500);
}

// prepare Data before sent 
void prepareData(double light1Data, double light2Data, float temp1, float rh1, float temp2, float rh2, float temp3, float rh3, float pressure, float windSpeed, float calWindDirection, float rain, double soilTemp1, double soilTemp2, double soilTemp3, float soilHumidity1, float soilHumidity2, float soilHumidity3, float CO2_1, float CO2_2, float volt1, float volt2, float volt3, float volt4, float volt5, String rssi) {
  payload = siteName;
  payload += ':';
  payload += String(light1Data);
  payload += ':';
  payload += String(light2Data);
  payload += ':';
  payload += String(temp1);
  payload += ':';
  payload += String(rh1);
  payload += ':';
  payload += String(temp2);
  payload += ':';
  payload += String(rh2);
  payload += ':';
  payload += String(temp3);
  payload += ':';
  payload += String(rh3);
  payload += ':';
  payload += String(pressure);
  payload += ':';
  payload += String(windSpeed);
  payload += ':';
  payload += String(calWindDirection);
  payload += ':';
  payload += String(rain);
  payload += ':';
  payload += String(soilTemp1);
  payload += ':';
  payload += String(soilTemp2);
  payload += ':';
  payload += String(soilTemp3);
  payload += ':';
  payload += String(soilHumidity1);
  payload += ':';
  payload += String(soilHumidity2);
  payload += ':';
  payload += String(soilHumidity3);
  payload += ':';
  payload += String(CO2_1);
  payload += ':';
  payload += String(CO2_2);
  payload += ':';
  payload += String(volt1);
  payload += ':';
  payload += String(volt2);
  payload += ':';
  payload += String(volt3);
  payload += ':';
  payload += String(volt4);
  payload += ':';
  payload += String(volt5);
  payload += ':';
  payload += rssi;
  payload += ':';
  payload += token;
}


//------- LCD Display ------------------------------------------------------------------------
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(lcd_address, 20, 4);

void display(int line, int position, String msg) {
  lcd.setCursor(position-1, line-1);
  lcd.print(msg);
}


//---------RTC Module -------------------------------------------------------------------------
#include "RTClib.h"
RTC_DS3231 RTC;


//--------SD Card Module -----------------------------------------------------------------------
#include "SD.h"
const int chipSelect = 53;
File dataFile;
bool fileError = false;
bool sdError = false;
unsigned int RecNum = 0;
int SDcount = 0;

void logHeader(bool fileError, bool sdError) {
  if (!fileError and !sdError) {
    dataFile.print("RecNum");
    dataFile.print(',');
    dataFile.print("Date");
    dataFile.print(',');
    dataFile.print("Time");
    dataFile.print(',');
    dataFile.print("Light1");
    dataFile.print(',');
    dataFile.print("Light2");
    dataFile.print(',');
    dataFile.print("Temp1(C)");
    dataFile.print(',');
    dataFile.print("RH1(%)");
    dataFile.print(',');
    dataFile.print("Temp(C)");
    dataFile.print(',');
    dataFile.print("RH2(%)");
    dataFile.print(',');
    dataFile.print("Temp3(C)");
    dataFile.print(',');
    dataFile.print("RH3(%)");
    dataFile.print(',');
    dataFile.print("Pressure(kPa)");
    dataFile.print(',');
    dataFile.print("Wind speed(m/s)");
    dataFile.print(',');
    dataFile.print("Wind Direction(degree)");
    dataFile.print(',');
    dataFile.print("Rain(mm)");
    dataFile.print(',');
    dataFile.print("Soil Temp1(C)");
    dataFile.print(',');
    dataFile.print("Soil Temp2(C)");
    dataFile.print(',');
    dataFile.print("Soil Temp3(C)");
    dataFile.print(',');
    dataFile.print("Soil Humidity1");
    dataFile.print(',');
    dataFile.print("Soil Humidity2");
    dataFile.print(',');
    dataFile.print("Soil Humidity3");
    dataFile.print(',');
    dataFile.print("CO2-1(ppm)");
    dataFile.print(',');
    dataFile.print("CO2-2(ppm)");
    dataFile.print(',');
    dataFile.print("Volt1");
    dataFile.print(',');
    dataFile.print("Volt2");
    dataFile.print(',');
    dataFile.print("Volt3");
    dataFile.print(',');
    dataFile.print("Volt4");
    dataFile.print(',');
    dataFile.print("Volt5");
    dataFile.print(',');
    dataFile.println("RSSI");
    dataFile.flush();
  }
}

void logData(double tmplight1Data, double tmplight2Data, float tmpTemp1, float tmpRh1, float tmpTemp2, float tmpRh2, float tmpTemp3, float tmpRh3, float tmpPressure, float tmpWindSpeed, float tmpCalWindDirection, float sumrain, double tmpSoilTemp1, double tmpSoilTemp2, double tmpSoilTemp3, float tmpSoilHumidity1, float tmpSoilHumidity2, float tmpSoilHumidity3, float tmpCO2_1, float tmpCO2_2, float tmpVolt1, float tmpVolt2, float tmpVolt3, float tmpVolt4, float tmpVolt5, String rssi) {
  if (!fileError && !sdError) {
    if(isnan(tmplight1Data)) {dataFile.print("NAN");}
    else {dataFile.print(tmplight1Data / SDcount);}  
    dataFile.print(',');
    if(isnan(tmplight2Data)) {dataFile.print("NaN");}
    else {dataFile.print(tmplight2Data / SDcount);}
    dataFile.print(',');
    if(isnan(tmpTemp1)) {dataFile.print("NaN");}
    else {dataFile.print(tmpTemp1 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpRh1)) {dataFile.print("NaN");}
    else {dataFile.print(tmpRh1 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpTemp2)) {dataFile.print("NaN");}
    else {dataFile.print(tmpTemp2 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpRh2)) {dataFile.print("NaN");}
    else {dataFile.print(tmpRh2 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpTemp3)) {dataFile.print("NaN");}
    else {dataFile.print(tmpTemp3 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpRh3)) {dataFile.print("NaN");}
    else {dataFile.print(tmpRh3 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpPressure)) {dataFile.print("NaN");}
    else {dataFile.print(tmpPressure / SDcount);}
    dataFile.print(',');
    if(isnan(tmpWindSpeed)) {dataFile.print("NaN");}
    else {dataFile.print(tmpWindSpeed / SDcount);}
    dataFile.print(',');
    if(isnan(tmpCalWindDirection)) {dataFile.print("NaN");}
    else {dataFile.print(tmpCalWindDirection / SDcount);}
    dataFile.print(',');
    if(isnan(sumrain)) {dataFile.print("NaN");}
    else {dataFile.print(sumrain);}
    dataFile.print(',');
    if(isnan(tmpSoilTemp1)) {dataFile.print("NaN");}
    else {dataFile.print(tmpSoilTemp1 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpSoilTemp2)) {dataFile.print("NaN");}
    else {dataFile.print(tmpSoilTemp2 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpSoilTemp3)) {dataFile.print("NaN");}
    else {dataFile.print(tmpSoilTemp3 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpSoilHumidity1)) {dataFile.print("NaN");}
    else {dataFile.print(tmpSoilHumidity1 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpSoilHumidity2)) {dataFile.print("NaN");}
    else {dataFile.print(tmpSoilHumidity2 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpSoilHumidity3)) {dataFile.print("NaN");}
    else {dataFile.print(tmpSoilHumidity3 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpCO2_1)) {dataFile.print("NaN");}
    else {dataFile.print(tmpCO2_1 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpCO2_2)) {dataFile.print("NaN");}
    else {dataFile.print(tmpCO2_2 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpVolt1)) {dataFile.print("NaN");}
    else {dataFile.print(tmpVolt1 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpVolt2)) {dataFile.print("NaN");}
    else {dataFile.print(tmpVolt2 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpVolt3)) {dataFile.print("NaN");}
    else {dataFile.print(tmpVolt3 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpVolt4)) {dataFile.print("NaN");}
    else {dataFile.print(tmpVolt4 / SDcount);}
    dataFile.print(',');
    if(isnan(tmpVolt5)) {dataFile.print("NaN");}
    else {dataFile.print(tmpVolt5 / SDcount);}
    dataFile.print(',');
    dataFile.println(rssi);
    dataFile.flush();
  }
}

//----- StopWatch -----------------------------------------------------------------------------
//--Interval time --------
StopWatch timerInterval(StopWatch::SECONDS);

//-- StopWatch for write to sd card
StopWatch SDInterval(StopWatch::SECONDS);


//Restart Program
#include <avr/wdt.h>

void restart(uint8_t prescaller) {
  wdt_enable(prescaller);
  while (1) {}
}


// temporary valuable for average data
double tmplight1Data, tmplight2Data, tmpSoilTemp1, tmpSoilTemp2, tmpSoilTemp3;
float tmpTemp1, tmpTemp2, tmpTemp3, tmpWindSpeed, sumRain, tmpVolt1, tmpVolt2, tmpVolt3, tmpVolt4, tmpVolt5;
int tmpRh1, tmpRh2, tmpRh3, tmpPressure, tmpWindDirection, tmpSoilHumid1, tmpSoilHumid2, tmpSoilHumid3, tmpCO2_1, tmpCO2_2;

void resetTempData() {
  tmplight1Data = 0;
  tmplight2Data = 0;
  tmpTemp1 = 0;
  tmpRh1 = 0;
  tmpTemp2 = 0;
  tmpRh2 = 0;
  tmpTemp3 = 0;
  tmpRh3 = 0;
  tmpPressure = 0;
  tmpWindSpeed = 0;
  tmpWindDirection = 0;
  sumRain = 0;
  tmpSoilTemp1 = 0;
  tmpSoilTemp2 = 0;
  tmpSoilTemp3 = 0;
  tmpSoilHumid1 = 0;
  tmpSoilHumid2 = 0;
  tmpSoilHumid3 = 0;
  tmpCO2_1 = 0;
  tmpCO2_2 = 0;
  tmpVolt1 = 0;
  tmpVolt2 = 0;
  tmpVolt3 = 0;
  tmpVolt4 = 0;
  tmpVolt5 = 0;
}


/***********************************************************************************************/
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  Wire.begin();
  // init LCD
  lcd.init();
  lcd.backlight();
  // init Soil Temp
  init_ds18b20();
  // init SHT31
  sht31_1.begin(sht31_1_addr);
  sht31_2.begin(sht31_2_addr);
  // init BME280
  bool status;
  status = bme.begin();
  if (!status) {
    Serial.println("BME280 sensor connect failed");
  }
  //init rain counter
  pinMode(rainInterruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainInterruptPin), rainCount, FALLING);
  //init wind speed and direction
  init_wind();
  //init soil Humidity
  init_SoilHumdity();
  
  // init RTC
  RTC.begin();
  DateTime now = RTC.now();
  Serial.println(now.year());
  String fileLog = String(now.year());
  if (now.month() < 10) {
    fileLog += "-0";
    fileLog += String(now.month());
  }
  else {
    fileLog += "-";
    fileLog += String(now.month());
  }
  fileLog += ".csv";
  Serial.print(fileLog);
  pinMode(chipSelect, OUTPUT);
  digitalWrite(chipSelect, HIGH);
  SD.begin(chipSelect);
  dataFile = SD.open(fileLog, FILE_WRITE);
  //write log header 
  logHeader(fileError, sdError);
  dataFile.close();
  //Display start up information data to LCD
  display(4,10,"YuiYui");
  display(2, 1,"bibi");
  if (setMode) {
    //----- initialize NBIOT -----------
    initAISNbIOT();
    //--- register site name -----------
    String regname = siteName;
    regname += "";
    UDPSend reg = AISnb.sendUDPmsgStr(serverIP, serverPort, regname);
    Serial.print("Reg Resp: ");
    Serial.println(reg.status);
    if (!reg.status) {
      restart(WDTO_2S);
    }
  }
  
  timerInterval.start();
  SDInterval.start();
}

void loop() {
  int curr = timerInterval.elapsed();

  //prepare date time to display
  DateTime now = RTC.now();
  String timeStamp = "";
  if (now.day() < 10) {timeStamp += '0';}
  timeStamp += String (now.day());
  timeStamp += '/';
  if (now.month() < 10) {timeStamp += '0';}
  timeStamp += String(now.month());
  timeStamp += '/';
  timeStamp += String(now.year());
  timeStamp += ' ';
  if (now.hour() < 10) {timeStamp += '0';}
  timeStamp += String(now.hour());
  timeStamp += ':';
  if (now.minute() < 10) {timeStamp += '0';}
  timeStamp += String(now.minute());
  timeStamp += ':';
  if (now.second() < 10) {timeStamp += '0';}
  timeStamp += String(now.second());
  display(1, 1, timeStamp);

  //turn on soilHumidity
  if (curr >= 55 ) {
    powerOnSoilHumidity();
  }
  if (curr >= 60) {
    timerInterval.stop();

    // get QuantumData and PyranometerData
    double light1Data = getLight1Data();
    double light2Data = getLight2Data();

    // get temp RH
    float temp1 = sht31_1.readTemperature();
    int rh1 = sht31_1.readHumidity();
    float temp2 = sht31_2.readTemperature();
    int rh2 = sht31_2.readHumidity();
    float temp3 = bme.readTemperature();
    int rh3 = bme.readHumidity();
    
    // get pressure
    int pressure = (bme.readPressure() / 1000.0F);

    //get windSpeed and direction
    getWindDirection();
    getWindSpeed();

    // get rain
    float rain = rainCnt * rainMuliplier;

    //get Soil Temperature
    getSoilTemp();

    //get Soil Humidity
    getSoilHumidity();

    // read CO2 val 
    sendRequest_1(readCO2);
    delay(100);
    unsigned long CO2_1 = getValue(response_1);
    if (CO2_1 >= 10000) {CO2_1 = 9999;}
    delay(100);
    sendRequest_2(readCO2);
    delay(100);
    unsigned long CO2_2 = getValue(response_2);
    if (CO2_2 >= 10000) {CO2_2 = 9999;}

    //get Voltage
    getVoltData();

    // check signal AIS Nb-IOT signal 
    signal chkRssi = AISnb.getSignal();
    rssi = chkRssi.rssi;

    //store data
    tmplight1Data += light1Data;
    tmplight2Data += light2Data;
    tmpTemp1 += temp1;
    tmpRh1 += rh1;
    tmpTemp2 += temp2;
    tmpRh2 += rh2;
    tmpTemp3 += temp3;
    tmpRh3 += rh3;
    tmpPressure += pressure;
    tmpWindSpeed += windSpeed;
    tmpWindDirection += calWindDirection;
    sumRain += rain;
    tmpSoilTemp1 += soilTemp1;
    tmpSoilTemp2 += soilTemp2;
    tmpSoilTemp3 += soilTemp3;
    tmpSoilHumid1 += soilHumidity1;
    tmpSoilHumid2 += soilHumidity2;
    tmpSoilHumid3 += soilHumidity3;
    tmpCO2_1 += CO2_1;
    tmpCO2_2 += CO2_2;
    tmpVolt1 += volt1;
    tmpVolt2 += volt2;
    tmpVolt3 += volt3;
    tmpVolt4 += volt4;
    tmpVolt5 += volt5;

    //----------------Display data to 20x4 LCD display -----------------------------------
    display(2, 1,"T1: ");
    display(2, 5, String(3141.5926)); // default 2 digit 
    display(2, 11,"T2: ");
    display(2, 15, String(int(temp1))); // no digit
    display(3, 1,"T2: ");
    display(3, 7, String(temp1)); // default 2 digit 



    //----------- prepare data to one String before sent data ---------------------------------
    prepareData(light1Data, light2Data, temp1, rh1, temp2, rh2, temp3, rh3, pressure, windSpeed, calWindDirection, rain, soilTemp1, soilTemp2, soilTemp3, soilHumidity1, soilHumidity2, soilHumidity3, CO2_1, CO2_2, volt1, volt2, volt3, volt4, volt5, rssi);
    // prepareData(NAN,NAN,temp1,rh1,temp2,rh2,temp3,rh3,pressure,NAN,NAN,NAN,soilTemp1,soilTemp2,soilTemp3,soilHumidity1,soilHumidity2,soilHumidity3,CO2_1,CO2_2,volt1,volt2,volt3,volt4,volt5,rssi);

    //sent data
    if (setMode) {UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, payload);}
    
    //turnoff soilHumidity
    powerOffSoilHumidity();

    SDcount++;
    payload = "";

    timerInterval.reset();
    timerInterval.start();
  }

  if (setMode) {UDPReceive resp = AISnb.waitResponse();}

  //write data to sd card 
  if (SDInterval.elapsed() >= SDTime) {
    String fileLog = String(now.year());
    if (now.month() < 10) {
        fileLog += "-0";
        fileLog += String(now.month());
    }
    else {
        fileLog += "-";
        fileLog += String(now.month());
    }
    fileLog += ".csv";
    SDInterval.stop();
    dataFile = SD.open(fileLog, FILE_WRITE);
    //write DateTime
    dataFile.print(RecNum);
    dataFile.print(',');
    dataFile.print(now.year(), DEC);
    dataFile.print('/');
    dataFile.print(now.month(), DEC);
    dataFile.print('/');
    dataFile.print(now.day(), DEC);
    dataFile.print(',');
    dataFile.print(now.hour(), DEC);
    dataFile.print(':');
    dataFile.print(now.minute(), DEC);
    dataFile.print(',');

    //send temp data to calculate avg.data and write data to log file
    logData(tmplight1Data, tmplight2Data, tmpTemp1, tmpRh1, tmpTemp2, tmpRh2, tmpTemp3, tmpRh3, tmpPressure, tmpWindSpeed, tmpWindDirection, sumRain, tmpSoilTemp1, tmpSoilTemp2, tmpSoilTemp3, tmpSoilHumid1, tmpSoilHumid2, tmpSoilHumid3, tmpCO2_1, tmpCO2_2, tmpVolt1, tmpVolt2, tmpVolt3, tmpVolt4, tmpVolt5, rssi);
    // logData(NAN,NAN,tmpTemp1,tmpRh1,tmpTemp2,tmpRh2,tmpTemp3,tmpRh3,tmpPressure,NAN,NAN,NAN,tmpSoilTemp1,tmpSoilTemp2,tmpSoilTemp3,tmpSoilHumid1,tmpSoilHumid2,tmpSoilHumid3,tmpCO2_1,tmpCO2_2,tmpVolt1,tmpVolt2,tmpVolt3,tmpVolt4,tmpVolt5,rssi);

    //reset temp file
    resetTempData();

    //close file
    dataFile.close();

    SDcount = 0;
    RecNum++;
    SDInterval.reset();
    SDInterval.start();
  }

  delay(10);
}

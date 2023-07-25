/*
 Weather Shield Example
 By: Nathan Seidle
 SparkFun Electronics
 Date: November 16th, 2013
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

 Much of this is based on Mike Grusin's USB Weather Board code: https://www.sparkfun.com/products/10586

 This code reads all the various sensors (wind speed, direction, rain gauge, humidty, pressure, light, batt_lvl)
 and reports it over the serial comm port. This can be easily routed to an datalogger (such as OpenLog) or
 a wireless transmitter (such as Electric Imp).

 Measurements are reported once a second but windspeed and rain gauge are tied to interrupts that are
 calcualted at each report.

 This example code assumes the GP-735 GPS module is attached.

 Updated by Joel Bartlett
 03/02/2017
 Removed HTU21D code and replaced with Si7021

 11/19/2017 - Travis Farmer
 added code for DS18B20 for extendid range temp, WiFi, MQQT.
 */


#include <SoftwareSerial.h> //Needed for GPS
#include <TinyGPS++.h> //GPS parsing - Available through the Library Manager.
#include <dhtnew.h>

DHTNEW mySensor(22);
TinyGPSPlus gps;

#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>

//#include <Wire.h>
//#include "SparkFun_AS3935.h"

// 0x03 is default, but the address can also be 0x02, 0x01.
// Adjust the address jumpers on the underside of the product.
#define AS3935_ADDR 0x03
#define INDOOR 0x12
#define OUTDOOR 0xE
#define LIGHTNING_INT 0x08
#define DISTURBER_INT 0x04
#define NOISE_INT 0x01

//SparkFun_AS3935 lightning(AS3935_ADDR);

// Interrupt pin for lightning detection
const int lightningInt = 23;

// This variable holds the number representing the lightning or non-lightning
// event issued by the lightning detector.
int intVal = 0;
int noise = 2; // Value between 1-7
int disturber = 2; // Value between 1-10


// Update these with values suitable for your hardware/network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xEE };
IPAddress server(192, 168, 1, 100);
IPAddress ip(192, 168, 0, 29);
IPAddress myDns(192, 168, 0, 1);
static const int RXPin = 5, TXPin = 4; //GPS is attached to pin 4(TX from GPS) and pin 5(RX into GPS)
SoftwareSerial ss(RXPin, TXPin);



//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// digital I/O pins
const byte WSPEED = 3;
const byte RAIN = 2;
const byte STAT1 = 7;
const byte STAT2 = 8;
const byte GPS_PWRCTL = 6; //Pulling this pin low puts GPS to sleep but maintains RTC and RAM

// analog I/O pins
const byte REFERENCE_3V3 = A3;
const byte LIGHT = A1;
const byte BATT = A2;
const byte WDIR = A0;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastSecond; //The millis counter to see when a second rolls by
byte seconds; //When it hits 60, increase the current minute
byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes; //Keeps track of where we are in various arrays of data
byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data


long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;

//We need to keep track of the following variables:
//Wind speed/dir each update (no storage)
//Wind gust/dir over the day (no storage)
//Wind speed/dir, avg over 2 minutes (store 1 per second)
//Wind gust/dir over last 10 minutes (store 1 per minute)
//Rain over the past hour (store 1 per minute)
//Total rain over date (store one per day)

byte windspdavg[120]; //120 bytes to keep track of 2 minute average
int winddiravg[120]; //120 ints to keep track of 2 minute average
float windgust_10m[10]; //10 floats to keep track of 10 minute max
int windgustdirection_10m[10]; //10 ints to keep track of 10 minute max
volatile float rainHour[60]; //60 floating numbers to keep track of 60 minutes of rain

//These are all the weather values that wunderground expects:
int winddir = 0; // [0-360 instantaneous wind direction]
float windspeedmph = 0.0; // [mph instantaneous wind speed]
float windgustmph = 0.0; // [mph current wind gust, using software specific time period]
int windgustdir = 0; // [0-360 using software specific time period]
float windspdmph_avg2m = 0.0; // [mph 2 minute average wind speed mph]
int winddir_avg2m = 0; // [0-360 2 minute average wind direction]
float windgustmph_10m = 0.0; // [mph past 10 minutes wind gust mph ]
int windgustdir_10m = 0; // [0-360 past 10 minutes wind gust direction]
float humidity = 0.0; // [%]
float tempf = 0.0; // [temperature F]
float rainin = 0.0; // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
volatile float dailyrainin = 0.0; // [rain inches so far today in local time]
//float baromin = 30.03;// [barom in] - It's hard to calculate baromin locally, do this in the agent
float pressure = 0.0;
//float dewptf; // [dewpoint F] - It's hard to calculate dewpoint locally, do this in the agent

float batt_lvl = 11.8; //[analog value from 0 to 1023]
float light_lvl = 455; //[analog value from 0 to 1023]

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// Singleton instance of the radio driver
RH_RF69 driver(23, 18); // SPI SS on 23, INT3 on UART tx01
//RH_RF69 driver(15, 16); // For RF69 on PJRC breakout board with Teensy 3.1
//RH_RF69 driver(4, 2); // For MoteinoMEGA https://lowpowerlab.com/shop/moteinomega
//RH_RF69 driver(8, 7); // Adafruit Feather 32u4

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

//Variables used for GPS
//float flat, flon; // 39.015024 -102.283608686
//unsigned long age;
//int year;
//byte month, day, hour, minute, second, hundredths;

// volatiles are subject to modification by IRQs
volatile unsigned long raintime, rainlast, raininterval, rain;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime = millis(); // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

    if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {
    dailyrainin += 0.011; //Each dump is 0.011" of water
    rainHour[minutes] += 0.011; //Increase this minute's amount of rain

    rainlast = raintime; // set up for next event
  }
}

void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = millis(); //Grab the current time
    windClicks++; //There is 1.492MPH for each click per second.
  }
}


void errorProc(int errorNum) {
  unsigned long lastErrorTim = 0UL;
  int delayON = (((1000 - 750) / errorNum) / 2);
  while(1) {
    digitalWrite(STAT1,LOW);
    delay(250);
    for (int i=0; i < errorNum; i++) {
      digitalWrite(STAT1,HIGH);
      delay(delayON);
      digitalWrite(STAT1,LOW);
      delay(delayON);
    }
  }
}


EthernetClient eClient;
PubSubClient client(eClient);

long lastReconnectAttempt = 0;

boolean reconnect() {
  if (client.connect("ArduinoWS")) {
    // Once connected, publish an announcement...
    // client.publish("test/outTopic","testing");
    // ... and resubscribe
    // client.subscribe("generator/Monitor/Platform_Stats/System_Uptime");
    printWeather();

  }
  return client.connected();
}

void setup()
{
  Serial.begin (9600);
  pinMode(STAT1, OUTPUT); //Status LED Blue
  pinMode(STAT2, OUTPUT); //Status LED Green
  digitalWrite(STAT1, LOW);
  digitalWrite(STAT2, LOW);

  if (!manager.init())
    digitalWrite(STAT1, HIGH);
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  driver.setTxPower(14, true);

  delay(1500);

  //ss.begin(9600); //Begin listening to GPS over software serial at 9600. This should be the default baud of the module.


  pinMode(GPS_PWRCTL, OUTPUT);
  digitalWrite(GPS_PWRCTL, HIGH); //Pulling this pin low puts GPS to sleep but maintains RTC and RAM

  pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
  pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor

  pinMode(REFERENCE_3V3, INPUT);
  pinMode(LIGHT, INPUT);


  seconds = 0;
  lastSecond = millis();

  // attach external interrupt pins to IRQ functions
  attachInterrupt(0, rainIRQ, FALLING);
  attachInterrupt(1, wspeedIRQ, FALLING);

  // turn on interrupts
  interrupts();

  //Serial.println("Weather Shield online!");
// When lightning is detected the interrupt pin goes HIGH.
  //pinMode(lightningInt, INPUT);
  //Wire.begin();
  //lightning.begin();
}


// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void loop()
{
  digitalWrite(STAT2, HIGH);

  // Send a message to manager_server
    calcWeather(); //Go calc all the various sensors
  char sz[10];

  sprintf(sz, "%d", winddir);
  String strWindDir = "0";
  strWindDir.concat(sz);
  char chrWindDir[10];
  strWindDir.toCharArray(chrWindDir, strWindDir.length());
  manager.sendtoWait(chrWindDir, sizeof(chrWindDir), SERVER_ADDRESS);

  dtostrf(windspeedmph, 4, 2, sz);
  String strWindSpeed = "1";
  strWindSpeed.concat(sz);
  char chrWindSpeed[10];
  strWindSpeed.toCharArray(chrWindSpeed, strWindSpeed.length());
  manager.sendtoWait(chrWindSpeed, sizeof(chrWindSpeed), SERVER_ADDRESS);

  dtostrf(humidity, 4, 2, sz);
  String strHumidity = "2";
  strHumidity.concat(sz);
  char chrHumidity[10];
  strHumidity.toCharArray(chrHumidity, strHumidity.length());
  manager.sendtoWait(chrHumidity, sizeof(chrHumidity), SERVER_ADDRESS);

  dtostrf(tempf, 4, 2, sz);
  String strTempF = "3";
  strTempF.concat(sz);
  char chrTempF[10];
  strTempF.toCharArray(chrTempF, strTempF.length());
  manager.sendtoWait(chrTempF, sizeof(chrTempF), SERVER_ADDRESS);

  dtostrf(rainin, 4, 2, sz);
  String strRainIn = "4";
  strRainIn.concat(sz);
  char chrRainIn[10];
  strRainIn.toCharArray(chrRainIn, strRainIn.length());
  manager.sendtoWait(chrRainIn, sizeof(chrRainIn), SERVER_ADDRESS);

  dtostrf(dailyrainin, 4, 2, sz);
  String strDRainIn = "5";
  strDRainIn.concat(sz);
  char chrDRainIn[10];
  strDRainIn.toCharArray(chrDRainIn, strDRainIn.length());
  manager.sendtoWait(chrDRainIn, sizeof(chrDRainIn), SERVER_ADDRESS);

  dtostrf(batt_lvl, 4, 2, sz);
  String strBatt = "6";
  strBatt.concat(sz);
  char chrBatt[10];
  strBatt.toCharArray(chrBatt, strBatt.length());
  manager.sendtoWait(chrBatt, sizeof(chrBatt), SERVER_ADDRESS);



  //Keep track of which minute it is
  if(millis() - lastSecond >= 1000)
  {

    lastSecond += 1000;

    //Take a speed and direction reading every second for 2 minute average
    if(++seconds_2m > 119) seconds_2m = 0;




  }

  //if(digitalRead(lightningInt) == HIGH){
    //intVal = lightning.readInterruptReg();
    //if(intVal == LIGHTNING_INT){
      //lightning_DistKM = lightning.distanceToStorm();
    //}
  //}

  smartdelay(1000); //Wait 1 second, and gather GPS data


}


//While we delay for a given amount of time, gather GPS data
static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial.available())
      gps.encode(Serial.read());
  } while (millis() - start < ms);
}


//Calculates each of the variables that wunderground is expecting
void calcWeather()
{
  //Calc winddir
  winddir = get_wind_direction();

  //Calc windspeed
  windspeedmph = get_wind_speed(); //This is calculated in the main loop

  int chk = mySensor.read();
  //Calc humidity
  humidity = mySensor.getHumidity();

  //tempf = mySensor.getTemperature();
  tempf = ((mySensor.getTemperature() * 1.8) + 32);
  //Total rainfall for the day is calculated within the interrupt
  //Calculate amount of rainfall for the last 60 minutes
  rainin = 0;
  for(int i = 0 ; i < 60 ; i++)
    rainin += rainHour[i];


  //Calc dewptf

  //Calc battery level
  batt_lvl = get_battery_level();

}

//Returns the voltage of the raw pin based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
//Battery level is connected to the RAW pin on Arduino and is fed through two 5% resistors:
//3.9K on the high side (R1), and 1K on the low side (R2)
float get_battery_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);

  float rawVoltage = analogRead(BATT);

  operatingVoltage = 3.30 / operatingVoltage; //The reference voltage is 3.3V

  rawVoltage = operatingVoltage * rawVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin

  rawVoltage *= 4.90; //(3.9k+1k)/1k - multiple BATT voltage by the voltage divider to get actual system voltage

  return(rawVoltage);
}

//Returns the instataneous wind speed
float get_wind_speed()
{
  float deltaTime = millis() - lastWindCheck; //750ms


  deltaTime /= 1000.0; //Covert to seconds

  float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4

  windClicks = 0; //Reset and start watching for new wind
  lastWindCheck = millis();

  windSpeed *= 1.492; //4 * 1.492 = 5.968MPH

  // Serial.println();
   //Serial.print("Windspeed:");
   //Serial.println(windSpeed);
  if (isnan(windSpeed) == true) {
    return(0.00);
  } else {
    return(windSpeed);
  }
}

//Read the wind direction sensor, return heading in degrees
int get_wind_direction()
{
  unsigned int adc;

  adc = analogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  if (adc < 380) return (113);
  if (adc < 393) return (68);
  if (adc < 414) return (90);
  if (adc < 456) return (158);
  if (adc < 508) return (135);
  if (adc < 551) return (203);
  if (adc < 615) return (180);
  if (adc < 680) return (23);
  if (adc < 746) return (45);
  if (adc < 801) return (248);
  if (adc < 833) return (225);
  if (adc < 878) return (338);
  if (adc < 913) return (0);
  if (adc < 940) return (293);
  if (adc < 967) return (315);
  if (adc < 990) return (270);
  return (0); // error, disconnected?
}

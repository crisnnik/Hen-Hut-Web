// CP'S HAPPY HEN HUT v17.8.15

/*
************************************************************************
  Remixed by Chris Peck
  Based upon the originating code of:
  David Naves
  Roger Reed
  THANK YOU!
  
  HAPPY HEN HUT COOP CONTROL

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
  02110-1301, USA.
************************************************************************* */

/*
********************************************************************************************************************************
  v17.8.15
  -Made adjustments to door close calculations with new int enable DoorCloseHr  
  v17.5.31
  -Changed INITSTATE to check door initial status using local time equivalent - UTC method was always failing and defaulting to door closed on reboot
  -used values derived in getLocalDoorTimes() for debug and display on LCD to limit calculations to that function
  -Modify lines 581-589 to toggle Supplemental Light (CURRENTLY DISABLED (only on when door opens))
  v17.5.13
  -Replaced CC3000 with ATWINC1500, modified code to function with this new module.
  -Changed serial debugging functions.  #define DEBUG ceased compiling, so changed to the new method.
  -Modify lines 70-80 to enable and disable easily.
  v17.5.07
  - Added ESP8266 running NodeMCU to host a web page for status and door control
  v17.3.25
  - Incorporates the Timezone Library for EST_DST compensation. FINALLY! RTC needs to be set to UTC time.
  - Changed recalculation of Alarms to just after midnight UTC
  v15.3.6
  - Modified code to disconnect cc3000 after each send message.
  - prior to sending sms message, call to connectWifi(); to connect to network.
  - connectWifi(); starts with cc3k.reboot();
  - This version has no integration for automatic adjustments due to DST conversion in the US.  I am still working on this aspect.
  Upon a time change (each FALL and SPRING), you must:
    A)  Reprogram the RTC to match the current time to the laptop using the Examples --> DS1307 --> SetTime Example Sketch.
    B)  Modify the TIMEZONE Integer Value below to match the current time zone status.
    C)  Re-Upload the modified sketch to the Arduino.
  Version 15.2.21
  - All functions running, but sms messages unreliable.  Usually fails to send all messages after a day or so.
********************************************************************************************************************************
*/

// Load Necessary Libraries

#include <Wire.h>
#include <WiFi101.h>
#include <WiFiSSLClient.h>
#include <OneWire.h>
#include <TimeLord.h>
#include "TimerObject.h"
#include <TimeLib.h>
#include <Timezone.h>
#include <TimeAlarms.h>
#include <DS1307RTC.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <TembooSSL.h>
#include "TembooAccount.h" // Temboo Account / Wifi Information
#include "Arduino.h"       // Library for the Degugging Function

// CHOOSE ONE OR THE OTHER BELOW FOR SERIAL MONITOR OUTPUT
// ********************* Disables Serial Monitor Output -- also //serialDubugState in the LOOPS **********************
//#define SPrint(a)
//#define SPrintln(a)
//#define SPrint2(a,b)
//#define SPrintln2(a,b)

// ********************** Enables Serial Monitor Output -- also enable serialDubugState in the LOOPS *****************
#define SPrint(a) (Serial.print(a))
#define SPrintln(a) (Serial.println(a))
#define SPrint2(a,b) (Serial.print(a,b))
#define SPrintln2(a,b) (Serial.println(a,b))


// Define Pins
const byte   PHOTOCELL = A0,                                      // Photocell on Analog 0
             ENABLE_COOP_DOOR_MOTOR = 7,                          // Enable Door Motor on digital pin 7
             DOOR_OPEN_SWITCH_PIN = 14,                           // Manual Switch to Open Door on digital pin 14
             DOOR_CLOSE_SWITCH_PIN = 15,                          // Manual Switch to Close Door on digital pin 15
             RUN_LIGHT_TOGGLE_BUTTON_PIN = 18,                    // Run Light Switch on digital pin 18
             COOP_LIGHT_TOGGLE_BUTTON_PIN = 19,                   // Coop Light Switch on digital pin 19
             IR_RECV_PIN = 33,                                    // IR Receiver pin on digital pin 33
             DOOR_MOTOR_CLOSE_PIN = 38,                           // Relay to CLOSE Coop Door on digital pin 38
             DOOR_MOTOR_OPEN_PIN = 39,                            // Relay to OPEN Coop Door on digital pin 39
             DOOR_OPEN_STOP_PIN = 41,                             // Top Reed Switch on digital pin 41
             AIR_TEMP_SENSOR_PIN = 42,                            // Temp Sensor on digital pin 42
             DOOR_CLOSE_STOP_PIN = 43,                            // Bottom Reed Switch on digital pin 43
             RUN_LIGHT_RELAY_PIN = 44,                            // Relay for 120V Run light on digital pin 44
             CASE_FAN_RELAY_PIN = 45,                             // Relay for Arduino Case Fan on digital pin 45
             DOOR_CLOSED_LED = 46,                                // Door LED Closed on digital pin 46
             DOOR_OPEN_LED = 47,                                  // Door LED Open on digital pin 47
             HVAC_LED = 48,                                       // Fan LED Indicator on digital pin 48
             COOP_FAN_RELAY_PIN = 49,                             // Relay for Coop Exhaust Fan on digital pin 50
             COOP_LIGHT_RELAY_PIN = 53;                           // Relap to Coop Light on digital pin 53



//US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};        //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};         //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

//If TimeChangeRules are already stored in EEPROM, comment out the three
//lines above and uncomment the line below.
//Timezone myTZ(100);       //assumes rules stored at EEPROM address 100
TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev
time_t utc, local;

TimerObject *updateWifiChipTimer = new TimerObject(5000); // Wifi chip variable updater -- pushes the state of variables to the esp8266 wifi chip to keep it updated

WiFiSSLClient client;

// Delay Constants
const int POST_IR_HANDLE_DELAY_MS = 1250,
          POST_BUTTON_HANDLE_DELAY_MS = 750;

// Time Constants
const int DOOR_SHUT_AFTER_SUNSET_MIN = 45,              // Close dooe 45 minutes after sunset
          DOOR_OPEN_AFTER_SUNRISE_MIN = 59,             // Open door 59 minutes after sunrise
          // LIGHT_PER_DAY_HRS = 12,                    // Only used for coop light based on Natural Daylight
          // LIGHT_ON_BEFORE_SUNSET_HRS = 1,            // Only used for coop light based on Natural Daylight
          LIGHT_ON_BEFORE_SUNRISE_HRS = 2,              // Fixed duration BEFORE sunrise
          LIGHT_ON_AFTER_SUNRISE_HRS = 1;               // Fixed duration AFTER sunrise

// Temperature Constants
const float MIN_REASONABLE_AIR_TEMP_F = 0.0f,           // low air temp we should never see; if we do most likely a temp probe issue
            MAX_REASONABLE_AIR_TEMP_F = 150.0f,         // high air temp we should never see
            INIT_TEMP = -1000.0f,
            FAN_ON_TEMP_F = 87.0f,                      // temperature fan is turned on
            FAN_OFF_TEMP_F = 82.0f;                     // temperature fan is turned off

// Location Constants
const float LATITUDE = 43.231681,                       // Latitude @ Barrington NH Castlerock Road
            LONGITUDE = -71.009833;                     // Longtitude @ Barrington NH Castlerock Road

int TZADJUST;                                           // to correct displayed local times
int enableDoorCloseHr;
int localHrOpen;
int localMinOpen;
int localHrClose;
int localMinClose;
int tZone;
int status = WL_IDLE_STATUS;                            // status for the ATCWINC1500

// Photocell Variables
int photocellReading;                                   // analog reading of the photocell
int photocellReadingLevel;                              // photocel reading levels (dark, twilight, light)

// Temperature Variables
OneWire airTempOneWire(AIR_TEMP_SENSOR_PIN);
boolean airTempError = false;
float airTemp = INIT_TEMP;

// Fan Status
int fanStatus;                                          // status of fan on or off for LCD if desired

// Network Status
boolean networkError = false;                           // To Trigger when a Network Error Occurs

// Door Status
boolean doorError = false;                              // To Trigger when a Door Error Occurs
String doorStatus = "Idle";
String doorState = "Unknown";
String fanDetail = "Off";
String coopLightDetail = "Off";
String runLightDetail = "Off";
String strDoorOpen;

// Time Variables
time_t timeEnd;                                         // Variable for holding the max door motor run time

TimeLord coopTimeLord;
long lastLoopSec = now();
byte openDoorHr, openDoorMin, closeDoorHr, closeDoorMin,
     coopLightOnHr, coopLightOnMin, coopLightOffHr, coopLightOffMin;

// LCD

// For I2C Display
// set the LCD address for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address Mine is 0x27

// ***************************************** THE SETUP *******************************************

void setup() {
  WiFi.setPins(8, 5, 4);                                // Pins for ATCWINC1500 Wifi Module (CS/IRQ/RST)
  Serial.begin(57600);                                  // Initialize the Hardware Monitor Serial Port
  Serial2.begin(115200);                                // Serial To/From ESP8266 (old 9600)
  Wire.begin();
  lcd.begin(20, 4);                                     // Set Columns, Rows
  // ------- Quick 3 blinks of backlight  -------------
  for (int i = 0; i < 3; i++)
  {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  lcd.backlight();
  SPrintln(F("Welcome To The Happy Hen Hut!"));
  delay(2000);
  SPrintln(F("Let's Get This Thing Rolling..."));
  delay(2000);
  setSyncProvider(RTC.get);                             // Sync Arduino Time with Real Time CLock
  if (timeStatus() != timeSet) {
    SPrintln(F("RTC FAILED to Set the System Time!"));
  } else {
    SPrintln(F("RTC has SUCCESSFULLY set the System Time!"));
  }
  SPrintln(F("Verify Wifi Functionality..."));
  connectWifi();
  rebootEsp8266();                                      // Restarts webserver on any power failure
  utc = now();
  printTime(utc, "UTC");
  local = myTZ.toLocal(utc, &tcr);
  printTime(local, tcr -> abbrev);
  getTzAdjust();
  SPrint(F("TZADJUST for Displayed Times = -"));
  SPrintln(TZADJUST);
  updateWifiChipTimer->setOnTimer(&sendVariablesToEsp8266);
  coopTimeLord.TimeZone(0);                             // Set Time Zone for all calculations to UTC
  coopTimeLord.Position(LATITUDE, LONGITUDE);           // Set defined Latitude and Longtitude
  setupOutputPins();                                    // Setup all output pins and initial state of each
  initState();                                          // Determine and set initial state based on current time of day
  scheduleDailyAlarms();                                // Calculate Daily Schedule Times
  scheduleTodayAlarms();                                // Today Alarms for Door and Light Functions
  SPrintln(F("\nHappy Hen Hut Initialized...\n"));
}

// ******************************************** THE LOOP ********************************************
void loop() {

  serialDebugState();
  handleDoor();
  serialComms();
  updateAirTemp();
  handleFan();
  handleLightSwitches();
  handleDoorSwitches();
  readPhotoCell();
  updateWifiChipTimer->Update();
  Alarm.delay(0);
  lastLoopSec = now();
  doLcdMsg();
}
// ******************************************** END OF LOOP ******************************************

// ****************************************** DOOR ERROR LOOP ****************************************
void errorLoop() {
  serialDebugState();
  serialComms();
  digitalWrite(DOOR_CLOSED_LED, HIGH);                      // turn OFF Green
  digitalWrite(DOOR_OPEN_LED, LOW);                         // turn ON Red
  delay(100);                                               // pause to FLASH
  digitalWrite(DOOR_OPEN_LED, HIGH);                        // turn OFF Red
  delay(100);                                               // pause to FLASH
  readPhotoCell();                                          // Read Light Level
  updateAirTemp();                                          // Update the Temp
  handleLightSwitches();                                    // Monitor Light Switches For Action
  updateWifiChipTimer->Update();
  handleFan();                                              // Handle Fan Control
  SPrintln(F("DOOR ERROR! - Loop Stopped - Clear Obstruction and Reset the System!"));
  SPrintln(F(""));
  doLcdMsg();                                               // Update the LCD Display
  errorLoop();                                              // Repeat Error Loop Repeatedly to Prevent Door Motor Damage Until the System is Reset
}

// **************************************** END DOOR ERROR LOOP **************************************

// ******************************************* TIME FUNCTIONS ****************************************

// Determine time offset for LCD and alarms
void getTzAdjust() {
  utc = now();
  local = myTZ.toLocal(utc, &tcr);
  String tzr = (tcr -> abbrev);
  SPrint(F("Current Time Zone Setting is: "));
  SPrint(tcr -> abbrev);
  SPrintln(F(""));
  if (tzr  == "EDT") {
    TZADJUST = 4;                    // Subtract 4 hours from UTC During EDT for local time display purposes
  } else {
    TZADJUST = 5;                    // Subtract 5 hours from UTC During EST for local time display purposes
  }
}

//Function to print time with timezone to serial
void printTime(time_t t, char *tz) {
  sPrintI00(hour(t));
  sPrintDigits(minute(t));
  sPrintDigits(second(t));
  SPrint(' ');
  SPrint(dayShortStr(weekday(t)));
  SPrint(' ');
  sPrintI00(day(t));
  SPrint(' ');
  SPrint(monthShortStr(month(t)));
  SPrint(' ');
  SPrint(year(t));
  SPrint(' ');
  SPrint(tz);
  SPrintln();
}
//Print an integer in "00" format (with leading zero).
//Input value assumed to be between 0 and 99.
void sPrintI00(int val) {
  if (val < 10) SPrint('0');
  SPrint2(val, DEC);
  return;
}
//Print an integer in ":00" format (with leading zero).
//Input value assumed to be between 0 and 99.
void sPrintDigits(int val) {
  SPrint(':');
  if (val < 10) SPrint('0');
  SPrint2(val, DEC);
}

//Function to print time with time zone to LCD
void lcdTime(time_t t, char *tz) {
  utc = now();
  local = myTZ.toLocal(utc, &tcr);
  lcdPrintI00(hour(t));
  lcdPrintDigits(minute(t));
  lcdPrintDigits(second(t));
  lcd.print(' ');
  lcd.print(tz);
}

//Print an integer in "00" format (with leading zero).
//Input value assumed to be between 0 and 99.
void lcdPrintI00(int val) {
  if (val < 10) lcd.print('0');
  lcd.print(val, DEC);
  return;
}

//Print an integer in ":00" format (with leading zero).
//Input value assumed to be between 0 and 99.
void lcdPrintDigits(int val) {
  lcd.print(':');
  if (val < 10) lcd.print('0');
  lcd.print(val, DEC);
}

// Get Local Door Times for WebServer
void getLocalDoorTimes() {
  localHrOpen = (openDoorHr - TZADJUST);
  localMinOpen = (openDoorMin);
  if (closeDoorHr == 0) {
    localHrClose = (24 - TZADJUST);
  } else {
    if (closeDoorHr == 1) {
      localHrClose = (25 - TZADJUST);
    } else {
      localHrClose = (closeDoorHr - TZADJUST);
    }
  }
  localMinClose = (closeDoorMin);
  tZone = (tcr -> abbrev);
}

// ************************************* END OF TIME FUNCTIONS *************************************

// ***************************************** SMS FUNCTIONS *****************************************

// Send SMS Alert(s)
void sendSMS() {
  SPrintln(F("Sending SMS Alert..."));
  TembooChoreoSSL SendSMSChoreo(client);

  // Invoke the Temboo Client
  SendSMSChoreo.begin();

  // Set Temboo Account Credentials
  String BodyValue = "";
  SendSMSChoreo.setAccountName(TEMBOO_ACCOUNT);
  SendSMSChoreo.setAppKeyName(TEMBOO_APP_KEY_NAME);
  SendSMSChoreo.setAppKey(TEMBOO_APP_KEY);
  SendSMSChoreo.setDeviceType(TEMBOO_DEVICE_TYPE);
  // Set profile to use for execution
  SendSMSChoreo.setProfile("TwilioAccount");
  SPrintln("Account Set");
  // Set Choreo Inputs
  SendSMSChoreo.addInput("To", ToValue);
  SPrintln(ToValue);
  SendSMSChoreo.addInput("From", FromValue);
  SPrintln(FromValue);

  {
    if (isDoorCloseStop() && doorError == FALSE) {
      SPrintln(F("Compose SMS for: Door Closed"));
      BodyValue = "The Coop Door Is CLOSED";
    }
    else if (isDoorOpenStop() && doorError == FALSE) {
      SPrintln(F("Compose SMS for: Door Open"));
      BodyValue = "The Coop Door Is OPEN";
    }
    else if (doorError == TRUE) {
      SPrintln(F("Compose SMS for: Door Error"));
      BodyValue = "ATTENTION!!  Coop Door ERROR!";
    }
  }

  SendSMSChoreo.addInput("Body", BodyValue);
  SPrintln(AuthTokenValue);
  SPrintln(AccountSIDValue);
  SPrintln(ToValue);
  SPrintln(FromValue);
  SPrintln(BodyValue);
  SPrintln(F("SMS Values Set"));

  // Identify the Choreo to run
  SendSMSChoreo.setChoreo("/Library/Twilio/SMSMessages/SendSMS");
  SPrintln(F("Library is Set"));
  // Run the Choreo; when results are available, print them to serial
  // tell the Process to run and wait for the results. The
  // return code (returnCode) will tell us whether the Temboo client
  // was able to send our request to the Temboo servers
  SPrintln(F("SendSMSChoreo Initiated"));       // Sending SMS #1

  // Run the Choreo; when results are available, print them to serial
  SendSMSChoreo.run();
  while (SendSMSChoreo.available()) {
    char c = SendSMSChoreo.read();
    SPrint(c);
  }
  SendSMSChoreo.close();
  SPrintln(F("Choreo Process Closed"));
}

// **************************************  END SMS FUNCTIONS  **************************************

// **************************************** FREE RAM CHECK *****************************************

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

// *************************************** END FREE RAM CHECK **************************************

// ***************************************** WIFI FUNCTIONS ****************************************

void connectWifi() {
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    SPrintln(F("WiFi shield not present"));
    // don't continue:
    while (true);
  }
  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    SPrint(F("Attempting to connect to SSID: "));
    SPrintln(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(5000);
  }
  SPrintln(F("Connected to wifi"));
  printWiFiStatus();
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  SPrint(F("Wifi Status:  "));
  SPrintln(WiFi.status());
  SPrint(F("SSID: "));
  SPrintln(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  SPrint(F("IP Address: "));
  SPrintln(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  SPrint(F("Signal Strength (RSSI):"));
  SPrint(rssi);
  SPrintln(F(" dBm"));
}

void rebootEsp8266() {
  // Restart wifi-ESP8266 for Web Server on startup. This is done in case the arduino restarts but the wifi chip doesn't.
  // It would need the ESP8266 chip to reboot as well, so then the chip will request updated variables, and this kicks off the interval to keep updating the variables on the wifi chip
  SPrintln(F("Send nodeMCU script to restart the ESP8266"));
  sendCmdToEsp8266("node.restart()");
  delay(5000);        //On restart I think this delay was causing the door motor to overrun
}

// ********************************************* END WIFI FUNCTIONS ****************************************

// ************************************************ ALARM METHODS ******************************************

// Calculates Alarm Times Used for Scheduling and Init State.
void calculateAlarmTimes() {
  int nowHour = hour(),
      nowMinute = minute(),
      nowDay = day(),
      nowMonth = month(),
      nowYear = year();

  byte timeLordSunRise[]  = {0, 0, 0, nowDay, nowMonth, nowYear};
  byte timeLordSunSet[]  = {0, 0, 0, nowDay, nowMonth, nowYear};

  coopTimeLord.SunRise(timeLordSunRise);
  coopTimeLord.SunSet(timeLordSunSet);
  SPrintln(F("VALUES DERIVED FROM void calculateAlarmTimes --"));
  SPrint(F("Sunrise Today: "));
  SPrint(timeLordSunRise[2]);
  SPrint(F(":"));
  SPrintln(timeLordSunRise[1]);
  SPrint(F("Sunset Today: "));
  SPrint(timeLordSunSet[2]);
  SPrint(F(":"));
  SPrintln(timeLordSunSet[1]);

  openDoorHr = (timeLordSunRise[2]);
  openDoorMin = timeLordSunRise[1];
  closeDoorHr = (timeLordSunSet[2]);
  closeDoorMin = timeLordSunSet[1];

  if (closeDoorMin + DOOR_SHUT_AFTER_SUNSET_MIN >= 60) {
    closeDoorHr += (closeDoorMin + DOOR_SHUT_AFTER_SUNSET_MIN) / 60;
    closeDoorMin = (closeDoorMin + DOOR_SHUT_AFTER_SUNSET_MIN) % 60;
  } else {
    closeDoorMin += DOOR_SHUT_AFTER_SUNSET_MIN;
  }

  if (openDoorMin + DOOR_OPEN_AFTER_SUNRISE_MIN >= 60) {
    openDoorHr += (openDoorMin + DOOR_OPEN_AFTER_SUNRISE_MIN) / 60;
    openDoorMin = (openDoorMin + DOOR_OPEN_AFTER_SUNRISE_MIN) % 60;
  } else {
    openDoorMin += DOOR_OPEN_AFTER_SUNRISE_MIN;
  }

// ***************** TEST SECTION

  if (closeDoorHr == 24){
    enableDoorCloseHr = 0;
  } else {
    if (closeDoorHr == 25){
      enableDoorCloseHr = 1;
    }else{
    enableDoorCloseHr = closeDoorHr;
   }
  }

// ***************** END TEST SECTION
  
  SPrintln(F("Per calculateAlarmTimes void:"));
  SPrint(F("Door Opens Today At: "));
  SPrint(openDoorHr);
  SPrint(":");
  SPrint(openDoorMin);
  SPrintln(F(" UTC"));
  SPrint(F("Door Closes Today At: "));
  SPrint(closeDoorHr);
  SPrint(F(":"));
  SPrint(closeDoorMin);
  SPrintln(F(" UTC"));

  getLocalDoorTimes();                                                // Convert OPEN and CLOSE Times to Local Equivalent for Displays

  float naturalDaylightHr;
  naturalDaylightHr = 12.0f - (timeLordSunRise[2] + timeLordSunRise[1] / 60.0f);
  naturalDaylightHr += (timeLordSunSet[2] + timeLordSunSet[1] / 60.0f) - 12.0f;

  SPrint(F("Today's Natural Daylight (hrs): "));
  SPrintln(naturalDaylightHr);

  // DO NOT ERASE - These 4 lines are used for Daylight compensation calculations using natural daylight method
  // coopLightOnHr = timeLordSunSet[2];
  // coopLightOnMin = timeLordSunSet[1];
  // coopLightOffHr = timeLordSunSet[2];
  // coopLightOffMin = timeLordSunSet[1];
  coopLightOnHr = (timeLordSunRise[2]);
  coopLightOnMin = timeLordSunRise[1];
  coopLightOffHr = (timeLordSunRise[2]);
  coopLightOffMin = timeLordSunRise[1];

  // FOR NO SUPPLEMENTAL LIGHT
  coopLightOnHr = openDoorHr;                                          // Light on when door opens
  coopLightOffHr = (openDoorHr + 1);                                   // Leave on 1 hour

  /*
  // For Supplemental Light 
  coopLightOnHr -= LIGHT_ON_BEFORE_SUNRISE_HRS;                        // Turns on coop light at sunrise MINUS LIGHT_ON_BEFORE_SUNRISE_HRS defined above in TIME CONSTANTS
  coopLightOffHr += LIGHT_ON_AFTER_SUNRISE_HRS;                        // Turns off coop light at sunrise PLUS LIGHT_ON_AFTER_SUNRISE_HRS defined above in TIME CONSTANTS
  */
  
  /*
    float lightNeededHrs = LIGHT_PER_DAY_HRS - naturalDaylightHr;      // DO NOT ERASE - Lines below are used for Daylight compensation calcs using natural daylight method
    int lightNeededMins = lightNeededHrs * 60;

    SPrint("light needed (mins): ");
    SPrintln(lightNeededMins);

    coopLightOnHr -= LIGHT_ON_BEFORE_SUNSET_HRS;

    if(coopLightOffMin + lightNeededMins >= 60){
    coopLightOffHr += (coopLightOffMin + lightNeededMins) / 60;
    coopLightOffMin = (coopLightOffMin + lightNeededMins) % 60;
    }else{
    coopLightOffMin += lightNeededMins;
    }
  */
  SPrint(F("Supplemental Lighting On: "));
  SPrint(coopLightOnHr);
  SPrint(F(":"));
  SPrintln(coopLightOnMin);
  SPrint(F("Supplemental Lighting Off: "));
  SPrint(coopLightOffHr);
  SPrint(F(":"));
  SPrintln(coopLightOffMin);
}

// Schedule daily alarms
void scheduleDailyAlarms() {
  Alarm.alarmRepeat(0, 1, 0, scheduleTodayAlarms);                     // Schedule repeating alarm task for daily alarms at midnight - 00:01:00 UTC
}

// Schedule Today Alarms.
void scheduleTodayAlarms() {
  calculateAlarmTimes();
  int nowHr = hourMinuteToHour(hour(), minute());

  if (nowHr < hourMinuteToHour(openDoorHr, openDoorMin)) {
    Alarm.alarmOnce(openDoorHr, openDoorMin, 0, enableDoorOpening);
  }

  if (nowHr < hourMinuteToHour(closeDoorHr, closeDoorMin)) {
    Alarm.alarmOnce(enableDoorCloseHr, closeDoorMin, 0, enableDoorClosing);
  }

  if (nowHr < hourMinuteToHour(coopLightOnHr, coopLightOnMin)) {
    Alarm.alarmOnce(coopLightOnHr, coopLightOnMin, 0, coopLightOn);
  }

  if (nowHr < hourMinuteToHour(coopLightOffHr, coopLightOffMin)) {
    Alarm.alarmOnce(coopLightOffHr, coopLightOffMin, 0, coopLightOff);
  }
}
// ************************************** END ALARM METHODS *****************************************

// *********************** INITIALIZE COOP STATE BASED ON CURRENT DATE AND TIME *********************
void initState() {
  fanOff();
  calculateAlarmTimes();
  int nowHr = hourMinuteToHour(hour(), minute());
  int local = (nowHr - TZADJUST);
  if (nowHr > hourMinuteToHour(coopLightOnHr, coopLightOnMin) && nowHr < hourMinuteToHour(coopLightOffHr, coopLightOffMin)) {
    SPrintln(F("Coop Light Initial State: ON"));
    coopLightOn();
  } else {
    SPrintln(F("Coop Light Initial State: OFF"));
    coopLightOff();
  }
  // Sets door state for web server if door is in proper position at any system restart
  if (isDoorOpenStop()) {
    doorState = "Open";
  } else {
    doorState = "Closed";
  }

  //if (nowHr > hourMinuteToHour(openDoorHr, openDoorMin) && nowHr < hourMinuteToHour(closeDoorHr, closeDoorMin)) {   //original code using UTC
  // the above line would always fail and default to door closed when resetting the system
  
  getLocalDoorTimes();
  if (local > hourMinuteToHour(localHrOpen, localMinOpen) && local < hourMinuteToHour(localHrClose, localMinClose)) { // Set Init State based upon local time equivalent
    SPrintln(F("Door Initial State: OPEN"));
    digitalWrite(DOOR_OPEN_LED, LOW);
    digitalWrite(DOOR_CLOSED_LED, HIGH);
    enableDoorOpening();
  } else {
    SPrintln(F("Door Initial State: CLOSED"));
    digitalWrite(DOOR_CLOSED_LED, LOW);
    digitalWrite(DOOR_OPEN_LED, HIGH);
    enableDoorClosing();
  }
}
// **************************************** END INITIAL STATE ***************************************

// **************************************** SETUP OUTPUT PINS ***************************************
void setupOutputPins() {

  // relays
  pinMode(DOOR_MOTOR_OPEN_PIN, OUTPUT);                     // Set door OPEN relay as output
  digitalWrite(DOOR_MOTOR_OPEN_PIN, LOW);                   // Set door relay to start OFF
  pinMode(DOOR_MOTOR_CLOSE_PIN, OUTPUT);                    // Set door CLOSE relay as output
  digitalWrite(DOOR_MOTOR_CLOSE_PIN, LOW);                  // Set door relay to start OFF
  pinMode(RUN_LIGHT_RELAY_PIN, OUTPUT);                     // Set Run Light relay as output
  digitalWrite(RUN_LIGHT_RELAY_PIN, HIGH);                  // Set Run Light relay to start OFF
  pinMode(COOP_LIGHT_RELAY_PIN, OUTPUT);                    // Set Coop Light relay as output
  digitalWrite(COOP_LIGHT_RELAY_PIN, HIGH);                 // Set Coop Light relay to start OFF
  pinMode(COOP_FAN_RELAY_PIN, OUTPUT);                      // Set Coop Fan relay as output
  digitalWrite(COOP_FAN_RELAY_PIN, HIGH);                   // Set Coop Fan relay to start OFF
  pinMode(CASE_FAN_RELAY_PIN, OUTPUT);                      // Set Case Fan Relay as output
  digitalWrite(CASE_FAN_RELAY_PIN, HIGH);                   // Set Case Fan Relay to start OFF

  // LEDs
  pinMode (DOOR_OPEN_LED, OUTPUT);                          // Set door open led output
  pinMode (DOOR_CLOSED_LED, OUTPUT);                        // Set door closed led output
  pinMode (HVAC_LED, OUTPUT);                               // Set HVAC Led output

  // Motor Enabler
  pinMode (ENABLE_COOP_DOOR_MOTOR, OUTPUT);                 // enable motor pin = output

  // Manual Coop Light Switch
  pinMode(COOP_LIGHT_TOGGLE_BUTTON_PIN, INPUT);             // set light button as input
  digitalWrite(COOP_LIGHT_TOGGLE_BUTTON_PIN, HIGH);         // activate button internal resistor

  // Manual Run Light Switch
  pinMode(RUN_LIGHT_TOGGLE_BUTTON_PIN, INPUT);              // set run light button as input
  digitalWrite(RUN_LIGHT_TOGGLE_BUTTON_PIN, HIGH);          // activate button internal resistor

  // Top Reed Switch
  pinMode(DOOR_OPEN_STOP_PIN, INPUT);                       // set door open reed switch as input
  digitalWrite(DOOR_OPEN_STOP_PIN, HIGH);                   // activate button internal resistor

  // Bottom Reed Switch
  pinMode(DOOR_CLOSE_STOP_PIN, INPUT);                      // set door closed reed switch as input
  digitalWrite(DOOR_CLOSE_STOP_PIN, HIGH);                  // activate button internal resistor

  // Door Open Switch Pin
  pinMode(DOOR_OPEN_SWITCH_PIN, INPUT);                     // set door open button as input
  digitalWrite(DOOR_OPEN_SWITCH_PIN, HIGH);                 // activate button internal resistor

  // Door Close Switch Pin
  pinMode(DOOR_CLOSE_SWITCH_PIN, INPUT);                    // set door close button as input
  digitalWrite(DOOR_CLOSE_SWITCH_PIN, HIGH);                // activate button internal resistor
}

//******************************************** DEBUG STATE *******************************************
void serialDebugState() {
  SPrintln(F(""));
  SPrintln(F("*************** BEGIN DEBUG DATA ***************"));
  SPrintln(freeRam());
  printWiFiStatus();
  SPrintln(F(""));
  utc = now();
  printTime(utc, "UTC");
  local = myTZ.toLocal(utc, &tcr);
  printTime(local, tcr -> abbrev);
  SPrintln(F(""));
  SPrint(F("Timezone Local Display Adjustment = -"));
  SPrintln(TZADJUST);
  SPrintln(F("** Per calculateAlarmTimes void **"));
  SPrint(F("enableDoorCloseHr claculates to:  "));
  SPrintln(enableDoorCloseHr);  
  SPrint(F("Door Opens Today At: "));
  SPrint(openDoorHr);
  SPrint(F(":"));
  SPrint(openDoorMin);
  SPrintln(F(" UTC"));
  SPrint(F("Door Closes Today At: "));
  SPrint(enableDoorCloseHr);
  SPrint(F(":"));
  SPrint(closeDoorMin);
  SPrintln(F(" UTC"));
  SPrint(F("Open Door:   "));
  if ((localHrOpen) < 10) {
    SPrint(0);
  }
  SPrint(localHrOpen);
  SPrint(F(":"));
  if ((localMinOpen) < 10) {
    SPrint(0);
  }
  SPrint(localMinOpen);
  SPrint(" ");
  SPrintln(tcr -> abbrev);
  SPrint(F("Close Door:  "));
  if ((localHrClose) < 10) {
    SPrint(0);
  }
  SPrint(localHrClose);
  SPrint(F(":"));
  if ((localMinClose) < 10) {
    SPrint(0);
  }
  SPrint(localMinClose);
  SPrint(" ");
  SPrintln(tcr -> abbrev);
  SPrintln(F(""));
  SPrintln(F("** Check State of Sensors and Buttons **"));
  SPrintln(F(""));
  SPrint(F("COOP_LIGHT_TOGGLE_BUTTON_PIN ("));
  SPrint2(COOP_LIGHT_TOGGLE_BUTTON_PIN, DEC);
  SPrint(F("): "));
  SPrint(digitalRead(COOP_LIGHT_TOGGLE_BUTTON_PIN));
  SPrint(F("  |  RUN_LIGHT_TOGGLE_BUTTON_PIN ("));
  SPrint2(RUN_LIGHT_TOGGLE_BUTTON_PIN, DEC);
  SPrint(F("): "));
  SPrint(digitalRead(RUN_LIGHT_TOGGLE_BUTTON_PIN));
  SPrint(F("  |  DOOR_OPEN_SWITCH_PIN ("));
  SPrint2(DOOR_OPEN_SWITCH_PIN, DEC);
  SPrint(F("): "));
  SPrint(digitalRead(DOOR_OPEN_SWITCH_PIN));
  SPrintln();
  SPrint(F("DOOR_CLOSE_SWITCH_PIN ("));
  SPrint2(DOOR_CLOSE_SWITCH_PIN, DEC);
  SPrint(F("): "));
  SPrint(digitalRead(DOOR_CLOSE_SWITCH_PIN));
  SPrint(F("  |  DOOR_CLOSE_STOP_PIN ("));
  SPrint2(DOOR_CLOSE_STOP_PIN, DEC);
  SPrint(F("): "));
  SPrint(digitalRead(DOOR_CLOSE_STOP_PIN));          //  > 500
  SPrint(F("  |  DOOR_OPEN_STOP_PIN ("));
  SPrint2(DOOR_OPEN_STOP_PIN, DEC);
  SPrint(F("): "));
  SPrint(digitalRead(DOOR_OPEN_STOP_PIN));           //  > 500
  SPrint(F("Coop Temp: "));
  if (!airTempError) {
    SPrint(airTemp);
    SPrintln();
  } else {
    SPrintln(F("ERROR"));
  }
}

// ****************************************** UPDATE ESP8266 *****************************************

void sendVariablesToEsp8266() {
  sendVariablesToEsp8266(false);
}

// sendCurrentVariablesToEsp8266
void sendVariablesToEsp8266(bool withDelay) {
  // Send variables to wifi: status, door position, coop temp, fan detail
  getLocalDoorTimes();
  // sendCmdToEsp8266("updateVariables(\"" + doorStatus + "\", \"" + doorState + "\", \"" + (airTemp) + "\")", withDelay);
     sendCmdToEsp8266("updateVariables(\"" + doorStatus + "\", \"" + doorState + "\", \"" + fanDetail + "\", \"" + coopLightDetail + "\", \"" + runLightDetail + "\", \"" + (airTemp) + "\")", withDelay);
}

void sendCmdToEsp8266(String str) {
  sendCmdToEsp8266(str, true);
}

// Send command to wifi chip. Allow some delay time for it to read each command from serial as a separate string.
void sendCmdToEsp8266(String str, bool withDelay) {
  SPrintln("* RAM: " + String(freeRam()));
  SPrintln("Send to Esp8266:");
  SPrintln(str);
  Serial2.print(str);
  Serial2.write("\r\n");
  if (withDelay) {
    delay(500);
  }
}

// ***************************************** END UPDATE ESP8266 **************************************

// *************************************** SERIAL COMMUNICATIONS *************************************
void serialComms() {
  String cmd = "";
  // Output to serial for debugging (commands from the wifi chip or from PC)
  if (Serial2.available()) {
    cmd = Serial2.readString();
    SPrintln("Wifi: " + cmd);
    //delay(5000);
  }
  else if (Serial.available()) {
    cmd = Serial.readString();
    SPrintln("PC: " + cmd);
    //delay(5000);
  }

  // Process the commands (normally coming from wifi)
  // (*) Update the wifi chip with the latest status
  if (cmd.startsWith("OpenDoor")) {
    enableDoorOpening();
  }
  else if (cmd.startsWith("CloseDoor")) {
    enableDoorClosing();
  }
  else if (cmd.startsWith("ToggleRun")) {
    toggleRunLight();
  }
  else if (cmd.startsWith("ToggleCoop")) {
    toggleCoopLight();
  } 
  else if (cmd.startsWith("RestartWifi")) {
    SPrintln("Restart wifi");
    sendCmdToEsp8266("node.restart()");
  }
  else if (cmd.startsWith("SendCurrentVariablesToWifi")) {
    // The wifi chip sends this command 10 seconds after it is ready. After this point, keep pushing updated variables to the chip at a regular interval
    SPrintln("Update the variables on the wifi");
    updateWifiChipTimer->Start();
    sendVariablesToEsp8266();
  }
  else if (cmd.startsWith("GetStatus")) {
    SPrintln("Position: " + doorState);
  }
  else {
    // Pass this message to the wifi chip             //Original sketch had this, I don't think it's necessary for anything
    //sendCmdToEsp8266(cmd);
  }
}

// ******************************************* END SERIAL COMMUNICATIONS ***********************************

// **************************************** START BUTTON AND SENSOR METHODS **********************************

// Read Photocell Level
void readPhotoCell() {
  photocellReading = analogRead(PHOTOCELL);
  SPrint(F("Photocell Analog Reading = "));
  SPrintln(photocellReading);

  //  Set Photocell Threshholds
  if (photocellReading >= 0 && photocellReading <= 29) {
    photocellReadingLevel = '1';
    SPrint(F("Photocell Reading Level: "));
    SPrintln(F(" Dark"));
  }  else if (photocellReading  >= 30 && photocellReading <= 199) {
    photocellReadingLevel = '2';
    SPrint(F("Photocell Reading Level: "));
    SPrintln(F(" Twilight"));
  }  else if (photocellReading  >= 201 ) {
    photocellReadingLevel = '3';
    SPrint(F("Photocell Reading Level: "));
    SPrintln(F(" Light"));
  }
}

// Handles Any Action Needed on the Light Buttons
void handleLightSwitches() {
  if (isCoopLightToggleButton()) {
    SPrintln(F("Coop Light Button"));
    toggleCoopLight();
    delay(POST_BUTTON_HANDLE_DELAY_MS);
  }
  if (isrunLightToggleButton()) {
    SPrintln(F("Run Light Button"));
    toggleRunLight();
    delay(POST_BUTTON_HANDLE_DELAY_MS);
  }
}

// Handles Any Action On the Door Buttons
void handleDoorSwitches () {
  if (isDoorOpenSwitch()) {
    SPrintln(F("Door Open Button"));
    enableDoorOpening();
  }
  if (isDoorCloseSwitch()) {
    SPrintln(F("Door Close Button"));
    enableDoorClosing();
  }
}

// Updates air temp and checks for probe error.
void updateAirTemp() {
  airTempError = !updateTemp(&airTempOneWire, &airTemp, airTempError);
  if (airTempError || airTemp <= MIN_REASONABLE_AIR_TEMP_F || airTemp >= MAX_REASONABLE_AIR_TEMP_F) {
    airTempError = true;
  }
}

// Updates A Single Temp
boolean updateTemp(OneWire * oneWireTherm, float * fltTemp, boolean previousError) {
  byte thermAddr[8], data[12];
  oneWireTherm->reset_search();
  oneWireTherm->search(thermAddr);

  if (OneWire::crc8(thermAddr, 7) != thermAddr[7]) {     // checksum invalid
    return false;
  }

  if (!oneWireTherm->reset()) {
    return false;
  }

  oneWireTherm->select(thermAddr);
  oneWireTherm->write(0x44, 1);                          // start conversation w/ parasite power

  if (previousError) {
    delay(1000);
  }

  if (!oneWireTherm->reset()) {
    return false;
  }

  oneWireTherm->select(thermAddr);
  oneWireTherm->write(0xBE);                             // read scratchpad
  for (int i = 0; i < 9; i++) {                          // need 9 bytes
    data[i] = oneWireTherm->read();
  }

  *fltTemp = getTemperature(data[0], data[1]);

  return true;
}

// Converts Celsius to Fahrenheit
float convertCeliusToFahrenheit(float c) {
  return ((c * 1.8) + 32);
}

// Converts Fahrenheit to Celsius
float convertFahrenheitToCelius(float f) {
  return ((f - 32) * 0.555555556);
}

float hourMinuteToHour(int hour, int minute) {
  return hour + minute / 60.0f;
}


// Gets  Temp From Bytes for OneWire
float getTemperature(int lowByte, int highByte) {
  int intPostDecimal, boolSign, intHexTempReading, intTempReadingBeforeSplit, preDecimal, i;
  float fltPostDecimal, fltTemp;
  intHexTempReading = (highByte << 8) + lowByte;
  boolSign = intHexTempReading & 0x8000;
  if (boolSign) {
    intHexTempReading = (intHexTempReading ^ 0xffff) + 1;
  }
  intTempReadingBeforeSplit = 6.25 * intHexTempReading;     // multiply by (100 * precision) = (100 * 0.0625) = 6.25 = 12-bit precision
  preDecimal = intTempReadingBeforeSplit / 100;
  intPostDecimal = intTempReadingBeforeSplit % 100;
  fltPostDecimal = intPostDecimal;
  if (intPostDecimal < 10) {
    fltTemp = preDecimal + (fltPostDecimal / 1000);
  }
  else {
    fltTemp = preDecimal + (fltPostDecimal / 100);
  }
  if (boolSign) {
    fltTemp = -fltTemp;
  }
  return convertCeliusToFahrenheit(fltTemp);
}

// ******************************* END BUTTON AND SENSOR METHODS *************************************

// ************************************* START LIGHT METHODS *****************************************

// Coop Light
void toggleCoopLight() {
  if (iscoopLightOn()) {
    coopLightOff();
  } else {
    coopLightOn();
  }
}

boolean iscoopLightOn() {
  return digitalRead(COOP_LIGHT_RELAY_PIN) == LOW;
}

void coopLightOn() {
  digitalWrite(COOP_LIGHT_RELAY_PIN, LOW);
  coopLightDetail = "On";
}

void coopLightOff() {
  digitalWrite(COOP_LIGHT_RELAY_PIN, HIGH);
  coopLightDetail = "Off";
}

boolean isCoopLightToggleButton() {
  return digitalRead(COOP_LIGHT_TOGGLE_BUTTON_PIN) == LOW;
}

// Run Light
void toggleRunLight() {
  if (isrunLightOn()) {
    runLightOff();
  } else {
    runLightOn();
  }
}

boolean isrunLightOn() {
  return digitalRead(RUN_LIGHT_RELAY_PIN) == LOW;
}

void runLightOn() {
  digitalWrite(RUN_LIGHT_RELAY_PIN, LOW);
  runLightDetail = "On";
}

void runLightOff() {
  digitalWrite(RUN_LIGHT_RELAY_PIN, HIGH);
    runLightDetail = "Off";
}

boolean isrunLightToggleButton() {
  return digitalRead(RUN_LIGHT_TOGGLE_BUTTON_PIN) == LOW;
}

/* All Lights On  ** Use if Using Natural Daylight Functions to Supplement Light **
  void allLightsOn(){
  coopLightOn();
  runLightOn();
  }

  // All Lights Off ** Use if Using Natural Daylight Functions to Supplement Light **
  void allLightsOff(){
  coopLightOff();
  runLightOff();
  }
*/

// **************************************** END LIGHT METHODS *************************************

// **************************************** START FAN METHODS *************************************

// Handles any action that needs to be taken on the fan.
void handleFan() {
  if (!airTempError) {
    if (airTemp >= FAN_ON_TEMP_F) {
      fanOn();
      //digitalWrite(HVAC_LED, LOW);                          // Turn On HVAC Led
      fanStatus = digitalRead(COOP_FAN_RELAY_PIN);            //Read and Hold Fan Status for LCD if Used  
    }
    if (airTemp <= FAN_OFF_TEMP_F) {
      fanOff();
      //digitalWrite(HVAC_LED, HIGH);                         // Turn Off HVAC Led
      fanStatus = digitalRead(COOP_FAN_RELAY_PIN);            //Read and Hold Fan Status for LCD if Used 
    }
  }
}

void toggleFan() {
  if (isFanOn()) {
    fanOff();
  } else {
    fanOn();
  }
}

boolean isFanOn() {
  return digitalRead(COOP_FAN_RELAY_PIN) == LOW;
}

void fanOn() {
  digitalWrite(COOP_FAN_RELAY_PIN, LOW);
  digitalWrite(CASE_FAN_RELAY_PIN, LOW);
  digitalWrite(HVAC_LED, LOW);                            // Turn On HVAC Led
  fanDetail = "On";
}

void fanOff() {
  digitalWrite(COOP_FAN_RELAY_PIN, HIGH);
  digitalWrite(CASE_FAN_RELAY_PIN, HIGH);
  digitalWrite(HVAC_LED, HIGH);                           // Turn Off HVAC Led
  fanDetail = "Off";
}
// ***************************************** END FAN METHODS ***************************************

// **************************************** START DOOR METHODS *************************************

// Handles any action that needs to be taken on the door motor.
void handleDoor() {
  int h = hour();                                                       // Get the hours right now and store them in an integer called h
  int m = minute();                                                     // Get the minutes right now and store them in an integer called m
  int s = second();                                                     // Get the seconds right now and store them in an integer called s
  if (isDoorClosing() && isDoorCloseStop()) {                           // If the door close motor is enabled AND the door closed reed switch is ON
    disableDoor();                                                      // Disable the Door Closing Function
    doorError = false;
    digitalWrite(DOOR_OPEN_LED, HIGH);                                  // Turns Off Door Open LED (Common Cathode to 5V)
    digitalWrite(DOOR_CLOSED_LED, LOW);                                 // Turns ON Door Closed LED (Common Cathode to 5V)
    SPrintln(F("Door Closed"));
    doorState = "Closed";
    //delay(2000);
    sendSMS();
  }

  if (isDoorOpening() && isDoorOpenStop()) {                             // If the door close motor is enabled AND the door closed reed switch is ON
    disableDoor();                                                       // Disable the Door Opening Function
    doorError = false;
    digitalWrite(DOOR_CLOSED_LED, HIGH);                                 // Turns Off Door Closed LED (Common Cathode to 5V)
    digitalWrite(DOOR_OPEN_LED, LOW);                                    // Turns ON Door Open LED (Common Cathode to 5V)
    SPrintln(F("Door Open"));
    doorState = "Open";
    //delay(2000);
    sendSMS();
  }

  // If the Door Motor has been running longer than the defined end time
  if ((isDoorClosing() == HIGH || isDoorOpening() == HIGH) && (((h * 60 * 60) + (m * 60) + s) >= (timeEnd))) {      // If current time in seconds is greater than or equal to the calculated end time
    disableDoor();                                                                                                  // Disable Door Function to prevent motor burnout
    doorError = true;                                                                                               // Flag Door Error for LCD
    SPrintln(F("Door Error"));
    doorState = "Error - Attention Needed!";
    //delay(2000);
    sendSMS();
    errorLoop();
  }
}

// Door State
boolean isDoorOpenSwitch() {
  return digitalRead(DOOR_OPEN_SWITCH_PIN) == LOW;
}
boolean isDoorCloseSwitch() {
  return digitalRead(DOOR_CLOSE_SWITCH_PIN) == LOW;
}
boolean isDoorClosing() {
  return digitalRead(DOOR_MOTOR_CLOSE_PIN) == HIGH;
}
boolean isDoorOpening() {
  return digitalRead(DOOR_MOTOR_OPEN_PIN) == HIGH;
}
boolean isDoorInMiddle() {
  return !isDoorOpenStop() && !isDoorCloseStop();
}
boolean isDoorCloseStop() {
  return digitalRead(DOOR_CLOSE_STOP_PIN) == LOW;
}
boolean isDoorOpenStop() {
  return digitalRead(DOOR_OPEN_STOP_PIN) == LOW;
}
// Close The Door
void enableDoorClosing() {
  if (!isDoorCloseStop()) {
    int h = hour();                                          // Get the hours right now and store them in an integer called h
    int m = minute();                                        // Get the minutes right now and store them in an integer called m
    int s = second();                                        // Get the seconds right now and store them in an integer called s
    timeEnd = (h * 60 * 60) + (m * 60) + s + 8;              // To calculate max run time of motor (add 8 seconds)
    doorStatus = "Closing";
    analogWrite(ENABLE_COOP_DOOR_MOTOR, 255);                // Enable Door Motor Full
    digitalWrite(DOOR_MOTOR_OPEN_PIN, LOW);                  // Ensure Open is off
    digitalWrite(DOOR_MOTOR_CLOSE_PIN, HIGH);                // Enable direction Motor CLOSED
  }
}
// Open The Door
void enableDoorOpening() {
  if (!isDoorOpenStop()) {
    int h = hour();                                          // Get the hours right now and store them in an integer called h
    int m = minute();                                        // Get the minutes right now and store them in an integer called m
    int s = second();                                        // Get the seconds right now and store them in an integer called s
    timeEnd = (h * 60 * 60) + (m * 60) + s + 8;              // To calculate max run time of motor (add 8 seconds)
    doorStatus = "Opening";
    analogWrite(ENABLE_COOP_DOOR_MOTOR, 255);                // Enable Door Motor Full
    digitalWrite(DOOR_MOTOR_CLOSE_PIN, LOW);                 // Ensure Close is off
    digitalWrite(DOOR_MOTOR_OPEN_PIN, HIGH);                 // Enable direction Motor OPEN
  }
}
// Disable All Door Functions
void disableDoor() {
  analogWrite(ENABLE_COOP_DOOR_MOTOR, 0);                   // Disable Door Motor
  digitalWrite(DOOR_MOTOR_OPEN_PIN, LOW);                   // Ensure Open is off
  digitalWrite(DOOR_MOTOR_CLOSE_PIN, LOW);                  // Ensure Close is off
  doorStatus = "Idle";
}

// ************************************** END DOOR METHODS *******************************************

// ************************************ LCD DISPLAY FUNCTIONS ****************************************
void doLcdMsg() {
  lcd.setCursor(0, 0);                          // set cursor to column 1,row 1
  lcd.print(F("HEN HUT"));                      // show HEN HUT
  lcd.setCursor(8, 0);                          // set cursor to row 0, column 8
  lcdTime(local, tcr ->abbrev);
  lcd.setCursor(0, 1);                          // set cursor to column 1, row 2
  lcd.print(F("Coop Temp:   "));                // Display "Coop Temp"
  if (!airTempError) {
    lcd.print(airTemp);
    lcd.print(F("F"));                          // show "F"
  } else {
    lcd.println(F("Error  "));                  // Error if Therm Failure
  }
  if (doorError == TRUE) {                      // If There is a Door Error
    lcd.setCursor(0, 2);                        // set cursor to column 1, row 3
    lcd.print(F(" *** COOP DOOR ***  "));       // Display "*** COOP DOOR ***"
    lcd.setCursor(0, 3);                        // set cursor to column 1, row 4
    lcd.print(F(" ***   ERROR   ***  "));       // Display "*** ERROR ***"
  } else {
    lcd.setCursor(0, 2);                        // set cursor to column 1, row 3
    lcd.print(F("Open Door:   "));              // display "Open Door:"
    if ((localHrOpen) < 10) {
      lcd.print(0);                             // Add a zero before if hour is less than 10
    }
    lcd.print(localHrOpen);                     // Display Open Door Hour
    lcd.print(F(":"));
    if ((localMinOpen) < 10) {
      lcd.print(0);                             // Add a zero before if minute is less than 10
    }
    lcd.print(localMinOpen);                    // Display Open Door Minute
    lcd.setCursor(0, 3);                        // set cursor to column 1, row 3
    lcd.print(F("Close Door:  "));              // Display "Close Door:"

    if ((localHrClose) < 10) {
      lcd.print(0);                               // Add a zero before if hour is less than 10
    }
    lcd.print(localHrClose);                    // Display close Door Hour
    lcd.print(F(":"));
    if ((localMinClose) < 10) {
      lcd.print(0);                             // Add a zero before if minute is less than 10
    }
    lcd.print(localMinClose);                   // Display Close Door Minute
  }
}


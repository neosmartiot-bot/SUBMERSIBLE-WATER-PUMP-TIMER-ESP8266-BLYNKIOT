/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************/

/************************************************************
-- a Product of Neo-Smart Coded by Saad 01Apr2026 14:00PM --
************************************************************/

/************************************************************
 * Project       : <Water Solutions with IoT>
 * File Name     : <BoreMotorTimerBlynk01Apr2026.ino>
 * Authorrrrr    : <Saad Ilyas>
 * Organization  : <Neo-Smart>
 *
 * Description   :
 *  <Brief technical description of what this file/module does.
 *   Mention key functionality, interfaces, or hardware used.>
 *
 * Hardware used :
 *   MCU used    : <ESP8266, ESP12E CHIP with a custom made
 *                  circuit developed by Neo-Smart>
 *   Peripherals : <Single Relay Module for Motor Contactor>
 *
 * Version       : v0.1.0
 * Dateeee       : <01-Apr-2026>
 *
 * Revision History:
 *   ----------------------------------------------------------
 *   Version        Date           Author        Description
 *   ----------------------------------------------------------
 *   v0.1.0    <01-Apr-2026>    <Saad ilyas>    Initial release
 *   v0.2.0    <date>              <name>       <Changes made>
 *   v0.3.0    <date>              <name>       <Changes made>
 *   v0.4.0    <date>              <name>       <Changes made>
 *
 * Dependencies :
 *   - <EEPROM.h>
 *   - <TimeLib.h>
 *   - <WidgetRTC.h>
 *   - <BlynkEdgent.h>
 *
 * Important Notes :
 *   - <Important implementation notes>
 *   This firmware is specifically meant for 28BYJ MOTOR
 *   - <Limitations or assumptions>
 *   User must count limitations ULN2003 driver IC, which
 *   contains darlington pairs transistors
 *
 * *********************************************************/

#define BLYNK_TEMPLATE_ID          "TMPLBXX0Jl_1"
#define BLYNK_TEMPLATE_NAME "Submersible Water Pump Timer"
#define BLYNK_FIRMWARE_VERSION        "0.1.0"
 
#define BLYNK_PRINT Serial

#include "BlynkEdgent.h"
#include <WidgetRTC.h>
#include <TimeLib.h>
#include <EEPROM.h>

// #define APP_DEBUG
// #define BLYNK_DEBUG
// #define USE_NODE_MCU_BOARD

// -------------------- Pins & Addresses -------------------- //

// #define DryRunPin    A0

#define DryRunPin    0   // GPIO0 (D3) Dry Run Sensor Pin
#define RelayyPin    3   // GPIO2 (D4) Motor + Green Led Pin
#define RedLEDPin    2   // GPIO14(TX) Red LED Pin for fault

WidgetRTC rtccccc;

// int waterValue = 0;

bool rtcValid = false;
bool RelayStatee = HIGH;               // Current motor state
bool RedLedState = HIGH;               // LOW side switching
bool waterPresent = true;
bool blynkOnline = false;

unsigned long lastEpoch = 0;
unsigned long lastMillisSync = 0;
unsigned long ledPreviousMillis = 0;

// Tune this threshold experimentally
int WATER_THRESHOLD = 400;   // example

String currentTime;
String currentDate;

// ---------------------- Runtime vars ---------------------- //

int OnnUnit = 1;   // default minutes
int OffUnit = 1;   // default minutes

// -------------------- EEPROM addresses -------------------- //

const int addrAutoManual   = 500;    // Auto/Manual mode
const int addrBlynkRelay   = 510;    // Blynk Relay state
const int addrOnnDuration  = 520;    // On duration (minutes)
const int addrOffDuration  = 530;    // Off duration (minutes)
const int addrLastStatee   = 540;    // Off duration (minutes)
const int addrLastStTime   = 550;    // store start time (seconds since epoch)

const int addrOnnUnit = 600;   // 0=sec, 1=min, 2=hour, 3=day, 4=week, 5=month, 6=year
const int addrOffUnit = 610;   // 0=sec, 1=min, 2=hour  3=day, 4=week, 5=month, 6=year

const int addrLastGoodEpoch = 700;
const int addrRTCValidFlag  = 710;

// -------------------- Blynk variables --------------------- //

int AutoManual = 0;                  // 0 = Manual, 1 = Auto
int RelayButton = 0;                  // 0 = OFF, 1 = ON
int OnnDuration = 1;                 // currently Minutes
int OffDuration = 1;                 // currently Minutes

// -------------------- Timing variables -------------------- //

bool manualCycleActive = false;

unsigned long currentMillis  = 0;
unsigned long previousMillis = 0;
unsigned long manualStartEpo = 0;    // when motor turned on

// --------------------- Safety Cap Constants --------------------- //

const unsigned long MAX_DURATION_SEC = 30UL * 24UL * 60UL * 60UL;   // 30 days in seconds

// ---------------- Helper: format epoch to string ---------------- //

String formatTimestamp(unsigned long epoch)
 
 {char buf[25];
  sprintf(buf, "%02d-%02d-%04d %02d:%02d:%02d",
          day(epoch), month(epoch), year(epoch),
          hour(epoch), minute(epoch), second(epoch));
  return String(buf);}

unsigned long toSeconds(unsigned long value, int unit) 
 
 {if (unit == 0) return value;           // seconds
  if (unit == 1) return value * 60UL;     // minutes
  if (unit == 2) return value * 3600UL;    // hoursss
  if (unit == 3) return value * 86400UL;    // dayssss
  if (unit == 4) return value * 604800UL;    // weeksss
  if (unit == 5) return value * 2419200UL;    // monthss
  if (unit == 6) return value * 29030400UL;    // yearsss
  return value;}

// --------------------- Forward declarations --------------------- //

void RequestTime();
void AutoRunMode();
void ManualRunMode();
void RestoreTimeCycle();

///////////////////////////////////////////////////////////////////////
///////////// ---------- Virtual Pins Handlers ---------- /////////////
///////////////////////////////////////////////////////////////////////

BLYNK_CONNECTED()
  
 {blynkOnline = true;
  Serial.println("[BLYNK] Connected");

  rtccccc.begin();
  Blynk.sendInternal("rtc", "sync");

  if (timeStatus() == timeSet)
     {rtcValid = true;
      lastEpoch = now();
      lastMillisSync = millis();

      EEPROM.write(addrRTCValidFlag, 1);
      EEPROM.commit();

      Serial.println("[RTC] Synced from server");
       
      if (abs((long)(now() - getCurrentTime())) > 30)
         {Serial.println("[RTC] Time correction applied");

          lastEpoch = now();
          lastMillisSync = millis();}
      else
         {lastEpoch = now();
          lastMillisSync = millis();}}

  else
      {Serial.println("[RTC] Waiting for sync...");}

  if (rtcValid)
     {unsigned long newEpoch = now();

      if (abs((long)(newEpoch - getCurrentTime())) > 10)
         {Serial.println("[WARNING] Time jump detected");}

      lastEpoch = newEpoch;
      lastMillisSync = millis();}
      
   Blynk.syncVirtual(V4);
   Blynk.syncVirtual(V5);
   Blynk.syncVirtual(V7);     // Onn Duration (default minutes)
   Blynk.syncVirtual(V8);     // Off Duration (default minutes)
   Blynk.syncVirtual(V11);    // Dropdown for OnnDuration unit
   Blynk.syncVirtual(V12);}   // Dropdown for OffDuration unit

BLYNK_DISCONNECTED()
 {blynkOnline = false;
  Serial.println("[BLYNK] Disconnected → Using EEPROM values");}

BLYNK_WRITE(V4)   // Blynk Auto/Manual Button

 {AutoManual = param.asInt();
  EEPROM.put(addrAutoManual, AutoManual);
  EEPROM.commit();
  Serial.print("[APP] Auto/Manual set to: ");
  Serial.println(AutoManual);}
  
BLYNK_WRITE(V5)   // Blynk Relay Control Button

 {RelayButton = param.asInt();
  EEPROM.put(addrBlynkRelay, RelayButton);
  EEPROM.commit();
  Serial.print("[APP] RelayState set to: ");
  Serial.println(RelayButton);}
  
  // Blynk.logEvent("Alert! You turned OFF the Motor Manual");}

BLYNK_WRITE(V7)   // On duration (minutes)

 {int newVal = param.asInt();

  if (newVal != OnnDuration)
     {OnnDuration = newVal;
      if ((unsigned long)OnnDuration * 60UL > MAX_DURATION_SEC)
         {OnnDuration = (MAX_DURATION_SEC / 60UL); // clamp to minutes
          Serial.println("[WARNING] OnNDuration capped to 30 days max");}
      EEPROM.put(addrOnnDuration, OnnDuration);
      EEPROM.commit();
      if (EEPROM.commit())
         {Serial.print("[APP] OnnDuration set to: ");
          Serial.println(OnnDuration);}}}
  
BLYNK_WRITE(V8)   // Off duration (minutes)

 {int newVal = param.asInt();

  if (newVal != OffDuration)
     {OffDuration = newVal;
      if ((unsigned long)OffDuration * 60UL > MAX_DURATION_SEC)
         {OffDuration = (MAX_DURATION_SEC / 60UL);
          Serial.println("[WARNING] OffDuration capped to 30 days max");}
      EEPROM.put(addrOffDuration, OffDuration);
      EEPROM.commit();
      if (EEPROM.commit())
         {Serial.print("[APP] OffDuration set to: ");
          Serial.println(OffDuration);}}}

BLYNK_WRITE(V11)   // Dropdown for OnnDuration unit
 
 {int newVal = param.asInt();

  if (newVal != OnnUnit)
     {OnnUnit = newVal;       // 0=sec,1=min,2=hour
      EEPROM.put(addrOnnUnit, OnnUnit);
      EEPROM.commit();
      if (EEPROM.commit())
         {Serial.print("[BLYNK] OnnDuration unit set to ");
          Serial.println(OnnUnit == 0 ? "Seconds" : OnnUnit == 1 ? "Minutes" : OnnUnit == 2 ? "Hours" : "Days");}}}


BLYNK_WRITE(V12)   // Dropdown for OffDuration unit

 {int newVal = param.asInt();

  if (newVal != OffUnit)
     {OffUnit = newVal;   // 0=sec,1=min,2=hour
      EEPROM.put(addrOffUnit, OffUnit);
      EEPROM.commit();
      if (EEPROM.commit())
         {Serial.print("[BLYNK] OffDuration unit set to ");
          Serial.println(OffUnit == 0 ? "Seconds" : OffUnit == 1 ? "Minutes" : OnnUnit == 2 ? "Hours" : "Days");}}}
  
/*BLYNK_WRITE(V9)

 {BlynkNote = param.asInt();}*/

///////////////////////////////////////////////////////////////////////
/////////////// ---------- Time Sync Handler ---------- ///////////////
///////////////////////////////////////////////////////////////////////

void RequestTime()
 
 {// Blynk.sendInternal("rtccccc", "sync");
 
  currentTime = String(hour()) + ":" + minute() + ":" + second();
  currentDate = String(day()) + " " + month() + " " + year();
  
  Blynk.virtualWrite(V0, currentDate);
  Blynk.virtualWrite(V1, currentTime);}

///////////////////////////////////////////////////////////////////////
/////////////// ---------- Time Sync Handler ---------- ///////////////
///////////////////////////////////////////////////////////////////////

unsigned long getCurrentTime()
 {if (rtcValid)            // Keep tracking drift using millis
     {unsigned long delta = (millis() - lastMillisSync) / 1000;
      return lastEpoch + delta;}
  else
     {return millis() / 1000;}} // fallback: millis-based time

///////////////////////////////////////////////////////////////////////
////////////// ---------- Water Check Function ---------- /////////////
///////////////////////////////////////////////////////////////////////

bool checkWaterPresence()
 {int value = digitalRead(DryRunPin);

  if (value == HIGH)
     {return true;}     // WATER FLOWING
     
  else
     {return false;}}   // DRY CONDITION
 
 /*
 
 {int sum = 0;

  for (int i = 0; i < 10; i++)
      {sum += analogRead(DryRunPin);
       delay(5);}

  waterValue = sum / 10;

  Serial.print("[SENSOR] Water ADC: ");
  Serial.println(waterValue);

  if (waterValue < WATER_THRESHOLD)
     {return false;}   // DRY CONDITION
  
  else
     {return true;}}   // WATER PRESENT */

///////////////////////////////////////////////////////////////////////
/////////////// ---------- Main Setup Handler ---------- //////////////
///////////////////////////////////////////////////////////////////////

void setup()

 {// rtc.begin();
  EEPROM.begin(4096);
  Serial.begin(9600);

  ////////// ---------- Load saved values ---------- //////////
  
  // AutoManual = EEPROM.get(addrAutoManual, AutoManual);
  // RelayButton = EEPROM.get(addrBlynkRelay, RelayButton);
  // OnnDuration = EEPROM.get(addrOnnDuration, OnnDuration);
  // OffDuration = EEPROM.get(addrOffDuration, OffDuration);

  EEPROM.get(addrAutoManual, AutoManual);
  EEPROM.get(addrBlynkRelay, RelayButton);
  EEPROM.get(addrOnnDuration, OnnDuration);
  EEPROM.get(addrOffDuration, OffDuration);

  Serial.println("---------[EEPROM] Loaded values---------");
  Serial.print("  AutoManual: "); Serial.println(AutoManual);
  Serial.print("  RelayState: "); Serial.println(RelayButton);
  Serial.print("  OnnDuration: "); Serial.println(OnnDuration);
  Serial.print("  OffDuration: "); Serial.println(OffDuration);

  // OnnUnit  = EEPROM.get(addrOnnUnit, OnnUnit);
  EEPROM.get(addrOnnUnit, OnnUnit);
  if (OnnUnit > 3) OnnUnit = 1;   // safety default minutes 

  // OffUnit = EEPROM.get(addrOffUnit, OffUnit);
  EEPROM.get(addrOffUnit, OffUnit);
  if (OffUnit > 3) OffUnit = 1;

  ////////// ------------- Setup Pins -------------- //////////

  pinMode(DryRunPin, INPUT);
  
  pinMode(RedLEDPin, OUTPUT);
  pinMode(RelayyPin, OUTPUT);

  digitalWrite(RedLEDPin, RedLedState);
  digitalWrite(RelayyPin, RelayStatee);

  ////////// ---------- Load saved values ---------- //////////
  
  // RestoreTimeCycle();
  RestoreLastGoodTime();

  BlynkEdgent.begin();
  
  RelayStatee = RelayButton;
  digitalWrite(RelayyPin, RelayStatee);
  
  edgentTimer.setInterval(1000L, RequestTime);}

///////////////////////////////////////////////////////////////////////
/////////////// ---------- Main Loop Handler ---------- ///////////////
///////////////////////////////////////////////////////////////////////

void loop()
 
 {BlynkEdgent.run();
  currentMillis = millis();
  waterPresent = checkWaterPresence();

  /////// ---------------- Restore Time Cycle ---------------- ////////

  static bool restoreDone = false;

  if (rtcValid && !restoreDone)
     {RestoreTimeCycle();
      restoreDone = true;}

  ///////// ---------------- Last Good Time ---------------- //////////

  static unsigned long lastSaveMillis = 0;

  if (rtcValid && millis() - lastSaveMillis > 60000)   // every 60 sec
     {lastSaveMillis = millis();
      SaveLastGoodTime();}

  ////////// ---------------- WATER CHECK ---------------- ////////////

  if (!waterPresent)   // 🚨 SAFETY OVERRIDE
     {if (RelayStatee == LOW)
         {Serial.println("[SAFETY] DRY RUN DETECTED → MOTOR OFF");

          RelayButton = 0;
          RelayStatee = HIGH;
          digitalWrite(RelayyPin, HIGH);

          unsigned long ledcurrentMillis = millis();

          if (ledcurrentMillis - ledPreviousMillis >= 500)
             {ledPreviousMillis = ledcurrentMillis;

              RedLedState = !RedLedState;
              digitalWrite(RedLEDPin, RedLedState);}

          Blynk.virtualWrite(V5, 0);
          Blynk.logEvent("[SAFETY] DRY RUN DETECTED → MOTOR OFF");}

      return;}  // ❗ STOP further logic
 
  ////////// ---------------- NORMAL LOGIC ---------------- ///////////

  if (waterPresent)
     {if (AutoManual == 1)
         {AutoRunMode();
          digitalWrite(RedLEDPin, LOW);}
      else
         {ManualRunMode();
          digitalWrite(RedLEDPin, LOW);}}}

///////////////////////////////////////////////////////////////////////
/////////////// ---------- Auto Mode Handler ---------- ///////////////
///////////////////////////////////////////////////////////////////////

void AutoRunMode() 

 {unsigned long nowEpoch = getCurrentTime();
  unsigned long elapsedd = nowEpoch - manualStartEpo;

  // unsigned long onnTime = (unsigned long)OnnDuration * 60UL;
  // unsigned long offTime = (unsigned long)OffDuration * 60UL;

  unsigned long onnTime = toSeconds(OnnDuration, OnnUnit);
  unsigned long offTime = toSeconds(OffDuration, OffUnit);

  if (RelayStatee == LOW && elapsedd >= onnTime) 
     {// Turn OFF
      RelayButton = 0;
      RelayStatee = HIGH;
      digitalWrite(RelayyPin, HIGH);
      manualStartEpo = nowEpoch;     // <-- reset reference time
      SaveCycleState(nowEpoch, 0);

      Blynk.virtualWrite(V5, 0);
      Blynk.virtualWrite(V6, 0);
      Blynk.logEvent("motor_off", "Motor turned OFF (Auto)");
      Serial.println("[AUTO] Motor OFF after OnnDuration");}
      
  else if (RelayStatee == HIGH && elapsedd >= offTime)
  
          {// Turn ON
           RelayButton = 1;
           RelayStatee = LOW;
           digitalWrite(RelayyPin, LOW);
           manualStartEpo = nowEpoch;      // <-- reset reference time
           SaveCycleState(nowEpoch, 1);

           Blynk.virtualWrite(V5, 1);
           Blynk.virtualWrite(V6, 255);
           Blynk.logEvent("motor_on", "Motor turned ON (Auto)");
           Serial.println("[AUTO] Motor ON after OffDuration");}}

///////////////////////////////////////////////////////////////////////
////////////// ---------- Manual Mode Handler ---------- //////////////
///////////////////////////////////////////////////////////////////////

void ManualRunMode()

 {unsigned long nowEpoch = getCurrentTime();
  unsigned long elapsedd = nowEpoch - manualStartEpo;
  // unsigned long targettt = (unsigned long)OnnDuration * 60UL;
  unsigned long targettt = toSeconds(OnnDuration, OnnUnit);   // ✅ now respects dropdown unit

  if (RelayButton == 1 && RelayStatee == HIGH) 
     {// Button pressed ON
      RelayStatee = LOW;
      digitalWrite(RelayyPin, LOW);
      manualStartEpo = nowEpoch;      // <-- reset reference time
      SaveCycleState(nowEpoch, 1);

      Blynk.logEvent("motor_on", "Motor turned ON (Manual)");
      Serial.println("[MANUAL] Motor ON (button press)");}

  if (RelayStatee == LOW && elapsedd >= targettt) 
     {// Auto shut OFF
      RelayButton = 0;
      RelayStatee = HIGH;
      digitalWrite(RelayyPin, HIGH);
      manualStartEpo = nowEpoch;     // <-- reset reference time
      SaveCycleState(nowEpoch, 0);

      Blynk.virtualWrite(V5, 0);
      Blynk.virtualWrite(V6, 0);
      Blynk.logEvent("motor_off", "Motor turned OFF (Manual duration complete)");
      Serial.println("[MANUAL] Motor OFF after OnnDuration");}}

////////////////////////////////////////////////////////////////////
/////////////// ----------- Boot Restore ----------- ///////////////
////////////////////////////////////////////////////////////////////

void RestoreTimeCycle()
  
 {if (!rtcValid)
     {Serial.println("[RESTORE] RTC not ready → skipping restore");
      return;}
  
  unsigned long lastStart = 0;
  int lastState = 0;

  EEPROM.get(addrLastStTime, lastStart);
  EEPROM.get(addrLastStatee, lastState);
  // lastState = EEPROM.get(addrLastStatee, lastState);

  unsigned long nowEpoch = getCurrentTime();   // RTC seconds
  unsigned long elapsedd = nowEpoch  -  lastStart;

  // unsigned long onnTime = (unsigned long)OnnDuration * 60UL;   // sec
  // unsigned long offTime = (unsigned long)OffDuration * 60UL;   // sec

  unsigned long onnTime = toSeconds(OnnDuration, OnnUnit);
  unsigned long offTime = toSeconds(OffDuration, OffUnit);

  if (lastState == 1)
     {if (elapsedd < onnTime) 
         {// Resume ON cycle
          RelayButton = 1;
          RelayStatee = LOW;
          digitalWrite(RelayyPin, LOW);
          Serial.print("[RESTORE] Motor still ON, ");
          Serial.print(onnTime - elapsedd);
          Serial.println(" seconds remaining");}
      else
         {// ON expired, switch OFF and start OFF cycle
          RelayButton = 0;
          RelayStatee = HIGH;
          digitalWrite(RelayyPin, HIGH);
          SaveCycleState(nowEpoch, 0);
          Serial.println("[RESTORE] ON expired, motor OFF (start OFF cycle)");}}
  
  else 
     
     {if (elapsedd < offTime)
         {// Resume OFF cycle
          RelayButton = 0;
          RelayStatee = HIGH;
          digitalWrite(RelayyPin, HIGH);
          Serial.print("[RESTORE] Motor still OFF, ");
          Serial.print(offTime - elapsedd);
          Serial.println(" seconds remaining");}
      else 
         {// OFF expired, switch ON and start ON cycle
          RelayButton = 1;
          RelayStatee = LOW;
          digitalWrite(RelayyPin, LOW);
          SaveCycleState(nowEpoch, 1);
          Serial.println("[RESTORE] OFF expired, motor ON (start ON cycle)");}}}

////////////////////////////////////////////////////////////////////////
//////////////// ---------- Save Cycle State ---------- ////////////////
////////////////////////////////////////////////////////////////////////

void SaveCycleState(unsigned long epoch, int state)
 
 {if (!rtcValid)
      {Serial.println("[WARNING] RTC not valid → skipping save");
       return;}
  
  EEPROM.put(addrLastStTime, epoch);
  EEPROM.put(addrLastStatee, state);
  EEPROM.commit();

  // String ts = formatTimestamp(epoch);

  char buf[25];
  sprintf(buf, "%02d-%02d-%04d %02d:%02d:%02d",
        day(), month(), year(),
        hour(), minute(), second());

  String ts = String(buf);
  
  Serial.print("[STATE] Saved cycle: ");
  Serial.print(state == 1 ? "ON" : "OFF");
  Serial.print(", at ");
  Serial.println(ts);

  if (state == 1)
     {Blynk.virtualWrite(V2, ts);}
  else
     {Blynk.virtualWrite(V3, ts);}}

////////////////////////////////////////////////////////////////////////
/////////////// ---------- Save Last Good Time ---------- //////////////
////////////////////////////////////////////////////////////////////////

void SaveLastGoodTime()
 {if (!rtcValid) return;

  unsigned long currentEpoch = now();

  EEPROM.put(addrLastGoodEpoch, currentEpoch);
  EEPROM.write(addrRTCValidFlag, 1);
  EEPROM.commit();

  Serial.print("[RTC] Saved epoch: ");
  Serial.println(currentEpoch);}

void RestoreLastGoodTime()
 {byte flag = EEPROM.read(addrRTCValidFlag);

  if (flag != 1)
     {Serial.println("[RTC] No valid saved time");
      return;}

  unsigned long savedEpoch;
  EEPROM.get(addrLastGoodEpoch, savedEpoch);

  if (savedEpoch < 100000) // sanity check (~1973)
     {Serial.println("[RTC] Invalid saved epoch");
      return;}

  lastEpoch = savedEpoch;
  lastMillisSync = millis();
  rtcValid = true;

  Serial.print("[RTC] Restored offline epoch: ");
  Serial.println(savedEpoch);}

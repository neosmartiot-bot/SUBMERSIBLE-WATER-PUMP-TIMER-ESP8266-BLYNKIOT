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
 * Version       : v1.0.0
 * Dateeee       : <01-Apr-2026>
 *
 * Revision History:
 *   ----------------------------------------------------------
 *   Version        Date           Author        Description
 *   ----------------------------------------------------------
 *   v1.0.0    <01-Apr-2026>    <Saad ilyas>    Initial release
 *   v1.1.0    <date>              <name>       <Changes made>
 *   v1.2.0    <date>              <name>       <Changes made>
 *   v1.3.0    <date>              <name>       <Changes made>
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
#define BLYNK_TEMPLATE_NAME      "Boring Motor Timer"
#define BLYNK_FIRMWARE_VERSION        "0.1.0"
 
#define BLYNK_PRINT Serial

// #include <Simpletimer.h>
#include "BlynkEdgent.h"
#include <WidgetRTC.h>
#include <TimeLib.h>
#include <EEPROM.h>
// #include <time.h>

// #define APP_DEBUG
// #define BLYNK_DEBUG
// #define USE_NODE_MCU_BOARD

// -------------------- Pins & Addresses -------------------- //

// #define DryRunPin    A0

#define DryRunPin    0   // GPIO0 (D3) Dry Run Sensor Pin
#define RelayyPin    2   // GPIO2 (D4) Motor + Green Led Pin
#define RedLEDPin    1   // GPIO14(TX) Red LED Pin for fault

WidgetRTC rtccccc;

// int waterValue = 0;

bool RelayStatee = LOW;               // Current motor state
bool RedLedState = LOW;               // LOW side switching
bool waterPresent = true;

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
  
  {rtccccc.begin();
   Blynk.syncVirtual(V4);
   Blynk.syncVirtual(V5);
   Blynk.syncVirtual(V7);    // Onn Duration (default minutes)
   Blynk.syncVirtual(V8);    // Off Duration (default minutes)
   Blynk.syncVirtual(V11);   // Dropdown for OnnDuration unit
   Blynk.syncVirtual(V12);   // Dropdown for OffDuration unit
   }

BLYNK_WRITE(V4)   // Blynk Auto/Manual Button

 {AutoManual = param.asInt();
  EEPROM.write(addrAutoManual, AutoManual);
  EEPROM.commit();
  Serial.print("[APP] Auto/Manual set to: ");
  Serial.println(AutoManual);}
  
BLYNK_WRITE(V5)   // Blynk Relay Control Button

 {RelayButton = param.asInt();
  EEPROM.write(addrBlynkRelay, RelayButton);
  EEPROM.commit();
  Serial.print("[APP] RelayState set to: ");
  Serial.println(RelayButton);}
  
  // Blynk.logEvent("Alert! You turned OFF the Motor Manual");}

BLYNK_WRITE(V7)   // On duration (minutes)

 {OnnDuration = param.asInt();
  if ((unsigned long)OnnDuration * 60UL > MAX_DURATION_SEC)
     {OnnDuration = (MAX_DURATION_SEC / 60UL); // clamp to minutes
      Serial.println("[WARNING] OnNDuration capped to 30 days max");}
  EEPROM.write(addrOnnDuration, OnnDuration);
  EEPROM.commit();
  Serial.print("[APP] OnnDuration set to: ");
  Serial.println(OnnDuration);}
  
BLYNK_WRITE(V8)   // Off duration (minutes)

 {OffDuration = param.asInt();
  if ((unsigned long)OffDuration * 60UL > MAX_DURATION_SEC)
     {OffDuration = (MAX_DURATION_SEC / 60UL);
      Serial.println("[WARNING] OffDuration capped to 30 days max");}
  EEPROM.write(addrOffDuration, OffDuration);
  EEPROM.commit();
  Serial.print("[APP] OffDuration set to: ");
  Serial.println(OffDuration);}

BLYNK_WRITE(V11)   // Dropdown for OnnDuration unit
 
 {OnnUnit = param.asInt();   // 0=sec,1=min,2=hour
  EEPROM.write(addrOnnUnit, OnnUnit);
  EEPROM.commit();
  Serial.print("[BLYNK] OnnDuration unit set to ");
  Serial.println(OnnUnit == 0 ? "Seconds" : OnnUnit == 1 ? "Minutes" : OnnUnit == 2 ? "Hours" : "Days");}


BLYNK_WRITE(V12)   // Dropdown for OffDuration unit

 {OffUnit = param.asInt();   // 0=sec,1=min,2=hour
  EEPROM.write(addrOffUnit, OffUnit);
  EEPROM.commit();
  Serial.print("[BLYNK] OffDuration unit set to ");
  Serial.println(OffUnit == 0 ? "Seconds" : OffUnit == 1 ? "Minutes" : OnnUnit == 2 ? "Hours" : "Days");}
  
/*BLYNK_WRITE(V9)

 {BlynkNote = param.asInt();}*/

///////////////////////////////////////////////////////////////////////
/////////////// ---------- Tiime Sync Handler ---------- //////////////
///////////////////////////////////////////////////////////////////////

void RequestTime()
 
 {// Blynk.sendInternal("rtccccc", "sync");
 
  currentTime = String(hour()) + ":" + minute() + ":" + second();
  currentDate = String(day()) + " " + month() + " " + year();
  
  Blynk.virtualWrite(V0, currentDate);
  Blynk.virtualWrite(V1, currentTime);}

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
  
  BlynkEdgent.begin();
  
  RestoreTimeCycle();

  pinMode(DryRunPin, INPUT);
  
  pinMode(RedLEDPin, OUTPUT);
  pinMode(RelayyPin, OUTPUT);

  digitalWrite(RedLEDPin, RedLedState);
  digitalWrite(RelayyPin, RelayStatee);

  // -------- Load saved values -------- //
  
  AutoManual = EEPROM.read(addrAutoManual);
  RelayButton = EEPROM.read(addrBlynkRelay);
  OnnDuration = EEPROM.read(addrOnnDuration);
  OffDuration = EEPROM.read(addrOffDuration);

  Serial.println("---------[EEPROM] Loaded values---------");
  Serial.print("  AutoManual: "); Serial.println(AutoManual);
  Serial.print("  RelayState: "); Serial.println(RelayButton);
  Serial.print("  OnnDuration: "); Serial.println(OnnDuration);
  Serial.print("  OffDuration: "); Serial.println(OffDuration);

  OnnUnit  = EEPROM.read(addrOnnUnit);
  if (OnnUnit > 3) OnnUnit = 1;   // safety default minutes 

  OffUnit = EEPROM.read(addrOffUnit);
  if (OffUnit > 3) OffUnit = 1;

  if (RelayButton)
     {digitalWrite(RelayyPin, HIGH);
      RelayStatee = HIGH;}

  edgentTimer.setInterval(1000L, RequestTime);}

///////////////////////////////////////////////////////////////////////
/////////////// ---------- Main Loop Handler ---------- ///////////////
///////////////////////////////////////////////////////////////////////

void loop()
 
 {BlynkEdgent.run();
  currentMillis = millis();
  waterPresent = checkWaterPresence();

  if (!waterPresent)   // 🚨 SAFETY OVERRIDE
     {if (RelayStatee == HIGH)
         {Serial.println("[SAFETY] DRY RUN DETECTED → MOTOR OFF");

          RelayButton = 0;
          RelayStatee = LOW;
          digitalWrite(RelayyPin, LOW);

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

 {unsigned long nowEpoch = now();
  unsigned long elapsedd = nowEpoch - manualStartEpo;

  // unsigned long onnTime = (unsigned long)OnnDuration * 60UL;
  // unsigned long offTime = (unsigned long)OffDuration * 60UL;

  unsigned long onnTime = toSeconds(OnnDuration, OnnUnit);
  unsigned long offTime = toSeconds(OffDuration, OffUnit);

  if (RelayStatee == HIGH && elapsedd >= onnTime) 
     {// Turn OFF
      RelayButton = 0;
      RelayStatee = LOW;
      digitalWrite(RelayyPin, LOW);
      manualStartEpo = nowEpoch;     // <-- reset reference time
      SaveCycleState(nowEpoch, 0);

      Blynk.virtualWrite(V5, 0);
      Blynk.virtualWrite(V6, 0);
      Blynk.logEvent("motor_off", "Motor turned OFF (Auto)");
      Serial.println("[AUTO] Motor OFF after OnnDuration");}
      
  else if (RelayStatee == LOW && elapsedd >= offTime)
  
          {// Turn ON
           RelayButton = 1;
           RelayStatee = HIGH;
           digitalWrite(RelayyPin, HIGH);
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

 {unsigned long nowEpoch = now();
  unsigned long elapsedd = nowEpoch - manualStartEpo;
  // unsigned long targettt = (unsigned long)OnnDuration * 60UL;
  unsigned long targettt = toSeconds(OnnDuration, OnnUnit);   // ✅ now respects dropdown unit

  if (RelayButton == 1 && RelayStatee == LOW) 
     {// Button pressed ON
      RelayStatee = HIGH;
      digitalWrite(RelayyPin, HIGH);
      manualStartEpo = nowEpoch;      // <-- reset reference time
      SaveCycleState(nowEpoch, 1);

      Blynk.logEvent("motor_on", "Motor turned ON (Manual)");
      Serial.println("[MANUAL] Motor ON (button press)");}

  if (RelayStatee == HIGH && elapsedd >= targettt) 
     {// Auto shut OFF
      RelayButton = 0;
      RelayStatee = LOW;
      digitalWrite(RelayyPin, LOW);
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
  
 {unsigned long lastStart = 0;
  int lastState = 0;

  EEPROM.get(addrLastStTime, lastStart);
  lastState = EEPROM.read(addrLastStatee);

  unsigned long nowEpoch = now();   // RTC seconds
  unsigned long elapsedd = nowEpoch  -  lastStart;

  // unsigned long onnTime = (unsigned long)OnnDuration * 60UL;   // sec
  // unsigned long offTime = (unsigned long)OffDuration * 60UL;   // sec

  unsigned long onnTime = toSeconds(OnnDuration, OnnUnit);
  unsigned long offTime = toSeconds(OffDuration, OffUnit);

  if (lastState == 1)
     {if (elapsedd < onnTime) 
         {// Resume ON cycle
          RelayButton = 1;
          RelayStatee = HIGH;
          digitalWrite(RelayyPin, HIGH);
          Serial.print("[RESTORE] Motor still ON, ");
          Serial.print(onnTime - elapsedd);
          Serial.println(" seconds remaining");}
      else
         {// ON expired, switch OFF and start OFF cycle
          RelayButton = 0;
          RelayStatee = LOW;
          digitalWrite(RelayyPin, LOW);
          SaveCycleState(nowEpoch, 0);
          Serial.println("[RESTORE] ON expired, motor OFF (start OFF cycle)");}}
  
  else 
     
     {if (elapsedd < offTime)
         {// Resume OFF cycle
          RelayButton = 0;
          RelayStatee = LOW;
          digitalWrite(RelayyPin, LOW);
          Serial.print("[RESTORE] Motor still OFF, ");
          Serial.print(offTime - elapsedd);
          Serial.println(" seconds remaining");}
      else 
         {// OFF expired, switch ON and start ON cycle
          RelayButton = 1;
          RelayStatee = HIGH;
          digitalWrite(RelayyPin, HIGH);
          SaveCycleState(nowEpoch, 1);
          Serial.println("[RESTORE] OFF expired, motor ON (start ON cycle)");}}}

////////////////////////////////////////////////////////////////////////
//////////////// ---------- Save Cycly State ---------- ////////////////
////////////////////////////////////////////////////////////////////////

void SaveCycleState(unsigned long epoch, int state)
 
 {EEPROM.put(addrLastStTime, epoch);
  EEPROM.write(addrLastStatee, state);
  EEPROM.commit();

  String ts = formatTimestamp(epoch);
  Serial.print("[STATE] Saved cycle: ");
  Serial.print(state == 1 ? "ON" : "OFF");
  Serial.print(", at ");
  Serial.println(ts);

  if (state == 1)
     {Blynk.virtualWrite(V2, ts);}
  else
     {Blynk.virtualWrite(V3, ts);}}

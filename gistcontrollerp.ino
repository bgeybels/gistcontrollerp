/*
 * TODO: starten zonder internet !!!
 * 
 * GIP Jobbe Geybels 2020-2021
 * Gistcontroller v6.x op een NODEMCU-board (ESP8266) 
 *    P-Controle: Simple Proportionele Controle
 * 
 * WifiSetup      via WiFiManager (IPscherm+select=reconnect)
 * I2C-ADXL345    Gyroscoop
 * I2C-LCD        LCD
 * RotaryEncoder  Druk- en draai-knop
 * DS18B20        Tempsensor: Wort/Frigo
 *    Wort  Sensor 2 : 0x28, 0x76, 0xCD, 0xF0, 0x3A, 0x19, 0x01, 0x3F
 *    Frigo Sensor 1 : 0x28, 0xAA, 0xD0, 0x6D, 0x59, 0x14, 0x01, 0xA2
 * Relays         Cooling - Heating
 * ESP8266        Email naar gist.controller@gmail.com
 * EEPROM         Bewaar settings (herstart): aanpassen via EEPROM_VER of menu
 * 
 */
 
#include <LiquidCrystal_I2C.h>             // Library voor LCD I2C
#include <ErriezRotaryFullStep.h>          // Encoder library
#include <Adafruit_ADXL345_U.h>            // Library ADXL345
#include <Adafruit_Sensor.h>               // Library ADXL345
#include <DallasTemperature.h>             // Libraryvoor DS18B20
#include <OneWire.h>                       // Library voor DS18B20
#include <ESP8266WiFi.h>                   // Library voor wifi
#include <WiFiManager.h>                   // Library voor WifiManager
#include <EEPROM.h>                        // Library EEPROM: bewaar parameters
#include <ESPDateTime.h>                   // Datum en tijd
#include "Gsender.h"                       // GMail settings
#include "parameters.h"                    // Parameters

// Initialiseer librarys 
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);
RotaryFullStep rotary(RO_LEFT, RO_RIGHT, true, 50);
Adafruit_ADXL345_Unified tilter = Adafruit_ADXL345_Unified(12345);
OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);
WiFiManager wifiManager;

void setup(void) { 
  // Initialseer LCD,Serieel en Serieel-message
  lcd_serial_msg_Init();

  // Initialiseer componenten
  initSuccess = initComponents();
  if (!initSuccess.isEmpty()) {
    while(1) {
      // endless loop = initialisatie was niet goed
      lcdShowInit("!! Probleem Init",2,0);
      lcdShowInit(initSuccess,3,0);
    }
  }
  
  // Initialiseer wortTemp/max-minWortTemp
  getTemperature();
  maxWortTemp = minWortTemp = wortTemp;
  if ( send_msg ) {sendInitMessage();}
}

void loop(void) {
  updateTemperature();                      // Haal de huidige temperatuur op
  controlState();                           // Update status: koel,inactief,verwarm
  updateTilted();                           // Controleer tiltsensor... elke sec=oké

  controlDisplayState();
  displayState();                           // update de LCDdisplay

  if (send_msg) {sendMessage();}            // Zend bericht als tijd verstreken 
}

/**
 * Haal de huidige temperatuur op en update MAX- en MIN-Temp
 */
void updateTemperature() {
  /*
  * targetTempF  targetTempW   Kp  wortTemp
  * 28,5         21,00         5   19,50
  * 26           21,00         5   20,00
  * 23,5         21,00         5   20,50
  * 21           21,00         5   21,00
  * 18,5         21,00         5   21,50
  * 16           21,00         5   22,00
  * 13,5         21,00         5   22,50
  * 11           21,00         5   23,00
  * 8,5          21,00         5   23,50
  * -4           21,00         5   26,00
  * -24          21,00         5   30,00
  */
  getTemperature();
  // automodus terug aanzetten als worttemp bereikt!
  if ( forced && wortTemp == targetTempW ) {
    digitalWrite(HEATING_PIN, LOW);
    digitalWrite(COOLING_PIN, LOW);
    forced=false;
  }
  
  float minvalue = targetTempW - TEMP_DANGER_C;
  float maxvalue = targetTempW + TEMP_DANGER_H;
  targetTempF=constrain(targetTempW-Kp*(wortTemp-targetTempW),minvalue,maxvalue);
 
  if ( debug_msgcsv && targetTempF != last_targetTempF ) {
    last_targetTempF = targetTempF;
    serialMsgCsv("targetTempF");
  }
  
  // Alert-message als er iets grondig fout loopt
  if ( send_msg ) {
    if ( wortTemp > (targetTempW + TEMP_DANGER_H) ) {sendAlertMessage();}
    if ( wortTemp < (targetTempW - TEMP_DANGER_C) ) {sendAlertMessage();}  
  }
  
  if (wortTemp > maxWortTemp) {maxWortTemp = wortTemp;}
  else if (wortTemp < minWortTemp) {minWortTemp = wortTemp;}
}

/**
 * Returns de huidige temperatuur van het wort
 */
void getTemperature() {
  sensors.requestTemperatures();            // Opvragen temperatuur
  wortTemp = sensors.getTempC(sensor1);
  frigoTemp = sensors.getTempC(sensor2);
}

/**
 * Wijzig status adhv temperatuur + bewaar de timers 
 * (HEATING) <=> (INACTIVE) <=> (COOLING)
 * debug_msgcsv=true --> serial print csv-formaat bij elke statusupdate
 */
void controlState() {
  currentMillis = millis();
  millisInControllerState = currentMillis - millisStateStart;

  if (debug) {
    if (currentMillis-logMillis > 60000) {
      logMillis = currentMillis;
      Serial.print(DateFormatter::format("%H:%M", DateTime.now()));
      Serial.print(F(";"));
      Serial.print(wortTemp);
      Serial.print(F(";"));
      Serial.print(frigoTemp);
      Serial.print(F(";"));
      Serial.print(targetTempF);
      switch (currentControllerState) {
        case STATE_COOLING:
          Serial.println(F(";COOL"));
          break;
        case STATE_INACTIVE:
          Serial.println(F(";INACTIVE"));
          break;
        case STATE_HEATING:
          Serial.println(F(";HEAT"));
          break;
      }
    }
  }

  // Momenteel niet in automatische status
  if (forced) {
      return;
  }
 
  switch ( currentControllerState ) {
    // Momenteel aan het KOELEN
    case STATE_COOLING:
      runTime = (unsigned long)(millis() - millisStateStart) / 1000;  // runtime in seconds
      // ensure minimum compressor runtime
      if (runTime < coolMinOn) break;
      if (frigoTemp < targetTempF) {
        if (debug_msgcsv) {serialMsgCsv("StartInactief");}
        //stop KOELEN + zet INACTIEF
        currentControllerState = STATE_INACTIVE;
        if (millisInControllerState > maxTimeCooling) {
          maxTimeCooling = millisInControllerState;
        }
        millisStateStart = currentMillis;
        digitalWrite(COOLING_PIN, LOW);
        if ( send_msg ) {sendStateMessage();}
      } else {
        // Alert-message als er iets grondig fout loopt
        if ( send_msg && millisInControllerState > STATE_DANGER ) {sendAlertMessage();}
      }
      break;
    // Momenteel INACTIEF = niks aan het doen
    case STATE_INACTIVE:
      if ( frigoTemp < ( targetTempF - Deadband ) ) {
        if (debug_msgcsv) {serialMsgCsv("StartWarmen");}
        //start VERWARMEN
        currentControllerState = STATE_HEATING;
        millisStateStart = currentMillis;
        countStatHeat++;
        countStatHeatTotal++;
        digitalWrite(HEATING_PIN, HIGH);
        if ( send_msg ) {sendStateMessage();}
      }
      else if (( frigoTemp > ( targetTempF + Deadband ) ) 
           && ((unsigned long)((millis() - stopTime) / 1000) > coolMinOff)) {
        if (debug_msgcsv) {serialMsgCsv("StartKoelen");}
        //start KOELEN
        currentControllerState = STATE_COOLING;
        millisStateStart = currentMillis;
        countStatCool++;
        countStatCoolTotal++;
        digitalWrite(COOLING_PIN, HIGH);
        if ( send_msg ) {sendStateMessage();}
      }
      break;
    // Momenteel aan het VERWARMEN
    case STATE_HEATING:
      { runTime = millis() - startTime;  // runtime in ms    
      if ( frigoTemp > targetTempF ) {
        if (debug_msgcsv) {serialMsgCsv("StartInactief");}
        //stop VERWARMEN    
        currentControllerState = STATE_INACTIVE;
        if (millisInControllerState > maxTimeHeating) {
          maxTimeHeating = millisInControllerState;
        }
        millisStateStart = currentMillis;
        digitalWrite(HEATING_PIN, LOW);
        if ( send_msg ) {sendStateMessage();}
      } else {
        // Alert-message als er iets grondig fout loopt
        if ( send_msg && millisInControllerState > 900000 ) {sendAlertMessage();}
      }
      break;
      }
  }
}

/*
 * Tiltsensor: tilt = 0 --> is beneden = 1 --> is naar boven
 */
void updateTilted() {
  currentMillis = millis();
  getTilt();
  if (currentTilt != lastTilt) {
    // enkel de neutrale stand (=beneden) telt
    if (currentTilt == 0) {
      millisBetweenTilts = currentMillis - millisLastTilt;
      millisLastTilt = currentMillis;
      ++countTilts;           // tel 1 bij het aantal tiltwijzigingen binnen 1 bericht
      ++countTiltsTotal;      // tel 1 bij totaal aantal tilts
      EEPROMWriteSettings();  // bewaar in EEPROM
    }
      lastTilt = currentTilt;
    }
    if (debug_tilt) {serialTilted();}
}

void getTilt() {
  sensors_event_t event; 
  tilter.getEvent(&event);
  gyro_X = (event.acceleration.x)/9.8;  
  gyro_Y = (event.acceleration.y)/9.8;
  // Negatieve meting omzetten naar positieve waarde
  if (gyro_X < 0) { gyro_X = gyro_X * -1; }
  if (gyro_Y < 0) { gyro_Y = gyro_Y * -1; }

  currentTilt = 0;
  // Vergelijk de x-y-meting met tiltThreshold
  // om te vermijden dat bij naar beneden vallen er door 
  // de terugslag een nieuwe tilt geregistreerd wordt
  if (gyro_X > tiltThreshold || gyro_Y > tiltThreshold) {
    currentTilt = 1;
  } 
}

/*
 * Stuur het bericht met tilt- en statusinfo als tijd om is 
 */
void sendMessage() {
  currentMillis = millis();
  if ( millisMessage > 0 && currentMillis - millisElapsedMessages > millisMessage) {
    if (WiFiConnected()) {
      millisElapsedMessages = currentMillis;
      String subject      = "Gist Controller " + versie + " Doel:" + targetTempW + "°C";
      String message      = "";
      message = fillMessage();
      mailSend = sendMail(subject,message);
      if ( mailSend ) {
             countTilts = 0;
             countStatHeat = 0;
             countStatCool = 0;
      }
    }
  }
}
/*
 * Stuur initieel bericht
 */
void sendInitMessage() {
  if (WiFiConnected()) {
    lcdShowInit("Sending InitMsg",2,0); 
    
    String subject      = "Init Gist Controller " + versie + " Doel:" + targetTempW + "°C";
    String message      = "";
    message = fillAlertMessage();
    mailSend = sendMail(subject,message);
    }
}
/*
 * Stuur bericht bij statuswissel
 */
void sendStateMessage() {
  if (WiFiConnected()) {
    String subject      = "Status Gist Controller " + versie + " Doel:" + targetTempW + "°C";
    String message      = "";
    message = fillStateMessage();
    mailSend = sendMail(subject,message);
    }
}
/*
 * Stuur een Alertmessage
 */
void sendAlertMessage() {
  --alertMaxTimer;
  if ( alertMaxTimer < 1) {
    if (WiFiConnected()) {
      alertMaxTimer=alertCountDown;
      String subject      = "ALERT Gist Controller " + versie + " Doel:" + targetTempW + "°C";
      String message      = "";
      message = fillAlertMessage();
      mailSend = sendMail(subject,message);
      }
  }
}

ICACHE_RAM_ATTR void potTurned() {
  // reset de tijd in deze LCD-status
  // anders bestaat de kans dat je naar hoofdscherm springt tijdens duwen
  millisLCDStart = millis();
  backlightStart = millis();
  whichButtonPressed=BUTTON_NONE;
  if (!isBacklightActive) {
    return;
  }
  
  int bturn;
  // Read rotary state (Counter clockwise) -2, -1, 0, 1, 2 (Clockwise)
  bturn = rotary.read();

  // Count up or down by using rotary speed
  if (bturn == 0) {
      return;
  } else if (abs(bturn) >= 2) {
      if (buttonPushed) {
        whichButtonPressed=BUTTON_UP;
        handleUpDown( whichButtonPressed, millis() );
      } else {  
        currentLCDState += bturn * 2;
      }
  } else {
      if (buttonPushed) {
        whichButtonPressed=BUTTON_DOWN;
        handleUpDown( whichButtonPressed, millis() );
      } else {  
        currentLCDState += bturn;
      }
  }
  // test of je niet te ver gaat tov NO_OF_LCD_STATES (momenteel 0 tem 8)
  currentLCDState += NO_OF_LCD_STATES;
  currentLCDState %= NO_OF_LCD_STATES;
}

ICACHE_RAM_ATTR void potPushed() {
  // reset de tijd in deze LCD-status
  // anders bestaat de kans dat je naar hoofdscherm springt tijdens duwen
  millisLCDStart = millis();
  backlightStart = millis();
  whichButtonPressed=BUTTON_NONE;
  int bpush = digitalRead(RO_PUSH);
  
  // Poll Rotary button pin
  if (bpush == 0) {
      if ( !isBacklightActive ) {
        buttonPushed = false;
        enableBacklight(millis());
      }
      else if ( ( currentLCDState == DISPLAY_IP ) && ( send_msg ) ) {
                  wifiSuccess=WiFiConnect();
      }
      else if ( ( currentLCDState == DISPLAY_RESET_EEPROM) ) {
                  EEPROMWritePresets();
                  EEPROMReadSettings();
                  if ( send_msg ) {sendInitMessage();}
                  currentLCDState = DISPLAY_SUMMARY;
                  displayState();
      }
      else if ( ( currentLCDState == DISPLAY_FORCE_ALLOUT) ) {
                  // terug naar automatisch COOL/HEAT
                  digitalWrite(HEATING_PIN, LOW);
                  digitalWrite(COOLING_PIN, LOW);    
                  currentControllerState = STATE_INACTIVE;
                  millisStateStart = millis();
                  forced=false;
                  currentLCDState = DISPLAY_SUMMARY;
                  displayState();
      } 
      else if ( ( currentLCDState == DISPLAY_FORCE_HEAT) ) {
                  // HEAT aan
                  digitalWrite(COOLING_PIN, LOW);
                  digitalWrite(HEATING_PIN, HIGH);
                  currentControllerState = STATE_HEATING;
                  millisStateStart = millis();
                  currentLCDState = DISPLAY_SUMMARY;
                  displayState();
                  forced=true;
      } 
      else if ( ( currentLCDState == DISPLAY_FORCE_COOL) ) {
                  // COOL aan
                  digitalWrite(HEATING_PIN, LOW);
                  digitalWrite(COOLING_PIN, HIGH);
                  currentControllerState = STATE_COOLING;
                  millisStateStart = millis();
                  currentLCDState = DISPLAY_SUMMARY;
                  displayState();
                  forced=true;
      } 
      else if ( ( currentLCDState == DISPLAY_RESET_NODEMCU) ) {
                  ESP.restart();
      } 
      else if ( ( currentLCDState == DISPLAY_SET_MSG_TIME ) ||  
                ( currentLCDState == DISPLAY_SET_TARGET ) ) {
                  if ( buttonPushed ) {
                    buttonPushed = false;
                    EEPROMWriteSettings();
                  } else {
                    buttonPushed = true;
                 }
      }
  }
}

/**
 * Gebruik knoppen, LCD-displaymodus aanpassen en LCD verversen
 */
void controlDisplayState() {
  currentMillis = millis();
  millisInLCDState = currentMillis - millisLCDStart;
  
  // Zet LCD uit na verstrijken timeout (REDIRECT_TIMEOUT * 5)
  checkBacklightTimeout(currentMillis);
  
  // Geen knop gedrukt EN momenteel niet in SUMMARY-status 
  if ( !whichButtonPressed && currentLCDState != DISPLAY_SUMMARY) {
    // Indien je te lang in huidige LCD-status zit = Terug naar SUMMARY-status
    if ( millisInLCDState >= REDIRECT_TIMEOUT ) {
      millisLCDStart = currentMillis;
      backlightStart = currentMillis;
      currentLCDState = DISPLAY_SUMMARY;
      displayState();
    }
    return;
  }  
}

/**
 * Zet LCD-backlight uit als timeout verstreken (REDIRECT_TIMEOUT * 5)
 */
void checkBacklightTimeout(int mtime) {
  backlightTimeout = currentMillis - backlightStart;
  if (   isBacklightActive 
      && (backlightTimeout) > (REDIRECT_TIMEOUT * 5) ) {
    disableBacklight(mtime);
  }
}
/**
 * disable LCDscherm
 */
void disableBacklight(int mtime) {
  currentLCDState = DISPLAY_SUMMARY;
  lcd.noBacklight();
  lcd.noDisplay();
  isBacklightActive = false;
}
/**
 * enable LCDscherm
 */
void enableBacklight(int mtime) {
  lcd.display();
  lcd.backlight();
  isBacklightActive = true;
  backlightStart = mtime;
}

/**
 * Knop = UP/DOWN = Wijzig parameters, afhankelijk van de huidige LCD-status
 */
void handleUpDown( int whichButtonPressed, int mtime ) {
   // reset de tijd in deze LCD-status
   // anders bestaat de kans dat je naar hoofdscherm springt tijdens duwen
   millisLCDStart = mtime;

   switch ( currentLCDState ) {
    case DISPLAY_SET_MSG_TIME:
      // Verhoog/verlaag de tijd tussen boodschappen
      if ( whichButtonPressed == BUTTON_UP ) {
        //BETWEEN_MSG_INCR = in minuten = 1000 * 60 * BETWEEN_MSG_INCR
        millisMessage += BETWEEN_MSG_INCR*60000; 
      }
      else if ( whichButtonPressed == BUTTON_DOWN ) {
        millisMessage -= BETWEEN_MSG_INCR*60000; 
        if ( millisMessage < 0 ) {
          millisMessage = 0;
        }
      }
      break;  
    case DISPLAY_SET_TARGET:
      // Verhoog/verlaag de gewenste temperatuur
      if ( whichButtonPressed == BUTTON_UP ) {
        // mag niet hoger gezet worden dan MAXIMUM_TARGET
        if ( targetTempW < MAXIMUM_TARGET ) {
          targetTempW += TEMP_INCR; 
        }
      }
      else if ( whichButtonPressed == BUTTON_DOWN ) {
        // mag niet lager gezet worden dan MINIMUM_TARGET
        if ( targetTempW > MINIMUM_TARGET ) {
        targetTempW -= TEMP_INCR; 
        }
      }
      break;
  }
}
  
String initComponents() {
  // initialiseer pins

  // Enable internal pull-up for the rotary button pin
  pinMode(RO_PUSH, INPUT_PULLUP);
  // Initialize pin change interrupt on both rotary encoder pins
  attachInterrupt(digitalPinToInterrupt(RO_LEFT), potTurned, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RO_RIGHT), potTurned, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RO_PUSH), potPushed, CHANGE);
  
  pinMode( COOLING_PIN, OUTPUT);           // output naar led/220V
  pinMode( HEATING_PIN, OUTPUT );          // output naar led/220V
  digitalWrite( COOLING_PIN, LOW );        // initieel inactief zetten
  digitalWrite( HEATING_PIN, LOW );        // initieel inactief zetten
  sensors.begin();                         // Start the DS18B20 sensor

  lcd.setCursor(0,1);
  lcd.print(F("Initialisatie:"));

  // Initialiseer wifi als send_msg = true
  if ( send_msg ) {
    wifiSuccess=WiFiConnect();
    if(!wifiSuccess) {return "Wifi";}
  } 

  // test DS18B20
  lcdShowInit("DS18B20 - Wort",2,0);
  getTemperature();
  if (wortTemp == -127) {
     lcdShowInit("Error",3,0);
     return "DS18B20 Wort";
  } else {lcdShowInit("gelukt",3,0);}
  lcdShowInit("DS18B20 - Frigo",2,0);
  if (frigoTemp == -127) {
     lcdShowInit("Error",3,0);
     return "DS18B20 Frigo";
  } else {lcdShowInit("gelukt",3,0);}
  // initialiseer gyro...
  lcdShowInit("ADXL345",2,0); 
  if(!tilter.begin()) {
    lcdShowInit("Error",3,0);
    return "ADXL345";
  } else {lcdShowInit("gelukt",3,0);}
  tilter.setRange(ADXL345_RANGE_4_G);        // initialiseer ADXL345
  
  // Ophalen DateTime
  lcdShowInit("DateTime",2,0);
  if (!datetimeSuccess) {datetimeSuccess = setupDateTime(10);}
  if (!datetimeSuccess) {
    lcdShowInit("Error",3,0);
    return "DateTime";
  } else {
    lcdShowInit("gelukt",3,0);
  }

  // Bewaar het startpunt
  startDateInt = DateTime.now();
  backlightStart = millis();
  millisElapsedMessages = millis();

  //EEPROM bewaren/ophalen waardes
  byte ver;
  lcdShowInit("EEPROM",2,0);
  EEPROM.begin(512);
  delay(1500);
  EEPROM.get(0, ver);  // eerste byte EEPROM = versienummer

  // EEPROM versienummer verschillend van EEPROM_VER
  if (ver != EEPROM_VER) {
    // Als versienummer niet goed = schrijf defaults
    EEPROMWritePresets();
    lcdShowInit("Reset",3,0);
    delay(1000);
  }
  // Haal waardes op uit EEPROM
  lcdShowInit("Loaded",3,0);
  EEPROMReadSettings();

  return "";
}

/*
 * Connecteer met WiFi
 */
boolean WiFiConnect() {
  lcdShowInit("WiFi",2,0);  
  if ( wm_reset ) {wifiManager.resetSettings();}
  //wifiManager.setDebugOutput(false);
  wifiManager.setMinimumSignalQuality(wm_quality);
  //Eerste parameter = naam accesspoint
  //Tweede parameter = paswoord
  wifiSuccess=wifiManager.autoConnect("GistController", "gistcontroller");
  if(!wifiSuccess) {
      ESP.restart();
  } else {
    lcdShowInit(WiFi.localIP().toString().c_str(),3,0);
    delay(2000);
    return true;
  } 
}

/*
 * WiFi nog connectie
 */
boolean WiFiConnected() {
  boolean state       = true;

  if (WiFi.status() != WL_CONNECTED) {state = false;}
  if (WiFi.localIP().toString() == "0.0.0.0") {state = false;}

  return state;
}

/*
 * Send Mail
 */
boolean sendMail(String subject, String message) {
  boolean state       = true;
  String error_msg    = "";
  
  Gsender *gsender = Gsender::Instance();    // Getting pointer to class instance
  if(gsender->Subject(subject)->Send("gist.controller@gmail.com", message)) {
    state = true;
  }
  else 
  {
    error_msg = gsender->getError(); 
    state = false;
  }
  return state;
}

/*
 * Ophalen actuele tijd
 */
boolean setupDateTime(int loops) {
  int cntLoops          = 0;
  boolean datetimeState = true; 
  DateTime.setTimeZone(1);
  DateTime.begin();
  while (!DateTime.isTimeValid()) {
    delay(100);
    if (debug_wifi) {Serial.print(F("."));}
    if (cntLoops > loops) {
      datetimeState = false;
      break;
    }
    cntLoops++;
    DateTime.begin();
  }
  return datetimeState;
}

/*
 * Initialiseer LCD 
 */
void lcd_serial_msg_Init() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print(F("Gist Controller"));
  lcd.setCursor(17,0);
  lcd.print(versie);
  
  if ( debug || debug_tilt || debug_wifi || debug_buttons || debug_msgcsv) {
      Serial.begin(lcdBaud);
      Serial.println(F("Initialisatie Gist-controller"));
  }
  delay(1000);

  if (debug_msgcsv) {
    Serial.print(F("startpunt;meetpunt;currentstatus;tijdinstatus;newstatus;"));
    Serial.print(F("doelwort;doelfrigo;worttemp;frigotemp;coolingsinframe;"));
    Serial.println(F("heatingsinframe;tiltsinframe;tijdtussentilts"));
    //targetTempF=constrain(targetTempW-Kp*(wortTemp-targetTempW),targetTempW-TEMP_DANGER,targetTempW+TEMP_DANGER);
  }
  
}

/*
 * LCD: display boodschap 'initmsg' op lijn 2 positie 'pos' 
 */
void lcdShowInit(String initmsg, int row, int pos) {
  for(int r = row; r < 4; r++) {
    lcd.setCursor(0,r);
    for(int n = 0; n < 20; n++) {lcd.print(F(" "));}
    }
  lcd.setCursor(pos,row);
  lcd.print(initmsg);
  delay(1000);
}

/**
 * In welke LCD-status zitten we nu en toon die LCD-status
 */
void displayState()  {
  switch ( currentLCDState ) {
    case DISPLAY_SUMMARY:
      displaySummary();
      break;
    case DISPLAY_TEMP_HISTORY:
      displayHistory();
      break;
    case DISPLAY_SET_MSG_TIME:
      displaysetmsgtime();
      break;
    case DISPLAY_STATUS_MAX:
      displayStatusmax();
      break;
    case DISPLAY_SET_TARGET:
      displaytargetTempW();
      break;
    case DISPLAY_IP:
      displayIP();
      break;
    case DISPLAY_FORCE_ALLOUT:
      displayforceallout();
      break;
    case DISPLAY_FORCE_HEAT:
      displayforceheat();
      break;
    case DISPLAY_FORCE_COOL:
      displayforcecool();
      break;
    case DISPLAY_RESET_EEPROM:
      displayEEPROM();
      break;
    case DISPLAY_RESET_NODEMCU:
      displayRESET();
      break;
    default:
      // Opvangen fout in schermdefinities
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Current State:"));
      lcd.setCursor(15,0);
      lcd.print( currentLCDState );
      lcd.setCursor(0,1);
      lcd.print(F("Invalid State..."));
  }
}

/**
 * summary display
 */ 
void displaySummary() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Temp Wort"));
  lcd.setCursor(11,0);
  lcd.print(wortTemp);
  lcd.setCursor(16,0);
  lcd.print((char)223);
  lcd.setCursor(17,0);
  lcd.print(F("C"));

  lcd.setCursor(0,1);
  lcd.print(F("Temp Frigo"));
  lcd.setCursor(11,1);
  lcd.print(frigoTemp);
  lcd.setCursor(16,1);
  lcd.print((char)223);
  lcd.setCursor(17,1);
  lcd.print(F("C"));
  
  lcd.setCursor(0,2);
  lcd.print(F("Doel"));
  lcd.setCursor(16,2);
  lcd.print((char)223);
  lcd.setCursor(17,2);
  lcd.print(F("C"));
  lcd.setCursor(11,2);
  lcd.print(targetTempW);

  lcd.setCursor(3,3);
  if (forced) {lcd.print(F("M"));}
  else {lcd.print(F("A"));}
    
  lcd.setCursor(5,3);
  switch ( currentControllerState ) {
    case STATE_COOLING:
      lcd.print(F("*"));
      break;
    case STATE_INACTIVE:
      lcd.print(F("-"));
      break;
    case STATE_HEATING:
      lcd.print(F("^"));
      break;
  }    
  lcd.setCursor(7,3);
  lcd.print(showTime(millisInControllerState/1000,false));
}

/**
 * display temp history MIN
 */ 
void displayHistory() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Temp Historiek:"));
  lcd.setCursor(1,1);
  lcd.print(F("Min: "));
  lcd.setCursor(6,1);
  lcd.print(minWortTemp);
  lcd.setCursor(1,2);
  lcd.print(F("Max: "));
  lcd.setCursor(6,2);
  lcd.print(maxWortTemp);
}

/**
 * display msg-time 
 */ 
void displaysetmsgtime() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Boodschap tijd: "));
  lcd.setCursor(0,1);
  lcd.print(millisMessage/60000);
  lcd.setCursor(7,1);
  lcd.print(F("[up/down]"));
  if ( buttonPushed ) {
    lcd.setCursor(18,1);
    lcd.print(F("<>"));
  }
}

/**
 * display status max
 */ 
void displayStatusmax() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Max Status:"));
  lcd.setCursor(1,1);
  lcd.print(F("Koelen"));
  lcd.setCursor(8,1);
  lcd.print(showTime(maxTimeCooling/1000,false));
  lcd.setCursor(1,2);
  lcd.print(F("Warmen"));
  lcd.setCursor(8,2);
  lcd.print(showTime(maxTimeHeating/1000,false));
}
/**
 * display target temp
 */ 
void displaytargetTempW() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Doel Temp: "));
  lcd.setCursor(0,1);
  lcd.print(targetTempW);
  lcd.setCursor(7,1);
  lcd.print(F("[up/down]"));
  if ( buttonPushed ) {
    lcd.setCursor(18,1);
    lcd.print(F("<>"));
  }
}

/**
 * display IP
 */
void displayIP() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("IP:"));
  lcd.setCursor(0,1);
  lcd.print(WiFi.localIP().toString().c_str());
}

/**
 * display EEPROM: reset on push
 */
void displayEEPROM() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Reset EEPROM?"));
  lcd.setCursor(0,1);
  lcd.print(F("Bevestig met push."));
}

/**
 * display RESET: reset NODEMCU
 */
void displayRESET() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Reset NodeMCU?"));
  lcd.setCursor(0,1);
  lcd.print(F("Bevestig met push."));
}

/**
 * COOL/HEAT automatisch
 */
void displayforceallout() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Temp automatisch?"));
  lcd.setCursor(0,1);
  lcd.print(F("Bevestig met push."));
}

/**
 * HEAT
 */
void displayforceheat() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Verwarmen?"));
  lcd.setCursor(0,1);
  lcd.print(F("Bevestig met push."));
}

/**
 * COOL
 */
void displayforcecool() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Koelen?"));
  lcd.setCursor(0,1);
  lcd.print(F("Bevestig met push."));
}

/**
 * Zet seconden om in een string "dagen hh:mm:ss"
 * val = seconds (millis eerst delen door 1000)
 */
String showTime(int val,bool showDays){
  int days                = 0;
  int hours               = 0;
  int minutes             = 0;
  int seconds             = 0;
  String result           = "";
  
  days    = elapsedDays(val);
  hours   = numberOfHours(val);
  minutes = numberOfMinutes(val);
  seconds = numberOfSeconds(val);
  if (showDays) {
    result  = String(zeroPad(days)) + " ";
  }
  result  = result + String(zeroPad(hours));
  result  = result + ":" + String(zeroPad(minutes));
  result  = result + ":" + String(zeroPad(seconds));
  return result;
}

// Verfraai getallen = voeg 0 toe als cijfer 1 positie is
String zeroPad( int value ) {
  String valueString = String(value);
  if (value < 10) {
    valueString = String("0") + valueString;
  }
  return valueString;
}

/*
 * vul string met informatie om te verzenden als message
 * <BR> = nieuwe lijn <p> = nieuwe paragraaf
 */
String fillMessage() {
  String bmsg       = "";

  bmsg = bmsg + WiFi.localIP().toString().c_str() + "<BR>";
  bmsg = bmsg + DateFormatter::format("Startpunt: %d/%m/%Y %H:%M:%S", startDateInt);
  bmsg = bmsg + DateFormatter::format(" Meetpunt: %d/%m/%Y %H:%M:%S", DateTime.now());
  bmsg += "<p>";
            
  bmsg += "Momenteel ";
  switch ( currentControllerState ) {
    case STATE_COOLING:
      bmsg = bmsg + "Koelen   : ";
      break;
    case STATE_INACTIVE:
      bmsg = bmsg + "Inactief : ";
      break;
    case STATE_HEATING:
      bmsg = bmsg + "Verwarmen: ";
      break;
  }
  bmsg = bmsg + showTime(millisInControllerState/1000,false) + "<BR>";
  bmsg = bmsg + "Wort temperatuur: " + wortTemp;
  bmsg = bmsg + " Frigo temperatuur: " + frigoTemp;
  bmsg += "<p>";
 
  bmsg = bmsg + "maxWortTemp: " + maxWortTemp + "<BR>";
  bmsg = bmsg + "minWortTemp: " + minWortTemp + "<BR>";
  bmsg = bmsg + "MaxTimeHeating: ";
  bmsg = bmsg + showTime(maxTimeHeating/1000,false) + "<BR>";
  bmsg = bmsg + "MaxTimeCooling: ";
  bmsg = bmsg + showTime(maxTimeCooling/1000,false);
  bmsg += "<p>";

  bmsg = bmsg + "Aantal Coolings: " + countStatCool;
  bmsg = bmsg + " Totaal: " + countStatCoolTotal + "<BR>";
  bmsg = bmsg + "Aantal Heatings: " + countStatHeat;
  bmsg = bmsg + " Totaal: " + countStatHeatTotal;
  bmsg += "<p>";  

  bmsg = bmsg + "Aantal tilts: " + countTilts;
  bmsg = bmsg + " Totaal: " + countTiltsTotal + "<BR>";
  bmsg = bmsg + "Laatste tilt-tijd: " + showTime(millisBetweenTilts/1000,true);
  bmsg += "<p>";
  
  return bmsg;
}

/*
 * Message bij statusupdate: cool/heat
 */
String fillStateMessage() {
  String bmsg       = "";

  bmsg = bmsg + WiFi.localIP().toString().c_str() + "<BR>";
  bmsg = bmsg + DateFormatter::format("Startpunt: %d/%m/%Y %H:%M:%S", startDateInt);
  bmsg = bmsg + DateFormatter::format(" Meetpunt: %d/%m/%Y %H:%M:%S", DateTime.now());
  bmsg += "<p>";
            
  bmsg += "Naar ";
  switch ( currentControllerState ) {
    case STATE_COOLING:
      bmsg = bmsg + "Koelen";
      break;
    case STATE_INACTIVE:
      bmsg = bmsg + "Inactief";
      break;
    case STATE_HEATING:
      bmsg = bmsg + "Verwarmen";
      break;
  }
  bmsg = bmsg + "--> Wort temperatuur: " + wortTemp;
  bmsg = bmsg + " Frigo temperatuur: " + frigoTemp;
  bmsg += "<p>";
   
  return bmsg;
}

/*
 * vul string met Alertinformatie om te verzenden
 * <BR> = nieuwe lijn <p> = nieuwe paragraaf
 */
String fillAlertMessage() {  
  String bmsg       = "";
  bmsg = bmsg + WiFi.localIP().toString().c_str() + "<BR>";
  bmsg = bmsg + DateFormatter::format("Startpunt: %d/%m/%Y %H:%M:%S", startDateInt);
  bmsg = bmsg + DateFormatter::format(" Meetpunt: %d/%m/%Y %H:%M:%S", DateTime.now());
  bmsg += "<p>";
            
  bmsg += "Momenteel ";
  switch ( currentControllerState ) {
    case STATE_COOLING:
      bmsg = bmsg + "Koelen   : ";
      break;
    case STATE_INACTIVE:
      bmsg = bmsg + "Inactief : ";
      break;
    case STATE_HEATING:
      bmsg = bmsg + "Verwarmen: ";
      break;
  }
  bmsg = bmsg + showTime(millisInControllerState/1000,false) + "<BR>";
  
  bmsg = bmsg + "Wort temperatuur: " + wortTemp;
  bmsg = bmsg + " Frigo temperatuur: " + frigoTemp;
  bmsg += "<p>";
  
  return bmsg;
}

void serialMsgCsv(String newstate ) {
  //startpunt;meetpunt;currentstatus;tijdinstatus;newstatus;
  //doelwort;doelfrigo;worttemp;frigotemp;coolingsinframe;
  //heatingsinframe;tiltsinframe;tijdtussentilts
  //targetTempF=constrain(targetTempW-Kp*(wortTemp-targetTempW),targetTempW-TEMP_DANGER,targetTempW+TEMP_DANGER);

  // startdatum
  Serial.print(DateFormatter::format("%d/%m/%Y %H:%M:%S", startDateInt));
  Serial.print(F(";"));
  // meetpunt
  Serial.print(DateFormatter::format("%d/%m/%Y %H:%M:%S", DateTime.now()));
  Serial.print(F(";"));
  // status op meetpunt
  switch ( currentControllerState ) {
    case STATE_COOLING:
      Serial.print(F("Koelen"));
      break;
    case STATE_INACTIVE:
      Serial.print(F("Inactief"));
      break;
    case STATE_HEATING:
      Serial.print(F("Verwarmen"));
      break;
  }
  Serial.print(F(";"));
  
  // tijd in huidige status
  Serial.print(showTime(millisInControllerState/1000,false));
  Serial.print(F(";"));
  // nieuwe status
  Serial.print(newstate);
  Serial.print(F(";"));
  // doeltemperatuur
  Serial.print(targetTempW);
  Serial.print(F(";"));
  Serial.print(targetTempF);
  Serial.print(F(";"));
  // huidige worttemp
  Serial.print(wortTemp);
  Serial.print(F(";"));
  // frigo temp
  Serial.print(frigoTemp);
  Serial.print(F(";"));

  // coolings in timeframe
  Serial.print(countStatCool);
  Serial.print(F(";"));
  // heatings in timeframe
  Serial.print(countStatHeat);
  Serial.print(F(";"));

  // tilts (naar 0) in timeframe
  Serial.print(countTilts);
  Serial.print(F(";"));
  // tijd tussen 0-1 laatste tilt
  Serial.print(showTime(millisBetweenTilts/1000,true));

  Serial.println(F(""));
}

void serialWifi(boolean wstate) {
  if (wstate) {
    Serial.println(F(""));
    //Serial.println(ssid);
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
  }
  else {
    Serial.println("");
    Serial.println(F("Connection failed."));
  }
}

void serialTilted() {
  Serial.print(F("Tilt = "));
  Serial.print(String(lastTilt));
  Serial.print(F(" op: "));
  Serial.print(showTime(millisLastTilt/1000,true));
  Serial.print(F(" Verschil: "));
  Serial.print(showTime(millisBetweenTilts/1000,true));
  Serial.print(F(" Count/Tot: "));
  Serial.print(countTilts);
  Serial.print(F(" / "));
  Serial.println(countTiltsTotal);
}

void EEPROMReadSettings() {  // read settings from EEPROM
  EEPROM.get(10, targetTempW);
  EEPROM.get(20, millisMessage);
  EEPROM.get(30, countTiltsTotal);
  EEPROM.get(100, startDateInt);
  Serial.print(F("Import from EEPROM: TempW:"));
  Serial.print(targetTempW);
  Serial.print(F(" millisMessage:"));
  Serial.print(millisMessage);
  Serial.print(F(" startDateInt:"));
  Serial.print(startDateInt);
  Serial.println("");
}

void EEPROMWriteSettings() {  // write current settings to EEPROM
  byte temp = EEPROM_VER;
  EEPROM.put(0, temp);
  EEPROM.put(10, targetTempW);
  EEPROM.put(20, millisMessage);
  EEPROM.put(30, countTiltsTotal);
  EEPROM.put(100, startDateInt);
  EEPROM.commit();
}

/*
 * Bewaar defaults in EEPROM
 */
void EEPROMWritePresets() {
  startDateInt = DateTime.now();
  byte temp = EEPROM_VER;
  // double=8 int=4 byte=1 bool=1 startDateInt
  EEPROM.put(0, temp);            // update EEPROM version
  float presettemp = 21.00;
  EEPROM.put(10, presettemp);          // targetTempW
  EEPROM.put(20, 3600000);        // millisMessage
  EEPROM.put(30, countTiltsTotal);// totaal tilts
  EEPROM.put(100, startDateInt);  // startdate
  EEPROM.commit();
}

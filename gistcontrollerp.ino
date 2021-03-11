/*
 * GIP Jobbe Geybels 2020-2021
 * Gistcontroller v6.x op een NODEMCU-board (ESP8266)
 *    P-Controle: Simpele Proportionele Controle
 * 
 * WifiSetup      via WiFiManager (On Demand via IPscherm+select)
 * I2C-ADXL345    Gyroscoop
 * I2C-LCD        LCD
 * RotaryEncoder  Druk- en draai-knop
 * DS18B20        Tempsensor: Wort/Frigo
 *    Frigo Sensor 1 : 0x28, 0xAA, 0xD0, 0x6D, 0x59, 0x14, 0x01, 0xA2
 *    Wort  Sensor 2 : 0x28, 0x76, 0xCD, 0xF0, 0x3A, 0x19, 0x01, 0x3F
 * Relays         Cooling - Heating
 * ESP8266        Email naar gist.controller@gmail.com
 * EEPROM         Bewaar settings (herstart): aanpassen via EEPROM_VER of menu
 * 
 */
 
#include <LiquidCrystal_I2C.h>             // Library voor LCD I2C
#include <ErriezRotaryFullStep.h>          // Encoder library
#include <Adafruit_ADXL345_U.h>            // Library ADXL345
#include <Adafruit_Sensor.h>               // Library ADXL345
#include <DallasTemperature.h>             // Library voor DS18B20
#include <OneWire.h>                       // Library voor DS18B20
#include <ESP8266WiFi.h>                   // Library voor wifi
#include <ESP8266WebServer.h>              // WebServer
#include <WiFiManager.h>                   // Library voor WifiManager
#include <EEPROM.h>                        // Library EEPROM: bewaar parameters
#include <ESPDateTime.h>                   // Datum en tijd
#include "Gsender.h"                       // GMail settings
#include "parameters.h"                    // Parameters

void setup(void) { 
  lcd_serial_msg_Init();                          // Init LCD,Serieel en Serieel-message-
  initSuccess = initComponents();                 // Initialiseer componenten
  if (!initSuccess.isEmpty()) {                   // endless loop = initialisatie was niet goed
    while(1) {
      lcdShowInit("!! Probleem Init",2,0);
      lcdShowInit(initSuccess,3,0);
    }
  }
  getTemperature();                               // Initialiseer wortTemp/max-minWortTemp
  maxWortTemp = minWortTemp = wortTemp;
  if ( send_msg ) sendInitMessage();

  currentLCDState == DISPLAY_SUMMARY;             // Toon hoofdscherm
  displayState();
}

void loop(void) {
  updateTemperature();                            // Ophalen temperatuur en bereken targetTempF
  if ( WiFiConnected() ) server.handleClient();   // Webserver
  controlState();                                 // Update status: koel,inactief,verwarm
  updateTilted();                                 // Controleer tiltsensor
  controlDisplayState();                          // Beheer LCD (display, backlight, button, ...)
  if ( send_msg  ) sendBaseMessage();             // Stuur BASIS-email als tijd verstreken 
}

/**
 * Ophalen huidige temperatuur, bereken targetTempF en update MAX/MIN-Temp
 */
void updateTemperature() {
  getTemperature();
  // automodus terug aanzetten als worttemp bereikt!
  if ( forced && wortTemp == targetTempW ) {
    digitalWrite(HEATING_PIN, LOW);
    digitalWrite(COOLING_PIN, LOW);
    forced=false;
  }  
  float minvalue = targetTempW - TEMP_DANGER_C;
  float maxvalue = targetTempW + TEMP_DANGER_H;
  // Bereken de gewenste FrigoTemp om de gewenste WortTemp te behouden
  targetTempF=constrain(targetTempW-Kp*(wortTemp-targetTempW),minvalue,maxvalue);

  if (wortTemp > maxWortTemp)       maxWortTemp = wortTemp;
  else if (wortTemp < minWortTemp)  minWortTemp = wortTemp;
  
  // Alert-message als er iets grondig fout loopt
  if ( send_msg ) {
    if ( wortTemp > (targetTempW + TEMP_DANGER_H) ) sendAlertMessage();
    if ( wortTemp < (targetTempW - TEMP_DANGER_C) ) sendAlertMessage(); 
  }
}

/**
 * Haal wort- frigo-temp op
 */
void getTemperature() {
  sensors.requestTemperatures();            // Opvragen temperatuur
  wortTemp = sensors.getTempC(sensor1);
  frigoTemp = sensors.getTempC(sensor2);
}

/**
 * Wijzig status adhv temperatuur + bewaar de timers 
 * (HEATING) <=> (INACTIVE) <=> (COOLING)
 */
void controlState() {
  currentMillis           = millis();
  millisInControllerState = currentMillis - millisStateStart;

  // Manuele status = geen verdere acties
  if (forced) return;

  // Automatische status
  // adhv huidige status en targetTempF wijzig STATUS
  switch ( currentControllerState ) {
    // Momenteel aan het KOELEN
    case STATE_COOLING:
      // bereken runtime in seconden = hoelang is frigo al bezig?
      runTime = (unsigned long)(millis() - millisStateStart) / 1000;
      // Compressor moet minstens een bepaalde tijd draaien voor hij af mag
      if (runTime < coolMinOn) break;
      if (frigoTemp < targetTempF) {
        //stop KOELEN + zet INACTIEF
        currentControllerState  = STATE_INACTIVE;
        if (millisInControllerState > maxTimeCooling) maxTimeCooling = millisInControllerState;
        millisStateStart        = currentMillis;
        digitalWrite(COOLING_PIN, LOW);
        if ( send_msg ) sendStateMessage();          // Stuur STATUS-email
      } else {
        // Alert-message als er iets grondig fout loopt
        if ( send_msg && millisInControllerState > STATE_DANGER ) sendAlertMessage();
      }
      break;
    // Momenteel INACTIEF = niks aan het doen
    case STATE_INACTIVE:
      if ( frigoTemp < ( targetTempF - Deadband ) ) {
        //start VERWARMEN
        currentControllerState  = STATE_HEATING;
        millisStateStart        = currentMillis;
        countStatHeat++;
        countStatHeatTotal++;
        digitalWrite(HEATING_PIN, HIGH);
        if ( send_msg ) sendStateMessage();       // Stuur STATUS-email
      }
      else if (( frigoTemp > ( targetTempF + Deadband ) ) 
           && ((unsigned long)((millis() - stopTime) / 1000) > coolMinOff)) {
        //start KOELEN
        currentControllerState  = STATE_COOLING;
        millisStateStart        = currentMillis;
        countStatCool++;
        countStatCoolTotal++;
        digitalWrite(COOLING_PIN, HIGH);
        if ( send_msg ) sendStateMessage();       // Stuur STATUS-email
      }
      break;
    // Momenteel aan het VERWARMEN
    case STATE_HEATING:
      runTime = millis() - startTime;  // runtime in ms    
      if ( frigoTemp > targetTempF ) {
        //stop VERWARMEN    
        currentControllerState    = STATE_INACTIVE;
        if (millisInControllerState > maxTimeHeating) maxTimeHeating = millisInControllerState;
        millisStateStart          = currentMillis;
        digitalWrite(HEATING_PIN, LOW);
        if ( send_msg ) sendStateMessage();       // Stuur STATUS-email
      } else {
        // Alert-message als er iets grondig fout loopt
        if ( send_msg && millisInControllerState > STATE_DANGER ) sendAlertMessage();
      }
      break;
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
      millisBetweenTilts  = currentMillis - millisLastTilt;
      millisLastTilt      = currentMillis;
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
  if (gyro_X < 0) gyro_X = gyro_X * -1;
  if (gyro_Y < 0) gyro_Y = gyro_Y * -1;

  currentTilt = 0;
  // Vergelijk de x-y-meting met tiltThreshold
  // om te vermijden dat bij naar beneden vallen er door 
  // de terugslag een nieuwe tilt geregistreerd wordt
  if (gyro_X > tiltThreshold || gyro_Y > tiltThreshold) {
    currentTilt = 1;
  } 
}

/*
 * Berichten: BASIS - INIT - STATUS - ALERT
 * Vul telkens 'subject' (subjectMsg) en 'message' (fillXXMessage)
 * sendMail     = Verstuur de email met subject en message
 */
void sendBaseMessage() {
  currentMillis = millis();
  if ( WiFiConnected() && millisMessage > 0 && currentMillis - millisElapsedMessages > millisMessage) {
      millisElapsedMessages = currentMillis;
      String subject        = subjectMsg("BASIS");
      String message        = fillBaseMessage();
      mailSend              = sendMail(subject,message);
      if ( mailSend ) {
             countTilts = 0;
             countStatHeat = 0;
             countStatCool = 0;
      }
  }
}
void sendInitMessage() {
  if (WiFiConnected()) {
    lcdShowInit("Sending InitMsg",2,0);     
    String subject          = subjectMsg("INIT");
    String message          = fillInitMessage();
    mailSend                = sendMail(subject,message);
    }
}
void sendStateMessage() {
  if (WiFiConnected()) {
    String subject          = subjectMsg("STATUS");
    String message          = fillStateMessage();
    mailSend                = sendMail(subject,message);
    }
}
void sendAlertMessage() {
  // gebruik een countdownteller 'alertMaxTimer'
  // om te vermijden dat alert constant blijft komen
  --alertMaxTimer;
  if ( alertMaxTimer < 1) {
    if (WiFiConnected()) {
      alertMaxTimer         = alertCountDown;
      String subject        = subjectMsg("ALERT");
      String message        = fillAlertMessage();
      mailSend              = sendMail(subject,message);
      }
  }
}
String subjectMsg(String mType) {
  return mType + " Gist Controller " + versie + " Doel:" + targetTempW + "°C";
}

ICACHE_RAM_ATTR void potTurned() {
  // enkel actie als backlight actief
  if (!isBacklightActive) return;

  // reset de tijd in deze LCD-status
  // anders bestaat de kans dat je naar hoofdscherm springt tijdens draaien
  millisLCDStart  = millis();
  backlightStart  = millis();
  buttonAction    = BUTTON_NONE;
  
  int bturn;
  bturn = rotary.read();
  // geen beweging
  if ( bturn == 0 ) return;
  // Links/Rechts afhankelijk van rotary-snelheid (abs= absolute (=positieve) waarde)
  if ( abs(bturn) >= 2 ) {
    if (buttonPushed) {             // werd er op een knop geduwd waar dat mocht
        buttonAction=BUTTON_UP;
        handleUpDown( buttonAction, millis() );
        return;
      } else {  // geen push = ga terug in het menu
        currentLCDState += bturn * 2;
      }
  } else {
      if (buttonPushed) {           // werd er op een knop geduwd waar dat mocht
        buttonAction=BUTTON_DOWN;
        handleUpDown( buttonAction, millis() );
        return;
      } else {  // geen push = ga verder in menu
        currentLCDState += bturn;
      }
  }
  // test of je niet te ver gaat tov NO_OF_LCD_STATES
  currentLCDState += NO_OF_LCD_STATES;
  currentLCDState %= NO_OF_LCD_STATES;
  // toon het nieuwe scherm
  displayState();
}

ICACHE_RAM_ATTR void potPushed() {
  int bpush = digitalRead(RO_PUSH);
  if (bpush != 0) return;             // geen Button push
  
  // reset de tijd in deze LCD-status
  // anders bestaat de kans dat je naar hoofdscherm springt tijdens duwen
  millisLCDStart = millis();
  backlightStart = millis();

  // als backlight uit staat, zet het aan + stop
  if ( !isBacklightActive ) {
    buttonPushed = false;
    enableBacklight(millis());
    return;
  }

  // push voor aanpassing msg_time, target
  if ( currentLCDState == DISPLAY_SET_MSG_TIME ) {
      if ( buttonPushed ) EEPROMWriteSettings();
      buttonPushed = !buttonPushed;
      displaysetmsgtimeData();
      return;
  }
  else if ( currentLCDState == DISPLAY_SET_TARGET) {
      if ( buttonPushed ) EEPROMWriteSettings();
      buttonPushed = !buttonPushed;
      displaytargetTempWData();
      return;
  }

  switch (currentLCDState) {
      case DISPLAY_RESET_WIFI:                // herconnecteer Wifi
        wifiSuccess=WiFiConnect();
        break;
      case DISPLAY_RESET_EEPROM:              // reset EEPROM
        EEPROMWritePresets();
        EEPROMReadSettings();
        if ( send_msg ) sendInitMessage();
        break;
      case DISPLAY_RESET_NODEMCU:             // reset Nodemcu
        ESP.restart();
        break;
      case DISPLAY_STATUS_AUTO:               // terug naar automatisch COOL/HEAT
        digitalWrite(HEATING_PIN, LOW);
        digitalWrite(COOLING_PIN, LOW);    
        currentControllerState  = STATE_INACTIVE;
        millisStateStart        = millis();
        forced                  = false;
        break;
      case DISPLAY_STATUS_HEAT:                // HEAT aan tot weer uitgezet of tempwort = gewenst
        digitalWrite(COOLING_PIN, LOW);
        digitalWrite(HEATING_PIN, HIGH);
        currentControllerState  = STATE_HEATING;
        millisStateStart        = millis();
        forced                  = true;
        break;
      case DISPLAY_STATUS_COOL:                // COOL aan tot weer uitgezet of tempwort = gewenst
        digitalWrite(HEATING_PIN, LOW);
        digitalWrite(COOLING_PIN, HIGH);
        currentControllerState  = STATE_COOLING;
        millisStateStart        = millis();
        forced                  = true;
        break;
    }    
}

/**
 * Backlite uitzetten, display waarde SUMMARY, terug naar hoofdscherm
 */
void controlDisplayState() {
  if (!isBacklightActive) return;
  
  currentMillis     = millis();
  millisInLCDState  = currentMillis - millisLCDStart;
  
  // Zet LCD uit na verstrijken timeout (REDIRECT_TIMEOUT * 5)
  checkBacklightTimeout(currentMillis);

  // Als hoofdscherm actief update dan de waardes
  if ( currentLCDState == DISPLAY_SUMMARY ) {
    displaySummaryData();
    return;
  }
  
  // Geen knop gedrukt EN momenteel niet in SUMMARY-status 
  if ( !buttonAction && currentLCDState != DISPLAY_SUMMARY) {
    // Indien je te lang in huidige LCD-status zit = Terug naar SUMMARY-status
    if ( millisInLCDState >= REDIRECT_TIMEOUT ) {
      millisLCDStart  = currentMillis;
      backlightStart  = currentMillis;
      currentLCDState = DISPLAY_SUMMARY;
      displayState();
    }
    return;
  }  
}

/**
 * Zet LCD-backlight uit als timeout verstreken (REDIRECT_TIMEOUT * 5)
 * Zet LCD uit
 * Zet LCD aan
 */
void checkBacklightTimeout(int mtime) {
  backlightTimeout = currentMillis - backlightStart;
  if ( isBacklightActive && backlightTimeout > (REDIRECT_TIMEOUT * 5) ) {
    disableBacklight(mtime);
  }
}
void disableBacklight(int mtime) {
  lcd.noBacklight();
  lcd.noDisplay();
  isBacklightActive = false;
}
void enableBacklight(int mtime) {
  lcd.display();
  lcd.backlight();
  currentLCDState   = DISPLAY_SUMMARY;
  isBacklightActive = true;
  backlightStart    = mtime;
  millisLCDStart    = mtime;
}

/**
 * Knop = UP/DOWN = Wijzig parameters, afhankelijk van de huidige LCD-status
 */
void handleUpDown( int buttonAction, int mtime ) {
   // reset de tijd in deze LCD-status
   // anders bestaat de kans dat je naar hoofdscherm springt tijdens duwen
   millisLCDStart = mtime;
   backlightStart = mtime;

   switch ( currentLCDState ) {
    case DISPLAY_SET_MSG_TIME:            // Verhoog/verlaag de tijd tussen boodschappen                                          
      switch ( buttonAction ) {           // BETWEEN_MSG_INCR = in minuten *60000 = millis
        case BUTTON_UP:
          millisMessage += BETWEEN_MSG_INCR*60000;
          break;
        default:
          millisMessage -= BETWEEN_MSG_INCR*60000;
          break; 
      }
      if ( millisMessage < 0 ) millisMessage = 0;
      displaysetmsgtimeData();
      break;  
    case DISPLAY_SET_TARGET:              // Verhoog/verlaag de gewenste temperatuur
      switch ( buttonAction ) {
        case BUTTON_UP:
          if ( targetTempW < MAXIMUM_TARGET ) targetTempW += TEMP_INCR;
          break;
        default:
          if ( targetTempW > MINIMUM_TARGET ) targetTempW -= TEMP_INCR; 
          break; 
      }
      displaytargetTempWData();
      break;
  }
}
  
String initComponents() {
  // initialiseer pins

  pinMode(RO_PUSH, INPUT_PULLUP);         // rotary pushbutton
  attachInterrupt(digitalPinToInterrupt(RO_LEFT), potTurned, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RO_RIGHT), potTurned, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RO_PUSH), potPushed, CHANGE);
  
  pinMode( COOLING_PIN, OUTPUT);           // koelen
  pinMode( HEATING_PIN, OUTPUT );          // verwarmen
  digitalWrite( COOLING_PIN, LOW );        // initieel inactief zetten
  digitalWrite( HEATING_PIN, LOW );        // initieel inactief zetten
  sensors.begin();                         // Start the DS18B20 sensors

  lcd.setCursor(0,1);
  lcd.print(F("Initialisatie:"));

  // Initialiseer wifi als send_msg = true
  wifiSuccess=WiFiConnect();
  if(!wifiSuccess) {return "Wifi";}
  // Webserver standaard routines
  server.on("/", handle_OnConnect);
  server.onNotFound(handle_NotFound);
  server.begin();

  // test DS18B20 Wort
  lcdShowInit("DS18B20 - Wort",2,0);
  getTemperature();
  if (wortTemp == -127) {
     lcdShowInit("Error",3,0);
     return "DS18B20 Wort";
  } else {lcdShowInit("gelukt",3,0);}
  
  // test DS18B20 Frigo
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
  tilter.setRange(ADXL345_RANGE_4_G);
  
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
  startDateInt          = DateTime.now();
  backlightStart        = millis();
  millisElapsedMessages = millis();
  
  //EEPROM bewaren/ophalen waardes
  // EEPROM bevat de herbruikbare waardes: 
  //    Gewenste WortTemp, Tijd tussen BasisMails, TotaalTilts sinds start, Starttijd
  // Bij herstart NODEMCU worden deze herbruikt
  // Tenzij EEPROM_VER wordt aangepast ... of reset via menu
  byte ver;
  lcdShowInit("EEPROM",2,0);
  EEPROM.begin(512);
  delay(1500);
  // haal eerste byte op = bevat versienummer EEPROM_VER
  EEPROM.get(0, ver);
  if ( ver != EEPROM_VER ) {
    // Als versienummer niet= EEPROM_VER = schrijf defaults
    EEPROMWritePresets();
    lcdShowInit("Reset",3,0);
    delay(1000);
  }
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
  wifiManager.setDebugOutput(false);
  wifiManager.setMinimumSignalQuality(wm_quality);
  //Parameter1 = naam accesspoint  Parameter2 = paswoord
  wifiSuccess=wifiManager.autoConnect("GistController", "gistcontroller");
  if(!wifiSuccess) {ESP.restart();}
  else {  
    lcdShowInit(WiFi.localIP().toString().c_str(),3,0);
    current_SSID = WiFi.SSID();
    delay(2000);
    return true;
  } 
}
          
/*
 * WiFi nog connectie
 */
boolean WiFiConnected() {
  boolean state       = true;
  if (WiFi.status() != WL_CONNECTED) return false;
  if (WiFi.localIP().toString() == "0.0.0.0") return false;
  return state;
}

/*
 * Send Mail
 */
boolean sendMail(String subject, String message) {
  boolean state       = true;
  String error_msg    = "";
  
  Gsender *gsender = Gsender::Instance();
  if ( gsender->Subject(subject)->Send("gist.controller@gmail.com", message) ) {
    state = true;
  }
  else {
    error_msg = gsender->getError(); 
    state     = false;
  }
  return state;
}

/*
 * Initialisatie DateTime
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
 * Initialiseer LCD/Serieel
 */
void lcd_serial_msg_Init() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print(F("Gist Controller"));
  lcd.setCursor(17,0);
  lcd.print(versie);  
  if ( debug || debug_tilt || debug_wifi || debug_buttons ) {
      Serial.begin(lcdBaud);
      Serial.println(F("Initialisatie Gist-controller"));
  }
  delay(1000);
}

/*
 * LCD: display boodschap 'initmsg' op lijn 'row' positie 'pos' 
 */
void lcdShowInit(String initmsg, int row, int pos) {
  // maak eerst de volledige lijn 'row' leeg
  for(int r = row; r < 4; r++) {
    lcd.setCursor(0,r);
    for(int n = 0; n < 20; n++) {lcd.print(F(" "));}
    }
  if ( initmsg != "" ) {
    lcd.setCursor(pos,row);
    lcd.print(initmsg);
    delay(500);
  }
}

/**
 * In welke LCD-status zitten we nu en toon die LCD-status
 */
void displayState() {
  if (!isBacklightActive) return;
  
  switch ( currentLCDState ) {
    case DISPLAY_SUMMARY:
      displaySummary();
      break;
    case DISPLAY_IP:
      displayIP();
      break;
    case DISPLAY_STATUS_MAX:
      displayStatusmax();
      break;
    case DISPLAY_TEMP_HISTORY:
      displayHistory();
      break;
    case DISPLAY_SET_MSG_TIME:
      displaysetmsgtime();
      break;
    case DISPLAY_SET_TARGET:
      displaytargetTempW();
      break;
    case DISPLAY_RESET_WIFI:
      displayreset_WIFI();
      break;
    case DISPLAY_RESET_EEPROM:
      displayreset_EEPROM();
      break;
    case DISPLAY_RESET_NODEMCU:
      displayreset_NODEMCU();
      break;
    case DISPLAY_STATUS_AUTO:
      displaystatus_auto();
      break;
    case DISPLAY_STATUS_HEAT:
      displaystatus_heat();
      break;
    case DISPLAY_STATUS_COOL:
      displaystatus_cool();
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
 * DISPLAY-schermen
 */ 
void displaySummary() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Temp Wort"));
  lcd.setCursor(16,0);
  lcd.print((char)223);
  lcd.setCursor(17,0);
  lcd.print(F("C"));

  lcd.setCursor(0,1);
  lcd.print(F("Temp Frigo"));
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
}
void displaySummaryData() {
  lcd.setCursor(11,0);
  lcd.print(wortTemp);

  lcd.setCursor(11,1);
  lcd.print(frigoTemp);
  
  lcd.setCursor(11,2);
  lcd.print(targetTempW);

  lcd.setCursor(2,3);
  if (wifiSuccess) {lcd.print(F("W"));}
  else {lcd.print(F(" "));}
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
void displayIP() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("IP"));
  lcd.setCursor(0,1);
  lcd.print(WiFi.localIP().toString().c_str());
}

/* 
 *  SET-schermen targetTemp, msgtime
 */
void displaytargetTempW() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Doel Temp: "));
  lcd.setCursor(0,1);
  lcd.print(targetTempW);
  lcd.setCursor(7,1);
  lcd.print(F("[up/down]"));
  lcd.setCursor(18,1);
  lcd.print(F("  "));
}
void displaytargetTempWData() {
  lcd.setCursor(0,1);
  lcd.print(targetTempW);
  if ( buttonPushed ) {
    lcd.setCursor(18,1);
    lcd.print(F("<>"));
  } else {lcd.print(F("  "));}
}
void displaysetmsgtime() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Boodschap tijd: "));
  lcd.setCursor(0,1);
  lcd.print(millisMessage/60000);
  lcd.setCursor(7,1);
  lcd.print(F("[up/down]"));
  lcd.setCursor(18,1);
  lcd.print(F("  "));
}
void displaysetmsgtimeData() {
  lcd.setCursor(0,1);
  lcd.print(millisMessage/60000);
  if ( buttonPushed ) {
    lcd.setCursor(18,1);
    lcd.print(F("<>"));
  } else {lcd.print(F("  "));}
}

/**
 * RESET-schermen
 */
void displayreset_WIFI() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Reset WiFi?"));
  lcd.setCursor(0,1);
  lcd.print(F("Bevestig met push."));
}
void displayreset_EEPROM() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Reset EEPROM?"));
  lcd.setCursor(0,1);
  lcd.print(F("Bevestig met push."));
}
void displayreset_NODEMCU() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Reset NodeMCU?"));
  lcd.setCursor(0,1);
  lcd.print(F("Bevestig met push."));
}

/**
 * Status Automatisch/COOL/HEAT
 */
void displaystatus_auto() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Temp automatisch?"));
  lcd.setCursor(0,1);
  lcd.print(F("Bevestig met push."));
}
void displaystatus_heat() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Verwarmen?"));
  lcd.setCursor(0,1);
  lcd.print(F("Bevestig met push."));
}
void displaystatus_cool() {
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
  if (showDays) result  = String(zeroPad(days)) + "d ";
  result  = result + String(zeroPad(hours));
  result  = result + ":" + String(zeroPad(minutes));
  result  = result + ":" + String(zeroPad(seconds));
  return result;
}

// Verfraai getallen = voeg 0 toe als cijfer 1 positie is
String zeroPad( int value ) {
  String valueString = String(value);
  if (value < 10) valueString = String("0") + valueString;
  return valueString;
}


/*
 * BASIS-email: opvullen tekst
 * <BR> = nieuwe lijn <p> = nieuwe paragraaf
 */
String fillBaseMessage() {
  String bmsg = msgStandard();            
  bmsg = bmsg + "Momenteel " + msgStatus();
  bmsg += paragraph;
  bmsg = bmsg + oneLineMessage("STATUS");
  return bmsg;
}

/*
 * STATUS-email: opvullen tekst
 */
String fillStateMessage() {
  String bmsg = msgStandard();       
  bmsg = bmsg + "Naar " + msgStatus();
  bmsg = bmsg + " --> Temp Wort: " + wortTemp;
  bmsg = bmsg + " Frigo: " + frigoTemp;
  bmsg += paragraph;

  bmsg = bmsg + oneLineMessage("StateCHANGE");  
  return bmsg;
}

/*
 * ALERT-email: opvullen tekst
 */
String fillAlertMessage() {  
  String bmsg = msgStandard();         
  bmsg = bmsg + "Momenteel " + msgStatus();
  bmsg = bmsg + showTime(millisInControllerState/1000,false);
  bmsg = bmsg + " Temp Wort: " + wortTemp;
  bmsg = bmsg + " Frigo: " + frigoTemp;
  bmsg += paragraph;

  bmsg = bmsg + oneLineMessage("ALERT");  
  return bmsg;
}

/*
 * INIT-email: opvullen tekst
 */
String fillInitMessage() {  
  String bmsg = msgStandard();         
  bmsg = bmsg + "Momenteel " + msgStatus();
  bmsg = bmsg + showTime(millisInControllerState/1000,false);
  bmsg = bmsg + " Temp Wort: " + wortTemp;
  bmsg = bmsg + " Frigo: " + frigoTemp;
  bmsg += paragraph;
  return bmsg;
}

/*
 * Eerste standaardlijnen voor emails
 */
String msgStandard() {
  String base = "";
  base = base + WiFi.localIP().toString().c_str();
  base = base + DateFormatter::format("Startpunt: %d/%m/%Y %H:%M:%S", startDateInt);
  base = base + DateFormatter::format(" Meetpunt: %d/%m/%Y %H:%M:%S", DateTime.now());
  base += paragraph;
  return base;
}
/*
 * Statusinformatie voor emails
 */
String msgStatus() {
  String base = "";
  switch ( currentControllerState ) {
    case STATE_COOLING:
      base = "Koelen   : ";
      break;
    case STATE_INACTIVE:
      base = "Inactief : ";
      break;
    case STATE_HEATING:
      base = "Verwarmen: ";
      break;
  }
  return base;
}
/*
 * Laatste standaardlijnen voor emails: gebruikt in excel
 */
String oneLineMessage(String newstate ) {
  String olmsg    = "";
  olmsg = olmsg + "startdt;startuur;nowdt;nowuur;currentstatus;tijdinstatus;newstatus;doelwort;doelfrigo;worttemp;";
  olmsg = olmsg + "frigotemp;coolingsinframe;heatingsinframe;tiltsinframe;tijdtussentilts" + lbreak;
  // startdatum
  olmsg = olmsg + DateFormatter::format("%d/%m/%Y;", startDateInt);
  olmsg = olmsg + DateFormatter::format("%H:%M:%S;", startDateInt);
  // meetpunt
  olmsg = olmsg + DateFormatter::format("%d/%m/%Y;", DateTime.now());
  olmsg = olmsg + DateFormatter::format("%H:%M:%S;", DateTime.now());
  // status op meetpunt
  switch ( currentControllerState ) {
    case STATE_COOLING:
      olmsg = olmsg + "Koelen;";
      break;
    case STATE_INACTIVE:
      olmsg = olmsg + "Inactief;";
      break;
    case STATE_HEATING:
      olmsg = olmsg + "Verwarmen;";
      break;
  }                                                     // tijd in huidige status
  olmsg = olmsg + showTime(millisInControllerState/1000,false) + ";"; 
  olmsg = olmsg + newstate + ";";                       // nieuwe status
  olmsg = olmsg + targetTempW + ";";                    // doeltemperatuur Wort
  olmsg = olmsg + targetTempF + ";";                    // doeltemperatuur Frigo
  olmsg = olmsg + wortTemp + ";";                       // huidige worttemp
  olmsg = olmsg + frigoTemp + ";";                      // frigo temp
  olmsg = olmsg + countStatCool + ";";                  // coolings in timeframe
  olmsg = olmsg + countStatHeat + ";";                  // heatings in timeframe
  olmsg = olmsg + countTilts + ";";                     // tilts in timeframe
  olmsg = olmsg + showTime(millisBetweenTilts/1000,true);// tijd tussen 0-1 laatste tilt
  return olmsg;
}

void serialWifi(boolean wstate) {
  if (wstate) {
    Serial.println(F(""));
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
  } else {
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

/*
 * EEPROM dingen
 */
void EEPROMReadSettings() {  // Lees waardes uit EEPROM
  EEPROM.get(10, targetTempW);
  EEPROM.get(20, millisMessage);
  EEPROM.get(30, countTiltsTotal);
  EEPROM.get(40, startDateInt);
}
void EEPROMWriteSettings() {  // Bewaar waardes in EEPROM
  byte temp = EEPROM_VER;
  EEPROM.put(0, temp);
  EEPROM.put(10, targetTempW);
  EEPROM.put(20, millisMessage);
  EEPROM.put(30, countTiltsTotal);
  EEPROM.put(40, startDateInt);
  EEPROM.commit();
}
void EEPROMWritePresets() {   // Reset waardes in EEPROM
  startDateInt = DateTime.now();
  byte temp = EEPROM_VER;
  // double=8 int=4 byte=1 bool=1 startDateInt
  EEPROM.put(0, temp);            // update EEPROM version
  float presettemp = 21.00;
  EEPROM.put(10, presettemp);     // targetTempW
  EEPROM.put(20, 3600000);        // millisMessage
  EEPROM.put(30, 0);              // totaal tilts
  EEPROM.put(40, startDateInt);   // startdate
  EEPROM.commit();
}

//Webservice
void handle_OnConnect() {
  String lastTilted       = showTime(millisBetweenTilts/1000,false);
  String lastMessage      = showTime((millis() - millisElapsedMessages)/1000,false);
  String currentStateTime = showTime(millisInControllerState/1000,false);
  String currentState     = "";
  
  switch (currentControllerState) {
    case STATE_COOLING:
      currentState="KOELEN";
      break;
    case STATE_INACTIVE:
      currentState="INACTIEF";
      break;
    case STATE_HEATING:
      currentState="VERWARMEN";
      break;
  }
  server.send(200, "text/html", SendHTML(wortTemp,frigoTemp,targetTempF,targetTempW,lastTilted,currentState,countTilts,countTiltsTotal,currentStateTime,lastMessage)); 
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

String SendHTML(float wortTemp,float frigoTemp,float targetTempF, float targetTempW, String lastTilted, String currentState,int countTilts, int countTiltsTotal, String currentStateTime, String lastMessage){
  String ptr = "<!DOCTYPE html>";
  ptr +="<html>";
  ptr +="<head>";
  ptr +="<title>GistController</title>";
  ptr +="<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  ptr +="<link href='https://fonts.googleapis.com/css?family=Open+Sans:300,400,600' rel='stylesheet'>";
  ptr +="<style>";
  ptr +="html { font-family: 'Open Sans', sans-serif; display: block; margin: 0px auto; text-align: center;color: #444444;}";
  ptr +="body{margin-top: 50px;} ";
  ptr +="h1 {margin: 50px auto 30px;} ";
  ptr +=".side-by-side{display: table-cell;vertical-align: middle;position: relative;}";
  ptr +=".text{font-weight: 600;font-size: 19px;width: 200px;}";
  ptr +=".temperature{font-weight: 300;font-size: 30px;padding-right: 15px;}";
  ptr +=".intervals{font-weight: 300;font-size: 20px;padding-right: 15px;}";
  ptr +=".tilt{font-weight: 300;font-size: 19px;padding-right: 15px;}";
  ptr +=".message{font-weight: 300;font-size: 19px;padding-right: 15px;}";
  ptr +=".wort-temp .temperature{color: #3B97D3;}";
  ptr +=".frigo-temp .temperature{color: #F29C1F;}";
  ptr +=".wort-target .temperature{color: #3B97D3;}";
  ptr +=".frigo-target .temperature{color: #3B97D3;}";
  ptr +=".last-tilted .tilt{color: #26B99A;}";
  ptr +=".count-tilted .tilt{color: #26B99A;}";
  ptr +=".last-message .message{color: #26B99A;}";
  ptr +=".superscript{font-size: 17px;font-weight: 600;position: absolute;right: -5px;top: 15px;}";
  ptr +=".data{padding: 10px;}";
  ptr +=".container{display: table;margin: 0 auto;}";
  ptr +=".icon{width:52px}";
  ptr +="</style>";
  ptr +="<script>\n";
  
  ptr +="setInterval(loadDoc,1000);\n";
  ptr +="function loadDoc() {\n";
  ptr +="var xhttp = new XMLHttpRequest();\n";
  ptr +="xhttp.onreadystatechange = function() {\n";
  ptr +="if (this.readyState == 4 && this.status == 200) {\n";
  ptr +="document.body.innerHTML =this.responseText}\n";
  ptr +="};\n";
  ptr +="xhttp.open(\"GET\", \"/\", true);\n";
  ptr +="xhttp.send();\n";
  ptr +="}\n";
  ptr +="</script>\n";
  ptr +="</head>";
  
  ptr +="<body>";
  
  ptr +="<h2>GistController</h2>";
  ptr +="<h3>";
  ptr +=currentState;
  ptr +=" (";
  ptr +=currentStateTime;
  ptr +=")</h3>";
  
  ptr +="<div class='container'>";  
  
  ptr +="<div class='data wort-temp'>";
  ptr +="<div class='side-by-side icon'>";
  ptr +="<svg viewBox='17.999 0 65 69.915' xmlns=http://www.w3.org/2000/svg>";
  ptr +="<path d='M 80.833 54.927 L 59.238 18.944 L 59.238 4.37 L 61.423 4.37 C 62.625 4.37 63.608 3.387 63.608 2.185 C 63.608 0.983 ";
  ptr +="62.625 0 61.423 0 L 39.574 0 C 38.373 0 37.389 0.983 37.389 2.185 C 37.389 3.387 38.373 4.37 39.574 4.37 L 41.759 4.37 L 41.759 ";
  ptr +="18.944 L 20.164 54.927 C 15.218 63.171 19.037 69.915 28.65 69.915 L 72.347 69.915 C 81.961 69.915 85.779 63.171 80.833 54.927 Z M ";
  ptr +="31.998 43.697 L 46.129 20.146 L 46.129 4.37 L 54.868 4.37 L 54.868 20.146 L 68.999 43.697 L 31.998 43.697 Z'/>";
  ptr +="</svg>";
  ptr +="</div>";
  ptr +="<div class='side-by-side text'>Wort</div>";  
  ptr +="<div class='side-by-side temperature'>";
  ptr +=(float)wortTemp;
  ptr +="<span class='superscript'>&deg;C</span></div>";
  ptr +="</div>";
  
  ptr +="<div class='data frigo-temp'>";
  ptr +="<div class='side-by-side icon'>";
  ptr +="<svg viewBox='18.534 26.978 65 37.624' xmlns=http://www.w3.org/2000/svg>";
  ptr +="<path d='M 57.783 47.694 C 57.783 47.694 58.667 35.794 47.761 34.437 C 38.413 33.482 35.569 42.169 35.569 42.169 C 35.569 42.169 ";
  ptr +="32.755 39.462 28.936 41.672 C 25.518 43.783 26.124 47.644 26.124 47.644 C 26.124 47.644 18.534 49.12 18.534 56.859 C 18.743 64.718 ";
  ptr +="26.728 64.523 26.728 64.523 L 56.594 64.602 C 56.594 64.602 63.742 64.685 65.044 57.321 C 65.548 49.282 57.783 47.694 57.783 47.694 ";
  ptr +="Z M 76.25 40.309 C 76.25 40.309 77.134 28.409 66.23 27.05 C 56.882 26.095 53.923 34.896 53.923 34.896 C 53.923 34.896 59.369 38.056 ";
  ptr +="59.6 46.597 C 63.062 47.637 66.752 50.493 66.868 57.188 L 75.06 57.217 C 75.06 57.217 82.209 57.3 83.51 49.936 C 84.015 41.893 76.25 ";
  ptr +="40.309 76.25 40.309 Z'/>";
  ptr +="</svg>";  
  ptr +="</div>";
  ptr +="<div class='side-by-side text'>Frigo</div>";
  ptr +="<div class='side-by-side temperature'>";
  ptr +=(float)frigoTemp;
  ptr +="<span class='superscript'>&deg;C</span></div>";
  ptr +="</div>";

  ptr +="<div class='data wort-target'>";
  ptr +="<div class='side-by-side icon'>";
  ptr +="<svg viewBox='17.999 0 65 69.915' xmlns=http://www.w3.org/2000/svg>";
  ptr +="<path d='M 80.833 54.927 L 59.238 18.944 L 59.238 4.37 L 61.423 4.37 C 62.625 4.37 63.608 3.387 63.608 2.185 C 63.608 0.983 ";
  ptr +="62.625 0 61.423 0 L 39.574 0 C 38.373 0 37.389 0.983 37.389 2.185 C 37.389 3.387 38.373 4.37 39.574 4.37 L 41.759 4.37 L 41.759 ";
  ptr +="18.944 L 20.164 54.927 C 15.218 63.171 19.037 69.915 28.65 69.915 L 72.347 69.915 C 81.961 69.915 85.779 63.171 80.833 54.927 Z M ";
  ptr +="31.998 43.697 L 46.129 20.146 L 46.129 4.37 L 54.868 4.37 L 54.868 20.146 L 68.999 43.697 L 31.998 43.697 Z'/>";
  ptr +="</svg>";
  ptr +="</div>";
  ptr +="<div class='side-by-side text'>Doel Wort</div>";
  ptr +="<div class='side-by-side temperature'>";
  ptr +=(float)targetTempW;
  ptr +="<span class='superscript'>&deg;C</span></div>";
  ptr +="</div>";
  
  ptr +="<div class='data frigo-target'>";
  ptr +="<div class='side-by-side icon'>";
  ptr +="<svg viewBox='18.534 26.978 65 37.624' xmlns=http://www.w3.org/2000/svg>";
  ptr +="<path d='M 57.783 47.694 C 57.783 47.694 58.667 35.794 47.761 34.437 C 38.413 33.482 35.569 42.169 35.569 42.169 C 35.569 42.169 ";
  ptr +="32.755 39.462 28.936 41.672 C 25.518 43.783 26.124 47.644 26.124 47.644 C 26.124 47.644 18.534 49.12 18.534 56.859 C 18.743 64.718 ";
  ptr +="26.728 64.523 26.728 64.523 L 56.594 64.602 C 56.594 64.602 63.742 64.685 65.044 57.321 C 65.548 49.282 57.783 47.694 57.783 47.694 ";
  ptr +="Z M 76.25 40.309 C 76.25 40.309 77.134 28.409 66.23 27.05 C 56.882 26.095 53.923 34.896 53.923 34.896 C 53.923 34.896 59.369 38.056 ";
  ptr +="59.6 46.597 C 63.062 47.637 66.752 50.493 66.868 57.188 L 75.06 57.217 C 75.06 57.217 82.209 57.3 83.51 49.936 C 84.015 41.893 76.25 ";
  ptr +="40.309 76.25 40.309 Z'/>";
  ptr +="</svg>";  
  ptr +="</div>";
  ptr +="<div class='side-by-side text'>Doel Frigo</div>";
  ptr +="<div class='side-by-side temperature'>";
  ptr +=(float)targetTempF;
  ptr +="<span class='superscript'>&deg;C</span></div>";
  ptr +="</div>";

  ptr +="<div class='data last-tilted'>";
  ptr +="<div class='side-by-side icon'>";
  ptr +="<svg viewBox='31.856 63.712 65 58.08' xmlns='http://www.w3.org/2000/svg'>";
  ptr +="<path d='M 91.323 103.088 L 89.1 114.855 L 39.612 114.855 L 37.389 103.088 L 31.856 103.088 L 31.856 121.792 L 96.856 121.792 L ";
  ptr +="96.856 103.088 L 91.323 103.088 Z M 89.922 82.188 L 73.631 82.188 L 73.631 75.391 L 55.086 75.391 L 55.086 82.188 L 38.95 82.188 ";
  ptr +="L 64.27 108.663 L 89.922 82.188 Z M 73.629 63.712 L 55.083 63.712 L 55.083 66.185 L 73.629 66.185 L 73.629 63.712 Z M 73.629 68.331 ";
  ptr +="L 55.083 68.331 L 55.083 73.059 L 73.629 73.059 L 73.629 68.331 Z'/>";
  ptr +="</svg>";
  ptr +="</div>";
  ptr +="<div class='side-by-side text'>TiltTijd</div>";
  ptr +="<div class='side-by-side intervals'>";
  ptr +=lastTilted;
  ptr +="</div>";

  ptr +="<div class='data count-tilted'>";
  ptr +="<div class='side-by-side icon'></div>";
  ptr +="<div class='side-by-side text'>Tilts</div>";
  ptr +="<div class='side-by-side intervals'>";
  ptr +=countTilts;
  ptr +="/";
  ptr +=countTiltsTotal;
  ptr +="</div>";

  ptr +="<div class='data last-message'>";
  ptr +="<div class='side-by-side icon'>";
  ptr +="<svg viewBox='31.856 63.712 65 58.08' xmlns='http://www.w3.org/2000/svg'>";
  ptr +="<path d='M464 64h-416c-26.4 0-48 21.6-48 48v320c0 26.4 21.6 48 48 48h416c26.4 0 48-21.6 48-48v-320c0-26.4-21.6-48-48-48zM199.37 ";
  ptr +="275.186l-135.37 105.446v-250.821l135.37 145.375zM88.19 128h335.62l-167.81 126-167.81-126zM204.644 280.849l51.356 55.151 51.355-55.151 ";
  ptr +="105.277 135.151h-313.264l105.276-135.151zM312.63 275.186l135.37-145.375v250.821l-135.37-105.446z'/>";
  ptr +="</svg>";
  ptr +="</div>";
  ptr +="<div class='side-by-side text'>Boodschap</div>";
  ptr +="<div class='side-by-side intervals'>";
  ptr +=lastMessage;
  ptr +="</div>";
  
  ptr +="</div>";
  ptr +="</body>";
  ptr +="</html>";
  return ptr;
}

#ifndef PARAMETERS_H
#define PARAMETERS_H

const int EEPROM_VER              = 15;    // EEPROM versie

bool    debug                     = false;
bool    debug_tilt                = false; // serieel tilt-test
bool    debug_buttons             = false; // serieel button-test
bool    debug_wifi                = false; // serieel wifi-test

bool    wm_reset                  = false; // true=reset wifimanager
int     wm_quality                = 10;    // %sterkte wifinetwerken
String  current_SSID              = "";    // SSID
String  versie                    = "7.1"; // versienummer
int     lcdBaud                   = 115200;// LCD baudrate
String  lbreak                    = "<BR>";// Line break in emails
String  paragraph                 = "<p>"; // Nieuwe paragraaf in emails

// 3600000=1h 1800000=30min 600000=10min 60000=1min
int     millisMessage             = 1800000;// Tijd tss twee messages 1000=1sec
int     millisElapsedMessages     = 0;     // Verstreken tijd
boolean mailSend                  = false; // true = mail verzenden gelukt
bool    send_msg                  = true;  // true=send mails (gewoon + alert)
char    sendStatusMsg             = "N";   // Verstuur email voor statusaanpassing?
int     alertMaxTimer             = 0;     // Countdown om alerts te beperken
int     alertCountDown            = 300;   // aantal tellen voor volgende alert

int     currentControllerState    = 1;     // 0=Koelen, 1=Niet-actief, 2=Verwarmen
long    millisInControllerState   = 0;     // millis doorgebracht in een status
long    millisStateStart          = 0;     // millis status gestart
long    millisStateStop           = 0;     // millis status gestopt
float   wortTemp                  = 0;     // Huidige temperatuur wort
float   frigoTemp                 = 0;     // Huidige temperatuur Frigo
float   targetTempW               = 21.0;  // Te handhaven temperatuur Wort
float   Kp                        = 6;     // Proportionele controle variabele

float   Deadband                  = 1;     // 1° verschil ter bescherming van de compressor
float   targetTempF               = 0;     // Berekening via de Kp-formule (Target temp Frigo)
float   last_targetTempF          = 0;     // vorige targetTempF (test)
float   maxWortTemp               = 0;     // hoogste temperatuur die bereikt werd
float   minWortTemp               = 0;     // laagste temperatuur die bereikt werd
int     maxTimeHeating            = 0;     // langste tijd status VERWARMEN (millis)
int     maxTimeCooling            = 0;     // langste tijd status KOELEN (millis)
boolean forced                    = false; // true = heat/cool werd via menu aangezet false=automatisch

boolean wifiSuccess               = false; // true = wifi connected
boolean datetimeSuccess           = false; // ophalen datetime gelukt/niet gelukt
String  initSuccess               = "";    // leeg als alles goed, anders aanduiding van fout
String  initMsg                   = "";    // display naar LCD tijdens initialisatie

boolean buttonPushed              = false; // werd er op de button gedrukt?
byte    buttonAction              = 0;     // BUTTON_NONE = 0
int     currentLCDState           = 0;     // Welk LCD momenteel actief: 0 tem ... 
int     millisInLCDState          = 0;     // millis in een LCDstaat
int     millisLCDStart            = 0;     // millis LCDstaat start
int     backlightTimeout          = 0;     // millis LCD uit 
int     backlightStart            = 0;     // millis backlight start
boolean isBacklightActive         = true;  // LCD momenteel actief?

float   gyro_X                    = 0;     // tilt-x-as
float   gyro_Y                    = 0;     // tilt-y-as
float   tiltThreshold             = 0.20;  // verschil op x/y-as... voorkomt 1-tel bij neervallen
int     currentTilt               = 0;     // de huidige tiltwaarde
int     lastTilt                  = 0;     // de vorige tiltwaarde
long    millisLastTilt            = 0;     // millis van de laaste 0-tilt
int     millisBetweenTilts        = 0;     // Tijd tussen twee tilts

int     countTilts                = 0;     // Aantal tilts in een millisMessage
int     countTiltsTotal           = 0;     // Aantal tilts sinds start
int     countStatHeat             = 0;     // Aantal HEAT binnen millisMessage
int     countStatHeatTotal        = 0;     // Aantal HEAT sinds start
int     countStatCool             = 0;     // Aantal COOL binnen millisMessage
int     countStatCoolTotal        = 0;     // Aantal COOL sinds start

int     currentMillis             = 0;     // millis op moment van de test = Nu!
int     logMillis                 = 0;
time_t  startDateInt              = 0;     // Startdatum/uur (DateTime)

// temp & delays: 1000millis = 1 sec
const int   DELAY                 = 100;   // 0.1 seconde (loop x10 = +-1sec delay)
const float TEMP_INCR             = 0.5;   // Tel bij targetTemp UP/DOWN
const float MINIMUM_TARGET        = 10.0;  // Minimum toegelaten targetTemp
const float MAXIMUM_TARGET        = 25.0;  // Maximum toegelaten targetTemp
const float TEMP_DANGER_C         = 1;     // temp + danger = ALERT
const float TEMP_DANGER_H         = 2;     // temp + dangerheat = ALERT
const int   REDIRECT_TIMEOUT      = 15000; // Terug naar hoofdscherm *5=Timeout (millis)
const int   BETWEEN_MSG_INCR      = 10;    // Tel bij targettimebetweenmsg (minuten)

// Compressor bescherming
const unsigned int coolMinOff     = 300;   // minimum compressor off time, seconds (5 min)
const unsigned int coolMinOn      = 90;    // minimum compressor on time, seconds (1.5 min)
unsigned long startTime           = 0;     // voor handaving coolminoff/on
unsigned long stopTime            = 0;     // voor handaving coolminoff/on
double runTime                    = 0;     // voor handaving coolminoff/on

// Pins
const int   RO_LEFT               = D3;    // Draai links
const int   RO_RIGHT              = D4;    // Draai rechts
const int   RO_PUSH               = D5;    // Drukknop
const int   COOLING_PIN           = D8;    // Koeler (led-blauw of 220V)
const int   HEATING_PIN           = D7;    // Verwarming (led-rood of 220V)
const int   TEMP_PIN              = D6;
uint8_t sensor2[8] = {0x28, 0x76, 0xCD, 0xF0, 0x3A, 0x19, 0x01, 0x3F};
uint8_t sensor1[8] = {0x28, 0xAA, 0xD0, 0x6D, 0x59, 0x14, 0x01, 0xA2};

// BUTTON-waardes
const int   BUTTON_NONE           = 0; 
const int   BUTTON_RIGHT          = 1;
const int   BUTTON_UP             = 2;
const int   BUTTON_DOWN           = 3;
const int   BUTTON_LEFT           = 4;

// STATUS-waardes
const int   STATE_COOLING         = 0;
const int   STATE_INACTIVE        = 1;
const int   STATE_HEATING         = 2;
const int   STATE_DANGER          = 3600000; // stuur alert als 30min in zelfde status

// LCDdisplay waardes = "DISP"
const int   DISPLAY_SUMMARY           = 0;
const int   DISPLAY_IP                = 1;
const int   DISPLAY_STATUS_MAX        = 2;
const int   DISPLAY_TEMP_HISTORY      = 3;
const int   DISPLAY_SET_TARGET        = 4;
const int   DISPLAY_SET_MSG_TIME      = 5;
const int   DISPLAY_RESET_WIFI        = 6;
const int   DISPLAY_RESET_EEPROM      = 7;
const int   DISPLAY_RESET_NODEMCU     = 8;
const int   DISPLAY_STATUS_AUTO       = 9;
const int   DISPLAY_STATUS_HEAT       = 10;
const int   DISPLAY_STATUS_COOL       = 11;
const int   DISPLAY_SEND_STATUSMSG    = 12;
const int   NO_OF_LCD_STATES          = 13;

// Omzetten millis/seconden naar leesbare vorm
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN)
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY) 

// Initialiseer librarys 
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);
RotaryFullStep rotary(RO_LEFT, RO_RIGHT, true, 50);
Adafruit_ADXL345_Unified tilter = Adafruit_ADXL345_Unified(12345);
OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);
WiFiManager wifiManager;
ESP8266WebServer server(80);

#endif // CONFIG

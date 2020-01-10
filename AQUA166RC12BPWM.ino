//**************************************************************************************************
// Cette esquisse est compatible avec l'AQUASHIELDS d'OLEG
// versions 3 et supérieures et le programme AquaCont_V1_4Х_Oleg_mod.ino
// version AQUA166 2016.01.23, la ligne du programme Text_table [185]

//**************************************************************************************************
#include <UTFT.h>
#include <UTouch.h>
#include <Wire.h>
#include <EEPROM.h>
#include "writeAnything.h"
#include <DS1307new.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SdFat.h>

#include <Boards.h>
#include <Firmata.h>
#include <FirmataConstants.h>
#include <FirmataDefines.h>
#include <FirmataMarshaller.h>
#include <FirmataParser.h>

//**************************************************************************************************
// Inverser le signal PWM si nécessaire
// true pour le pilote dont le 0 sur l'entrée PWM correspond à 0% de la luminosité
// false pour le pilote dont le 0 sur l'entrée PWM correspond à 100% de la luminosité
boolean RECOM_RCD = true;               // For Mean Well drivers change "true" to "false"
//boolean RECOM_RCD = false;

//**************************************************************************************************
// Inverse les sorties de la minuterie et la sortie du chauffage (pour contrôler le niveau bas actif)
boolean OUT_INVERSE = true;   // Décommenter pour un contrôle standard
//boolean OUT_INVERSE = false;   // Décommenter pour le contrôle inverse

//**************************************************************************************************
boolean ButtonPress = false;
#define NO false        // deux lignes dans le bouton

//*********************** AMENDEMENT COORDONNÉ ÉPAISSEUR CACHÉ ******************************************
//#define Mirror_X   //  Pour les écrans avec une coordonnée x inversée
//#define Mirror_Y   //  Pour les écrans avec des coordonnées de l'axe Y inversées

//********************* TEMPS DE PAUSE POUR LES MINUTEURS 1 ET 3 À L'ALIMENTATION **********************(*******
byte Tpause = 120;   //     120 - dix minutes

//*****************************CHOISIR LA SÉLECTION DU TYPE ***************************************************

//#define Aqua_shield_v2        // Ne pas commenter lors de l'utilisation d'AQUASHILD V2,
#define Aqua_shield_v3          // Ne pas commenter lorsque vous utilisez AQUASHILD V3 ou plus ancien

//*******************************SÉLECTION DU TYPE ET DISPONIBILITÉ DU CAPTEUR PH **********************************
//#define ADC_1115                // Décommenter s'il y a un capteur РН sur l'ADC 1115
//#define PH_sensor_I2C         // Décommenter s'il y a un capteur РН sur le bus I2C
#define Analog_PH_sensor      // Décommenter s'il y a un capteur РН avec un module analogique

//*************************CHOIX DE LA MER - HARE ****************************************************
#define freshwater              // Décommenter pour Presnya
//#define seawater              // Uncomment pour la mer

//*********************** Résolution PWM 11 ou 12 bits *******************************************
#define Timers_11bit        // 0-1985 value for standart Shield or Aqua_shield_v3, 11bit PWM for all colour
//#define Timers_12bit          // 0-4000 value for standart Shield or Aqua_shield_v3, 12bit PWM for all colour 

//************************************************************************************************
//mixzt
#define MAX_LEVEL                 500

#ifdef Timers_12bit
#define MAX_LINEAR_LEVEL        3984
#endif

#ifdef Timers_11bit
#define MAX_LINEAR_LEVEL        1992
#endif

unsigned long msec_offset;
byte sec_tmp;
//mixzt

//****************************** Fréquence PWM  ***************************************************
#ifdef Timers_11bit
byte PWM_FRQ_Value = 2;         // PWM_FRQ_Value=2    11bit PWM Frequency = 1khz for all colours exept Moon
                                // PWM_FRQ_Value=3    11bit PWM Frequency = 125hz
#endif

byte PWM_FRQ_Value_Fan = 5;     // PWM_FRQ_Value_Fan=5 PWM Frequency = 30 Hz for Fans
                                // PWM_FRQ_Value_Fan=4 PWM Frequency = 122 Hz
//***********************************************************************************************

//********************** Couleurs de canal **********************************************************

byte rgbCh0[] = {238, 236, 225};  //  Blanc chaud   |
byte rgbCh1[] = {255, 255, 255};  //  Blanc froid   |
byte rgbCh2[] = {79, 129, 189};   //  Bleu          |
byte rgbCh3[] = {192, 80, 77};    //  Rouge         |
byte rgbCh4[] = {128, 100, 162};  //  Violet        |
byte rgbCh5[] = {228, 150, 70};   //  Orange        |
byte rgbCh6[] = {155, 187, 89};   //  Vert          |
byte rgbCh8[] = {216, 216, 216};  //  Lune          |

//************************************************************************************************
UTFT myGLCD(ILI9341_16, 38, 39, 40, 41);     // Écran 240x320
UTouch myTouch(6, 5, 4, 3, 2);

byte xdate;                               // variable de date
int rtcSetMin, rtcSetHr, rtcSetDy, rtcSetMon, rtcSetYr, rtcSetSec, rtcSetDw;

// аналоговые часы
int clockCenterX = 159;                   // coordonnée horizontale
int clockCenterY = 119;                   // coordonnée verticale
int oldsec = 0;

int displayDOW = 0;                        // Hide=0 || Show=1 (change in prog)
int yTime;                                 // Setting clock stuff

int timeDispH, timeDispM, xTimeH, xTimeM10, xTimeM1, xColon;
String time, day;

int setClockOrBlank = 0;             // Clock Screensaver=0 || Blank Screen=1 (change in prog)
int setScreensaverOnOff = 0;         // OFF=0 || ON=1 Turns it ON/OFF (change in prog)
int setScreensaverDOWonOff = 0;      // OFF=0 || ON=1 Shows/Hides DOW in Screensaver (change in prog)
int setScreensaverTupe = 0;          // type d'économiseur d'écran

int SS_DOW_x;                        // Moves the DOW to correct position
int setSSmintues;                    // Time in (minutes) before Screensaver comes on (change in program)
int TempSSminutes;                   // Temporary SetSSminutes used in calcs and prints, etc.
int setScreenSaverTimer;             // how long in (minutes) before Screensaver comes on (change in program)
int screenSaverCounter = 0;          // counter for Screen Saver
boolean SCREEN_RETURN = true;        // Auto Return to mainScreen() after so long of inactivity
int returnTimer = 0;                 // counter for Screen Return
int setReturnTimer;                  // Return to main screen 75% of time before the screensaver turns on

// декларируем шрифты
extern uint8_t SmallFont[];           // маленький шрифт
extern uint8_t BigRusFont[];          // большой шрифт
extern uint8_t DotMatrix_M_Num[];     // большой матричный шрифт (только цифры)
extern uint8_t SevenSegNumFont[];     // большой шрифт
extern uint8_t RusFont1[];            // маленькая кириллица
extern uint8_t RusFont2[];            // большая кириллица
extern uint8_t RusFont3[];            // средняя кириллица
extern uint8_t RusFont6[];            // средняя кириллица (Small Font)

float linhaR;
float linhaG;
float linhaB;

// true - activé par défaut, false - désactivé
#define LARGE false     // grande police
#define SMALL true    // petite police (par défaut)

#define BlUE_BAC true // bleu (par défaut)
#define GREEN_BAC false // vert

//************************* Define for 11bit timer *********************************
#ifndef cbi_mix
#define cbi_mix(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi_mix
#define sbi_mix(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Résolution de 11 bits, peut travailler sur les contacts(2, 3, 5, 6, 7, 8, 11, 12, 13, 44, 45, 46)

//************************* Brochage  *********************************

// ------------- FOR AQUA SHIELD V2 Brochage pour Oleg Akvashild-------------------------------
#ifdef Aqua_shield_v2

// ВХОДЫ
#define SensLight         A13 // capteur de lumière, sur broche analogique
#define SensLevel         A12 // capteur de niveau d'eau, sur broche analogique 
OneWire OneWireBus(A15);      // Датчик температуры  (Dallas pin)

// ВЫХОДЫ
#define ledPinWarmWhite   7   // WWT  Теплый белый    (2) - 11 бит   
#define ledPinCoolWhite   8   // CWT  Холодный белый  (3) - 11 бит
#define ledPinRoyBlue     11   // RBL  Глубой          (5) - 11 бит
#define ledPinRed         12   // RED  Красный         (6) - 11 бит
#define ledPinUV          13   // UVL  Фиолетовый      (7) - 11 бит
#define ledPinOrange      44   // ORG  Оранжевый       (8) - 11 бит
#define ledPinGr          45   // GRN  Зеленый        (11) - 11 бит
#define ledPinMoon        46   // Moon Led pin     (4) - ( луна 8 бит - 255 )
#define LCDbrightPin      47   // подсветка LCD   (13) - (8 бит)
#define tempHeatPin        A10    // A5   Нагреватель вкл./выкл. 
#define tempChillPin       A11    // 46   Холодильник вкл./выкл. 
#define tempAlarmPin        7    // A11  Buzzer Alarm 
#define autoFeeder         A12    // А7   Пин кормушки
#define Heatsink1_FansPWM  48    // 44   Вентилятор на радиаторе 1 
#define Heatsink2_FansPWM  49     // 9    Вентилятор на радиаторе 2 
// Таймеры
#define timer1             A0    // pin analog
#define timer2             A1    // pin analog 
#define timer3             A2    // pin analog
#define timer4             A3    // pin analog
#define timer5             A4    // pin analog
// Дозаторы
#define pump1              A5    // pin analog A5          Дозатор1
#define pump2              A6    // pin analog A6          Дозатор2
#define pump3              A7    // pin analog A10         Дозатор3
#define pump4              A8    // pin analog A14         Дозатор4
#define vacpump            A9   // pin analog A6  Вакуумный насос
#define SDchipSelect       53 // SD card attached to SPI bus
//ШИМ управление помпами (Jebao PWM Pump)
#define PWMPinA             9         // шим пин для помпы 1 (Pump 1)
#define PWMPinB            10         // шим пин для помпы 2 (Pump 2)
#endif


// Contacts occupés(50-53 for SD Card)  myTouch(42, 49, 47, 48, 43)
// Résolution de 11 bits, peut travailler sur les contacts(2, 3, 5, 6, 7, 8, 11, 12, 13, 44, 45, 46)
// ------------- FOR AQUA SHIELD V3.7 -------------------------------------
#ifdef Aqua_shield_v3

// ВЫХОДЫ ШИМ на светильник
#define ledPinGr           7     // GRN  Vert         arduino port 2,  shield sch pwm0  //пин 25 аквашилд 3.7
#define ledPinUV           8     // UVL  Violet       arduino port 3,  shield sch pwm1  //пин 23 аквашилд 3.7
#define ledPinMoon         9     // Moon La lune      arduino port 4,  shield sch pwm8  //пин 26 аквашилд 3.7
#define ledPinCoolWhite    10    // CWT  Blanc froid  arduino port 5,  shield sch pwm4  //пин 15 аквашилд 3.7
#define ledPinRed          11    // RED  Rouge        arduino port 6,  shield sch pwm5  //пин 13 аквашилд 3.7
#define ledPinRoyBlue      12    // RBL  Bleu         arduino port 7,  shield sch pwm3  //пин 17 аквашилд 3.7
#define ledPinWarmWhite    13    // WWT  Blanc chaud  arduino port 12, shield sch pwm2  //пин 21 аквашилд 3.7 

// il faudra déporter l'ecran pour réaliser le montage via une pull-up sur le 5 volts, controle bright via soft
#define LCDbrightPin       42    // illumination LCD   (13) - voir le message n ° 3695, 369    

// pas utilisée pour le moment
#define ledPinOrange       43    // ORG  Orange      arduino port 11, shield sch pwm7  //пин 9  аквашилд 3.7
 

// Capteur de température (broche Dallas)
OneWire OneWireBus(A15);         // Sonde de témpérature en cascade (3) Eau, ventilo 1 et 2

// SD card
#define SDchipSelect       53		 // SD card attached to SPI bus

//Ventilateurs de refroidissement
#define Heatsink1_FansPWM  44     // 44 // pin 40 aquashield 3.7 // Ventilateur sur le radiateur 1 (via l'interrupteur d'alimentation FAN-PWM0)
#define Heatsink2_FansPWM  45     // 45 // pin 34 aquashield 3.7 // Ventilateur sur le radiateur 2 (via l'interrupteur d'alimentation PWS-OUT2)
#define tempChillPin       46     // 46 // pin 34 aquashield 3.7 // Ventilateur sur le radiateur 0 (via l'interrupteur d'alimentation PWS-OUT0)

// Alarme
#define tempAlarmPin       A11     // A11  //Buzzer Alarm (внутренний сигнал)

// Timers (sortie)
#define timer1             A0 // broche analogique 0 // broche 6 aquashield 3.7 // Aération
#define timer2             A1 // broche analogique 1 // broche 8 aquashield 3.7 // CO2
#define timer3             A2 // broche analogique 3 // broche 12 aquashield 3.7 // Filtre
#define timer4             A3 // broche analogique 2 // broche 10 aquashield 3.7 // Lampe UV
#define timer5             A4 // broche analogique 4 // broche 14 aquashield 3.7 // Topping

// Sorties supplémentaires
#define tempHeatPin        A5 // Chauffage
#define vacpump            A6 // Pompe d'évacuation de la réserve (eventuellement pour remplissage cuve
#define autoFeeder         A7 // Distributeur de nourriture

// Distributeurs (sortie)
#define pump1              47 // pin 10            // pin 36 aquashild 3.7 // Dispenser1, (via l'interrupteur d'alimentation FAN-PWM1)
#define pump2              48 // pin analog A9     // pin 30 aquashild 3.7 // Dispenser2, (via l'interrupteur d'alimentation PWS-OUT0)
#define pump3              49 // pin analog A12    // pin 39 aquashild 3.7 // Dispenser3, (via l'interrupteur d'alimentation PWS-OUT4)
#define pump4              50 // pin analog A14    // pin 32 aquashild 3.7 // Dispenser4, (via l'interrupteur d'alimentation PWS-OUT3)

//  Capteurs (entrée)
#define SensLevel          A8 // capteur de niveau d'eau
#define PHSens             A9 // capteur PH analogique (DO NOT RANGE)
// # définir XXXX         A13 // broche analogique 13

// Commande de pompe PWM (Pompe Jebao PWM)
#define PWMPinA            51 // pin 8 // Pompe de brassage 1
#define PWMPinB            52 // pin 9 // Pompe de Brassage 2
// ^^^^^^^^^^ Lorsque vous utilisez Aquashield, un moyen direct de prendre avec mega ^^^^^^^^^^
#endif

int sensorValue;
//const int maxModos = 6;  // nombre de pas pour le commutateur de mode
int Mode1, Mode2, Mode3, Mode4, Mode5, Mode6;
int cmode = 0;     // modes de commutation
int ModeSel = 6;   // 6 mode de fonctionnement par défaut - les pompes sont éteintes
int SpeedPump;     // valeur de vitesse convertie

long times = 0;         // 0
//int ValPoten = 0;  // résistance variable

long starttime = 0;
int startvalue = 255;
//byte periodtime = 50000;   // temps pour passer en mode 3
//int value = 0;     // valeur de niveau
int Pump1PWM = 0;    // la valeur de niveau pour la pompe 1 = 0
int Pump2PWM = 0;

int Pump1 = 0;        //la valeur de niveau pour la pompe 1 = 0
int Pump2 = 0;

byte MinPower = 0;
byte MaxPower = 0;

int maxP1;      // par défaut pour la pompe 1 min de puissance
int minP1;      // par défaut pour la pompe 1 puissance maximale
int maxP2;      // par défaut pour la pompe 2 min de puissance
int minP2;      // par défaut pour la pompe 2 puissance maximale

byte value = 0;    // valeur de niveau
int periode;       // la valeur des périodes pour le mode normal (dans setup-5000}

int aclock;
int sec, sec1, sec2, sec3, sec4;
//int waterlevel = 0;
int waterlevel;

// *************la valeur des périodes pour le mode normal (dans setup-5000}***************************//

#define PHADDRESS 0x4D // Adresse du capteur de PH
int RoomTempI2CAddress = B1001011; // adresse du capteur de température sur le module PH

double SetvalPH;
float volt7 = 0.6852;                     // Tension d'étalonnage du capteur PH
float volt10 = 0.3842;
byte co = 1;
float calibrationTempC = 20;

double phVolt;

double voltsPerPH;

double realPHVolt;
double phUnits;
double measuredPH;

double roomTempC;
double roomTempCompensatedMeasuredPH;
int sampleSize = 200;

double avgMeasuredPH = 0;
double avgRoomTempC = 0;
double avgPHVolts = 0;
double avgRoomTemperatureCompensatedMeasuredPH = 0;

double tempAdjusted10;

float adjustPHBasedOnTemp(float PH, float temp)
{
  // http://www.omega.com/Green/pdf/pHbasics_REF.pdf
  // When the temperature is other than 25degC and the ph is other than 7
  // the temperature error is 0.03ph error/ph unit/10degC
  // which means error = 0.03*(ph away from 7)*(tempdiffC/10)

  float phDifference = abs(PH - 7);
  float tempDifferenceC = abs(temp - 25);
  float phAdjust = (0.03 * phDifference) * (tempDifferenceC / 10);

  if (PH > 7 && temp < 25)
    phAdjust = phAdjust;

  if (PH > 7 && temp > 25)
    phAdjust = phAdjust * -1;

  if (PH < 7 && temp > 25)
    phAdjust = phAdjust;

  if (PH < 7 && temp < 25)
    phAdjust = phAdjust * -1;

  float tempAdjustedPH = PH + phAdjust;
  return tempAdjustedPH;
}

double getPHVolts()
{
  byte ad_high;
  byte ad_low;

  Wire.requestFrom(PHADDRESS, 2);        //requests 2 bytes
  while (Wire.available() < 2);          //while two bytes to receive

  ad_high = Wire.read();
  ad_low = Wire.read();
  double units = (ad_high * 256) + ad_low;

  double volts =  (units / 4096) * 3;
  return volts;
}

double getRoomTemperatureC()
{
  Wire.requestFrom(RoomTempI2CAddress, 2);
  byte MSB = Wire.read();
  byte LSB = Wire.read();

  int TemperatureSum = ((MSB << 8) | LSB) >> 4;
  double celsius = TemperatureSum * 0.0625;

  return celsius;
}

void SetRoomTemperatureResolutionBits(int ResolutionBits)
{
  if (ResolutionBits < 9 || ResolutionBits > 12) exit;
  Wire.beginTransmission(RoomTempI2CAddress);
  Wire.write(B00000001);                     //addresses the configuration register
  Wire.write((ResolutionBits - 9) << 5);     //writes the resolution bits
  Wire.endTransmission();

  Wire.beginTransmission(RoomTempI2CAddress); //resets to reading the temperature
  Wire.write((byte)0x00);
  Wire.endTransmission();
}

// *************Travailler avec un capteur PH sur l'ADC 1115***************************//
uint8_t adc_addr;

//calibration defines
int adc_low = 0x8000F;    //adc_low : for calibrate insert here adc value for LOW buffer
int adc_high = 0x7FFF;    //adc_high : for calibrate insert here adc value for HIGH buffer
long ph_buf_low = 0L;     //buffer low 4.01, change if you have different low buffer value
long ph_buf_high = 1400L; //buffer high 9.18, change if you have different high buffer value

long ph;
int adc;
byte highbyte, lowbyte;

// find ADC chip address on i2c bus
uint8_t i2c_adr_find(uint8_t adr_begin, uint8_t adr_end)
{
  for (uint8_t i = adr_begin; i <= adr_end; i++)
  {
    if (Wire.requestFrom(i, 1) != 0)
    {
      return i;
    }
  }
  return 0;
}

//ADC initialisation & gain setting
void adc_init(uint8_t i)
{
  Wire.beginTransmission(i); // ADC inint
  Wire.write(0b00000001); //Select register address
  Wire.write(0b10001010); //OS MUX2 MUX1 MUX0 PGA2 PGA1 PGA0 MODE ****** 0b10001010 = diff ch0-ch1, gain(spain) +- 0.256V
  Wire.write(0b00100011); //DR2 DR1 DR0 COMP_MODE COMP_POL COMP_LAT COMP_QUE1 COMP_QUE0 ****** DR2 DR1 DR0 = 001 (16 measurements per second)
  Wire.endTransmission();
  Wire.beginTransmission(i);
  Wire.write(0b00000000); //Select data address
  Wire.endTransmission();
}

//кормушка

byte feedTime;
byte FEEDTime1, FEEDTime2,
     FEEDTime3, FEEDTime4;
byte ManFeed = 0;
byte FeedPause = 0;

byte feedFish1H, feedFish1M,          //Times to feed the fish
     feedFish2H, feedFish2M,
     feedFish3H, feedFish3M,
     feedFish4H, feedFish4M;

boolean FeedWaveCtrl_0 = false;
boolean FeedWaveCtrl_1 = false;
boolean FeedWaveCtrl_2 = false;
boolean FeedWaveCtrl_3 = false;
boolean FeedWaveCtrl_4 = false;
boolean FeedWaveCtrl_5 = false;

int AM_PM;

int  xTimeAMPM;
byte setAutoStop = 0;
byte fiveTillBackOn0, fiveTillBackOn1, fiveTillBackOn2,
     fiveTillBackOn3, fiveTillBackOn4, fiveTillBackOn5;
//дозатор удо

byte dozTime;
byte DOZTime1, DOZTime2,
     DOZTime3, DOZTime4;

byte dozPump1H, dozPump1M,
     dozPump2H, dozPump2M,
     dozPump3H, dozPump3M,
     dozPump4H, dozPump4M;

byte numDoz1, numDoz2, numDoz3, numDoz4,
     intDoz1, intDoz2, intDoz3, intDoz4;

int dozVal1, dozVal2, dozVal3, dozVal4;
int dozPart1, dozPart2, dozPart3, dozPart4;
int dozCal1, dozCal2, dozCal3, dozCal4;
int DosSec1, DosSec2, DosSec3, DosSec4;
int CalMode;

byte shiftH12, shiftH13, shiftH14;
byte shiftH22, shiftH23, shiftH24;
byte shiftH32, shiftH33, shiftH34;
byte shiftH42, shiftH43, shiftH44;

int Doscalibrate1, Doscalibrate2 , Doscalibrate3 , Doscalibrate4;

// oneWire reference to Dallas Temperature.
DallasTemperature sensors(&OneWireBus);

// SD карта
SdFat sd;
SdFile myFile;
#define error(s) sd.errorHalt_P(PSTR(s))

// Assign the addresses of temperature sensors.  Add/Change addresses as needed.
// ROM = 28 14 4F CC 3 0 0 D0
DeviceAddress tempDeviceAddress;
DeviceAddress Heatsink1Thermometer; // capteur de température du radiateur 1
DeviceAddress Heatsink2Thermometer; // capteur de température du radiateur 2
DeviceAddress waterThermometer;     // capteur de température de l'eau dans l'aquarium

byte  resolution = 11; // résolution du capteur
unsigned long lastTempRequest = 0;
int  delayInMillis = 0;
//unsigned long logtempminutoantes = 0;  // Pour enregistrer le journal de température sur une carte flash

byte counterB1 = 0;          // compteur 1
byte counterB2 = 0;          // compteur 2
byte counterB3 = 0;
byte numberOfDevices = 0;    // quantity of Dallas sensor connected to board

unsigned long previousMillis1sec;

float tempW = 0;                          // Capteur de température dans l'aquarium
float tempH1 = 0;                         // Capteur de température sur le radiateur 1
float tempH2 = 0;                         // Capteur de température sur le radiateur 2

float setTempToBeginHeatsink1FanC = 0.0;  // Temperature to Turn on Heatsink1 Fans (in Degrees C)
float setTempToBeginHeatsink2FanC = 0.0;  // Temperature to Turn on Heatsink2 Fan (in Degrees C)
int  setTempToSoundAlarmC = 0;            // Temperature to Heatsink Sound alarm (in Degrees C)

float temp2beHFan;                   // Temporary Temperature Values
float temp2beSFan;                   // Temporary Temperature Values

float FanOn = 0.2;                   // Starts Fan(s) at 20% Duty Cycle (choose 0.2 or higher)

int Heatsink1TempInterval = 0;       // Used for PWM Duty calculations
int Heatsink2TempInterval = 0;       // Used for PWM Duty calculations
byte Heatsink1PWM = 0;               // Used for PWM Duty calculations
byte Heatsink2PWM = 0;               // Used for PWM Duty calculations
byte HeatsinkLoPWM = 5;
byte HeatsinkHiPWM = 100;
byte HeatLoPWM;
byte HeatHiPWM;

byte CoolFanPWM = 100;               // Refroidisseur PWM
byte CoolFanMIN = 0;                 // Vitesse minimale du ventilateur 0 - désactivée
byte HiPWM;
byte LowPWM;

float setTempC = 0.0;                // Desired Water Temperature (User input in program)
float offTempC = 0.0;                // Desired Water Temp. Offsets for Heater & Chiller (User input in program)
float alarmTempC = 0.0;              // Temperature the Alarm will sound (User input in program)
boolean tempCoolflag = 0;            // 1 if cooling on
boolean tempHeatflag = 0;            // 1 if heating on
boolean tempAlarmflag = 0;           // 1 if alarm on

boolean tempAlarmflagH1 = 0;         // 1 if alarm on
boolean tempAlarmflagH2 = 0;         // 1 if alarm on
boolean AlarmflagON ;
boolean RefreshAfterError = false;

float temp2beS;                      // Temporary Temperature Values
float temp2beO;                      // Temporary Temperature Values
float temp2beA;                      // Temporary Temperature Values

float MaxTempW;       // température maximale pour le capteur d'eau
float MaxTempH1;      //  --//--               pour capteur sur le radiateur 1
float MaxTempH2;      //  --//--               pour capteur sur le radiateur 2

int DimmL = 0;                       // gradation manuelle 1-oui, 0-non
int setLEDsDimPercentL = 0;
int TempsetLEDsDimPercentL;
float PercentDimL = 0.0;

// int ledON = 1; // Lorsque les préréglages sont définis, les canaux continuent à fonctionner en mode nominal

byte setDimLEDsOnOff = 0;             // If/When LEDs reach a certain temp they can dim down (feature off by default)
byte setLEDsDimTempC = 0;             // Default value is 0, set this value in program
byte TempLEDsDimTemp;                 // Temporary LED Dimming Temp
byte setLEDsDimPercent = 0;           // Choose value to failsafe dim LEDs in program
byte TempLEDsDimPercent;              // Temporary LED Dimming Percent
float PercentDim = 0.0;               // Converts saved value in EEPROM to a percentage
//int tempLED=setLEDsDimTempC+5;
float PercentSoftStart = 0.0;         // change from 0 to 1 with 0.1 increment, every 5sec after programm is started

extern unsigned int up[0x5D0];        // flèche vers le haut pour les minuteurs
extern unsigned int down[0x5D0];      // flèche vers le bas pour les minuteurs
extern unsigned int clos[0x240];      // croix (fermer)
extern unsigned int preset[0x195];    // preset

// Картинки фаз луны  http: //arduino.cc/forum/index.php/topic,134649.0.html
uint_farptr_t MoonPic;              // Pointer to the Lunar Phase Pic
extern unsigned int Full_Moon[0xAF9];         // Lunar Phase Pics
extern unsigned int First_Quarter[0xAF9];

float LC = 29.53059;       // 1 Lunar Cycle = 29.53059 days
String LP;                 // LP = Lunar Phase - variable used to print out Moon Phase
double AG;
byte tMaxI;                // Maximum Illumination of Moon (User Defined/Set in Prog. -- Default = 0)
byte tMinI;                // Minimum Illumination of Moon (User Defined/Set in Prog. -- Default = 0)
byte MinI = 5;             // luminosité minimale de la lune par défaut
byte MaxI = 90;            // le maximum

float lunar_perc;

//================================= Текстовые надписи в printHeader (верхний баннер) ========================================
PROGMEM const char Header_Text_string0[] = "MENU PRINCIPAL";                          // ГЛАВНОЕ МЕНЮ
PROGMEM const char Header_Text_string1[] = "HEURE ET DATE";              // УСТАНОВКА ВРЕМЕНИ И ДАТЫ
PROGMEM const char Header_Text_string2[] = "REGLAGE DE LA TEMPERATURE";            // УСТАНОВКА ТЕМПЕРАТУРЫ ВОДЫ
PROGMEM const char Header_Text_string3[] = "TEST DE CANAL AUTOMATIQUE";   // АВТОМАТИЧЕСКОЕ ТЕСТИРОВАНИЕ КАНАЛОВ
PROGMEM const char Header_Text_string4[] = "TEST ECLAIRAGE MANUEL";                 // РУЧНОЙ ТЕСТ ОСВЕЩЕНИЯ
PROGMEM const char Header_Text_string5[] = "CONFIGURATION DES CANAUX PAR COULEURS";                       // УСТАНОВКА ЦВЕТА
PROGMEM const char Header_Text_string6[] = "REGLAGES COULEUR LUNAIRE";                // УСТАНОВКА ЯРКОСТИ ЛУНЫ
PROGMEM const char Header_Text_string7[] = "FONCTIONNEMENT DE REFROIDISSEMENT";              // РЕЖИМ РАБОТЫ ОХЛАЖДЕНИЯ
PROGMEM const char Header_Text_string8[] = "AJUSTEMENT DES DOSEURS";                   // НАСТРОЙКА ДОЗАТОРОВ
PROGMEM const char Header_Text_string9[] = "AJUSTER LE PH";          // НАСТРОЙКА ПОДАЧИ УГЛЕКИСЛОТЫ
PROGMEM const char Header_Text_string10[] = "REGLAGES PRINCIPAUX, PAGE 1";             // ГЛАВНЫЕ НАСТРОЙКИ, СТР. 1
PROGMEM const char Header_Text_string11[] = "REGLAGES PRINCIPAUX, PAGE 2";             // ГЛАВНЫЕ НАСТРОЙКИ, СТР. 2
PROGMEM const char Header_Text_string12[] = "REGLAGES PRINCIPAUX, PAGE 3";             // ГЛАВНЫЕ НАСТРОЙКИ, СТР. 3
PROGMEM const char Header_Text_string13[] = "RETROECLAIRAGE ECRAN";             // НАСТРОЙКА ЯРКОСТИ ЭКРАНА (подсветка)
PROGMEM const char Header_Text_string14[] = "REGLAGE TEMPERATURE FANS"; // УСТАНОВКА ТЕМПЕРАТУРЫ ВКЛ. ВЕНТИЛЯТОРОВ
PROGMEM const char Header_Text_string15[] = "REDUCTION LUMINOSITE SI SURCHAUFFE";     // УМЕНЬШЕНИЕ ЯРКОСТИ ПРИ ПЕРЕГРЕВЕ
PROGMEM const char Header_Text_string16[] = "REGLAGE DE L'ECRAN";           // УСТАНОВКИ ХРАНИТЕЛЯ ЭКРАНА
PROGMEM const char Header_Text_string17[] = "REGLAGE DE MINUTERIE";                     // УСТАНОВКИ ТАЙМЕРОВ
# ifdef freshwater
PROGMEM const char Header_Text_string18[] = "REGLAGE TIMER AERATION";          // УСТАНОВКИ ТАЙМЕРА 1 АЭРАЦИЯ
PROGMEM const char Header_Text_string19[] = "REGLAGE TIMER CO2";              // УСТАНОВКИ ТАЙМЕРА 2 СО2
PROGMEM const char Header_Text_string20[] = "REGLAGE TIMER FILTRE";           // УСТАНОВКИ ТАЙМЕРА 3 ФИЛЬТР
PROGMEM const char Header_Text_string21[] = "REGLAGE TIMER LAMPE";         // УСТАНОВКИ ТАЙМЕРА 4 УФ ЛАМПА
PROGMEM const char Header_Text_string22[] = "REGLAGE TIMER PLUS LONG";            // УСТАНОВКИ ТАЙМЕРА 5 ДОЛИВ
# endif
# ifdef seawater
PROGMEM const char Header_Text_string18[] = "REGLAGE TIMER 1";                  // УСТАНОВКИ ТАЙМЕРА 1
PROGMEM const char Header_Text_string19[] = "REGLAGE TIMER 2";                  // УСТАНОВКИ ТАЙМЕРА 2
PROGMEM const char Header_Text_string20[] = "REGLAGE TIMER 3";                  // УСТАНОВКИ ТАЙМЕРА 3
PROGMEM const char Header_Text_string21[] = "REGLAGE TIMER 4";                  // УСТАНОВКИ ТАЙМЕРА 4
PROGMEM const char Header_Text_string22[] = "REGLAGE TIMER 5";                  // УСТАНОВКИ ТАЙМЕРА 5
# endif
PROGMEM const char Header_Text_string23[] = "GESTION MANUELLE DES TIMERS";          // РУЧНОЕ УПРАВЛЕНИЕ ТАЙМЕРАМИ
PROGMEM const char Header_Text_string24[] = "RECHERCHE AUTOMATIQUE SONDES TEMPERATURE";       // АВТОПОИСК ДАТЧИКОВ ТЕМПЕРАТУРЫ
PROGMEM const char Header_Text_string25[] = "SAVE, LOAD SETTINGS";       // СОХРАНИТЬ, ЗАГРУЗИТЬ НАСТРОЙКИ
PROGMEM const char Header_Text_string26[] = "REGLAGES HORAIRE DOSEURS";          // УСТАНОВКА ВРЕМЕНИ ДОЗАТОРОВ
PROGMEM const char Header_Text_string27[] = "REGLAGE TIMER FEEDER";          // УСТАНОВКА ВРЕМЕНИ КОРМЛЕНИЯ
PROGMEM const char Header_Text_string28[] = "REGLAGE DE LA POMPE";               // НАСТРОЙКА ПОМП ТЕЧЕНИЯ

const PROGMEM char* const PROGMEM Header_Text_table[]  = {
  Header_Text_string0, Header_Text_string1, Header_Text_string2, Header_Text_string3, Header_Text_string4,
  Header_Text_string5, Header_Text_string6, Header_Text_string7, Header_Text_string8, Header_Text_string9,
  Header_Text_string10, Header_Text_string11, Header_Text_string12, Header_Text_string13, Header_Text_string14,
  Header_Text_string15, Header_Text_string16, Header_Text_string17, Header_Text_string18, Header_Text_string19,
  Header_Text_string20, Header_Text_string21, Header_Text_string22, Header_Text_string23, Header_Text_string24,
  Header_Text_string25, Header_Text_string26, Header_Text_string27, Header_Text_string28,
};

//====================== Текстовые надписи в меню, хранящиеся во флеш ==========================
PROGMEM const char Text_string0[] = "HEURE:";                // ВРЕМЯ:
PROGMEM const char Text_string1[] = "DATE:";                 // ДАТА:
PROGMEM const char Text_string2[] = ""; // -----------------------------------------------------
PROGMEM const char Text_string3[] = "FORMAT";             // В ФОРМАТЕ (в меню часов)
PROGMEM const char Text_string4[] = "BLANK";                 // ПУСТО
PROGMEM const char Text_string5[] = "HORLOGE";                  // ЧАСЫ
PROGMEM const char Text_string6[] = "Type d'ecran";            // Тип экрана
PROGMEM const char Text_string7[] = "MONTRER LA DATE";       // ПОКАЗЫВАТЬ ДАТУ
PROGMEM const char Text_string8[] = "Oui";                    // ДА
PROGMEM const char Text_string9[] = "Non";                   // НЕТ
PROGMEM const char Text_string10[] = "Analo.";               // А.ЧАСЫ (аналоговые часы)
PROGMEM const char Text_string11[] = "Digit.";               // Ц.ЧАСЫ (цифровые часы)
PROGMEM const char Text_string12[] = "INFOS";                // Датч. D`rw. // ДАТЧ.
PROGMEM const char Text_string13[] = "EVENEMENTS";      // МОНИТОР СОБЫТИЙ
PROGMEM const char Text_string14[] = "SECTEUR HORAIRE";            // СЕКТОР ВРЕМЕНИ
PROGMEM const char Text_string15[] = "24HR";                  // 24ч.(формат)
PROGMEM const char Text_string16[] = "C";                    // С (цельсий)
PROGMEM const char Text_string17[] = "OK";                   // OK
PROGMEM const char Text_string18[] = "SET TEMP.";          // УСТАН.ТЕМП.
PROGMEM const char Text_string19[] = "Enregistrer les parametres";  // Сохранить Установки
PROGMEM const char Text_string20[] = "Parametres de telechargement";  // Загрузить Установки
PROGMEM const char Text_string21[] = "SET UP";            // НАСТРОИТЬ
PROGMEM const char Text_string22[] = "FISHPINO VERSION"; // УПРАВЛЕНИЕ АКВАРИУМОМ
PROGMEM const char Text_string23[] = "LUMINOSITE LEDS";      // ЯРКОСТЬ КАНАЛОВ
PROGMEM const char Text_string24[] = "PHASE LUNE";            // ФАЗА ЛУНЫ
PROGMEM const char Text_string25[] = "TEMPERATURE";            // ТЕМПЕРАТ. (RELOEP@RSP@)  (reloep.)
PROGMEM const char Text_string26[] = "OFF";                // (ВЫКЛ.)
PROGMEM const char Text_string27[] = "EAU:";                // ВОДА :  ТЕМП.ВОДЫ (RELO BND[:)
PROGMEM const char Text_string28[] = "FAN1";                // РАД:1   РАДИАТ Д1 (P@DH@R D1:)
PROGMEM const char Text_string29[] = "FAN2";                // РАД:2   РАДИАТ Д2 (P@DH@R D2:)
PROGMEM const char Text_string30[] = "**.*";               // Ошибка
PROGMEM const char Text_string31[] = "  COOLER  ";           // ОХЛАЖДЕНИЕ
PROGMEM const char Text_string32[] = " CHAUFFAGE ";          // НАГРЕВАТЕЛЬ
PROGMEM const char Text_string33[] = "     ALARME!";          // ТРЕВОГА !
PROGMEM const char Text_string34[] = "(JJ/MM/AAAA)";         // (ДД/MM/ГГГГ)  (день, месяц, год)
PROGMEM const char Text_string35[] = "CALENDRIER DE LA JOURNEE"; // ГРАФИК СВЕТОВОГО ДНЯ
PROGMEM const char Text_string36[] = "";    // РЕЗЕРВ
PROGMEM const char Text_string37[] = "Temperature specifiee:"; // Заданная Температура
PROGMEM const char Text_string38[] = "Offset Temperature"; // Гистерезис Температуры
PROGMEM const char Text_string39[] = "Alarme Offset";   // Гистерезис Тревоги
PROGMEM const char Text_string40[] = "Temps";                // Время
PROGMEM const char Text_string41[] = "Avancer";               // Вперед
PROGMEM const char Text_string42[] = "RETOUR";                // Назад
PROGMEM const char Text_string43[] = "TEST PROCESSUS";      // ТЕСТ В ПРОЦЕССЕ
PROGMEM const char Text_string44[] = "TEMPS";                 // ВРЕМЯ
PROGMEM const char Text_string45[] = "NIVEAUX DE SORTIES (0-100%)"; // УРОВНИ ВЫХОДОВ (0-100%)
PROGMEM const char Text_string46[] = "000";                    // 000 (три нуля)
PROGMEM const char Text_string47[] = "";    // РЕЗЕРВ
PROGMEM const char Text_string48[] = "";    // РЕЗЕРВ
PROGMEM const char Text_string49[] = "";    // РЕЗЕРВ
PROGMEM const char Text_string50[] = "LUNE";                         // ЛУНА
PROGMEM const char Text_string51[] = "LUMINOSITE ECRAN";     // ЯРКОСТЬ ПОДСВЕТКИ ЭКРАНА
PROGMEM const char Text_string52[] = "REDUIRE LUMINOSITE DE 50%";     // УМЕНЬШИТЬ ЯРКОСТЬ НА 50%
PROGMEM const char Text_string53[] = "PERIODE";                     // В ПЕРИОД
PROGMEM const char Text_string54[] = "TEMPS";                      // ВРЕМЕНИ
PROGMEM const char Text_string55[] = "C";                            // С
PROGMEM const char Text_string56[] = "DN";                           // ДО
PROGMEM const char Text_string57[] = "Ch.";                           // Ч.
PROGMEM const char Text_string58[] = "VITESSE MINIMUM DES FANS";          // МИНИМАЛЬНЫЕ ОБОРОТЫ ВЕНТИЛЯТОРОВ
PROGMEM const char Text_string59[] = "VITESSE MAXIMUM DES FANS";         // МАКСИМАЛЬНЫЕ ОБОРОТЫ ВЕНТИЛЯТОРОВ
PROGMEM const char Text_string60[] = "EAU DE REFROIDISSEMENT";                // ОХЛАЖДЕНИЯ ВОДЫ
PROGMEM const char Text_string61[] = "FANS 1,2";               // РАДИАТОРОВ 1,2
PROGMEM const char Text_string62[] = "";    // РЕЗЕРВ
PROGMEM const char Text_string63[] = "Minimum";          // Минимальная
PROGMEM const char Text_string64[] = "Maximum";         // Максимальная
PROGMEM const char Text_string65[] = "Luminosite";              // Яркость (в меню луны)
PROGMEM const char Text_string66[] = "NOUVELLE LUNE";           // НОВАЯ ЛУНА
PROGMEM const char Text_string67[] = "LUNE COMPLETE";          // ПОЛНАЯ ЛУНА
PROGMEM const char Text_string68[] = "";    // РЕЗЕРВ
PROGMEM const char Text_string69[] = "";    // РЕЗЕРВ
PROGMEM const char Text_string70[] = "";    // РЕЗЕРВ
PROGMEM const char Text_string71[] = "";    // РЕЗЕРВ
PROGMEM const char Text_string72[] = "DOSEUR";                // ДОЗАТОР
PROGMEM const char Text_string73[] = "EAU";                    // ВОДЫ
PROGMEM const char Text_string74[] = "FEED";                // КОРМУШКА
PROGMEM const char Text_string75[] = "FILTRE OFF";        // ВЫКЛЮЧИТЬ ФИЛЬТР
PROGMEM const char Text_string76[] = "NIVEAU";                 // УРОВЕНЬ
PROGMEM const char Text_string77[] = "DRAIN";                    // СЛИВ
PROGMEM const char Text_string78[] = "REGLAGE DU PH";     //  Установка уровня РН
PROGMEM const char Text_string79[] = "PH";              //  РН
PROGMEM const char Text_string80[] = "Actu";                 //  Текущее
PROGMEM const char Text_string81[] = "Signification";                //  Значение
PROGMEM const char Text_string82[] = "PH";                      //  PH
PROGMEM const char Text_string83[] = "";         //  РЕЗЕРВ
PROGMEM const char Text_string84[] = "7";                       // 7
PROGMEM const char Text_string85[] = "10";                      // 10
PROGMEM const char Text_string86[] = "";         // РЕЗЕРВ
PROGMEM const char Text_string87[] = "FILTRE";                  // ФИЛЬТР
PROGMEM const char Text_string88[] = "";         // РЕЗЕРВ
PROGMEM const char Text_string89[] = "ON / OFF";             // ВКЛ./ВЫКЛ.
PROGMEM const char Text_string90[] = "ASSIGNATION COULEUR";           // ПРИСВОЕНИЕ ЦВЕТА
PROGMEM const char Text_string91[] = "CANAUX";                    // КАНАЛАМ
PROGMEM const char Text_string92[] = "FONCTIONNEMENT";               // РЕЖИМ РАБОТЫ
PROGMEM const char Text_string93[] = "REFROIDISSEMENT";                 // ОХЛАЖДЕНИЯ
PROGMEM const char Text_string94[] = "";           // РЕЗЕРВ
PROGMEM const char Text_string95[] = "";           // РЕЗЕРВ
PROGMEM const char Text_string96[] = "";           // РЕЗЕРВ
PROGMEM const char Text_string97[] = "";           // РЕЗЕРВ
PROGMEM const char Text_string98[] = "";           // РЕЗЕРВ
PROGMEM const char Text_string99[] = "";           // РЕЗЕРВ
PROGMEM const char Text_string100[] = "Pause";                    // ПАУЗА
PROGMEM const char Text_string101[] = "ALIM.";                    // КОРМ.
PROGMEM const char Text_string102[] = "";          // РЕЗЕРВ
PROGMEM const char Text_string103[] = "";          // РЕЗЕРВ
PROGMEM const char Text_string104[] = "";          // РЕЗЕРВ
PROGMEM const char Text_string105[] = "";          // РЕЗЕРВ
PROGMEM const char Text_string106[] = "";          // РЕЗЕРВ
PROGMEM const char Text_string107[] = "";          // РЕЗЕРВ
PROGMEM const char Text_string108[] = "";          // РЕЗЕРВ
PROGMEM const char Text_string109[] = "";          // РЕЗЕРВ
PROGMEM const char Text_string110[] = "";          // РЕЗЕРВ
PROGMEM const char Text_string111[] = "Minutes";                            // Минут
PROGMEM const char Text_string112[] = "Secondes";                           // Секунд
PROGMEM const char Text_string113[] = "";          // РЕЗЕРВ
PROGMEM const char Text_string114[] = "LED A:";                        // CВЕТОДИОДОВ ДО:
PROGMEM const char Text_string115[] = "NOMBRE DE DOSES   >";                      // КОЛИЧЕСТВО ДОЗ
PROGMEM const char Text_string116[] = "INTERVALLE >";                      // ИНТЕРВАЛ
PROGMEM const char Text_string117[] = "ENTRER LE VOLUME >";                        // ВВЕДИТЕ ОБЪЕМ
PROGMEM const char Text_string118[] = "Sec.";                                   // Сек.
PROGMEM const char Text_string119[] = "DOSAGE >";                      // ОБЪЕМ дозы
PROGMEM const char Text_string120[] = "DUREE DOSAGE";                             // Время работы дозатора
PROGMEM const char Text_string121[] = "Mл";                                   // Mл
PROGMEM const char Text_string122[] = "CALIBRATION";                            // КАЛИБРОВКА
PROGMEM const char Text_string123[] = "DOSEUR";                               // ДОЗАТОР
PROGMEM const char Text_string124[] = "MONTRES";                                  // ЧАСЫ
PROGMEM const char Text_string125[] = "MINUTES";                                // МИНУТЫ
PROGMEM const char Text_string126[] = "SUITE";                              // ПРОДОЛЖ.
PROGMEM const char Text_string127[] = "REGLAGE T.";                        // УСТАНОВ. ТЕМП.
PROGMEM const char Text_string128[] = "ON";                                  // ВКЛ.
PROGMEM const char Text_string129[] = "Niveaux";                                // Уровни
PROGMEM const char Text_string130[] = "SAUVER";                             // СОХРАНИТЬ
PROGMEM const char Text_string131[] = "Servo";                                 // Servo
PROGMEM const char Text_string132[] = "MINUTES";                                 // МИНУТ
PROGMEM const char Text_string133[] = "ECRAN LUMINEUX";                      // ПОДСВЕТКА ЭКРАНА
PROGMEM const char Text_string134[] = "TEMP. INCLUSIONS FANS";                  // ТЕМП. ВКЛЮЧЕНИЯ ВЕНТИЛЯТОРА
PROGMEM const char Text_string135[] = "REDUCTION LUMINOSITE";                  // АВТО-УМЕНЬШ. ЯРКОСТИ
PROGMEM const char Text_string136[] = "OVERHOUSE";                         // ПРИ ПЕРЕГРЕВЕ
PROGMEM const char Text_string137[] = "REGLAGES";                             // УСТАНОВКИ
PROGMEM const char Text_string138[] = "ECRAN";                      // ХРАНИТЕЛЯ ЭКРАНА
PROGMEM const char Text_string139[] = "LIMITATION PUISSANCE";                  // ОГРАНИЧЕНИЕ МОЩНОСТИ
PROGMEM const char Text_string140[] = "RADIATEUR-1 REFROIDISSEMENT";     // ВЕНТИЛЯТОР ОХЛАЖДЕНИЯ РАДИАТОРА-1
PROGMEM const char Text_string141[] = "RADIATEUR-2 REFROIDISSEMENT";     // ВЕНТИЛЯТОР ОХЛАЖДЕНИЯ РАДИАТОРА-2
PROGMEM const char Text_string142[] = "SUR LA TEMPERATURE";                 // ТЕМПЕРАТУРА ВКЛЮЧЕНИЯ
PROGMEM const char Text_string143[] = "amplitude signal des canaux";         // величина сигнала всех каналов
PROGMEM const char Text_string144[] = "(25-50)";                               // (25-50) граница температур
PROGMEM const char Text_string145[] = "AVEC CHAUFFAGE RADIATEUR";                 // ПРИ НАГРЕВЕ РАДИАТОРА
PROGMEM const char Text_string146[] = "AVANT TEMPERATURE:";                       // ДО ТЕМПЕРАТУРЫ:
PROGMEM const char Text_string147[] = "REDUIRE LA LUMINOSITE";                     // УМЕНЬШАТЬ ЯРКОСТЬ
PROGMEM const char Text_string148[] = "CLOCK / BLANK";                   // ЧАСЫ / ПУСТОЙ ЭКРАН
PROGMEM const char Text_string149[] = "ACTIVATION";                             // АКТИВАЦИЯ
PROGMEM const char Text_string150[] = "Apres:";                                // ПОСЛЕ

# ifdef freshwater
PROGMEM const char Text_string151[] = "AIR";                              // АЭРАЦИЯ  (меню ручное управление)
PROGMEM const char Text_string152[] = "CO2";                                  // СО2
PROGMEM const char Text_string153[] = "FILTRE";                               // ФИЛЬТР
PROGMEM const char Text_string154[] = "LAMPE";                             // УФ ЛАМПА
PROGMEM const char Text_string155[] = "DUREE";                                // ДОЛИВ
#endif
# ifdef seawater
PROGMEM const char Text_string151[] = "TIMER-1";                              // Таймер 1  (меню ручное управление)
PROGMEM const char Text_string152[] = "TIMER-2";                              // Таймер 2
PROGMEM const char Text_string153[] = "TIMER-3";                              // Таймер 3
PROGMEM const char Text_string154[] = "TIMER-4";                              // Таймер 4
PROGMEM const char Text_string155[] = "TIMER-5";                              // Таймер 5
#endif
PROGMEM const char Text_string156[] = "AUTO";                                  // АВТО
PROGMEM const char Text_string157[] = "TEMPERATURE QUOTIDIENNE";             // ТЕМПЕРАТУРА ВОДЫ ЗА СУТКИ
PROGMEM const char Text_string158[] = "";       // РЕЗЕРВ
PROGMEM const char Text_string159[] = "";       // РЕЗЕРВ
PROGMEM const char Text_string160[] = "";       // РЕЗЕРВ
PROGMEM const char Text_string161[] = "";       // РЕЗЕРВ
PROGMEM const char Text_string162[] = "";       // РЕЗЕРВ
PROGMEM const char Text_string163[] = "";       // РЕЗЕРВ
PROGMEM const char Text_string164[] = "";       // РЕЗЕРВ
PROGMEM const char Text_string165[] = "";       // РЕЗЕРВ
PROGMEM const char Text_string166[] = "(24h)";                                 // (24HR)
PROGMEM const char Text_string167[] = "REGLAGE DE L'ALARME SON";            // УСТАНОВКА ЗВУКОВОЙ ТРЕВОГИ
PROGMEM const char Text_string168[] = "(Fonct. simultane pompes)";           //(Одновременная работа помп)
PROGMEM const char Text_string169[] = "TIMERS";                               // Таймеры
PROGMEM const char Text_string170[] = "Reglage!";                             // Изменено ! (меню часов)
PROGMEM const char Text_string171[] = "OFF";                                   // Вык (выключено)
PROGMEM const char Text_string172[] = "ON";                                   // Вкл (включено)
PROGMEM const char Text_string173[] = "";        // РЕЗЕРВ
PROGMEM const char Text_string174[] = "Enregistrement sur la carte flash";        // Запись настроек на флеш карту
PROGMEM const char Text_string175[] = "Parametres bien enregistres.";          // Настройки успешно сохранены
PROGMEM const char Text_string176[] = "Rien";                               // Отключ.
PROGMEM const char Text_string177[] = "Eau";                                // Д.Воды (Датчик в аквариуме)
PROGMEM const char Text_string178[] = "Fan1";                               // Д.Рад:1 (Радиатор Датчик 1)
PROGMEM const char Text_string179[] = "Fan2";                               // Д.Рад:2 (Радиатор Датчик 2)
PROGMEM const char Text_string180[] = "NOURRISAGE";                      // ПОКОРМИТЬ СЕЙЧАС
PROGMEM const char Text_string181[] = "ALIMENTATION";                             // КОРМЛЕНИЕ
PROGMEM const char Text_string182[] = "HEURE";                                 // ВРЕМЯ
PROGMEM const char Text_string183[] = "PAS INSTALLE";                        // НЕ УСТАНОВЛЕНО
PROGMEM const char Text_string184[] = "OFF";                                 // ВЫКЛ.
PROGMEM const char Text_string185[] = "166 RC.";                          // programm version ENGLISH !!!!!

const PROGMEM char* const PROGMEM Text_table[]  = {
  Text_string0, Text_string1, Text_string2, Text_string3, Text_string4, Text_string5, Text_string6, Text_string7,
  Text_string8, Text_string9, Text_string10, Text_string11, Text_string12, Text_string13, Text_string14, Text_string15,
  Text_string16, Text_string17, Text_string18, Text_string19, Text_string20, Text_string21, Text_string22, Text_string23,
  Text_string24, Text_string25, Text_string26, Text_string27, Text_string28, Text_string29, Text_string30, Text_string31,
  Text_string32, Text_string33, Text_string34, Text_string35, Text_string36, Text_string37, Text_string38, Text_string39,
  Text_string40, Text_string41, Text_string42, Text_string43, Text_string44, Text_string45, Text_string46, Text_string47,
  Text_string48, Text_string49, Text_string50,  Text_string51, Text_string52, Text_string53, Text_string54, Text_string55,
  Text_string56, Text_string57, Text_string58, Text_string59, Text_string60, Text_string61, Text_string62, Text_string63,
  Text_string64, Text_string65, Text_string66, Text_string67, Text_string68, Text_string69, Text_string70, Text_string71,
  Text_string72, Text_string73, Text_string74, Text_string75, Text_string76, Text_string77, Text_string78, Text_string79,
  Text_string80, Text_string81, Text_string82, Text_string83, Text_string84, Text_string85, Text_string86, Text_string87,
  Text_string88, Text_string89, Text_string90, Text_string91, Text_string92, Text_string93, Text_string94, Text_string95,
  Text_string96, Text_string97, Text_string98, Text_string99, Text_string100, Text_string101, Text_string102, Text_string103,
  Text_string104, Text_string105, Text_string106, Text_string107, Text_string108, Text_string109, Text_string110,
  Text_string111, Text_string112, Text_string113, Text_string114, Text_string115, Text_string116, Text_string117,
  Text_string118, Text_string119, Text_string120, Text_string121, Text_string122, Text_string123, Text_string124,
  Text_string125, Text_string126, Text_string127, Text_string128, Text_string129, Text_string130, Text_string131,
  Text_string132, Text_string133, Text_string134, Text_string135, Text_string136, Text_string137, Text_string138,
  Text_string139, Text_string140, Text_string141, Text_string142, Text_string143, Text_string144, Text_string145,
  Text_string146, Text_string147, Text_string148, Text_string149, Text_string150, Text_string151, Text_string152,
  Text_string153, Text_string154, Text_string155, Text_string156, Text_string157, Text_string158, Text_string159,
  Text_string160, Text_string161, Text_string162, Text_string163, Text_string164, Text_string165, Text_string166,
  Text_string167, Text_string168, Text_string169, Text_string170, Text_string171, Text_string172, Text_string173,
  Text_string174, Text_string175, Text_string176, Text_string177, Text_string178, Text_string179, Text_string180,
  Text_string181, Text_string182, Text_string183, Text_string184, Text_string185
};

char buffer[40];
int PrintStringIndex;
char* print_text[]  = {
  "<< MENU",           // print_text[0]  МЕНЮ
  "ACCUEIL",            // print_text[1]  ОТМЕНА
  "<< RETOUR",          // print_text[2]  НАЗАД
  "SAUVER",         // print_text[3]  СОХРАНИТЬ
  "STOP",              // print_text[4]  СТОП
  "+ COULEURS",       // print_text[5]  ДРУГОЙ ЦВЕТ
  "CHANGER",          // print_text[6]  ИЗМЕНИТЬ
  "LUN",               // print_text[7]  ПОН.
  "MAR",               // print_text[8]  ВТО.
  "MER",               // print_text[9]  СРД.
  "JEU",               // print_text[10] ЧТВ.
  "VEN",               // print_text[11] ПТН.
  "SAM",               // print_text[12] СУБ.
  "DIM",               // print_text[13] ВСК.
  "AUTO FEEDER",       // print_text[14]
  "RAND",              // print_text[15]
  "ORAGE",             // print_text[16]
  "NEXT >>",          // print_text[17]
  "<<",                // print_text[18]
  ">>",                // print_text[19]
  "1",                 // print_text[20]  (W) white  10 аква
  "2",                 // print_text[21]  (B) blue   11 аква
  "3",                 // print_text[22]  (RB)royal   5 аква
  "4",                 // print_text[23]  (R) red     8 аква
  "5",                 // print_text[24]  ultra       9 аква
  "6",                 // print_text[25]  oLed
  "7",                 // print_text[26]  green
  "+",                 // print_text[27]
  "-",                 // print_text[28]
  "<< AVANT",         // print_text[29]  << Сектор
  "APRES>>",          // print_text[30]  Сектор >>
  "Preset 1",          // print_text[31]  Пресет 1  OPEQER 1
  "Preset 2",          // print_text[32]  Пресет 2
  "Preset 3",          // print_text[33]  Пресет 3
  "Preset 4",          // print_text[34]  Пресет 4
  "YES",                // print_text[35]  ДА
  "NO",               // print_text[36]  НЕТ
  "10",                // print_text[37]  10
  "20",                // print_text[38]  20
  "30",                // print_text[39]  30
  "40",                // print_text[40]  40
  "50",                // print_text[41]  50
  "60",                // print_text[42]  60
  "70",                // print_text[43]  70
  "80" ,               // print_text[44]  80
  "90",                // print_text[45]  90
  "100",               // print_text[46]  100
  "1",                 // print_text[47]  1  "@]P@VH_"
  "2",                 // print_text[48]  2 "OND@W@ QN"
  "3",                 // print_text[49]  3 "THK&RP"
  "4",                 // print_text[50]  4 "ST K@LO@"
  "5",                 // print_text[51]  5 "QKHB"
  "O-1",               // print_text[52]
  "O-2",               // print_text[53]
  "O-3",               // print_text[54]
  "O-4",               // print_text[55]
  ":",                 // print_text[56]  двоеточие
  "C",                 // print_text[57]
  "---",               // print_text[58]
  "+++",               // print_text[59]
  "-1",                // print_text[60]
  "+5",                // print_text[61]
# ifdef freshwater
  "AIR",              // print_text[62]                         Таймеры
  "CO2",               // print_text[63]
  "FILTRE",            // print_text[64]
  "LAMPE",                // print_text[65]
  "DUREE",             // print_text[66]
#endif
# ifdef seawater
  "-1-",               // print_text[62]                         Таймеры
  "-2-",               // print_text[63]
  "-3-",               // print_text[64]
  "-4-",               // print_text[65]
  "-5-",               // print_text[66]
#endif
  "temps sur",   // print_text[67] время вкл.
  "temps libre",  // print_text[68] время выкл.
  "22",                // print_text[69]
  "23",                // print_text[70]
  "24",                // print_text[71]
  "25",                // print_text[72]
  "26",                // print_text[73]
  "27",                // print_text[74]
  "28",                // print_text[75]
  "29",                // print_text[76]
  "%",                 // print_text[77]  знак процентов
  "31",                // print_text[78]
  "6",                 // print_text[79]
  "8",                 // print_text[80]
  "12",                // print_text[81]
  "14",                // print_text[82]
  "16",                // print_text[83]
  "18",                // print_text[84]
  "E",                 // print_text[85]
  "D",                 // print_text[86]
  "FAN1",             // print_text[87]  датчик температуры радиатора 1   ДАТЧ1
  "FAN2",             // print_text[88]  датчик температуры радиатора 2   ДАТЧ2
  "Lune",              // print_text[89]  Луна
  "32",                // print_text[90]  32
  "34",                // print_text[91]  34
  "36",                // print_text[92]  36
  "38",                // print_text[93]  38
  "42",                // print_text[94]  42
  "OFF",              // print_text[95]   ВЫКЛ
  "START",             // print_text[96]   СТАРТ
  "CANAL-1  :",        // print_text[97]   БЕЛЫЙ   канал    (главный экран)
  "CANAL-2  :",        // print_text[98]   ГОЛУБОЙ   канал
  "CANAL-3  :",        // print_text[99]   СИНИЙ   канал
  "CANAL-4  :",        // print_text[100]  КРАСНЫЙ   канал
  "CANAL-5  :",        // print_text[101]  УЛЬТРА   канал
  "CANAL-6  :",        // print_text[102]  ОРАНЖЕВЫЙ  канал
  "CANAL-7  :",        // print_text[103]  ЗЕЛЕНЫЙ   канал
  "CANAL-1:",          // print_text[104]  БЕЛЫЙ (меню тест)
  "CANAL-2:",          // print_text[105]  ГОЛУБОЙ
  "CANAL-3:",          // print_text[106]  СИНИЙ
  "CANAL-4:",          // print_text[107]  КРАСНЫЙ
  "CANAL-5:",          // print_text[108]  УЛЬТРА
  "CANAL-6:",          // print_text[109]  ОРАНЖ.
  "CANAL-7:",          // print_text[110]  ЗЕЛЕНЫЙ
  "   ",               // print_text[111]  (три пробела)
  "Nouvelle Lune",         // print_text[112]  Новолуние
  "Croissant",         // print_text[113]  Растущая
  "Quartier",        // print_text[114]  1 Четверть
  "Croissant",         // print_text[115]  Растущая
  "Pleine lune",        // print_text[116]  Полнолуние
  "Decroissant",        // print_text[117]  Убывающая
  "3 trimestre",        // print_text[118]  3 Четверть
  "AQUARIUM",          // print_text[119]  АКВАРИУМ
  "FAN",          // print_text[120]  РАДИАТОР
  "CALENDRIER TEMP.", // print_text[121]  ГРАФИК ТЕМПЕРАТУР (menu)
  "REGLAGES",         // print_text[122]   УСТАНОВКА    (главное меню)
  "HEURE",           // print_text[123]   ВРЕМЕНИ
  "TEMPERATURE",       // print_text[124]   ТЕМПЕРАТУРЫ
  "FILTRE",            // print_text[125]   ФИЛЬТР
  "DOSEUR",           // print_text[126]   ДОЗАТОР
  "ADDITIF",           // print_text[127]   ДОБАВОК
  "REGLAGES",          // print_text[128]   ОСНОВНЫЕ
  "PRINCIPAL",         // print_text[129]   НАСТРОЙКИ
  "AUTO",              // print_text[130]   АВТО
  "TEST",              // print_text[131]   ТЕСТ
  "CALENDR.",            // print_text[132]   ГРАФИК
  "CANAUX",           // print_text[133]   КАНАЛОВ
  "COULEURS",             // print_text[134]   ЦВЕТА
  "CANAUX",            // print_text[135]   CЕКТОР
  "HEURE",             // print_text[136]   ВРЕМЯ
  "SAUVER",            // print_text[137]   ЗАПИСЬ
  "PRESETS",          // print_text[138]   ПРЕСЕТОВ
  "TIMERS",          // print_text[139]   СУТОЧНЫЕ
  "QUOTIDIEN",           // print_text[140]   ТАЙМЕРЫ
  "pH",       // print_text[141]   УГЛЕКИСЛОТЫ
  "%",                 // print_text[142]   % (проценты)
  "Matin",           // print_text[143]   рассвет
  "Soir",             // print_text[144]   закат
  "Max",              // print_text[145]   Макс
  "Actu",             // print_text[146]   Текущ
  "BACKUP.TXT",        // print_text[147]   BACKUP.TXT
  "DANS LE FICHIER TEXTE", // print_text[148]   В ТЕКСТОВОМ ФАЙЛЕ
  "TOUS LES PARAMETRES SONT SUR LA CARTE", // print_text[149]  ВСЕ НАСТРОЙКИ НАХОДЯТСЯ НА ФЛЕШ КАРТЕ
  "PARAMETRES",         // print_text[150]   Настройки
  "Temperature Fan par jour",       // print_text[151]  температура радиаторов за сутки
  "Alarme",             // print_text[152]   Alarm (в меню установки температуры радиатора)
  "    ",              // print_text[153]  (четыре пробела)  (Cегодня Qecndm$)
  "TEST",              // print_text[154]   ТЕСТ
  "/",                 // print_text[155]   // | /
  "Lundi",       // print_text[156]   Понедельник (бар даты)
  "Mardi",           // print_text[157]   Вторник
  "Mercr.",             // print_text[158]   Среда
  "Jeudi",           // print_text[159]   Четверг
  "Vendr.",           // print_text[160]   Пятница
  "Samedi",           // print_text[161]   Суббота
  "Diman.",       // print_text[162]   Воскресенье
  "Janvier",            // print_text[163]   Января
  "Fevrier",           // print_text[164]   Февраля
  "Mars",             // print_text[165]   Марта
  "Avril",            // print_text[166]   Апреля
  "Mai",               // print_text[167]   Мая
  "Juin",              // print_text[168]   Июня
  "Juillet",              // print_text[169]   Июля
  "Aout",           // print_text[170]   Августа
  "Septembre",          // print_text[171]   Сентября
  "Octobre",           // print_text[172]   Октября
  "Novembre",            // print_text[173]   Ноября
  "Decembre",           // print_text[174]   Декабря
  "Enregistrement parametres", // print_text[175]   Сохранение настроек
  "0...100 %",         // print_text[176]  (0--100) В меню настройки луны
  "Lune",              // print_text[177]  Луна
  "*7*",               // print_text[178]  Green (GRN)
  "*6*",               // print_text[179]  Orange(ORG
  "*5*",               // print_text[180]  UV    (UVL)
  "*4*",               // print_text[181]  Red   (RED)
  "*3*",               // print_text[182]  Royal (RBL)
  "*2*",               // print_text[183]  Blue  (BLU)
  "*1*",               // print_text[184]  White (WHT)
  "VOUKEZ-VOUS SAUVEGARDER,", // print_text[185]  ВЫ ХОТИТЕ СОХРАНИТЬ НАСТРОЙКИ,
  "AVANT DE QUITTER DU MENU?^ ?",        // print_text[186]      ПЕРЕД ВЫХОДОМ ИЗ МЕНЮ ?
  "0",                 // print_text[187]  0 (один ноль)
  "00",                // print_text[188]  00 (два нуля)
  "000",               // print_text[189]  000 (три нуля)
  "TEST",           // print_text[190]  ТЕСТИРОВАНИЕ
  "Nettoyage memoire",   // print_text[191] ПОДОЖДИТЕ, идет очистка памяти
  "RESTAURER",         // print_text[192]  ЗАГРУЗИТЬ
  "RESET",             // print_text[193]  СБРОС (RESET)
  "BACKUP",             // print_text[194]  БЕКАП(BACKUP)
  "RECHERCHE",        // print_text[195]  ПОИСК ДАТЧ.
  "Restaurer les parametres", // print_text[196] Восстановление настроек завершено
  "Veuillez patienter ...",           // print_text[197] Пожалуйста подождите....
  "Lecture des parametres sur la carte",// print_text[198] Чтение файла настроек на флеш карте
  "Erreur d'ouverture du fichier", // print_text[199] Ошибка при открытии файла Настроек
  "RÉINITIALISER REGLAGES?",               // print_text[200] СБРОСИТЬ НАСТРОЙКИ ?
  "Initialisation terminee avec succes",    // print_text[201] Инициализация успешно завершена
  "Erreur! initialisation carte SD",  // print_text[202] Ошибка ! инициализации флеш карты
  "Initialisation carte SD ..",         // print_text[203] Инициализация Флеш карты..
  "RECHERCHE SONDES",                     // print_text[204] ПОИСК ДАТЧИКОВ   ОБНОВИТЬ (NAMNBHR;)
  "Connecte K",                        // print_text[205] Подключен К
  "Sonde",                             // print_text[206] Датчик
  "Sondes",                           // print_text[207] Датчиков
  "Trouvee:",                           // print_text[208] Найдено:
  "REINIT PARAMETRES",                 // print_text[209] СБРОСИТЬ НАСТРОЙКИ (в меню)
  "SAUVEGARDE PARAMETRES",              // print_text[210] СОХР. НАСТР. НА КАРТУ
  "RECHERCHE SONDE TEMP.",             // print_text[211] ПОИСК ДАТЧИКОВ ТЕМПЕР.
  "ON",                                // print_text[212] ВКЛ
  "OFF",                                // print_text[213] ВЫК
  "Temperature eau",                   // print_text[214] Температура воды
  "AJUSTEMENT",                             // print_text[215] НАСТР.
  "FEEDER",                          // print_text[216] КОРМЛЕНИЯ
  "GESTION",                              // print_text[217] ПОМПЫ
  "DES POMPES",                            // print_text[218] ТЕЧЕНИЯ
};

int dispScreen = 0;
/*
  0-Main Screen,                          ГЛАВНЫЙ ЭКРАН
  1-Menu,                                 ГЛАВНОЕ МЕНЮ
  2-Clock Setup,                          УСТАНОВКА ДАТЫ/ВРЕМЕНИ
  3-Temp Control,                         УСТАНОВКА ТЕМПЕРАТУРЫ ВОДЫ
  4-LED Test Options (sector),            НАСТРОЙКА ОСВЕЩЕНИЯ ПО СЕКТОРАМ
  5-Test LED Arrays,                      АВТОМАТИЧЕСКИЙ ТЕСТ ОСВЕЩЕНИЯ
  6-Test Individual,                      РУЧНОЙ ТЕСТ ОСВЕЩЕНИЯ
  7-                                РЕЗЕРВ
  8-                                РЕЗЕРВ
  9-General Set.(Page3),                  ГЛАВНЫЕ НАСТРОЙКИ СТР.3
  10- Dose calibrate,                     КАЛИБРОВКА ДОЗАТОРОВ
  11-PH Settings,                         УСТАНОВКА И КАЛИРОВКА РН
  12-PWM Pump,                            ПОМПЫ В РЕЖИМЕ ШИМ
  13-Led Preset,                          ЗАПИСЬ ПРЕСЕТОВ
  14-General Set.(Page1),                 ГЛАВНЫЕ НАСТРОЙКИ СТР.1
  15-General Set.(Page2),                 ГЛАВНЫЕ НАСТРОЙКИ СТР.2
  16-Heatsink FAN Temp,                   НАСТРОЙКА ВЕНТИЛЯТОРОВ РАДИАТОРОВ
  17-Led Dim (temp),                      ТЕМПЕРАТУРА АВТОУМЕНЬШЕНИЯ ЯРКОСТИ
  18-Set Screensaver,                     УСТАНОВКИ СКРИНСЕЙВЕРА
  19-Timer,                               ТАЙМЕРЫ
  20-T1, 21-T2, 22-T3, 23-T4, 24-T5,      ТАЙМЕРЫ 1,2,3,4.5
  25-Manual Timer control,                РУЧНОЕ УПРАВЛЕНИЕ ТАЙМЕРАМИ
  26-Moon Seting,                         НАСТРОЙКА ЯРКОСТИ ЛУНЫ
  27-Log Temp Radiator,                   ГРАФИК ТЕМПЕРАТУРЫ РАДИАТОРОВ
  28-Log Temp Water,                      ГРАФИК ТЕМПЕРАТУРЫ ВОДЫ
  29-Test Led Graph,                      ТЕСТ ОСВЕЩЕНИЯ С ГРАФИКАМИ
  30-Dallas auto-detect,                  ПОИСК ДАТЧИКОВ ТЕМПЕРАТУРЫ
  31-Backup Setting,                      СОХРАНЕНИЕ НАСТРОЕК
  32-Dimm,                                ОГРАНИЧЕНИЕ МОЩНОСТИ
  33-LCDbright,                           ПОДСВЕТКА ЭКРАНА
  34-                               РЕЗЕРВ
  35-Sound Alarm,                         НАСТРОЙКА ТРЕВОГИ
  36-AutoDoser,                           ЭКРАН ДОЗАТОРОВ ДОБАВОК
  37-Set AutoDoser Times,                 УСТАНОВКА ВРЕМЕНИ ДОЗАТОРОВ
  38-Fish feeder,                         ЭКРАН АВТОКОРМУШКИ
  39-Set feeder times,                    УСТАНОВКА ВРЕМЕНИ КОРМЛЕНИЯ
  40-Set Chanel colors,                   РАСКРАСКА КАНАЛОВ
*/
int x, y;                            // touch coordinates

long previousMillisLED = 0;          // Used in the Test LED Array Function
long previousMillisWave = 0;         // Used in the WaveMaker Function feed_output()
long previousMillisFive = 0;          // Used in the Main Loop (Checks Time,Temp,LEDs,Screen)
long previousMillisTen = 0;
long previousMillisLEDoff = 0;       // Used in Shut LEDs Off if Temp too high
long previousMillisAlarm = 0;        // Used in the Alarm
long previousMillisOne = 0;          // Used in the Main Loop (LEDs level) для уровней каналов в 11бит режиме

long previousMillisLewel = 0;        // check led lewel
long previousMillisAlarm2 = 0;
long previousMillisGraph = 0;

// вместо Delay
unsigned long currentTime;
unsigned long loopTime;
// your feed seconds are douswtled)
long previousMillisCt = 0;           // stores the last update for the Countdown Timer
long intervalCt = 1000;              // One Second Interval for Countdown Timer
int countDown = 5 * 60 + 0;          // Countdown for 5 minutes and zero seconds
int MIN_O = 5;                       // Start the Time at 5 (for looks only)
int SEC_T = 0;
int SEC_O = 0;

int LedChangTime = 0;                // LED change page, time and values
byte oldLCT ;
int min_cnt;                         // Used to determine the place in the color arrays
int sec_cnt;
byte temp_sector;                    // used in led test arrays, min_cnt/15

boolean LEDtestTick = false;         // for testing leds and speed up clock
//boolean firstTouch  = false;       // #oleg выход из скринсейва
boolean SliderSwitch  = false;       // slider

//byte LedShannelStatusByte =255;      // ON=1/OFF=0 channel, one bit - one channel
byte LedShannelStatusByte = 2000; // 11 bit ON=1/OFF=0 channel, one bit - one channel
byte GlobalStatus2Byte = 0x0;        // byte Led preset status

// bit 0 - preset 1 (ON=1/OFF), bit 1 - preset 2, bit 2 - preset 3, bit 3 - preset 4
byte  AddressShift;
byte GlobalStatus1Byte;		     // bit 0 for Y/N button in reset default function
byte setTimeFormat = 0;

int whiteLed, blueLed, rblueLed, redLed, uvLed, orLed, grLed;     // предыдущие значения яркости LED
int wwtled_out, cwtled_out, rbled_out, rled_out, uvled_out, oLed_out, moonled_out, gled_out; // текущие значения яркости LED
int wwtcol_out, cwtcol_out, rbcol_out, rcol_out, uvcol_out, ocol_out, grcol_out, mooncol_out; // Current LED output values for Test Ind. Color LEDs
int wwt_out, cwt_out, rb_out, r_out, uv_out, o_out, gr_out, moon_out;  // Current LED output values for Test Ind. Color LEDs

int R_color, G_color, B_color, temp_R_color, temp_G_color, temp_B_color;                                                      // Переменные для изменения цвета слайдеров

byte COLOR = 0, WHITE = 1, BLUE = 2, ROYAL = 3, RED = 4, ULTRA = 5, ORANGE = 6, GREEN = 7, MOON = 8; // View / Change Color LED Values

boolean colorLEDtest = false;         // To test individual color LEDs

boolean RGBcolorSet = false;         // To RGB individual color Set
//int sbR, sbG, sbB, sbX1, sbX2;        // Used in the Slider Bars
byte sbR, sbG, sbB;
int sbX1, sbX2;                      // Used in the Slider Bars
//int tSlide=0;
int yWHT = 0, yBLU = 0, yRBL = 0, yRED = 0, yUVL = 0, ySMP = 0, yLUN = 0, yGRN = 0;

boolean DrawStaticElement = false;    // Allows selection of changing LED values

int yStore = 0, k = 0, tSlide = 0;
boolean TopRows = false;              // Allows selection of changing LED values

int x1Bar = 0, x2Bar = 0, xValue = 0, yValue = 0, LEDlevel, yBar; // Used in LED Output Chart on Test Ind. LEDs Screen
int setmode1 = 0, setmode2 = 0, setmode3 = 0;

int timer;  // variable de temporisateur
// Activer les minuteries d'éclairage réglées 0 = NON 1 = OUI
int prog1, prog2, prog3, prog4, prog5; // minuteries 1-5

// Temps de commutation de la lumière
int on1, on2, on3, on4, on5;   // minuteries 1-5

// Eteindre le temps
int off1, off2, off3, off4, off5; // minuteries 1-5

// Variable de temps Temps d'allumage de l'éclairage
int tempon1, tempon2, tempon3, tempon4, tempon5;  // minuteries 1-5

// Eteindre le temps
int tempoff1, tempoff2, tempoff3, tempoff4, tempoff5; // minuteries 1-5

// Statut des minuteries d'activation / désactivation (contrôle manuel)
byte timer1Status;   // Statut du minuteur 1
byte timer2Status;   // Statut du minuteur 2
byte timer3Status;   // Statut du minuteur 3
byte timer4Status;   // Statut du minuteur 4
byte timer5Status;   // Statut du minuteur 5

byte LCDbright;       // luminosité de l'écran par défaut
byte HightbrH;       // temps de luminosité
byte LowbrH;       // temps de variation

long previousMillisA = 0;  // Timer stuff
long previousMillisB = 0;
long previousMillisC = 0;

int DelayPeriodA = 200;    // Combien de temps attendre (en millisecondes) entre changer la luminosité de la LED (vitesse de balayage)
int DelayPeriodB = 100;
int DelayPeriodC = 50;

int DegreesCounterA = 30;  // Используется для прокрутки сигнала синус / 300
int DegreesCounterB = 30;
int DegreesCounterC = 30;

float RadiansCounterA = 0; // Radians version of DegreesCounter
float RadiansCounterB = 0;
float RadiansCounterC = 0;

float Sin_of_Rad_CtrA = 0; // Sine feed calculation stuff
float Sin_of_Rad_CtrB = 0;
float Sin_of_Rad_CtrC = 0;

int BrightnessA = 0;       // Valeur de luminosité, LED
int BrightnessB = 0;
int BrightnessC = 0;

int StepSize = 1;  // augmentation de la luminosité

// Графики
byte StopTime;
byte StartTime;
int TopSldY ;	    // top slider
int BotSldY ;       // bot slider
boolean LedShannelFlag_on_off;   // flag led on/off channel status
int  EndScale;
byte LightDay;
boolean W = true;  // true - activé par défaut, false - désactivé
boolean RB = true;
boolean B = true;
boolean R = true;
boolean UV = true;
boolean SU = true;
boolean OR = true;

boolean F1 = true;  // capteur 1 sur le radiateur
boolean F2 = true;  // capteur 2
float Th;
int TimeW = 1;

//********************** VALEURS DES CARACTÈRES DES CANAUX PAR DÉFAUT ***********************************
byte wwtled[96] = { // Canal 1 Blanc chaud pas utilisé
  0, 0, 0, 0, 0, 0, 0, 0,                   //0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,                   //2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,                   //4 - 5
  0, 0, 0, 0, 0, 0, 0, 0,                   //6 - 7
  5, 5, 5, 5, 10, 10, 10, 10,               //8 - 9
  10, 10, 10, 10, 15, 15, 15, 25,           //10 - 11
  25, 30, 40, 50, 75, 80, 80, 85,           //12 - 13
  85, 85, 85, 85, 90, 90, 90, 90,           //14 - 15
  85, 85, 85, 85, 85, 85, 85, 75,           //16 - 17
  75, 75, 75, 75, 70, 70, 70, 60,           //18 - 19
  55, 50, 50, 40, 30, 20, 10, 5,            //20 - 21
  0, 0, 0, 0, 0, 0, 0, 0                    //22 - 23
};


byte swtled[96] = { // canal 2 Blanc froid utilisé
  0, 0, 0, 0, 0, 0, 0, 0,                   //0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,                   //2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,                   //4 - 5
  0, 0, 0, 0, 0, 0, 0, 0,                   //6 - 7
  5, 5, 5, 5, 10, 10, 10, 10,               //8 - 9
  10, 10, 10, 10, 15, 15, 15, 25,           //10 - 11
  25, 30, 40, 50, 75, 80, 80, 85,           //12 - 13
  85, 85, 85, 85, 90, 90, 90, 90,           //14 - 15
  85, 85, 85, 85, 85, 85, 85, 75,           //16 - 17
  75, 75, 75, 75, 70, 70, 70, 60,           //18 - 19
  55, 50, 50, 40, 30, 20, 10, 5,            //20 - 21
  0, 0, 0, 0, 0, 0, 0, 0                    //22 - 23
};


byte rswtled[96] = { // Canal 3 Bleu utilisé
  1, 1, 1, 1, 1, 1, 1, 1,                   //0 - 1
  1, 1, 1, 1, 1, 1, 1, 1,                   //2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,                   //4 - 5
  0, 0, 0, 0, 0, 0, 0, 0,                   //6 - 7
  5, 5, 5, 5, 10, 10, 10, 10,               //8 - 9
  10, 10, 10, 10, 15, 15, 15, 25,           //10 - 11
  25, 30, 40, 50, 75, 80, 80, 75,           //12 - 13
  75, 75, 75, 75, 75, 75, 75, 75,           //14 - 15
  75, 75, 70, 70, 70, 70, 70, 70,           //16 - 17
  70, 70, 70, 70, 70, 70, 70, 70,           //18 - 19
  70, 65, 65, 60, 50, 40, 30, 20,           //20 - 21
  10, 8, 5, 1, 1, 1, 1, 1                   //22 - 23
};

byte rled[96] = { // Canal 4 Rouge utilisé
  0, 0, 0, 0, 0, 0, 0, 0,                   //0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,                   //2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,                   //4 - 5
  0, 0, 0, 0, 0, 0, 0, 0,                   //6 - 7
  5, 5, 5, 5, 10, 10, 10, 10,               //8 - 9
  10, 10, 10, 10, 15, 15, 15, 25,           //10 - 11
  20, 20, 20, 20, 30, 40, 40, 50,           //12 - 13
  50, 50, 50, 50, 50, 55, 55, 50,           //14 - 15
  50, 50, 50, 50, 50, 50, 50, 50,           //16 - 17
  50, 50, 50, 50, 40, 40, 40, 50,           //18 - 19
  50, 50, 45, 45, 35, 25, 15, 10,           //20 - 21
  5, 4, 2, 0, 0, 0, 0, 0                    //22 - 23
};

byte uvled[96] = { // Canal 5 UV pas utilisé
  0, 0, 0, 0, 0, 0, 0, 0,                   //0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,                   //2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,                   //4 - 5
  0, 0, 0, 0, 0, 0, 0, 0,                   //6 - 7
  0, 0, 0, 0, 0, 0, 0, 0,                   //8 - 9
  0, 0, 0, 2, 2, 5, 10, 10,                 //10 - 11
  10, 10, 10, 10, 10, 10, 10, 15,           //12 - 13
  15, 15, 15, 15, 10, 10, 10, 10,           //14 - 15
  10, 10, 10, 10, 10, 10, 10, 10,           //16 - 17
  10, 10, 10, 10, 5, 5, 5, 3,               //18 - 19
  2, 0, 0, 0, 0, 0, 0, 0,                   //20 - 21
  0, 0, 0, 0, 0, 0, 0, 0                    //22 - 23
};

byte oLed[96] = {  // Canal 6 Orange pas utilisé
  0, 0, 0, 0, 0, 0, 0, 0,                   //0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,                   //2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,                   //4 - 5
  0, 0, 0, 0, 0, 0, 0, 0,                   //6 - 7
  0, 0, 0, 0, 0, 0, 0, 0,                   //8 - 9
  0, 0, 0, 2, 2, 5, 10, 10,                 //10 - 11
  10, 10, 10, 10, 10, 10, 10, 15,           //12 - 13
  15, 15, 15, 15, 10, 10, 10, 10,           //14 - 15
  10, 10, 10, 10, 10, 10, 10, 10,           //16 - 17
  10, 10, 10, 10, 5, 5, 5, 3,               //18 - 19
  2, 0, 0, 0, 0, 0, 0, 0,                   //20 - 21
  0, 0, 0, 0, 0, 0, 0, 0                    //22 - 23
};

byte gled[96] = {  // Canal 7 Vert utilisé
  0, 0, 0, 0, 0, 0, 0, 0,                   //0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,                   //2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,                   //4 - 5
  0, 0, 0, 0, 0, 0, 0, 0,                   //6 - 7
  0, 0, 0, 0, 0, 0, 0, 0,                   //8 - 9
  0, 0, 0, 2, 2, 5, 10, 10,                 //10 - 11
  10, 10, 10, 10, 10, 10, 10, 15,           //12 - 13
  15, 15, 15, 15, 10, 10, 10, 10,           //14 - 15
  10, 10, 10, 10, 10, 10, 10, 10,           //16 - 17
  10, 10, 10, 10, 5, 5, 5, 3,               //18 - 19
  2, 0, 0, 0, 0, 0, 0, 0,                   //20 - 21
  0, 0, 0, 0, 0, 0, 0, 0                    //22 - 23
};

byte tled[96];     // tableau temporaire pour stocker les valeurs de led
word tFA[48];      // Tableau temporaire pour la température dans l'aquarium
word media[48] ;
word tF1[48] ;     // Réseau temporaire pour la température (capteur 1)
word tF2[48] ;     // Réseau temporaire pour la température (capteur 2)

boolean refreshAll,  PlsMnsPress, ButtonDisable;
int XShift, YShift, ButDist, Min, Max, Step, PlsMnsCount;

/********************************Boutons virtuels dans le menu principal *********************************/
const int tanD[] = {4, 22, 104, 58};       // Время и Дата
const int temC[] = {4, 66, 104, 102};      // Температура Воды в Аквариуме
const int feed[] = {4, 110, 104, 146};     // Установка времени кормления
const int gSet[] = {4, 154, 104, 190};     // Основные настройки
const int tesT[] = {110, 22, 210, 58};     // Тест установок яркости, за сутки (авто-тест обычный)
const int ledChM[] = {110, 66, 210, 102};  // Настройка каналов по цветам
const int Sector[] = {110, 110, 210, 146}; // Настройка каналов по секторам времени
const int timday[] = {110, 154, 210, 190}; // Суточный Tаймер
const int tesT2[] = {216, 22, 316, 58};    // Тест установок яркости, за сутки (тест с графиками каналов)
const int Preset[] = {216, 66, 316, 102};  // Запись присетов
const int PHset[] = {216, 110, 316, 146};  // Настройка PH
const int Pumpset[] = {216, 154, 316, 190}; // Настройка дозирующих помп
const int logW[] = {16, 208, 92, 224};     // график температуры воды за сутки
const int logH[] = {110, 208, 186, 224};   // график температуры радиатора за сутки

/************************* Виртуальные кнопки в Окне настройки времени и даты **********************/
const int houU[] = {106, 22, 136, 44};     // Hour up
const int houD[] = {106, 76, 136, 98};     // Hour down
const int minU[] = {176, 22, 206, 44};     // Min up
const int minD[] = {176, 76, 206, 98};     // Min down
const int dayU[] = {106, 112, 136, 134};   // Day up
const int dayD[] = {106, 166, 136, 188};   // Day down
const int monU[] = {176, 112, 206, 134};   // Month up
const int monD[] = {176, 166, 206, 188};   // Month down
const int yeaU[] = {264, 112, 294, 134};   // Year up
const int yeaD[] = {264, 166, 294, 188};   // Year down

/************************ Виртуальные кнопки в Окне контроля температуры *******************/
const int temP[] = {214, 48, 254, 78};         // Температура плюс
const int temM[] = {64, 48, 104, 78};          // Температура минус
const int offP[] = {214, 98, 254, 128};        // Offset plus
const int offM[] = {64, 98, 104, 128};         // Offset minus
const int SoundAlarmTp[] = {214, 148, 254, 178}; // Sound Alarm Temp +
const int SoundAlarmTm[] = {64, 148, 104, 178}; // Sound Alarm Temp -
const int weatH[] = {4, 80, 38, 160};          // Лог температуры

/**************************** LED TESTING MENU BUTTONS *******************************/
const int tstLA[] = {40, 59, 280, 99};     // "Test LED Array Output" settings
const int cntIL[] = {40, 109, 280, 149};   // "Control Individual Leds" settings

/*************************Виртуальные кнопки в меню тест LED ***************************/
const int stsT[] = {110, 105, 200, 175};   // Start/stop
const int tenM[] = {20, 120, 90, 160};     // -10s
const int tenP[] = {220, 120, 290, 160};   // +10s

/************************* CHANGE LED VALUES MENU BUTTONS ****************************/
const int btCIL[] = {5, 188, 90, 220};     // Back to Change Individual LEDs Screen
const int ledChV[] = {110, 200, 210, 220}; // LED Change Values
const int eeprom[] = {215, 200, 315, 220}; // Save to EEPROM (Right Button)

/********************** кнопки установки яркости луны ****************************/
const int MINiM[] = {12, 142, 44, 172};    // MinI minus
const int MINiP[] = {114, 142, 146, 172};  // MinI plus
const int MAXiM[] = {174, 142, 206, 172};  // MaxI minus
const int MAXiP[] = {276, 142, 308, 172};  // MaxI plus

/**************************WAVEMAKER BUTTONS (вейв ШИМ) ************************/
const int ModePump1[] = {15, 18, 51, 47};  // Mode 1 Pulse
const int ModePump2[] = {59, 18, 95, 47};  // Mode 2 Ramp
const int ModePump3[] = {103, 18, 139, 47}; // Mode 3 ReefCrazy
const int ModePump4[] = {147, 18, 183, 47}; // Mode 4 Pump ON
const int ModePump5[] = {191, 18, 226, 47}; // Mode 5 Nutrient Zero
const int ModePump6[] = {236, 27, 269, 46}; // Pump OFF
const int ModePump7[] = {277, 27, 310, 46}; // Pump Max Level

const int SpeedPplus[] = {40, 100, 70, 122};   // Cкорость помпы +
const int SpeedPminus[] = {40, 160, 70, 182};  // Cкорость помпы -

const int SpeedMax[] = {7, 107, 22, 137}; // Максимальная скорость
const int SpeedMin[] = {7, 148, 22, 178}; // Минимальная скорость

const int MaxPlus1[] = {92, 105, 112, 125};   // Максимальная мощность помпы 1 +
const int MaxMinus1[] = {92, 155, 112, 175};  // Максимальная мощность помпы 1 -
const int MinPlus1[] = {122, 105, 142, 125};  // Минимальная мощность помпы 1 +
const int MinMinus1[] = {122, 155, 142, 175}; // Минимальная мощность помпы 1 -
const int MaxPlus2[] = {160, 105, 180, 125};  // Максимальная мощность помпы 2 +
const int MaxMinus2[] = {160, 155, 180, 175}; // Максимальная мощность помпы 2 -
const int MinPlus2[] = {190, 105, 210, 125};  // Минимальная мощность помпы 2 +
const int MinMinus2[] = {190, 155, 210, 175}; // Минимальная мощность помпы 2 -

/************************ кнопки установки времени кормления ************************/
const int houP[] = {110, 34, 140, 56};      //hour up
const int minP[] = {180, 34, 210, 56};      //min up
const int ampmP[] = {265, 34, 295, 56};     //AM/PM up
const int houM[] = {110, 93, 140, 115};     //hour down
const int minM[] = {180, 93, 210, 115};     //min down
const int ampmM[] = {265, 93, 295, 115};    //AM/PM down

/*********************** кнопки дозатора удобрений***********************************/
const int dos1b[] = {5, 31, 100, 56};        // дозатор1
const int dos2b[] = {5, 75, 100, 100};       // дозатор2
const int dos3b[] = {5, 120, 100, 145};      // дозатор3
const int dos4b[] = {5, 163, 100, 188};      // дозатор4
const int dosT1[] = {10, 21, 210, 51};       // время дозатора 1
const int dosT2[] = {10, 65, 210, 95};       // время дозатора 2
const int dosT3[] = {10, 110, 210, 140};     // время дозатора 3
const int dosT4[] = {10, 153, 210, 183};     // время дозатора 4
const int dosval1[] = {230, 26, 315, 56};    // объем дозатора 1
const int dosval2[] = {230, 70, 315, 100};   // объем дозатора 2
const int dosval3[] = {230, 115, 315, 145};  // объем дозатора 3
const int dosval4[] = {230, 158, 315, 188};  // объем дозатора 4
const int dosCal[] = {16, 208, 91, 223};     // калибровка

const int valUP[] =  {170, 44, 178, 75};      // объем вверх
const int valDOWN[] = {265, 44, 313, 75};     // объем вниз
const int numUP[] =  {170, 79, 178, 110};     // количество доз вверх
const int numDOWN[] = {265, 79, 313, 110};    // количество доз вниз
const int intUP[] =  {170, 114, 178, 145};    // интервал часы вверх
const int intDOWN[] = {265, 114, 313, 145};   // интервал часы вниз
const int calUP[] =  {170, 155, 178, 181};    // калибровка вверх
const int calDOWN[] = {265, 150, 313, 181};   // калибровка вниз

/*************************** PARAMÈTRES DE COULEURS DU BOUTON **************************/
const int RGBch1[] = {20, 90, 55, 120};        // 1 канал
const int RGBch2[] = {60, 90, 95, 120};        // 2 канал
const int RGBch3[] = {100, 90, 135, 120};      // 3 канал
const int RGBch4[] = {140, 90, 175, 120};      // 4 канал
const int RGBch5[] = {40, 125, 75, 155};       // 5 канал
const int RGBch6[] = {80, 125, 115, 155};      // 6 канал
const int RGBch7[] = {120, 125, 155, 155};     // 7 канал
const int MOONch[] = {60, 165, 135, 195};      // le canal de la lune

/***************************** MISCELLANEOUS BUTTONS *********************************/
const int backGS[] = {4, 200, 78, 223};    // BACK  (маленькая)
const int nextGS[] = {83, 200, 157, 223};  // NEXT  (маленькая)
const int prSAVEgs[] = {162, 200, 236, 223}; // SAVE  (маленькая)
const int canCgs[] = {241, 200, 315, 223}; // CANCEL(маленькая)

const int HoodFanTm[] = {80, 53, 115, 83}; // Hood Fan Temp -  температура радиатора
const int HoodFanTp[] = {205, 53, 240, 83}; // Hood Fan Temp +
const int SumpFanTm[] = {80, 144, 115, 174}; // Hood 2 Fan Temp -
const int SumpFanTp[] = {205, 144, 240, 174}; // Hood 2 Fan Temp +

const int SalaRm[] = {4, 145, 56, 165};    // кнопка установки настройки звуковой тревоги
const int SoundATp[] = {215, 117, 257, 151}; // Sound Alarm Temp +
const int SoundATm[] = {63, 117, 105, 151}; // Sound Alarm Temp -

const int back[] = {5, 200, 105, 223};     // BACK   (большая)
const int prSAVE[] = {110, 200, 210, 223}; // SAVE   (большая)
const int canC[] = {215, 200, 315, 223};   // CANCEL (большая)

const int ButMns[] = {0, 0, 40, 30};          // universal +/- button counter, defaul coordinate, 24x24 size

// Графики освещенности LED
const int Wg[] = {37, 0, 67, 15};          // белый канал
const int Bg[] = {77, 0, 107, 15};         // голубой канал
const int RBg[] = {117, 0, 147, 15};       // канал роял
const int Rg[] = {157, 0, 187, 15};        // красный канал
const int UVg[] = {197, 0, 227, 15};       // фиолет канал
const int ORg[] = {237, 0, 267, 15};       // оранжевый канал
const int GRg[] = {277, 0, 307, 15};       // зеленый канал

// кнопки утановки времени таймеров
const int btonhup[] = {20, 44, 68, 75};        // часы вкл вверх
const int btonhdn[] = {20, 149, 68, 180};       // часы вкл вниз
const int btonmup[] = {89, 44, 137, 75};       // минуты вкл вверх
const int btonmdn[] = {89, 149, 137, 180};     // минуты вкл вниз
const int btofhup[] = {185, 44, 233, 75};      // часы выкл вверх
const int btofhdn[] = {185, 149, 233, 180};     // часы выкл вниз
const int btofmup[] = {255, 44, 303, 75};      // минуты выкл вверх
const int btofmdn[] = {255, 149, 303, 180};    // минуты выкл вниз

// начало светового дня, конец светового дня (рассвет, закат)
const int StartDay[] = {5, 0, 74, 14};     // рассвет
const int StopDay[] = {241, 0, 315, 14};   // закат
const int Psect[] = {140, 200, 222, 220};  // Prev SECTOR (предыдущий сектор)
const int Nsect[] = {233, 200, 315, 220};  // NEXT SECTOR (следующий сектор)
const int Yes[] = {110, 125, 150, 145};    // Yes
const int No[] = {170, 125, 210, 145};     // No

// пресеты
const int LedPres1[] = {7, 0, 78, 14};      // Led Preset 1 button
const int LedPres2[] = {85, 0, 156, 14};    // Led Preset 2 button
const int LedPres3[] = {163, 0, 234, 14};   // Led Preset 3 button
const int LedPres4[] = {241, 0, 312, 14};   // Led Preset 4 button

// кнопки в Окне ЛОГ температуры радиаторов 1,2
const int Fg1[] = {112, 200, 157, 218};     // Датч1
const int Fg2[] = {163, 200, 208, 218};     // Датч2

//------------------------ Подсветка экрана --------------------------------
const int gseB[] = {64, 44};               // brightness bar

//----------------- кнопки YES  NO в меню главных настроек -----------------
const int ClocBlno[] = {255, 51, 305, 71}; // setScreensaverDOWonOff  NO
const int ClocBlyes[] = {185, 51, 235, 71}; // setScreensaverDOWonOff  YES
const int DledOn[] = {195, 27, 235, 47};   // setDimLEDsOnOff ON
const int DledOff[] = {255, 27, 295, 47};  // setDimLEDsOnOff OFF
const int SsavOn[] = {195, 101, 235, 121}; // setScreensaverOnOff ON
const int SsavOff[] = {255, 101, 295, 121}; // setScreensaverOnOff OFF
const int DimmLOn[] = {195, 169, 235, 189}; // DimmL ON
const int DimmLOff[] = {255, 169, 295, 189}; // DimmL OFF

/*******************************MESURE PH I2C****************************************/
#ifdef PH_sensor_I2C

void CheckPH() {

  int x;
  for (x = 0; x < sampleSize; x++)
  {

    phVolt = getPHVolts();
    tempAdjusted10 = adjustPHBasedOnTemp(10, calibrationTempC);
    voltsPerPH = abs((volt10 - volt7) / (tempAdjusted10 - 7));

    realPHVolt = (volt7 - phVolt);
    phUnits = realPHVolt / voltsPerPH;
    measuredPH = 7 + phUnits;

    roomTempC =  getRoomTemperatureC();
    roomTempCompensatedMeasuredPH = adjustPHBasedOnTemp(measuredPH, roomTempC);

    avgMeasuredPH += measuredPH;
    avgRoomTemperatureCompensatedMeasuredPH += roomTempCompensatedMeasuredPH;
    avgRoomTempC += roomTempC;
    avgPHVolts += phVolt;

  }

  avgMeasuredPH /= sampleSize;
  avgRoomTemperatureCompensatedMeasuredPH /= sampleSize;
  avgRoomTempC /= sampleSize;
  avgPHVolts /= sampleSize;

  setFont(SMALL, 0, 255, 255, 0, 0, 0);
  if (dispScreen == 0 && screenSaverCounter < setScreenSaverTimer && avgMeasuredPH > 3 && avgMeasuredPH < 9.9) {
    myGLCD.setFont(BigRusFont);
    myGLCD.printNumF(avgMeasuredPH, 1, 116, 110);
  }
  else {
    if (dispScreen == 0 && screenSaverCounter < setScreenSaverTimer) {
      //    myGLCD.drawBitmap(128, 108, 24, 24, clos, 1);    // картинка  крестик
      myGLCD.setFont(BigRusFont);
      myGLCD.print("*.*", 116, 110);
    }
  }
  if (dispScreen == 11 && screenSaverCounter < setScreenSaverTimer && avgMeasuredPH > 3 && avgMeasuredPH < 14) {
    myGLCD.setFont(BigRusFont);
    myGLCD.printNumF(avgMeasuredPH, 1, 232, 45);
    myGLCD.printNumF(avgPHVolts, 4, 215, 60);
    myGLCD.printNumF(volt7, 4, dosval3[0] + 2, dosval3[1] + 6);
    myGLCD.printNumF(volt10, 4, dosval4[0] + 2, dosval4[1] + 6);
  }

  if (avgMeasuredPH >= SetvalPH) {
    co = 1;
  } else {
    co = 0;
  }
}
#endif

//**************** fonctionne avec un capteur de PH analogique ******************//
#ifdef Analog_PH_sensor

#define SensorPin 10         //pH meter Analog output to Arduino Analog Input 10
unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10], temp;

void CheckAPH()
{
  for (int i = 0; i < 10; i++) //Get 10 sample value from the sensor for smooth the value
  {
    buf[i] = analogRead(SensorPin);
    delay(10);
  }
  for (int i = 0; i < 9; i++) //sort the analog from small to large
  {
    for (int j = i + 1; j < 10; j++)
    {
      if (buf[i] > buf[j])
      {
        temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }
  avgValue = 0;
  for (int i = 2; i < 8; i++)               //take the average value of 6 center sample
    avgValue += buf[i];
  float phValue = (float)avgValue * 5.0 / 1024 / 6; //convert the analog into millivolt

  phVolt = phValue;

  voltsPerPH = abs((volt10 - volt7) / 3);

  realPHVolt = (volt7 - phVolt);
  phUnits = realPHVolt / voltsPerPH;
  measuredPH = 7 + phUnits;

  avgMeasuredPH = measuredPH;

  avgPHVolts = phVolt;

  setFont(SMALL, 0, 255, 255, 0, 0, 0);
  if (dispScreen == 0 && screenSaverCounter < setScreenSaverTimer && avgMeasuredPH > 3 && avgMeasuredPH < 9.9) {
    myGLCD.setFont(BigRusFont);
    myGLCD.printNumF(avgMeasuredPH, 1, 116, 110);
  }
  else {
    if (dispScreen == 0 && screenSaverCounter < setScreenSaverTimer) {
      //   myGLCD.drawBitmap(128, 108, 24, 24, clos, 1);    // картинка  крестик
      myGLCD.setFont(BigRusFont);
      myGLCD.print("*.*", 116, 110);
    }
  }
  if (dispScreen == 11 && screenSaverCounter < setScreenSaverTimer && avgMeasuredPH > 3 && avgMeasuredPH < 14) {
    myGLCD.setFont(BigRusFont);
    myGLCD.printNumF(avgMeasuredPH, 1, 232, 45);
    myGLCD.printNumF(avgPHVolts, 4, 215, 60);
    myGLCD.printNumF(volt7, 4, dosval3[0] + 2, dosval3[1] + 6);
    myGLCD.printNumF(volt10, 4, dosval4[0] + 2, dosval4[1] + 6);
  }
  if (avgMeasuredPH >= SetvalPH) {
    co = 1;
  } else {
    co = 0;
  }
}
#endif

void CheckADC1115PH() {
  // ADC ADS1115 read code
  Wire.requestFrom(adc_addr, 2);
  while (Wire.available()) // ensure all the data comes in
  {
    highbyte = Wire.read(); // high byte * B11111111
    lowbyte = Wire.read(); // low byte
  }
  adc = highbyte << 8;
  adc = adc + lowbyte;

  phVolt = abs(adc * 0.00001);
  voltsPerPH = abs((volt10 - volt7) / 3);
  realPHVolt = (volt7 - phVolt);
  phUnits = realPHVolt / voltsPerPH;
  measuredPH = 7 + phUnits;
  avgMeasuredPH = measuredPH;
  avgPHVolts = phVolt;

  setFont(SMALL, 0, 255, 255, 0, 0, 0);
  if (dispScreen == 0 && screenSaverCounter < setScreenSaverTimer && avgMeasuredPH > 3 && avgMeasuredPH < 9.9) {
    myGLCD.setFont(BigRusFont);
    myGLCD.printNumF(avgMeasuredPH, 1, 116, 110);
  }
  else {
    if (dispScreen == 0 && screenSaverCounter < setScreenSaverTimer) {
      //   myGLCD.drawBitmap(128, 108, 24, 24, clos, 1);    // картинка  крестик
      myGLCD.setFont(BigRusFont);
      myGLCD.print("*.*", 116, 110);
    }
  }
  if (dispScreen == 11 && screenSaverCounter < setScreenSaverTimer && avgMeasuredPH > 3 && avgMeasuredPH < 14) {
    myGLCD.setFont(BigRusFont);
    myGLCD.printNumF(avgMeasuredPH, 1, 232, 45);
    myGLCD.printNumF(avgPHVolts, 4, 215, 60);
    myGLCD.printNumF(volt7, 4, dosval3[0] + 2, dosval3[1] + 6);
    myGLCD.printNumF(volt10, 4, dosval4[0] + 2, dosval4[1] + 6);
  }

  if (avgMeasuredPH >= SetvalPH) {
    co = 1;
  } else {
    co = 0;
  }
}

/***********************FONCTIONS TEXTE ET GRAPHIQUES******************************************/
void CountDown(byte timer) {                          // Pause compte à rebours pour l'alimentation
  setFont(SMALL, 255, 255, 0, 0, 0, 0);
  timer = Tpause - timer;
  myGLCD.printNumI((timer * 5 / 60), 274, 56, 2, '0');                           // minutes
  myGLCD.print(print_text[56], 289, 56);                                         // :
  myGLCD.printNumI((timer * 5 - ((timer * 5 / 60) * 60)), 297, 56, 2, '0');
}                // секунды

//********************************** Обычный текст из массива Menu_Text***********************************//
void Menu_Text(byte MenuText0, int x1, int y1, byte font = 0, byte background = 2 )
{
  strcpy_P(buffer, (char*) pgm_read_word_near (&(Text_table [MenuText0])));

  if (font == 0) {
    myGLCD.setFont(BigRusFont); // Big Rus font
  }
  if (font == 1) {
    myGLCD.setFont(RusFont1); // Small Rus font
  }
  if (font == 2) {
    myGLCD.setFont(RusFont2);
  }
  if (font == 3) {
    myGLCD.setFont(RusFont3);
  }
  if (font == 6) {
    myGLCD.setFont(RusFont6);
  }

  if (background == 0) {
    myGLCD.setColor(38, 195, 178);              // текст бирюзовый
    myGLCD.setBackColor(30, 30, 30);
  }          // фон серый

  if (background == 1) {
    myGLCD.setColor(255, 255, 0);               // текст желтый
    myGLCD.setBackColor(64, 64, 64);
  }           // фон серый

  if (background == 2) {
    myGLCD.setColor(0, 255, 100);               // текст зеленый
    myGLCD.setBackColor(0, 0, 0);
  }              // фон черный

  if (background == 3) {
    myGLCD.setColor(255, 50, 50);                // текст красный
    myGLCD.setBackColor(0, 0, 0);
  }               // фон черный

  if (background == 4) {
    myGLCD.setColor(50, 50, 255);                // текст синий
    myGLCD.setBackColor(0, 0, 0);
  }               // фон черный

  if (background == 5) {
    myGLCD.setColor(255, 255, 255);               // текст белый
    myGLCD.setBackColor(0, 0, 255);
  }              // фон синий

  if (background == 6) {
    myGLCD.setColor(0, 0, 0);                     // текст черный
    myGLCD.setBackColor(255, 240, 255);
  }          // фон белый (серый)

  if (background == 7) {
    myGLCD.setColor(0, 0, 0);                     // текст черный
    myGLCD.setBackColor(0, 255, 0);
  }              // фон зеленый

  if (background == 8) {
    myGLCD.setColor(32, 255, 255);                // текст бирюзовый
    myGLCD.setBackColor(0, 0, 0);
  }                // фон черный

  myGLCD.print(buffer, x1, y1);
}

void drawUpButtonSlide(int x, int y) { // кнопка в верх, со стрелкой в меню слайдера
  myGLCD.setColor(64, 64, 128);
  myGLCD.fillRoundRect(x, y, x + 30, y + 22);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(x, y, x + 30, y + 22);
  myGLCD.setColor(128, 128, 255);
  for (int i = 0; i < 15; i++) myGLCD.drawLine(x + 5 + (i / 1.5), y + 18 - i, x + 26 - (i / 1.5), y + 18 - i);
}

void drawDownButtonSlide(int x, int y) {
  myGLCD.setColor(64, 64, 128);
  myGLCD.fillRoundRect(x, y, x + 30, y + 22);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(x, y, x + 30, y + 22);
  myGLCD.setColor(128, 128, 255);
  for (int i = 0; i < 15; i++) myGLCD.drawLine(x + 5 + (i / 1.5), y + 4 + i, x + 26 - (i / 1.5), y + 4 + i);
}

void printButton(char* text, int x1, int y1, int x2, int y2, boolean fontsize = false, boolean background = false) {
  int stl = strlen(text);
  int fx, fy;

  if (background == true) {                   // белый фонт, зелёный фон
    myGLCD.setColor(0, 180, 86);            // цвет зеленый для ON
    myGLCD.fillRoundRect (x1, y1, x2, y2);
    myGLCD.setColor(255, 255, 255);         // текст белый
    myGLCD.drawRoundRect (x1, y1, x2, y2);
    myGLCD.setBackColor(0, 180, 86);        // вокруг текста  зелёный фон (70, 200, 0);
    myGLCD.setColor(0, 0, 0);
  } else {
    myGLCD.setColor(0, 0, 255);         // белый фонт, голубой фон
    myGLCD.fillRoundRect (x1, y1, x2, y2);
    myGLCD.setColor(255, 255, 255);
    myGLCD.drawRoundRect (x1, y1, x2, y2);
    myGLCD.setBackColor(0, 0, 255);
  }
  if (fontsize) {
    myGLCD.setFont(BigRusFont);   // большой шрифт
    fx = x1 + (((x2 - x1 + 1) - (stl * 16)) / 2); fy = y1 + (((y2 - y1 + 1) - 16) / 2);
    myGLCD.print(text, fx, fy);
  } else {
    myGLCD.setFont(RusFont6); // маленький шрифт
    fx = x1 + (((x2 - x1) - (stl * 8)) / 2); fy = y1 + (((y2 - y1 - 1) - 8) / 2);
    myGLCD.print(text, fx + 2, fy - 1);
  }
}

//--------------------- Кнопки на русском -----------------------
void printButtonRUS(char* text, int x1, int y1, int x2, int y2, boolean fontsize = false) {
  int stl = strlen(text);
  int fx, fy;
  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect (x1, y1, x2, y2);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect (x1, y1, x2, y2);
  myGLCD.setBackColor(0, 0, 255);
  myGLCD.setFont(RusFont6); fx = x1 + (((x2 - x1) - (stl * 8)) / 2); fy = y1 + (((y2 - y1 - 1) - 6) / 2);
  myGLCD.print(text, fx + 1, fy - 2);
} // fx - по горизонтали, fy - по вертикали

// кнопки со стрелками для меню серво (маленькие)
void drawUpButton1(int x, int y) { // вверх
  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect(x, y, x + 18, y + 18);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(x, y, x + 18, y + 18);
  for (int i = 0; i < 14; i++) myGLCD.drawLine(x + 1 + (i / 1.5), y + 16 - i, x + 17 - (i / 1.5), y + 16 - i);
}

void drawDownButton1(int x, int y) { // вниз
  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect(x, y, x + 18, y + 18);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(x, y, x + 18, y + 18);
  for (int i = 0; i < 14; i++) myGLCD.drawLine(x + 1 + (i / 1.5), y + 2 + i, x + 17 - (i / 1.5), y + 2 + i);
}

// кнопки со стрелками (обычные) в меню настройки времени
void drawUpButton(int x, int y) {  // вверх
  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect(x, y, x + 25, y + 25);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(x, y, x + 25, y + 25);
  for (int i = 0; i < 15; i++) myGLCD.drawLine(x + 3 + (i / 1.5), y + 19 - i, x + 22 - (i / 1.5), y + 19 - i);
}

void drawDownButton(int x, int y) { // вниз
  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect(x, y, x + 25, y + 25);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(x, y, x + 25, y + 25);
  for (int i = 0; i < 15; i++) myGLCD.drawLine(x + 3 + (i / 1.5), y + 6 + i, x + 22 - (i / 1.5), y + 6 + i);
}

// ----- виртуальные кнопки серые ------
void printButton100(char* text, int x1, int y1, int x2, int y2, boolean fontsize = false) {
  int stl = strlen(text);
  int fx, fy;
  myGLCD.setColor(100, 100, 100);
  myGLCD.fillRoundRect (x1, y1, x2, y2);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect (x1, y1, x2, y2);
  myGLCD.setBackColor(100, 100, 100);
  if (fontsize) {
    myGLCD.setFont(BigRusFont);
    fx = x1 + (((x2 - x1 + 1) - (stl * 16)) / 2); fy = y1 + (((y2 - y1 + 1) - 16) / 2);
    myGLCD.print(text, fx, fy);
  } else {
    myGLCD.setFont(RusFont6);
    fx = x1 + (((x2 - x1) - (stl * 8)) / 2); fy = y1 + (((y2 - y1 - 1) - 8) / 2);
    myGLCD.print(text, fx + 2, fy - 1);
  }
}

// ----- виртуальные кнопки R ------
void printButton104(char* text, int x1, int y1, int x2, int y2, boolean fontsize = false) {
  int stl = strlen(text);
  int fx, fy;
  myGLCD.setColor(255, 0, 0);             // цвет внутреннего заполнения кнопки - красный
  myGLCD.fillRoundRect (x1, y1, x2, y2);
  myGLCD.setColor(255, 255, 255);         // цвет рамки вокруг кнопки - бирюзовый (88, 255, 238);
  myGLCD.drawRoundRect (x1, y1, x2, y2);
  myGLCD.setBackColor(255, 0, 0);
  if (fontsize) {
    myGLCD.setFont(BigRusFont);
    fx = x1 + (((x2 - x1 + 1) - (stl * 16)) / 2); fy = y1 + (((y2 - y1 + 1) - 16) / 2);
    myGLCD.print(text, fx, fy);
  } else {
    myGLCD.setFont(RusFont6);
    fx = x1 + (((x2 - x1) - (stl * 8)) / 2); fy = y1 + (((y2 - y1 - 1) - 8) / 2);
    myGLCD.print(text, fx + 2, fy - 1);
  }
}

void printVBar(int val, int x1, int y1, const byte rgb[]) { // бар яркости подсветки LCD
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRect (x1 - 1, y1 + 21, x1 + 30, y1 + 125); // белая рамка
  myGLCD.setColor(0, 0, 125); // цвет темно-синий
  //                         V -  ширина активного бара
  myGLCD.drawRect (x1, y1 + 22, x1 + 29, y1 + 124);
  myGLCD.setColor(rgb[0], rgb[1], rgb[2]);
  //                                         v - ширина
  myGLCD.fillRect (x1 + 1, y1 + (124 - val), x1 + 28, y1 + 124); // заполнение цветом при изменении уровня
  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRect (x1 + 1, y1 + 22, x1 + 28, y1 + (124 - val));
  drawUpButtonSlide (x1, y1 - 15);   // кнопка вверх (x1, y1-10);
  drawDownButtonSlide (x1, y1 + 140); // кнопка вниз  (x1, y1+134);
  setFont(SMALL, 255, 255, 255, 0, 0, 0);
  //                                     V - яркость в %
  if (val > -1) myGLCD.printNumI(val, x1 + 16 - (intlen(val) * 4), y1 + 66);
}

int intlen(int number) {   // number of digits
  int length;
  char tmpChar[5];
  itoa(number, tmpChar, 10); length = strlen(tmpChar);
  return length;
}

void printCoun() {
  myGLCD.setFont(RusFont3);
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(0, 0, 255);
}
//setFont(RUS3, 255, 255, 255, 0, 0, 255); }

void printFont() { // шрифт и фонт в таймерах
  myGLCD.setFont(DotMatrix_M_Num);
  myGLCD.setColor(0, 255, 255);
  myGLCD.setBackColor(0, 0, 0);
}

void waitForIt(int x1, int y1, int x2, int y2) {   // Красная рамка при нажатии на экран
  myGLCD.setColor(255, 0, 0);
  myGLCD.drawRoundRect (x1, y1, x2, y2);
  waitForTouchRelease();
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect (x1, y1, x2, y2);
}

void OnOffTimer1() { // графика on off для Таймера 1
  Menu_Text(156, 105, 25, 0, 4 ); // AUTO
  myGLCD.drawRect(102, 17, 171, 49);
  myGLCD.setColor(80, 80, 80);
  myGLCD.print(print_text[213], 259, 25);  // OFF
  myGLCD.print(print_text[212], 188, 25);  // ON
  myGLCD.setColor(190, 190, 190);        // серый
  myGLCD.drawRect(177, 19, 242, 47);     // on
  myGLCD.drawRect(175, 17, 244, 49);     // ON
  myGLCD.drawRect(248, 17, 317, 49);     // OFF
  myGLCD.drawRect(250, 19, 315, 47);
}       // off

void OnOffTimer2() { // графика on off для Таймера 2
  Menu_Text(156, 105, 61, 0, 4 );   // AUTO
  myGLCD.drawRect(102, 53, 171, 85);  // auto off
  myGLCD.setColor(80, 80, 80);
  myGLCD.print(print_text[213], 259, 61);   // OFF
  myGLCD.print(print_text[212], 188, 61);   // ON
  myGLCD.setColor(190, 190, 190);         // серый
  myGLCD.drawRect(177, 55, 242, 83);      // on
  myGLCD.drawRect(175, 53, 244, 85);      // ON
  myGLCD.drawRect(250, 55, 315, 83);      // off
  myGLCD.drawRect(248, 53, 317, 85);
}        // OFF

void OnOffTimer3() { // графика on off для Таймера 3
  Menu_Text(156, 105, 97, 0, 4 );         // AUTO
  myGLCD.drawRect(102, 89, 171, 121);     // auto on
  myGLCD.setColor(80, 80, 80);
  myGLCD.print(print_text[213], 259, 97);   // OFF
  myGLCD.print(print_text[212], 188, 97);   //  ON
  myGLCD.setColor(190, 190, 190);         // серый
  myGLCD.drawRect(177, 91, 242, 119);     // on
  myGLCD.drawRect(175, 89, 244, 121);     // ON
  myGLCD.drawRect(250, 91, 315, 119);     // off
  myGLCD.drawRect(248, 89, 317, 121);
}       // OFF

void OnOffTimer4() { // графика on off для Таймера 4
  Menu_Text(156, 105, 134, 0, 4 );         // AUTO
  myGLCD.drawRect(102, 125, 171, 157);     // auto on
  myGLCD.setColor(80, 80, 80);
  myGLCD.print(print_text[213], 259, 133);   // OFF
  myGLCD.print(print_text[212], 188, 134);   // ON
  myGLCD.setColor(190, 190, 190);          // серый
  myGLCD.drawRect(177, 127, 242, 155);     // on
  myGLCD.drawRect(175, 125, 244, 157);     // ON
  myGLCD.drawRect(250, 127, 315, 155);     // off
  myGLCD.drawRect(248, 125, 317, 157);
}       // OFF

void OnOffTimer5() { // графика on off для Таймера 5
  Menu_Text(156, 105, 168, 0, 4 );         // AUTO
  myGLCD.drawRect(102, 161, 171, 193);     // auto on
  myGLCD.setColor(80, 80, 80);
  myGLCD.print(print_text[213], 259, 168);   // OFF
  myGLCD.print(print_text[212], 188, 168);   // ON
  myGLCD.setColor(190, 190, 190);          // серый
  myGLCD.drawRect(177, 163, 242, 191);     // on
  myGLCD.drawRect(175, 161, 244, 193);     // ON
  myGLCD.drawRect(250, 163, 315, 191);     // off
  myGLCD.drawRect(248, 161, 317, 193);
}       // OFF

/***************************** Олег / Кнопки плюс минус для целых чисел*******************************/
int PlusMinusCountI ( boolean refreshAll, boolean ButtonDisable, int XShift, int YShift, byte ButDist, int Min, int Max, int Step, int PlsMnsCount ) // =/- всех значений
{
  /*
    int XShift - X fist left "-" button coordinate / Первая левая координата по оси X
    int YShift   Y fist left "-" button coordinate / Первая левая координата по оси Y
    byte ButDist - distance between "-" and "+" buttons / Расстояние между кнопками + и -
    int Min - minimum value of counter  / Минимальное значение счетчика
    int Max - maximum counter  / Максимальное значение счетчика
    int Step - step  / Шаг отсчета
    int PlsMnsCount - input/output value / изменяемая переменнная
    boolean refreshAll - if "false" - draw only static element, and display values without change / Только нарисовать (значения не изменяются)
    boolean ButtonDisable - if "false" - disable touch button function  / Отключение чувствительности к нажатию тача
  */
  // ---------------------------------------------    draw static element / Отрисовка статичных элементов
  if (refreshAll == false) {
    PlsMnsPress = false;
    myGLCD.setColor(255, 255, 255);
    myGLCD.drawRoundRect (ButMns[0] + XShift, ButMns[1] + YShift, ButMns[2] + XShift + ButDist, ButMns[3] + YShift);
    printButton("-", (ButMns[0] + XShift), (ButMns[1] + YShift), (ButMns[2] + XShift), (ButMns[3] + YShift), LARGE);
    printButton("+", (ButMns[0] + XShift + ButDist), (ButMns[1] + YShift), (ButMns[2] + XShift + ButDist), (ButMns[3] + YShift), LARGE);
    setFont(LARGE, 255, 255, 255, 0, 0, 0);
    //   myGLCD.print("   ", (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-24),ButMns[1]+YShift+3);
    if (PlsMnsCount >= 0 && PlsMnsCount < 10 ) {
      myGLCD.printNumI(PlsMnsCount, (((ButMns[2] + XShift) + (ButMns[0] + XShift + ButDist)) / 2 - 8), ButMns[1] + YShift + 6);
    }
    if (PlsMnsCount >= 10 && PlsMnsCount < 100) {
      myGLCD.printNumI(PlsMnsCount, (((ButMns[2] + XShift) + (ButMns[0] + XShift + ButDist)) / 2 - 16), ButMns[1] + YShift + 6);
    }
    if (PlsMnsCount >= 100) {
      myGLCD.printNumI(PlsMnsCount, (((ButMns[2] + XShift) + (ButMns[0] + XShift + ButDist)) / 2 - 24), ButMns[1] + YShift + 6);
    }
  }
  //  ---------------------------wait for press button + and - / Ожидание  нажатия и отпускания кнопок + и -
  if (ButtonDisable == true) {				// Minus button / Кнопка минус
    if (x >= (ButMns[0] + XShift) && x <= (ButMns[2] + XShift) && y >= (ButMns[1] + YShift) && y <= (ButMns[3] + YShift)) //press "-" / нажатие "-"
    { waitForIt(ButMns[0] + XShift, ButMns[1] + YShift, ButMns[2] + XShift, ButMns[3] + YShift);
      PlsMnsCount -= Step;
      PlsMnsPress = true;
    }											// Plus button / Кнопка плюс
    if (x >= (ButMns[0] + XShift + ButDist) && x <= (ButMns[2] + XShift + ButDist) && y >= (ButMns[1] + YShift) && y <= (ButMns[3] + YShift)) //press "+"  / Нажатие "+"
    { waitForIt(ButMns[0] + XShift + ButDist, ButMns[1] + YShift, ButMns[2] + XShift + ButDist, ButMns[3] + YShift);
      PlsMnsCount += Step;
      PlsMnsPress = true;
    }
  }
  // ------------------------------ print counter values / вывод на экран значения счетчика --------------
  if (PlsMnsPress == true ) {
    //	  PlsMnsPress = false;
    if (PlsMnsCount > Max) {
      PlsMnsCount = Max;
    }
    if (PlsMnsCount < Min) {
      PlsMnsCount = Min;
    }
    myGLCD.setColor(0, 0, 0);
    myGLCD.fillRect(ButMns[2] + XShift + 2, ButMns[3] + YShift - 2, ButMns[0] + XShift + ButDist - 2, ButMns[1] + YShift + 2);
    setFont(LARGE, 255, 255, 255, 0, 0, 0);
    //	  myGLCD.print("   ", (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-24),ButMns[1]+YShift+3);
    if (PlsMnsCount >= 0 && PlsMnsCount < 10 ) {
      myGLCD.printNumI(PlsMnsCount, (((ButMns[2] + XShift) + (ButMns[0] + XShift + ButDist)) / 2 - 8), ButMns[1] + YShift + 6);
    }
    if (PlsMnsCount >= 10 && PlsMnsCount < 100) {
      myGLCD.printNumI(PlsMnsCount, (((ButMns[2] + XShift) + (ButMns[0] + XShift + ButDist)) / 2 - 16), ButMns[1] + YShift + 6);
    }
    if (PlsMnsCount >= 100) {
      myGLCD.printNumI(PlsMnsCount, (((ButMns[2] + XShift) + (ButMns[0] + XShift + ButDist)) / 2 - 24), ButMns[1] + YShift + 6);
    }
  }
  return PlsMnsCount;
  return PlsMnsPress;
}

/***************************** Олег / Кнопки плюс минус для чисел с запятой *******************************/
float PlusMinusCountF ( boolean refreshAll, boolean ButtonDisable, int XShift, int YShift, byte ButDist, float Min, float Max, float Step, float PlsMnsCount ) // increase/decrease all values counter
{
  /*
    int XShift - X fist left "-" button coordinate / Первая левая координата по оси X
    int YShift   Y fist left "-" button coordinate / Первая левая координата по оси Y
    byte ButDist - distance between "-" and "+" buttons / Расстояние между кнопками + и -
    int Min - minimum value of counter  / Минимальное значение счетчика
    int Max - maximum counter  / Максимальное значение счетчика
    int Step - step  / Шаг отсчета
    int PlsMnsCount - input/output value / изменяемая переменнная
    boolean refreshAll - if "false" - draw only static element, and display values without change / Только нарисовать (значения не изменяются)
    boolean ButtonDisable - if "false" - disable touch button function  / Отключение чувствительности к нажатию тача
  */
  // ---------------------------------------------    draw static element / Отрисовка статичных элементов
  if (refreshAll == false) {
    PlsMnsPress = false;
    myGLCD.setColor(255, 255, 255);
    myGLCD.drawRoundRect (ButMns[0] + XShift, ButMns[1] + YShift, ButMns[2] + XShift + ButDist, ButMns[3] + YShift);
    printButton("-", (ButMns[0] + XShift), (ButMns[1] + YShift), (ButMns[2] + XShift), (ButMns[3] + YShift), LARGE);
    printButton("+", (ButMns[0] + XShift + ButDist), (ButMns[1] + YShift), (ButMns[2] + XShift + ButDist), (ButMns[3] + YShift), LARGE);
    setFont(LARGE, 255, 255, 255, 0, 0, 0);
    //   myGLCD.print("   ", (((ButMns[2]+XShift)+(ButMns[0]+XShift+ButDist))/2-36),ButMns[1]+YShift+3);
    if (PlsMnsCount >= 0 && PlsMnsCount < 10 ) {
      myGLCD.printNumF(PlsMnsCount, 1, (((ButMns[2] + XShift) + (ButMns[0] + XShift + ButDist)) / 2 - 26), ButMns[1] + YShift + 6);
    }
    if (PlsMnsCount >= 10 && PlsMnsCount < 100) {
      myGLCD.printNumF(PlsMnsCount, 1, (((ButMns[2] + XShift) + (ButMns[0] + XShift + ButDist)) / 2 - 28), ButMns[1] + YShift + 6);
    }
    if (PlsMnsCount >= 100) {
      myGLCD.printNumF(PlsMnsCount, 1, (((ButMns[2] + XShift) + (ButMns[0] + XShift + ButDist)) / 2 - 36), ButMns[1] + YShift + 6);
    }
  }
  //  --------------------------wait for press button + and - / Ожидание  нажатия и отпускания кнопок + и -
  if (ButtonDisable == true) {				// Minus button / кнопка минус
    if (x >= (ButMns[0] + XShift) && x <= (ButMns[2] + XShift) && y >= (ButMns[1] + YShift) && y <= (ButMns[3] + YShift)) //press "-"
    { waitForIt(ButMns[0] + XShift, ButMns[1] + YShift, ButMns[2] + XShift, ButMns[3] + YShift);
      PlsMnsCount -= Step;
      PlsMnsPress = true;
    }
    // Plus button / кнопка плюс
    if (x >= (ButMns[0] + XShift + ButDist) && x <= (ButMns[2] + XShift + ButDist) && y >= (ButMns[1] + YShift) && y <= (ButMns[3] + YShift)) //press "+"
    { waitForIt(ButMns[0] + XShift + ButDist, ButMns[1] + YShift, ButMns[2] + XShift + ButDist, ButMns[3] + YShift);
      PlsMnsCount += Step;
      PlsMnsPress = true;
    }
  }
  // ------------------------------ print counter values / вывод на экран значения счетчика --------------
  if (PlsMnsPress == true ) {
    //      PlsMnsPress = false;
    if (PlsMnsCount > Max + 0.05) {
      PlsMnsCount = Max;
    }
    if (PlsMnsCount < Min) {
      PlsMnsCount = Min;
    }
    myGLCD.setColor(0, 0, 0);
    myGLCD.fillRect(ButMns[2] + XShift + 2, ButMns[3] + YShift - 2, ButMns[0] + XShift + ButDist - 2, ButMns[1] + YShift + 2);
    setFont(LARGE, 255, 255, 255, 0, 0, 0);
    if (PlsMnsCount >= 0 && PlsMnsCount < 10 ) {
      myGLCD.printNumF(PlsMnsCount, 1, (((ButMns[2] + XShift) + (ButMns[0] + XShift + ButDist)) / 2 - 26), ButMns[1] + YShift + 6);
    }
    if (PlsMnsCount >= 10 && PlsMnsCount < 100) {
      myGLCD.printNumF(PlsMnsCount, 1, (((ButMns[2] + XShift) + (ButMns[0] + XShift + ButDist)) / 2 - 28), ButMns[1] + YShift + 6);
    }
    if (PlsMnsCount >= 100) {
      myGLCD.printNumF(PlsMnsCount, 1, (((ButMns[2] + XShift) + (ButMns[0] + XShift + ButDist)) / 2 - 36), ButMns[1] + YShift + 6);
    }
  }
  return PlsMnsCount;
  return PlsMnsPress;
}

/************************** КНОПКИ НАЗАД-СОХРАНИТБЬ-ОТМЕНА **********************************/
void PrintBSC3b() {
  printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
  printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);
  printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);
}

/*************************** ОКНО УСТАНОВОК ТАЙМЕРОВ *********************************/
void TimerSetPic() {
  printFramework();               // кайма вокруг часов включения
  printPicture();                 // картинки стрелок
  printButGreen(print_text[67]);  // время вкл.
  printButRed(print_text[68]);    // время выкл.
  PrintBSC3b();
}             // назад, сохранить, отмена

/************************************ ОБРАБОТКА НАЖАТИЙ (ОЛЕГ)*******************************************/
int PressButton(byte MenuTextNumb0, byte MenuTextNumb1, int x1, int y1, int x2, int y2, boolean ButtonEn = true )
{
  if ((x >= x1) && (x <= x2) && (y >= y1) && (y <= y2) && (ButtonEn == true))
  { myGLCD.setColor(255, 0, 0);
    myGLCD.drawRoundRect (x1, y1, x2, y2);
    while ( myTouch.dataAvailable() == true )
      myTouch.read();
    myGLCD.setColor(255, 255, 255);
    myGLCD.drawRoundRect (x1, y1, x2, y2);
    ButtonPress = true;
  } else {
    ButtonPress = false;
  }
  return ButtonPress;
}

/******************Работа с памятью*****************EEPROM FUNCTIONS***************/
struct config_t {    // температура
  int tempset;
  int tempoff;
  int tempalarm;
  int tempSoundAlarmC;
} tempSettings;

struct config_ph {    // PH
  float Pvolt7;
  float Pvolt10;
  float PSetvalPH;
} PHsettings;

struct config_m {    // луна
  int MinI_t;
  int MaxI_t;
} MinMaxIsettings;

struct config_L {
  int DimLEDsOnOff;
  int LEDsDimTempC;
  int LEDsDimPercent;
} LEDsFailsafeSettings;

struct config_g {     // главные настройки
  int ShowHideDOW;
  int Heatsink1FanTempC;
  int Heatsink2FanTempC;
  int SCREENsaver;
  int ScreensaverClockOrBlank;
  int ScreensaverDOWonOff;
  int ScreensaverTimer;
  int SetScreensaverTupe;
  int coolFanMIN;
  int coolFanPWM;
  int heatsinkLoPWM;
  int heatsinkHiPWM;
} GENERALsettings;

struct config_h { // таймеры
  int on1;
  int on2;
  int on3;
  int on4;
  int on5;
  int off1;
  int off2;
  int off3;
  int off4;
  int off5;
} TIMERsettings;

struct config_preset { // пресеты
  byte wwtcol_out;
  byte cwtcol_out;
  byte rbcol_out;
  byte rcol_out;
  byte uvcol_out;
  byte ocol_out;
  byte grcol_out;
} LedPresetSetting;

struct config_s { // для помпы в режиме ШИМ
  int periode;
  int ModeSel;
  int minP1;
  int minP2;
  int maxP1;
  int maxP2;
} PWMPUMPsettings;

struct config_f
{
  int feedFish1h;
  int feedFish1m;
  int feedFish2h;
  int feedFish2m;
  int feedFish3h;
  int feedFish3m;
  int feedFish4h;
  int feedFish4m;
  int feedTime1;
  int feedTime2;
  int feedTime3;
  int feedTime4;
} FEEDERsettings;

struct config_d
{
  int DozTime1h;
  int DozTime1m;
  int DozTime2h;
  int DozTime2m;
  int DozTime3h;
  int DozTime3m;
  int DozTime4h;
  int DozTime4m;
  int dozTime1;
  int dozTime2;
  int dozTime3;
  int dozTime4;
  int doz1sec;
  int doz2sec;
  int doz3sec;
  int doz4sec;
  int doz1num;
  int doz2num;
  int doz3num;
  int doz4num;
  int doz1int;
  int doz2int;
  int doz3int;
  int doz4int;
  int doz1cal;
  int doz2cal;
  int doz3cal;
  int doz4cal;
} DOSINGsettings;

struct config_D {    // Dimm
  int DimmL;
  int setLEDsDimPercentL;
}  DIMMsettings;

struct config_LCD {  // Яркость подсветки
  byte lCDbright;
  byte hightbrH ;
  byte lowbrH ;
} LCDbrightSettings;

// Fonction de nettoyage de la mémoire
void(* resetFunc) (void) = 0; // Reset MC function

void EraseAllEEPROM_SaveDefault() { // check memory for my code format #oleg
  int k = EEPROM.read(0);
  if (k != 125) { // 125 проверочное число
    myGLCD.setFont(RusFont1);
    myGLCD.setColor(255, 0, 0);
    myGLCD.setBackColor(0, 0, 0);
    myGLCD.print(print_text[191], CENTER, 110);       // ПОДОЖДИТЕ, идет очистка памяти
    for (int i = 0; i < 4096; i++) {
      EEPROM.write(i, 0);
      if (i == 0) {
        EEPROM.write(0, 125);
      }
      if ((i == 0) || (i == 512) || (i == 1024) || (i == 1536) || (i == 2048) || (i == 2560) || (i == 3072) || (i == 3584) || (i == 4094)) {
        setFont(LARGE, 200, 200, 200, 0, 0, 0);
        myGLCD.printNumI(i, CENTER, 140);
      }
    }
    COLOR = 0;
    EEPROM.write(780, 0xFF);  // enable all led channel
    SaveLEDToEEPROM();
  }
}    // save default

void SaveLEDToEEPROM() {     // сохранить настройки яркости
  EEPROM.write(0, 125);      // to determine if data available in EEPROM
  if ((COLOR == 1) || (COLOR == 0)) {
    for (int i = 1; i < 97; i++) {
      EEPROM.write(i + (96 * 0), wwtled[i - 1]); // xled[] address must be from 1 to 96
    }
  }
  if ((COLOR == 2) || (COLOR == 0)) {
    for (int i = 1; i < 97; i++) {
      EEPROM.write(i + (96 * 1), swtled[i - 1]); // xled[] address must be from 1 to 96
    }
  }
  if ((COLOR == 3) || (COLOR == 0)) {
    for (int i = 1; i < 97; i++) {
      EEPROM.write(i + (96 * 2), rswtled[i - 1]);
    }
  }
  if ((COLOR == 4) || (COLOR == 0)) {
    for (int i = 1; i < 97; i++) {
      EEPROM.write(i + (96 * 3), rled[i - 1]);
    }
  }
  if ((COLOR == 5) || (COLOR == 0)) {
    for (int i = 1; i < 97; i++) {
      EEPROM.write(i + (96 * 4), uvled[i - 1]);
    }
  }
  if ((COLOR == 6) || (COLOR == 0)) {
    for (int i = 1; i < 97; i++) {
      EEPROM.write(i + (96 * 5), oLed[i - 1]);
    }
  }
  if ((COLOR == 7) || (COLOR == 0)) {
    for (int i = 1; i < 97; i++) {
      EEPROM.write(i + (96 * 6), gled[i - 1]);
    }
  }
}

void SaveOnOffLedStatus(boolean LedShannelFlag_on_off = false) { // вкл / вык канал
  if (LedShannelFlag_on_off == true) {
    if (COLOR == 1) {
      bitSet(LedShannelStatusByte, 0);
    }
    if (COLOR == 2) {
      bitSet(LedShannelStatusByte, 1);
    }
    if (COLOR == 3) {
      bitSet(LedShannelStatusByte, 2);
    }
    if (COLOR == 4) {
      bitSet(LedShannelStatusByte, 3);
    }
    if (COLOR == 5) {
      bitSet(LedShannelStatusByte, 4);
    }
    if (COLOR == 6) {
      bitSet(LedShannelStatusByte, 5);
    }
    if (COLOR == 7) {
      bitSet(LedShannelStatusByte, 6);
    }
  } else {
    if (COLOR == 1) {
      bitClear(LedShannelStatusByte, 0);
    }
    if (COLOR == 2) {
      bitClear(LedShannelStatusByte, 1);
    }
    if (COLOR == 3) {
      bitClear(LedShannelStatusByte, 2);
    }
    if (COLOR == 4) {
      bitClear(LedShannelStatusByte, 3);
    }
    if (COLOR == 5) {
      bitClear(LedShannelStatusByte, 4);
    }
    if (COLOR == 6) {
      bitClear(LedShannelStatusByte, 5);
    }
    if (COLOR == 7) {
      bitClear(LedShannelStatusByte, 6);
    }
  }
  EEPROM.write(780, LedShannelStatusByte);
}

void ReadOnOffLedStatus() { // if reader bit =0 LedShannelFlag_on_off = false, else LedShannelFlag_on_off = true;
  if (COLOR == 1) {
    LedShannelFlag_on_off = bitRead(LedShannelStatusByte, 0);
  }
  if (COLOR == 2) {
    LedShannelFlag_on_off = bitRead(LedShannelStatusByte, 1);
  }
  if (COLOR == 3) {
    LedShannelFlag_on_off = bitRead(LedShannelStatusByte, 2);
  }
  if (COLOR == 4) {
    LedShannelFlag_on_off = bitRead(LedShannelStatusByte, 3);
  }
  if (COLOR == 5) {
    LedShannelFlag_on_off = bitRead(LedShannelStatusByte, 4);
  }
  if (COLOR == 6) {
    LedShannelFlag_on_off = bitRead(LedShannelStatusByte, 5);
  }
  if (COLOR == 7) {
    LedShannelFlag_on_off = bitRead(LedShannelStatusByte, 6);
  }
}

void SaveMoonLEDToEEPROM() { // сохранение минимальной / макс. яркостей луны
  MinMaxIsettings.MinI_t = int(MinI);
  MinMaxIsettings.MaxI_t = int(MaxI);
  EEPROM_writeAnything(600 + 200, MinMaxIsettings);
}

void SaveLEDsFailsafeToEEPROM() { // запись настроек авто-умен. яркости при перегреве
  LEDsFailsafeSettings.DimLEDsOnOff = int(setDimLEDsOnOff);
  LEDsFailsafeSettings.LEDsDimTempC = int(setLEDsDimTempC);
  LEDsFailsafeSettings.LEDsDimPercent = int(setLEDsDimPercent);
  EEPROM_writeAnything(610 + 200, LEDsFailsafeSettings);
}

void SaveTempToEEPROM() {
  tempSettings.tempset = int(setTempC * 10);
  tempSettings.tempoff = int(offTempC * 10);
  tempSettings.tempalarm = int(alarmTempC * 10);
  tempSettings.tempSoundAlarmC =  int(setTempToSoundAlarmC);
  EEPROM_writeAnything(640 + 200, tempSettings);
}

void SavePHsetToEEPROM() {   // запись настроек ph
  PHsettings.Pvolt7 = volt7 * 10000;
  PHsettings.Pvolt10 = volt10 * 10000;
  PHsettings.PSetvalPH = SetvalPH * 10;
  EEPROM_writeAnything(2300, PHsettings);
}

void SaveGenSetsToEEPROM() {
  GENERALsettings.Heatsink1FanTempC = int(setTempToBeginHeatsink1FanC * 10);
  GENERALsettings.Heatsink2FanTempC = int(setTempToBeginHeatsink2FanC * 10);
  GENERALsettings.SCREENsaver = int(setScreensaverOnOff);
  GENERALsettings.ScreensaverClockOrBlank = int(setClockOrBlank);
  GENERALsettings.ScreensaverDOWonOff = int(setScreensaverDOWonOff);
  GENERALsettings.ScreensaverTimer = int(setSSmintues);
  GENERALsettings.SetScreensaverTupe = int(setScreensaverTupe);
  GENERALsettings.coolFanMIN = CoolFanMIN;
  GENERALsettings.coolFanPWM = CoolFanPWM;
  GENERALsettings.heatsinkLoPWM = HeatsinkLoPWM;
  GENERALsettings.heatsinkHiPWM = HeatsinkHiPWM;
  EEPROM_writeAnything(660 + 200, GENERALsettings);
}

void SaveFeedTimesToEEPROM() {
  FEEDERsettings.feedFish1h = int(feedFish1H);
  FEEDERsettings.feedFish1m = int(feedFish1M);
  FEEDERsettings.feedFish2h = int(feedFish2H);
  FEEDERsettings.feedFish2m = int(feedFish2M);
  FEEDERsettings.feedFish3h = int(feedFish3H);
  FEEDERsettings.feedFish3m = int(feedFish3M);
  FEEDERsettings.feedFish4h = int(feedFish4H);
  FEEDERsettings.feedFish4m = int(feedFish4M);
  FEEDERsettings.feedTime1 = int(FEEDTime1);
  FEEDERsettings.feedTime2 = int(FEEDTime2);
  FEEDERsettings.feedTime3 = int(FEEDTime3);
  FEEDERsettings.feedTime4 = int(FEEDTime4);
  EEPROM_writeAnything(2000 + 200, FEEDERsettings);
}

void SaveDoseTimesToEEPROM() {
  DOSINGsettings.DozTime1h = int(dozPump1H);
  DOSINGsettings.DozTime1m = int(dozPump1M);
  DOSINGsettings.DozTime2h = int(dozPump2H);
  DOSINGsettings.DozTime2m = int(dozPump2M);
  DOSINGsettings.DozTime3h = int(dozPump3H);
  DOSINGsettings.DozTime3m = int(dozPump3M);
  DOSINGsettings.DozTime4h = int(dozPump4H);
  DOSINGsettings.DozTime4m = int(dozPump4M);
  DOSINGsettings.dozTime1 = int(DOZTime1);
  DOSINGsettings.dozTime2 = int(DOZTime2);
  DOSINGsettings.dozTime3 = int(DOZTime3);
  DOSINGsettings.dozTime4 = int(DOZTime4);
  DOSINGsettings.doz1sec = int(dozVal1);
  DOSINGsettings.doz2sec = int(dozVal2);
  DOSINGsettings.doz3sec = int(dozVal3);
  DOSINGsettings.doz4sec = int(dozVal4);
  DOSINGsettings.doz1num = int(numDoz1);
  DOSINGsettings.doz2num = int(numDoz2);
  DOSINGsettings.doz3num = int(numDoz3);
  DOSINGsettings.doz4num = int(numDoz4);
  DOSINGsettings.doz1int = int(intDoz1);
  DOSINGsettings.doz2int = int(intDoz2);
  DOSINGsettings.doz3int = int(intDoz3);
  DOSINGsettings.doz4int = int(intDoz4);
  DOSINGsettings.doz1cal = int(dozCal1);
  DOSINGsettings.doz2cal = int(dozCal2);
  DOSINGsettings.doz3cal = int(dozCal3);
  DOSINGsettings.doz4cal = int(dozCal4);
  EEPROM_writeAnything(1900 + 200, DOSINGsettings);
}

void SaveTimerEEPROM() {   // запись в память настроек времени
  TIMERsettings.on1 = int(on1);
  TIMERsettings.on2 = int(on2);
  TIMERsettings.on3 = int(on3);
  TIMERsettings.on4 = int(on4);
  TIMERsettings.on5 = int(on5);
  TIMERsettings.off1 = int(off1);
  TIMERsettings.off2 = int(off2);
  TIMERsettings.off3 = int(off3);
  TIMERsettings.off4 = int(off4);
  TIMERsettings.off5 = int(off5);
  EEPROM_writeAnything(1200, TIMERsettings);
}

void SaveRGBbarsToEEPROM() {
  {
    for (int i = 1; i < 4; i++) {
      EEPROM.write(2500 + i + (3 * 0), rgbCh0[i - 1]); // Enregistrer la couleur pour le canal 1
    }
  }
  {
    for (int i = 1; i < 4; i++) {
      EEPROM.write(2500 + i + (3 * 1), rgbCh1[i - 1]); // Сохранение цвета 2 канала
    }
  }
  {
    for (int i = 1; i < 4; i++) {
      EEPROM.write(2500 + i + (3 * 2), rgbCh2[i - 1]); // Сохранение цвета 3 канала
    }
  }
  {
    for (int i = 1; i < 4; i++) {
      EEPROM.write(2500 + i + (3 * 3), rgbCh3[i - 1]); // Сохранение цвета 4 канала
    }
  }
  {
    for (int i = 1; i < 4; i++) {
      EEPROM.write(2500 + i + (3 * 4), rgbCh4[i - 1]); // Сохранение цвета 5 канала
    }
  }
  {
    for (int i = 1; i < 4; i++) {
      EEPROM.write(2500 + i + (3 * 5), rgbCh5[i - 1]); // Сохранение цвета 6 канала
    }
  }
  {
    for (int i = 1; i < 4; i++) {
      EEPROM.write(2500 + i + (3 * 6), rgbCh6[i - 1]); // Сохранение цвета 7 канала
    }
  }
}

// запись в память датчиков температуры
void SaveDallasAddress () {
  for (byte i = 0; i < 8; i++) {
    EEPROM.write(900 + i, Heatsink1Thermometer [i]); // sensor address
    EEPROM.write(900 + i + 9, Heatsink2Thermometer [i]);
    EEPROM.write(900 + i + 18, waterThermometer [i]);
  }
  EEPROM.write(900 + 8, counterB1);		 // config byte
  EEPROM.write(900 + 17, counterB2);
  EEPROM.write(900 + 26, counterB3);
}

void SaveLEDPresetToEEPROM() { // сохранение настроек пресетов
  LedPresetSetting.wwtcol_out = wwtcol_out;
  LedPresetSetting.cwtcol_out = cwtcol_out;
  LedPresetSetting.rbcol_out = rbcol_out;
  LedPresetSetting.rcol_out =  rcol_out;
  LedPresetSetting.uvcol_out = uvcol_out;
  LedPresetSetting.ocol_out = ocol_out;
  LedPresetSetting.grcol_out = grcol_out;
  EEPROM_writeAnything(1000 + AddressShift, LedPresetSetting);
}

void SavePwmToEEPROM() {   // запись настроек для помп в режиме ШИМ
  PWMPUMPsettings.periode = int(periode);
  PWMPUMPsettings.ModeSel = int(ModeSel);
  PWMPUMPsettings.minP1 = int(minP1);
  PWMPUMPsettings.minP2 = int(minP2);
  PWMPUMPsettings.maxP1 = int(maxP1);
  PWMPUMPsettings.maxP2 = int(maxP2);
  EEPROM_writeAnything(1400, PWMPUMPsettings);
}

void SaveDimmLEDToEEPROM() {       // Dimm
  DIMMsettings.DimmL = int(DimmL);
  DIMMsettings.setLEDsDimPercentL = int(setLEDsDimPercentL);
  EEPROM_writeAnything(1500, DIMMsettings);
}

void SaveLCDbrightToEEPROM() {  // сохранение настроек подсветки
  LCDbrightSettings.lCDbright = LCDbright;
  LCDbrightSettings.hightbrH = HightbrH;
  LCDbrightSettings.lowbrH = LowbrH;
  EEPROM_writeAnything(1900, LCDbrightSettings);
}

void ReadLEDPresetFromEEPROM() {
  EEPROM_readAnything(1000 + AddressShift, LedPresetSetting);
  wwtcol_out = LedPresetSetting.wwtcol_out;
  cwtcol_out = LedPresetSetting.cwtcol_out;
  rbcol_out = LedPresetSetting.rbcol_out;
  rcol_out = LedPresetSetting.rcol_out;
  uvcol_out = LedPresetSetting.uvcol_out;
  ocol_out = LedPresetSetting.ocol_out;
  grcol_out = LedPresetSetting.grcol_out;
}

void ReadLedFromEEPROM() { // COLOR=0 - read all colours
  byte k = EEPROM.read(0);
  int Temp;
  if (k == 125) {
    if ((COLOR == 1) || (COLOR == 0)) {
      for (byte i = 1; i < 97; i++) {
        Temp = EEPROM.read(i + (96 * 0));
        if (Temp > 100) {
          Temp = 100;
        } wwtled[i - 1] = Temp;
      }
    }
    if ((COLOR == 2) || (COLOR == 0)) {
      for (byte i = 1; i < 97; i++) {
        Temp = EEPROM.read(i + (96 * 1));
        if (Temp > 100) {
          Temp = 100;
        } swtled[i - 1] = Temp;
      }
    }
    if ((COLOR == 3) || (COLOR == 0)) {
      for (byte i = 1; i < 97; i++) {
        Temp = EEPROM.read(i + (96 * 2));
        if (Temp > 100) {
          Temp = 100;
        } rswtled[i - 1] = Temp;
      }
    }
    if ((COLOR == 4) || (COLOR == 0)) {
      for (byte i = 1; i < 97; i++) {
        Temp = EEPROM.read(i + (96 * 3));
        if (Temp > 100) {
          Temp = 100;
        } rled[i - 1] = Temp;
      }
    }
    if ((COLOR == 5) || (COLOR == 0)) {
      for (byte i = 1; i < 97; i++) {
        Temp = EEPROM.read(i + (96 * 4));
        if (Temp > 100) {
          Temp = 100;
        } uvled[i - 1] = Temp;
      }
    }
    if ((COLOR == 6) || (COLOR == 0)) {
      for (byte i = 1; i < 97; i++) {
        Temp = EEPROM.read(i + (96 * 5));
        if (Temp > 100) {
          Temp = 100;
        } oLed[i - 1] = Temp;
      }
    }
    if ((COLOR == 7) || (COLOR == 0)) {
      for (byte i = 1; i < 97; i++) {
        Temp = EEPROM.read(i + (96 * 6));
        if (Temp > 100) {
          Temp = 100;
        } gled[i - 1] = Temp;
      }
    }

    if ((COLOR == 8) || (COLOR == 0)) { // moon
      EEPROM_readAnything(600 + 200, MinMaxIsettings); // 200 байт для мин. макс. значений яркости луны
      MinI = MinMaxIsettings.MinI_t;
      if ( MinI > 100 ) {
        MinI = 10; // set default = 10
      }
      MaxI = MinMaxIsettings.MaxI_t;
      if ( MaxI > 100 ) {
        MaxI = 90; // set default = 90
      }
    }
    LedShannelStatusByte = EEPROM.read(780);
  }

  EEPROM_readAnything(610 + 200, LEDsFailsafeSettings); //температура авто-уменьшения яркости при перегреве
  setDimLEDsOnOff = LEDsFailsafeSettings.DimLEDsOnOff;
  if ( setDimLEDsOnOff > 1) {
    setDimLEDsOnOff = 1; // set default = 1 (on / off)
  }
  setLEDsDimTempC = LEDsFailsafeSettings.LEDsDimTempC;
  if ( setLEDsDimTempC > 99 || setLEDsDimTempC < 40 ) {
    setLEDsDimTempC = 50; // set default = 50C
  }
  setLEDsDimPercent = LEDsFailsafeSettings.LEDsDimPercent;
  if ( setLEDsDimPercent > 80 || setLEDsDimPercent < 10 ) {
    setLEDsDimPercent = 50; // set default = 50%
  }
}

void ReadRGBColorFromEEPROM() {
  byte TempRGB;
  for (byte i = 1; i < 4; i++) {
    TempRGB = EEPROM.read(2500 + i + (3 * 0));  // Чтение цвета 1 канала
    rgbCh0[i - 1] = TempRGB;
  }
  for (byte i = 1; i < 4; i++) {
    TempRGB = EEPROM.read(2500 + i + (3 * 1));  // Чтение цвета 2 канала
    rgbCh1[i - 1] = TempRGB;
  }
  for (byte i = 1; i < 4; i++) {
    TempRGB = EEPROM.read(2500 + i + (3 * 2));  // Чтение цвета 3 канала
    rgbCh2[i - 1] = TempRGB;
  }
  for (byte i = 1; i < 4; i++) {
    TempRGB = EEPROM.read(2500 + i + (3 * 3));  // Чтение цвета 4 канала
    rgbCh3[i - 1] = TempRGB;
  }
  for (byte i = 1; i < 4; i++) {
    TempRGB = EEPROM.read(2500 + i + (3 * 4));  // Чтение цвета 5 канала
    rgbCh4[i - 1] = TempRGB;
  }
  for (byte i = 1; i < 4; i++) {
    TempRGB = EEPROM.read(2500 + i + (3 * 5));  // Чтение цвета 6 канала
    rgbCh5[i - 1] = TempRGB;
  }
  for (byte i = 1; i < 4; i++) {
    TempRGB = EEPROM.read(2500 + i + (3 * 6));  // Чтение цвета 7 канала
    rgbCh6[i - 1] = TempRGB;
  }
  if (rgbCh0[0] == 0 && rgbCh0[1] == 0 && rgbCh0[2] == 0 &&
      rgbCh1[0] == 0 && rgbCh1[1] == 0 && rgbCh1[2] == 0 &&
      rgbCh2[0] == 0 && rgbCh2[1] == 0 && rgbCh2[2] == 0 &&
      rgbCh3[0] == 0 && rgbCh3[1] == 0 && rgbCh3[2] == 0 &&
      rgbCh4[0] == 0 && rgbCh4[1] == 0 && rgbCh4[2] == 0 &&
      rgbCh5[0] == 0 && rgbCh5[1] == 0 && rgbCh5[2] == 0 &&
      rgbCh6[0] == 0 && rgbCh6[1] == 0 && rgbCh6[2] == 0)
  {
    //********************** Couleurs de canal par défaut **********************************************
    rgbCh0[0] = 255; rgbCh0[1] = 255; rgbCh0[2] = 205;
    rgbCh1[0] = 255; rgbCh1[1] = 255; rgbCh1[2] = 200;
    rgbCh2[0] = 58;  rgbCh2[1] = 95;   rgbCh2[2] = 205;
    rgbCh3[0] = 255; rgbCh3[1] = 0;   rgbCh3[2] = 0;
    rgbCh4[0] = 255; rgbCh4[1] = 102; rgbCh4[2] = 255;
    rgbCh5[0] = 255; rgbCh5[1] = 143; rgbCh5[2] = 32;
    rgbCh6[0] = 0;   rgbCh6[1] = 255; rgbCh6[2] = 0;
    //************************************************************************************************
  }
}

void ReadFromEEPROM() {     // read from eeprom all data exclude LED SETTING
  int k = EEPROM.read(0);
  char tempString[3];

  // предустановки темперетуры по умолчанию
  EEPROM_readAnything(640 + 200, tempSettings);
  setTempC = tempSettings.tempset; setTempC /= 10;
  if (setTempC < 10 || setTempC > 40) {
    setTempC = 26.0; // set default = 26 C
  }
  offTempC = tempSettings.tempoff; offTempC /= 10;
  if (offTempC < 0 || offTempC > 5) {
    offTempC = 1.0; // set default = 1 C
  }
  alarmTempC = tempSettings.tempalarm; alarmTempC /= 10;
  if (alarmTempC == 255) {
    alarmTempC = 255;
  } else {
    if (alarmTempC < 0 || alarmTempC > 9.9) {
      alarmTempC = 9.9; // set default = 9.9 C
    }
  }

  setTempToSoundAlarmC = tempSettings.tempSoundAlarmC;              // звуковая тревога
  if (setTempToSoundAlarmC == 255) {
    setTempToSoundAlarmC = 255;
  } else {
    if (setTempToSoundAlarmC < 40 || setTempToSoundAlarmC > 99) {
      setTempToSoundAlarmC = 50; // set default = 50 C
    }
  }

  EEPROM_readAnything(2300, PHsettings); // ph
  volt7 = PHsettings.Pvolt7; volt7 /= 10000.0;

  if (volt7 < 0.1 || volt7 > 0.9) {
    volt7 = 0.6939;
  }
  volt10 = PHsettings.Pvolt10; volt10 /= 10000.0;

  if (volt10 < 0.1 || volt10 > 0.9) {
    volt10 = 0.3846;
  }
  SetvalPH = PHsettings.PSetvalPH; SetvalPH /= 10.0;

  if (SetvalPH < 4 || SetvalPH > 12) {
    SetvalPH = 7.0;
  }


  EEPROM_readAnything(660 + 200, GENERALsettings); // температура вентеляторов на радиаторе	660+200
  setTempToBeginHeatsink1FanC = GENERALsettings.Heatsink1FanTempC; setTempToBeginHeatsink1FanC /= 10;
  if (setTempToBeginHeatsink1FanC < 25 || setTempToBeginHeatsink1FanC > 50) {
    setTempToBeginHeatsink1FanC = 30.0; // set default = 30 C
  }
  setTempToBeginHeatsink2FanC = GENERALsettings.Heatsink2FanTempC; setTempToBeginHeatsink2FanC /= 10;
  if (setTempToBeginHeatsink2FanC < 25 || setTempToBeginHeatsink2FanC > 50) {
    setTempToBeginHeatsink2FanC = 30.0; // set default = 30 C
  }

  setScreensaverOnOff = GENERALsettings.SCREENsaver;		 // OFF=0 || ON=1
  if (setScreensaverOnOff > 1) {
    setScreensaverOnOff = 1;
  }
  setClockOrBlank = GENERALsettings.ScreensaverClockOrBlank;         // Clock Screensaver=0 || Blank Screen=1
  if (setClockOrBlank > 0) {
    setClockOrBlank = 1;
  }
  setScreensaverDOWonOff = GENERALsettings.ScreensaverDOWonOff;      // OFF=0 || ON=1 Shows/Hides DOW
  if (setScreensaverDOWonOff > 1) {
    setScreensaverDOWonOff = 1;
  }
  setSSmintues = GENERALsettings.ScreensaverTimer;			 //
  if (setSSmintues < 1 || setSSmintues > 99) {
    setSSmintues = 10; // 1....99 min
  }
  setScreensaverTupe = GENERALsettings.SetScreensaverTupe;
  if (setScreensaverTupe > 0) {
    setScreensaverTupe = 1;
  }
  CoolFanMIN = GENERALsettings.coolFanMIN;
  if (CoolFanMIN > 100) {
    CoolFanMIN = 0;
  }
  CoolFanPWM = GENERALsettings.coolFanPWM;
  if ((CoolFanPWM > 100) || (CoolFanPWM == 0))  {
    CoolFanPWM = 100;
  }
  HeatsinkLoPWM = GENERALsettings.heatsinkLoPWM;
  if ((HeatsinkLoPWM > 100) || (HeatsinkHiPWM < 5)) {
    HeatsinkLoPWM = 5;
  }
  HeatsinkHiPWM = GENERALsettings.heatsinkHiPWM;
  if ((HeatsinkHiPWM > 100) || (HeatsinkHiPWM == 0))  {
    HeatsinkHiPWM = 100;
  }

  EEPROM_readAnything(2000 + 200, FEEDERsettings); //680
  feedFish1H = FEEDERsettings.feedFish1h;
  if (feedFish1H > 24) {
    feedFish1H = 20; // 0....24 h
  }
  feedFish1M = FEEDERsettings.feedFish1m;
  if (feedFish1M > 60) {
    feedFish1M = 0; // 0....60 min
  }
  feedFish2H = FEEDERsettings.feedFish2h;
  if (feedFish2H > 24) {
    feedFish2H = 20; // 0....24 h
  }
  feedFish2M = FEEDERsettings.feedFish2m;
  if (feedFish2M > 60) {
    feedFish2M = 0; // 0....60 min
  }
  feedFish3H = FEEDERsettings.feedFish3h;
  if (feedFish3H > 24) {
    feedFish3H = 20; // 0....24 h
  }
  feedFish3M = FEEDERsettings.feedFish3m;
  if (feedFish3M > 60) {
    feedFish3M = 0; // 0....60 min
  }
  feedFish4H = FEEDERsettings.feedFish4h;
  if (feedFish4H > 24) {
    feedFish4H = 20; // 0....24 h
  }
  feedFish4M = FEEDERsettings.feedFish4m;
  if (feedFish4M > 60) {
    feedFish4M = 0; // 0....60 min
  }
  FEEDTime1 = FEEDERsettings.feedTime1;
  if (FEEDTime1 > 1) {
    FEEDTime1 = 0;
  }
  FEEDTime2 = FEEDERsettings.feedTime2;
  if (FEEDTime2 > 1) {
    FEEDTime2 = 0;
  }
  FEEDTime3 = FEEDERsettings.feedTime3;
  if (FEEDTime3 > 1) {
    FEEDTime3 = 0;
  }
  FEEDTime4 = FEEDERsettings.feedTime4;
  if (FEEDTime4 > 1) {
    FEEDTime4 = 0;
  }

  EEPROM_readAnything(1900 + 200, DOSINGsettings); //680
  dozPump1H = DOSINGsettings.DozTime1h;
  if (dozPump1H > 24) {
    dozPump1H = 20; // 0....24 h
  }
  dozPump1M = DOSINGsettings.DozTime1m;
  if (dozPump1M > 60) {
    dozPump1M = 0; // 0....60 min
  }
  dozPump2H = DOSINGsettings.DozTime2h;
  if (dozPump2H > 24) {
    dozPump2H = 20; // 0....24 h
  }
  dozPump2M = DOSINGsettings.DozTime2m;
  if (dozPump2M > 60) {
    dozPump2M = 0; // 0....60 min
  }
  dozPump3H = DOSINGsettings.DozTime3h;
  if (dozPump3H > 24) {
    dozPump3H = 20; // 0....24 h
  }
  dozPump3M = DOSINGsettings.DozTime3m;
  if (dozPump3M > 60) {
    dozPump3M = 0; // 0....60 min
  }
  dozPump4H = DOSINGsettings.DozTime4h;
  if (dozPump4H > 24) {
    dozPump4H = 20; // 0....24 h
  }
  dozPump4M = DOSINGsettings.DozTime4m;
  if (dozPump4M > 60) {
    dozPump4M = 0; // 0....60 min
  }
  DOZTime1 = DOSINGsettings.dozTime1;
  if (DOZTime1 > 1) {
    DOZTime1 = 0;
  }
  DOZTime2 = DOSINGsettings.dozTime2;
  if (DOZTime2 > 1) {
    DOZTime2 = 0;
  }
  DOZTime3 = DOSINGsettings.dozTime3;
  if (DOZTime3 > 1) {
    DOZTime3 = 0;
  }
  DOZTime4 = DOSINGsettings.dozTime4;
  if (DOZTime4 > 1) {
    DOZTime4 = 0;
  }
  numDoz1 = DOSINGsettings.doz1num;
  numDoz2 = DOSINGsettings.doz2num;
  numDoz3 = DOSINGsettings.doz3num;
  numDoz4 = DOSINGsettings.doz4num;
  intDoz1 = DOSINGsettings.doz1int;
  intDoz2 = DOSINGsettings.doz2int;
  intDoz3 = DOSINGsettings.doz3int;
  intDoz4 = DOSINGsettings.doz4int;
  dozVal1 = DOSINGsettings.doz1sec;
  dozVal2 = DOSINGsettings.doz2sec;
  dozVal3 = DOSINGsettings.doz3sec;
  dozVal4 = DOSINGsettings.doz4sec;
  dozCal1 = DOSINGsettings.doz1cal;
  dozCal2 = DOSINGsettings.doz2cal;
  dozCal3 = DOSINGsettings.doz3cal;
  dozCal4 = DOSINGsettings.doz4cal;
  DosSec1 = (dozVal1 * 10 / dozCal1); //Восстановление переменных DosSecX
  DosSec2 = (dozVal2 * 10 / dozCal2); //после перезагрузки
  DosSec3 = (dozVal3 * 10 / dozCal3);
  DosSec4 = (dozVal4 * 10 / dozCal4);

  EEPROM_readAnything(1200, TIMERsettings); // таймеры // 700+200
  on1 = TIMERsettings.on1;
  on2 = TIMERsettings.on2;
  on3 = TIMERsettings.on3;
  on4 = TIMERsettings.on4;
  on5 = TIMERsettings.on5;
  off1 = TIMERsettings.off1;
  off2 = TIMERsettings.off2;
  off3 = TIMERsettings.off3;
  off4 = TIMERsettings.off4;
  off5 = TIMERsettings.off5;

  EEPROM_readAnything(1400, PWMPUMPsettings); // прочитать из памяти скорость помп в режиме ШИМ
  ModeSel = PWMPUMPsettings.ModeSel;
  minP1 = PWMPUMPsettings.minP1;
  if (minP1 == 0) {
    minP1 = 99;
  }
  minP2 = PWMPUMPsettings.minP2;
  if (minP2 == 0) {
    minP2 = 99;
  }
  maxP1 = PWMPUMPsettings.maxP1;
  maxP2 = PWMPUMPsettings.maxP2;
  periode = PWMPUMPsettings.periode;
  if (periode == 0) {
    periode = 13608;
  }

  EEPROM_readAnything(1500, DIMMsettings);      // Dimm
  DimmL = DIMMsettings.DimmL;
  setLEDsDimPercentL = DIMMsettings.setLEDsDimPercentL;
}

// чтение из памяти адресов датчиков температуры
void ReadDallasAddress () {
  for (byte i = 0; i < 8; i++) {
    Heatsink1Thermometer [i] = EEPROM.read(900 + i);  // sensor address
    Heatsink2Thermometer [i] = EEPROM.read(900 + i + 9);
    waterThermometer [i] = EEPROM.read(900 + i + 18);
  }
  counterB1 = EEPROM.read(900 + 8);	         // config byte
  counterB2 = EEPROM.read(900 + 17);
  counterB3 = EEPROM.read(900 + 26);
}

void ReadLCDbright() { // чтение из памяти настроек яркости
  EEPROM_readAnything(1900, LCDbrightSettings);
  LCDbright = LCDbrightSettings.lCDbright;
  if (LCDbright == 0) {
    LCDbright = 50;
  }
  HightbrH = LCDbrightSettings.hightbrH;
  LowbrH = LCDbrightSettings.lowbrH;
}

// RTC FUNCTIONS
void TimeDateBar(boolean refreshAll = false) {
  RTC.getTime();

  Menu_Text(0, 224, 229, 1, 0 ) ;  // надпись на русском "Время"

  myGLCD.setFont(RusFont2);        // большой шрифт
  myGLCD.setColor(255, 255, 0);    // цвет шрифта желтый
  myGLCD.setBackColor(30, 30, 30); // фон серый

  // Отображение времени в главном экране
  if (RTC.hour < 10) {
    myGLCD.print(" ", 271, 227);
    myGLCD.printNumI(RTC.hour, 280, 227);
  } else myGLCD.printNumI(RTC.hour, 270, 227);

  myGLCD.print(":", 285, 227);

  if (RTC.minute < 10) {
    myGLCD.print("0", 293, 227);
    myGLCD.printNumI(RTC.minute, 300, 227);
  } else myGLCD.printNumI(RTC.minute, 293, 227);
}

void titledate(boolean refreshAll = false) { // Отображение даты и дня недели в нижней части экрана
  //             Time t;
  //       RTC.getTime();

  // Отображать день недели   RTC.day - дата, RTC.month  - месяц, RTC.year - год, RTC.dow  - день недели
  myGLCD.setFont(RusFont1);  // Выбор шрифта
  myGLCD.setColor(190, 190, 190);  // серый шрифт
  myGLCD.setBackColor(30, 30, 30);


  if (RTC.dow == 1) {
    myGLCD.print(print_text[156], 5, 230); // Понедельник / Monday
  }
  if (RTC.dow == 2) {
    myGLCD.print(print_text[157], 5, 230); // Вторник / Tuesday
  }
  if (RTC.dow == 3) {
    myGLCD.print(print_text[158], 5, 230); // Среда / Wednesday
  }
  if (RTC.dow == 4) {
    myGLCD.print(print_text[159], 5, 230); // Четверг / Thursday
  }
  if (RTC.dow == 5) {
    myGLCD.print(print_text[160], 5, 230); // Пятница / Friday
  }
  myGLCD.setColor(255, 0, 0); // шрифт красный для(субботы, воскресенья)
  if (RTC.dow == 6) {
    myGLCD.print(print_text[161], 5, 230); // Суббота / Saturday
  }
  if (RTC.dow == 0) {
    myGLCD.print(print_text[162], 5, 230); // Воскресенье / Sunda
  }

  // Позиция отображения даты, в зависимости от длинны имени дня недели
  if (RTC.dow == 1 || RTC.dow == 0 ) {
    xdate = 93; // 92                   // позиция для Понедельник, Воскресенье.
  }
  if (RTC.dow == 2 || RTC.dow == 4 || RTC.dow == 5 || RTC.dow == 6 ) {
    xdate = 61; // позиция для Вторник, Четверг, Пятница, Суббота.
  }
  if (RTC.dow == 3 ) {
    xdate = 45; // позиция для Среда.
  }

  // Отображать дату
  setFont(SMALL, 160, 255, 255, 30, 30, 30);
  myGLCD.printNumI(RTC.day, xdate + 5, 227);

  // Отображать месяц   // позиция месяца xdate+24
  myGLCD.setFont(RusFont1);  // Выбор шрифта
  myGLCD.setColor(255, 255, 255);  // белый шрифт
  myGLCD.setBackColor(30, 30, 30); // фон серый
  //setFont(RUS1, 255, 255, 255, 30, 30, 30); // белый шрифт, фон серый
  if (RTC.month == 1) {
    myGLCD.print(print_text[163], xdate + 25, 230); // Января / January
  }
  if (RTC.month == 2) {
    myGLCD.print(print_text[164], xdate + 25, 230); // Февраля / February
  }
  if (RTC.month == 3) {
    myGLCD.print(print_text[165], xdate + 25, 230); // Марта  / March
  }
  if (RTC.month == 4) {
    myGLCD.print(print_text[166], xdate + 25, 230); // Апреля / Avril
  }
  if (RTC.month == 5) {
    myGLCD.print(print_text[167], xdate + 25, 230); // Мая / May
  }
  if (RTC.month == 6) {
    myGLCD.print(print_text[168], xdate + 25, 230); // Июня / June
  }
  if (RTC.month == 7) {
    myGLCD.print(print_text[169], xdate + 25, 230); // Июля / July
  }
  if (RTC.month == 8) {
    myGLCD.print(print_text[170], xdate + 25, 230); // Августа / August
  }
  if (RTC.month == 9) {
    myGLCD.print(print_text[171], xdate + 25, 230); // Сентября / September
  }
  if (RTC.month == 10) {
    myGLCD.print(print_text[172], xdate + 25, 230); // Октября / October
  }
  if (RTC.month == 11) {
    myGLCD.print(print_text[173], xdate + 25, 230); // Ноября / November
  }
  if (RTC.month == 12) {
    myGLCD.print(print_text[174], xdate + 25, 230); // Декабря / December
  }

  // Позиция отображения года, в зависимости от длинны имени месяца
  if (RTC.month == 1 || RTC.month == 4 || RTC.month == 8 || RTC.month == 11) {
    xdate = xdate + 90; // Января, Апрель, Август, Ноябрь
  }
  if (RTC.month == 3 || RTC.month == 6 || RTC.month == 7) {
    xdate = xdate + 70; // Март, Июнь, Июль
  }
  if (RTC.month == 2 || RTC.month == 10 || RTC.month == 12) {
    xdate = xdate + 86; // Февраль, Октябрь, Декабрь
  }
  if (RTC.month == 5) {
    xdate = xdate + 60; // Май
  }
  if (RTC.month == 9) {
    xdate = xdate + 95; // Сентябрь
  }

  // Отображать год
  setFont(SMALL, 248, 255, 120, 30, 30, 30);
  myGLCD.printNumI(RTC.year, xdate, 227);
}

byte calcDOW(byte d, byte m, int y) {
  int dow;
  byte mArr[12] = {6, 2, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  dow = (y % 100);
  dow = dow * 1.25;
  dow += d;
  dow += mArr[m - 1];
  if (((y % 4) == 0) && (m < 3))dow -= 1; while (dow > 7)dow -= 7; return dow;
}

byte validateDate(byte d, byte m, word y) {
  byte mArr[12] = {31, 0, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  byte od;
  if (m == 2) {
    if ((y % 4) == 0) {
      if (d == 30) od = 1; else if (d == 0) od = 29; else od = d;
    } else {
      if (d == 29) od = 1; else if (d == 0) od = 28; else od = d;
    }
  } else {
    if (d == 0) od = mArr[m - 1]; else if (d == (mArr[m - 1] + 1)) od = 1; else od = d;
  } return od;
}

byte validateDateForMonth(byte d, byte m, word y) {
  byte mArr[12] = {31, 0, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  byte od;
  boolean dc = false;
  if (m == 2) {
    if ((y % 4) == 0) {
      if (d > 29) {
        d = 29;
      }
    } else {
      if (d > 28) {
        d = 28;
      }
    }
  } else {
    if (d > mArr[m - 1]) {
      d = mArr[m - 1];
    }
  } return d;
}

/************************************* LED LEVELS *************************************/
#ifdef Timers_12bit

void LED_levelo_output() {
  int sector, sstep, t1, t2 ;

  if (colorLEDtest == true)
  {
    wwtled_out = calcExponentialToLinear(wwtcol_out * 5); // new value 0-100%,
    cwtled_out = calcExponentialToLinear(cwtcol_out * 5);
    rbled_out = calcExponentialToLinear(rbcol_out * 5);
    rled_out = calcExponentialToLinear(rcol_out * 5);
    uvled_out = calcExponentialToLinear(uvcol_out * 5);
    oLed_out = calcExponentialToLinear(ocol_out * 5);
    gled_out = calcExponentialToLinear(grcol_out * 5);
    moonled_out = map(mooncol_out, 0, 100, 0, 255);
  }
  else
  {
    if (min_cnt >= 1440) {
      min_cnt = 1; // 24 hours of minutes
    }
    sector = min_cnt / 15;         // divided by gives sector -- 15 minute
    sstep = min_cnt % 15;          // remainder gives add on to sector value
    t1 = sector;

    if (t1 == 95) {
      t2 = 0;
    }
    else {
      t2 = t1 + 1;
    }

    if (sstep == 0)
    {

      wwtled_out = calcExponentialToLinear(wwtled[t1] * 5); // new value 0-100%
      cwtled_out = calcExponentialToLinear(swtled[t1] * 5); // new value 0-100%
      rbled_out = calcExponentialToLinear(rswtled[t1] * 5); // new value 0-100%
      rled_out =  calcExponentialToLinear(rled[t1] * 5); // new value 0-100%
      uvled_out =  calcExponentialToLinear(uvled[t1] * 5); // new value 0-100%
      oLed_out =  calcExponentialToLinear(oLed[t1] * 5); // new value 0-100%
      gled_out = calcExponentialToLinear(gled[t1] * 5); // new value 0-100%

    } else
    {
      wwtled_out  = check(calcExponentialToLinear(wwtled[t1] * 5), calcExponentialToLinear(wwtled[t2] * 5), sstep);
      cwtled_out  = check(calcExponentialToLinear(swtled[t1] * 5), calcExponentialToLinear(swtled[t2] * 5), sstep);
      rbled_out  = check(calcExponentialToLinear(rswtled[t1] * 5), calcExponentialToLinear(rswtled[t2] * 5), sstep);
      rled_out  = check(calcExponentialToLinear(rled[t1] * 5), calcExponentialToLinear(rled[t2] * 5), sstep);
      uvled_out  = check(calcExponentialToLinear(uvled[t1] * 5), calcExponentialToLinear(uvled[t2] * 5), sstep);
      oLed_out  = check(calcExponentialToLinear(oLed[t1] * 5), calcExponentialToLinear(oLed[t2] * 5), sstep);
      gled_out = check(calcExponentialToLinear(gled[t1] * 5), calcExponentialToLinear(gled[t2] * 5), sstep);
    }

    float lunarCycle = moonPhase(RTC.year, RTC.month, RTC.day);             // get a value for the lunar cycle
    moonled_out = map ((MinI * (1 - lunarCycle) + MaxI * lunarCycle + 0.5), 0, 100, 0, 255);
  }

  if (setDimLEDsOnOff == 1) {
    HOT_LEDs(); //
  }
  Soft_Start_Led();          // increase led brigtness after start programm during 50sec


  if (bitRead(LedShannelStatusByte, 0) == false) {
    wwtled_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 1) == false) {
    cwtled_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 2) == false) {
    rbled_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 3) == false) {
    rled_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 4) == false) {
    uvled_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 5) == false) {
    oLed_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 6) == false) {
    gled_out = 0;
  }

  // Диммирование вручную (адаптация)----- Dimm
  if (DimmL == 1) {
    PercentDimL = setLEDsDimPercentL * 0.01;
    wwtled_out = PercentDimL * wwtled_out;
    cwtled_out = PercentDimL * cwtled_out;
    rbled_out = PercentDimL * rbled_out;
    rled_out = PercentDimL * rled_out;
    uvled_out = PercentDimL * uvled_out;
    oLed_out = PercentDimL * oLed_out;
    gled_out = PercentDimL * gled_out;

    if (dispScreen == 0 && screenSaverCounter < setScreenSaverTimer) { // отображения уровней в главном экране
      myGLCD.setColor(255, 125, 0);
      myGLCD.setBackColor(0, 0, 0);
      myGLCD.setFont(SmallFont);
      myGLCD.printNumI(setLEDsDimPercentL, 141, 15);
      myGLCD.print(print_text[142], 159, 15);
    }
  }  // %

  if (RECOM_RCD == true) {  // обычное управление 0 - 2000
    wwt_out = wwtled_out;
    cwt_out = cwtled_out;
    rb_out = rbled_out;
    r_out = rled_out;
    uv_out = uvled_out;
    o_out = oLed_out;
    gr_out = gled_out;
    moon_out = moonled_out;
  } else {     //  инверсное управление 2000 - 0
    wwt_out = MAX_LINEAR_LEVEL - wwtled_out;   // (11бит)
    cwt_out = MAX_LINEAR_LEVEL - cwtled_out;
    rb_out = MAX_LINEAR_LEVEL - rbled_out;
    r_out = MAX_LINEAR_LEVEL - rled_out;
    uv_out = MAX_LINEAR_LEVEL - uvled_out;
    o_out = MAX_LINEAR_LEVEL - oLed_out;
    gr_out = MAX_LINEAR_LEVEL - gled_out;
    moon_out = moonled_out;
  }

  //----------- check negative values  ----------------------------------------
  if ( wwt_out < 0)  {
    wwt_out = 0 ;
  }
  if ( cwt_out < 0)  {
    cwt_out = 0 ;
  }
  if ( rb_out < 0 )  {
    rb_out = 0 ;
  }
  if ( r_out < 0  )  {
    r_out = 0 ;
  }
  if ( uv_out < 0 )  {
    uv_out = 0 ;
  }
  if ( o_out < 0 )   {
    o_out = 0 ;
  }
  if ( gr_out < 0)   {
    gr_out = 0 ;
  }

  //----------- 11 bit PWM outputs
  if (rb_out)   sbi_mix(TCCR4A, COM4B1); else cbi_mix(TCCR4A, COM4B1); // T4B port 7
  if (uv_out)   sbi_mix(TCCR3A, COM3C1); else cbi_mix(TCCR3A, COM3C1); // T3C port 3
  //if (tv_out)   sbi_mix(TCCR4A, COM4C1); else cbi_mix(TCCR4A, COM4C1); // T4C port 8
  if (r_out)    sbi_mix(TCCR4A, COM4A1); else cbi_mix(TCCR4A, COM4A1); // T4A port 6
  if (wwt_out)  sbi_mix(TCCR1A, COM1B1); else cbi_mix(TCCR1A, COM1B1); // T1B port 12
  if (cwt_out)  sbi_mix(TCCR3A, COM3A1); else cbi_mix(TCCR3A, COM3A1); // T3A port 5
  if (gr_out)   sbi_mix(TCCR3A, COM3B1); else cbi_mix(TCCR3A, COM3B1); // T3B port 2
  if (o_out)    sbi_mix(TCCR1A, COM1A1); else cbi_mix(TCCR1A, COM1A1); // T1A port 11

  // reload Output Compare Register
  OCR4B = rb_out;
  OCR3C = uv_out;
  //OCR4C = tv_out;
  OCR4A = r_out;
  OCR1B = wwt_out;
  OCR3A = cwt_out;
  OCR3B = gr_out;
  OCR1A = o_out;

  analogWrite(ledPinMoon, moon_out);  // 8 bit MOON PWM

  /************************Подсветка экрана*********************************/
  byte bout = map(LCDbright, 1, 100, 4, 255);             // подсветка экрана

  if ((RTC.hour >= HightbrH) && (RTC.hour < LowbrH)) {
    analogWrite(LCDbrightPin, bout);
  }
  else {
    bout = bout / 4; // 25% яркости
  }
  analogWrite(LCDbrightPin, bout);
}
#endif

#ifdef Timers_11bit

void LED_levelo_output() {
  int sector, sstep, t1, t2 ;

  if (colorLEDtest == true)
  {
    wwtled_out = calcExponentialToLinear(wwtcol_out * 5); // new value 0-100%,
    cwtled_out = calcExponentialToLinear(cwtcol_out * 5);
    cwtled_out = calcExponentialToLinear(rbcol_out * 5);
    rled_out = calcExponentialToLinear(rcol_out * 5);
    uvled_out = calcExponentialToLinear(uvcol_out * 5);
    oLed_out = calcExponentialToLinear(ocol_out * 5);
    gled_out = calcExponentialToLinear(grcol_out * 5);
    moonled_out = map(mooncol_out, 0, 100, 0, 255);
  }
  else
  {
    if (min_cnt >= 1440) {
      min_cnt = 1; // 24 hours of minutes
    }
    sector = min_cnt / 15;         // divided by gives sector -- 15 minute
    sstep = min_cnt % 15;          // remainder gives add on to sector value
    t1 = sector;

    if (t1 == 95) {
      t2 = 0;
    }
    else {
      t2 = t1 + 1;
    }

    if (sstep == 0)
    {
      wwtled_out = calcExponentialToLinear(wwtled[t1] * 5); // new value 0-100%
      cwtled_out = calcExponentialToLinear(swtled[t1] * 5); // new value 0-100%
      rbled_out = calcExponentialToLinear(rswtled[t1] * 5); // new value 0-100%
      rled_out =  calcExponentialToLinear(rled[t1] * 5); // new value 0-100%
      uvled_out =  calcExponentialToLinear(uvled[t1] * 5); // new value 0-100%
      oLed_out =  calcExponentialToLinear(oLed[t1] * 5); // new value 0-100%
      gled_out = calcExponentialToLinear(gled[t1] * 5); // new value 0-100%
    } else
    {
      wwtled_out  = check(calcExponentialToLinear(wwtled[t1] * 5), calcExponentialToLinear(wwtled[t2] * 5), sstep);
      cwtled_out  = check(calcExponentialToLinear(swtled[t1] * 5), calcExponentialToLinear(swtled[t2] * 5), sstep);
      rbled_out  = check(calcExponentialToLinear(rswtled[t1] * 5), calcExponentialToLinear(rswtled[t2] * 5), sstep);
      rled_out  = check(calcExponentialToLinear(rled[t1] * 5), calcExponentialToLinear(rled[t2] * 5), sstep);
      uvled_out  = check(calcExponentialToLinear(uvled[t1] * 5), calcExponentialToLinear(uvled[t2] * 5), sstep);
      oLed_out  = check(calcExponentialToLinear(oLed[t1] * 5), calcExponentialToLinear(oLed[t2] * 5), sstep);
      gled_out = check(calcExponentialToLinear(gled[t1] * 5), calcExponentialToLinear(gled[t2] * 5), sstep);
    }

    float lunarCycle = moonPhase(RTC.year, RTC.month, RTC.day);             // get a value for the lunar cycle
    moonled_out = map ((MinI * (1 - lunarCycle) + MaxI * lunarCycle + 0.5), 0, 100, 0, 255);
  }

  if (setDimLEDsOnOff == 1) {
    HOT_LEDs(); //
  }
  Soft_Start_Led();          // increase led brigtness after start programm during 50sec


  if (bitRead(LedShannelStatusByte, 0) == false) {
    wwtled_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 1) == false) {
    cwtled_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 2) == false) {
    rbled_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 3) == false) {
    rled_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 4) == false) {
    uvled_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 5) == false) {
    oLed_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 6) == false) {
    gled_out = 0;
  }

  // Диммирование вручную (адаптация)----- Dimm
  if (DimmL == 1) {
    PercentDimL = setLEDsDimPercentL * 0.01;
    wwtled_out = PercentDimL * wwtled_out;
    cwtled_out = PercentDimL * cwtled_out;
    rbled_out = PercentDimL * rbled_out;
    rled_out = PercentDimL * rled_out;
    uvled_out = PercentDimL * uvled_out;
    oLed_out = PercentDimL * oLed_out;
    gled_out = PercentDimL * gled_out;

    if (dispScreen == 0 && screenSaverCounter < setScreenSaverTimer) { // отображения уровней в главном экране
      myGLCD.setColor(255, 125, 0);
      myGLCD.setBackColor(0, 0, 0);
      myGLCD.setFont(SmallFont);
      myGLCD.printNumI(setLEDsDimPercentL, 141, 15);
      myGLCD.print(print_text[142], 159, 15);
    }
  }  // %

  if (RECOM_RCD == true) {  // обычное управление 0 - 2000
    wwt_out = wwtled_out;
    cwt_out = cwtled_out;
    rb_out = rbled_out;
    r_out = rled_out;
    uv_out = uvled_out;
    o_out = oLed_out;
    gr_out = gled_out;
    moon_out = moonled_out;
  } else {     //  инверсное управление 2000 - 0
    wwt_out = MAX_LINEAR_LEVEL - wwtled_out;   // (11бит)
    cwt_out = MAX_LINEAR_LEVEL - cwtled_out;
    rb_out = MAX_LINEAR_LEVEL - rbled_out;
    r_out = MAX_LINEAR_LEVEL - rled_out;
    uv_out = MAX_LINEAR_LEVEL - uvled_out;
    o_out = MAX_LINEAR_LEVEL - oLed_out;
    gr_out = MAX_LINEAR_LEVEL - gled_out;
    moon_out = moonled_out;
  }

  //----------- check negative values  ----------------------------------------
  if ( wwt_out < 0)  {
    wwt_out = 0 ;
  }
  if ( cwt_out < 0)  {
    cwt_out = 0 ;
  }
  if ( rb_out < 0 )  {
    rb_out = 0 ;
  }
  if ( r_out < 0  )  {
    r_out = 0 ;
  }
  if ( uv_out < 0 )  {
    uv_out = 0 ;
  }
  if ( o_out < 0 )   {
    o_out = 0 ;
  }
  if ( gr_out < 0)   {
    gr_out = 0 ;
  }

  //----------- 11 bit PWM outputs
  if (rb_out)   sbi_mix(TCCR4A, COM4B1); else cbi_mix(TCCR4A, COM4B1); // T4B port 7
  if (uv_out)   sbi_mix(TCCR3A, COM3C1); else cbi_mix(TCCR3A, COM3C1); // T3C port 3
  //if (tv_out)   sbi_mix(TCCR4A, COM4C1); else cbi_mix(TCCR4A, COM4C1); // T4C port 8
  if (r_out)    sbi_mix(TCCR4A, COM4A1); else cbi_mix(TCCR4A, COM4A1); // T4A port 6
  if (wwt_out)  sbi_mix(TCCR1A, COM1B1); else cbi_mix(TCCR1A, COM1B1); // T1B port 12
  if (cwt_out)  sbi_mix(TCCR3A, COM3A1); else cbi_mix(TCCR3A, COM3A1); // T3A port 5
  if (gr_out)   sbi_mix(TCCR3A, COM3B1); else cbi_mix(TCCR3A, COM3B1); // T3B port 2
  if (o_out)    sbi_mix(TCCR1A, COM1A1); else cbi_mix(TCCR1A, COM1A1); // T1A port 11

  // reload Output Compare Register
  OCR4B = rb_out;
  OCR3C = uv_out;
  //OCR4C = tv_out;
  OCR4A = r_out;
  OCR1B = wwt_out;
  OCR3A = cwt_out;
  OCR3B = gr_out;
  OCR1A = o_out;

  analogWrite(ledPinMoon, moon_out);  // 8 bit MOON PWM

  /************************Подсветка экрана*********************************/
  byte bout = map(LCDbright, 1, 100, 4, 255);             // подсветка экрана

  if ((RTC.hour >= HightbrH) && (RTC.hour < LowbrH)) {
    analogWrite(LCDbrightPin, bout);
  }
  else {
    bout = bout / 4; // 25% яркости
  }
  analogWrite(LCDbrightPin, bout);
}
#endif

// pt1 и pt2 от чего и к чему диммируем(границы), lstep сколько минут прошло от начала интервала
int check( int pt1, int pt2, int lstep) {
  int result;
  float fresult;

  if (pt1 == pt2) {
    result = pt1;
  } else        // No change
    if (pt1 < pt2) {                          // Increasing brightness
      fresult = ((float(pt2 - pt1) / 900) * float(lstep * 60 + RTC.second)) + float(pt1);
      result = int(fresult);
    }
  // Decreasing brightness
    else {
      fresult = -((float(pt1 - pt2) / 900) * float(lstep * 60 + RTC.second)) + float(pt1);
      result = int(fresult);
    } return result;
}

/******************************** TEMPERATURE FUNCTIONS *******************************/
void checkTempC() {

  HiPWM = map(CoolFanPWM, 0, 100, 0, 255);
  LowPWM = map(CoolFanMIN, 0, 100, 0, 255);
  HeatHiPWM = map(HeatsinkHiPWM, 0, 100, 150, 255);
  HeatLoPWM = map(HeatsinkLoPWM, 0, 100, 5, 150);

  if (millis() - lastTempRequest >= delayInMillis) {    // waited long enough??
    tempW = (sensors.getTempC(waterThermometer));         // чтение датчика температуры воды
    tempH1 = (sensors.getTempC(Heatsink1Thermometer));    // read Heatsink1's heatsink temperature
    tempH2 = (sensors.getTempC(Heatsink2Thermometer));    // read Heatsink2's heatsink temperature

    delayInMillis = 750 / (1 << (12 - resolution));
    lastTempRequest = millis();
    //      digitalWrite(A15, HIGH);              // вкл / выкл вентелятора
    sensors.requestTemperatures();              // temperature request to all devices on the bus
  }

  //--------------- Чтение температуры Воды ------------------
  if (counterB1 == 1 || counterB2 == 1 || counterB3 == 1) { // check Water temperature if B1 or B2 or B3 = 1 (Water)
    if (tempW == -127 || tempW == -196) {                  // sensor disconnected
      digitalWrite(tempHeatPin, LOW);		             // off heater and chiller
      analogWrite(tempChillPin, LowPWM);         // (минимум оборотов)
      tempAlarmflag = true;                                 // turn on alarm
      tempCoolflag = false;
      tempHeatflag = false;
    }

    if (tempW < (setTempC + offTempC + alarmTempC) && tempW > (setTempC - offTempC - alarmTempC)) { // turn off alarm after OverHeating/OverCooling
      tempAlarmflag = false;
      AlarmflagON = false;
      digitalWrite(tempAlarmPin, LOW);
    }                      // OFF alarm

    if (tempW < setTempC) {
      tempCoolflag = false;
      analogWrite(tempChillPin, LowPWM);
    }         // холодильник выкл (минимум оборотов)

    if (tempW > setTempC) {
      tempHeatflag = false;
      if (OUT_INVERSE == true) {
        digitalWrite(tempHeatPin, LOW);
      }                  // нагреватель выкл
      else  {
        digitalWrite(tempHeatPin, HIGH); // нагреватель выкл (инверсия)
      }
    }

    if (tempW >= (setTempC + offTempC)) {
      tempCoolflag = true;
      analogWrite(tempChillPin, HiPWM);            // холодильник  вкл
      if (OUT_INVERSE == true) {
        digitalWrite(tempHeatPin, LOW);
      }           // нагреватель выкл
      else  {
        digitalWrite(tempHeatPin, HIGH); // нагреватель выкл (инверсия)
      }
    }

    if (tempW <= (setTempC - offTempC)) {
      tempHeatflag = true;
      if (OUT_INVERSE == true) {
        digitalWrite(tempHeatPin, HIGH);
      }                 // нагреватель вкл
      else  {
        digitalWrite(tempHeatPin, LOW); // нагреватель вкл (инверсия)
      }

      analogWrite(tempChillPin, LowPWM);
    }              //  холодильник выкл (минимум оборотов)

    if (alarmTempC > 0) {                          // turn on alarm
      if ((tempW >= (setTempC + offTempC + alarmTempC)) || (tempW <= (setTempC - offTempC - alarmTempC))) {
        tempAlarmflag = true;
      }
    }

    //----- максимум температуры для датчика в аквариуме -----
    if (tempW >= MaxTempW) {
      MaxTempW = tempW; // store max temp
    }
  }  else {
    digitalWrite(tempHeatPin, LOW);                 // выключить нагреватель
    analogWrite(tempChillPin, LowPWM);                // выключить холодильник (минимум оборотов)
    tempCoolflag = false;
    tempHeatflag = false;
  }

  //------------ Fan Controller for Heatsink1 ---------------
  if (counterB1 == 2 || counterB2 == 2 || counterB3 == 2) {           // check Heatsink 1 temperature if B1 or B2 or B3 = 2
    Heatsink1TempInterval = (tempH1 - setTempToBeginHeatsink1FanC); // Sets the interval to start from 0
    if (Heatsink1TempInterval >= 20) {
      Heatsink1TempInterval = 20;  // Set maximal value PWM =255 if current temp >= setTempToBeginHeatsink2FanC+20C
    }
    if (Heatsink1TempInterval <= -1) {
      Heatsink1PWM = 0;  // 255
      analogWrite(Heatsink1_FansPWM, Heatsink1PWM);
      bitClear(GlobalStatus1Byte, 6);                      // GlobalStatus1Byte.6 = StartUPFan1 ;
      bitClear(GlobalStatus1Byte, 7);
    }                      // GlobalStatus1Byte.6 = StartUPFan1_Timeout;

    if (Heatsink1TempInterval >= 1) {
      Heatsink1PWM = map (Heatsink1TempInterval, -1 , 20, HeatLoPWM, HeatHiPWM);  // maximum PWM = setTempToBeginHeatsink2FanC+20C, minimum speed 40% of max
      if (bitRead(GlobalStatus1Byte, 7) == false) {                       // set flag startup process
        analogWrite(Heatsink1_FansPWM, 255); // 255

        unsigned long cMillis = millis();
        previousMillisAlarm = cMillis;                            // reset 1 sec timer for fan startup process
        bitSet(GlobalStatus1Byte, 6);
      }
      else {
        analogWrite(Heatsink1_FansPWM, Heatsink1PWM);
      }
    }
  }

  //------------------------ Sound alarm Section ------------------------------
  if (tempH1 >= setTempToSoundAlarmC || tempH1 == -127) {
    tempAlarmflagH1 = true;
    analogWrite(Heatsink1_FansPWM, HeatHiPWM);
  }      // set maximum fan speed
  else {
    tempAlarmflagH1 = false;  // OFF alarm
    digitalWrite(tempAlarmPin, LOW);
  }

  //----------- Максимум температуры за день для датчика на радиаторе 1 ------
  if (tempH1 >= MaxTempH1) {
    MaxTempH1 = tempH1; // store max temp
  }

  //-------------- Fan Controller for Heatsink 2 ----------------
  if (counterB1 == 3 || counterB2 == 3 || counterB3 == 3) { 	  // check Heatsink 2 temperature if B1 or B2 or B3 = 3
    Heatsink2TempInterval = (tempH2 - setTempToBeginHeatsink2FanC);  // Sets the interval to start from 0
    if (Heatsink2TempInterval >= 20 ) {
      Heatsink2TempInterval = 20;  // Set maximal value PWM =255 if current temp >= setTempToBeginHeatsink2FanC+20C
    }
    if (Heatsink2TempInterval <= -1) {
      Heatsink2PWM = 0; //
      analogWrite(Heatsink2_FansPWM, Heatsink2PWM);
      bitClear(GlobalStatus1Byte, 4);                // GlobalStatus1Byte.4 = StartUPFan2 ;
      bitClear(GlobalStatus1Byte, 5);
    }               // GlobalStatus1Byte.5 = StartUPFan2_Timeout;

    if (Heatsink2TempInterval >= 1) {       //                5  200
      Heatsink2PWM = map (Heatsink2TempInterval, -1 , 20, HeatLoPWM, HeatHiPWM);  // maximum PWM = setTempToBeginHeatsink2FanC+20C

      if (bitRead(GlobalStatus1Byte, 5) == false) {			      // set flag startup process
        analogWrite(Heatsink2_FansPWM, 255);  // 255
        unsigned long cMillis = millis();
        previousMillisAlarm = cMillis;			       // reset 1 sec timer for fan startup process
        bitSet(GlobalStatus1Byte, 4);
      }
      else {
        analogWrite(Heatsink2_FansPWM, Heatsink2PWM);
      }
    }
  }

  //---------------------- Sound alarm Section ------------------------------
  if (tempH2 >= setTempToSoundAlarmC || tempH2 == -127) {
    tempAlarmflagH2 = true;
    analogWrite(Heatsink2_FansPWM, HeatHiPWM);
  }          // set maximum fan speed
  else {
    tempAlarmflagH2 = false;  // OFF alarm
    digitalWrite(tempAlarmPin, LOW);
  }

  //----------- Максимум температуры за день для датчика на радиаторе 2 ------
  if (tempH2 >= MaxTempH2) {
    MaxTempH2 = tempH2; // store max temp
  }

  sensors.requestTemperatures();					  // call sensors.requestTemperatures() to issue a global
  // temperature request to all devices on the bus
  //-------- Clear Max Temp every Day -- очистка максимумов температур --------
  if (min_cnt / 15 == StartTime) {
    MaxTempW = 0;
    MaxTempH1 = 0;
    MaxTempH2 = 0;
  }

  // ------------ запоминание температуры по времени для ЛОГ и ЛОГ Темп1,2,3 ----------------
  //                            V-колличество минут в сутки
  for ( int i = 0; i < 1440; i += 30 ) {
    if (timer == i) {
      media[(i - 30) / 30 + 1] = tempW * 10; // температура воды в аквариуме
    }
  }
  for ( int i = 0; i < 1440; i += 30 ) {      // температура радиатора датч 1, 2
    if (timer == i) {
      tF1[(i - 30) / 30 + 1] = tempH1 * 10;
      tF2[(i - 30) / 30 + 1] = tempH2 * 10;
    }
  }
}

//----------------------------------------------------------------------------------
void Alarm() {  // Тревога
  unsigned long cMillis = millis();
  if (cMillis - previousMillisAlarm > 700) {
    previousMillisAlarm = cMillis;

    //----------------- FAN start UP ------------------------
    if (bitRead(GlobalStatus1Byte, 6) == true) { // startup for FAN1
      bitClear(GlobalStatus1Byte, 6);
      bitSet(GlobalStatus1Byte, 7);
    }

    if (bitRead(GlobalStatus1Byte, 4) == true) { // startup for FAN2
      bitClear(GlobalStatus1Byte, 4);
      bitSet(GlobalStatus1Byte, 5);
    }


    //------------- Sound Alarm -------------------------- звуковая тревога

    if ((tempAlarmflag == true && counterB1 != 0) || (tempAlarmflagH1 == true  && counterB2 != 0) || (tempAlarmflagH2 == true && counterB3 != 0)) {

      if (AlarmflagON == true) {
        digitalWrite(tempAlarmPin, HIGH);   // ON alarm
        AlarmflagON = false;
      }
      else {
        if (AlarmflagON == false) {
          digitalWrite(tempAlarmPin, LOW);   // OFF alarm
          AlarmflagON = true;
        }
      }
    }
  }
}

void Speed_Fan() { // Скорость вентеляторов в главном экране
  //------------ Скорость вентилятора на радиаторе датчик 1 -------------

  if ((dispScreen == 0) && (screenSaverCounter < setScreenSaverTimer)) {
    myGLCD.setFont(RusFont1);
    myGLCD.setColor(0, 255, 0);  // green
    myGLCD.setBackColor(0, 0, 0);
    myGLCD.print(F("#"), 270, 18);  // вентелятор
    myGLCD.print(F("#"), 270, 28); // 2
    myGLCD.setColor(255, 255, 255);
    myGLCD.print(F("1"), 283, 18);  // 1
    myGLCD.print(F("2"), 283, 28);  // 2 номер датчика

    if (Heatsink1PWM != 0) {
      myGLCD.setColor(0, 250, 0);
      Heatsink1PWM = map (Heatsink1PWM, 0 , 255, 1, 99);
      if (Heatsink1PWM < 10) {
        myGLCD.print(" ", 294, 18);
        myGLCD.printNumI( Heatsink1PWM, 302, 18);
      }
      else {
        myGLCD.printNumI( Heatsink1PWM, 294, 18);  // вентелятор 1
      }
      myGLCD.print(print_text[142], 310, 18);
    } else {
      Menu_Text(171, 294, 18, 1, 3 ); // Вык
    }

    if (Heatsink2PWM != 0) {
      myGLCD.setColor(0, 250, 0);
      Heatsink2PWM = map (Heatsink2PWM, 0 , 255, 1, 99);
      if (Heatsink2PWM < 10) {
        myGLCD.print(" ", 294, 28);
        myGLCD.printNumI( Heatsink2PWM, 302, 28);
      }
      else {
        myGLCD.printNumI( Heatsink2PWM, 294, 28); // вентелятор 2
      }
      myGLCD.print(print_text[142], 310, 28);
    } else {
      Menu_Text(171, 294, 28, 1, 3 ); // Вык
    }

    if (tempH1 == -127 || tempH1 == -196 || tempH1 == 0) { // при обрыве или неподключенном датчике
      myGLCD.setFont(SmallFont);
      myGLCD.setColor(255, 153, 153);
      analogWrite(Heatsink1_FansPWM, 255);
      myGLCD.print("max", 294, 15);
    }   // вентелятор 1 максимальная скорость

    if (tempH2 == -127 || tempH2 == -196 || tempH2 == 0) {
      myGLCD.setFont(SmallFont);
      myGLCD.setColor(255, 153, 153);
      analogWrite(Heatsink2_FansPWM, 255);
      myGLCD.print("max", 294, 25);
    }
  } // вентелятор 2 максимальная скорость

}

// *************************** NIVEAU D'EAU SUR L'ECRAN PRINCIPAL *********** (en développement) ** ************** //
void waterlevels() {

  if ((dispScreen == 0) && (screenSaverCounter < setScreenSaverTimer)) {
    if (waterlevel == HIGH) {
      myGLCD.setColor(200, 0, 0);
      myGLCD.fillRoundRect(121, 171, 157, 177);
      myGLCD.fillRoundRect(121, 181, 157, 187);
      myGLCD.fillRoundRect(121, 191, 157, 197);
      myGLCD.fillRoundRect(121, 201, 157, 217);
      digitalWrite(tempAlarmPin, HIGH);
    }
    else if (waterlevel == LOW) {
      myGLCD.setColor(0, 150, 0);
      myGLCD.fillRoundRect(121, 171, 157, 177);
      myGLCD.fillRoundRect(121, 181, 157, 187);
      myGLCD.fillRoundRect(121, 191, 157, 197);
      myGLCD.fillRoundRect(121, 201, 157, 217);
      digitalWrite(tempAlarmPin, LOW);
    }
  }
}

//************ Soft Led Start function **************
void Soft_Start_Led() { // increase led brigtness after start programm during 50sec

  if (PercentSoftStart < 1.1) {
    wwtled_out = PercentSoftStart * wwtled_out;
    cwtled_out = PercentSoftStart * cwtled_out;
    rbled_out = PercentSoftStart * rbled_out;
    rled_out = PercentSoftStart * rled_out;
    uvled_out = PercentSoftStart * uvled_out;
    oLed_out = PercentSoftStart * oLed_out;
    gled_out = PercentSoftStart * gled_out;
    PercentSoftStart += 0.1;
  }
}

/*********** DIM LEDs WHEN HOT FUNCTION *************************** Уменьшение яркости каналов при перегреве */
void HOT_LEDs() {
  //     датчик на радиаторе 1        датчик на радиаторе 2             датчик в аквариуме
  if (tempH1 >= setLEDsDimTempC + 1 || tempH2 >= setLEDsDimTempC + 1) { //|| tempW>=setLEDsDimTempC+1){
    bitSet(GlobalStatus1Byte, 1);
  }

  if (tempH1 <= setLEDsDimTempC - 1 && tempH2 <= setLEDsDimTempC - 1) { //&& tempW<=setLEDsDimTempC-1){
    bitClear(GlobalStatus1Byte, 1);
  }

  if (bitRead(GlobalStatus1Byte, 1) == true) {
    PercentDim = setLEDsDimPercent * 0.01;
    wwtled_out = PercentDim * wwtled_out;   // белый
    cwtled_out = PercentDim * cwtled_out;
    rbled_out = PercentDim * rbled_out;
    rled_out = PercentDim * rled_out;
    uvled_out = PercentDim * uvled_out;
    oLed_out = PercentDim * oLed_out; // оранжевый
    gled_out = PercentDim * gled_out;

    screenSaverCounter = 0; // обнулить счетчик хранителя экрана

    if (dispScreen == 0) {
      myGLCD.setFont(RusFont3);
      myGLCD.setColor(255, 76, 0);                      // красный
      //   myGLCD.drawRect(265, 144, 302, 169);              // красная рамка вокруг макс. темп
      myGLCD.printNumI(setLEDsDimPercent, 130, 32);     // мощность в процентах
      myGLCD.print(print_text[142], 150, 32);           // %
      myGLCD.print(F("OverHeat"), 25, 32);           // мощность
      myGLCD.print("reduction leds", 16, 45);
    }
  }
} // ПЕРЕГРЕВ РАДИАТОРА ;

/******************************* LUNAR PHASE FUNCTION *********************************/
float moonPhase(int moonYear, int moonMonth, int moonDay) {
  float phase; double IP;

  long YY, MM, K1, K2, K3, JulianDay;
  YY = moonYear - floor((12 - moonMonth) / 10);
  MM = moonMonth + 9;
  if (MM >= 12) {
    MM = MM - 12;
  }
  K1 = floor(365.25 * (YY + 4712));
  K2 = floor(30.6 * MM + 0.5);
  K3 = floor(floor((YY / 100) + 49) * 0.75) - 38;
  JulianDay = K1 + K2 + moonDay + 59;
  if (JulianDay > 2299160) {
    JulianDay = JulianDay - K3;
  }
  IP = MyNormalize((JulianDay - 2451550.1) / LC);
  AG = IP * LC; phase = 0;

  // Determine the Moon Illumination %
  if ((AG >= 0) && (AG <= LC / 2)) {
    phase = (2 * AG) / LC;  // FROM New Moon 0% TO Full Moon 100%
  }
  if ((AG > LC / 2) && (AG <= LC)) {
    phase = 2 * (LC - AG) / LC; // FROM Full Moon 100% TO New Moon 0%
  }

  // Determine the Lunar Phase
  if ((AG >= 0) && (AG <= 1.85)) {
    LP = print_text[112];  // ~0-12.5% Новолуние
    MoonPic = pgm_get_far_address(First_Quarter);
  }
  if ((AG > 1.85) && (AG <= 5.54)) {
    LP = print_text[113];  // ~12.5-37.5% Растущая
    MoonPic = pgm_get_far_address(First_Quarter);
  }
  if ((AG > 5.54) && (AG <= 9.23)) {
    LP = print_text[114];  // ~37.5-62.5% 1 Четверть
    MoonPic = pgm_get_far_address(First_Quarter);
  }
  if ((AG > 9.23) && (AG <= 12.92)) {
    LP = print_text[115];  // ~62.5-87.5% Растущая
    MoonPic = pgm_get_far_address(First_Quarter);
  }
  if ((AG > 12.92) && (AG <= 16.61)) {
    LP = print_text[116];  // ~87.5-100-87.5% Полнолуние
    MoonPic = pgm_get_far_address(Full_Moon);
  }
  if ((AG > 16.61) && (AG <= 20.30)) {
    LP = print_text[117];  // ~87.5-62.5% Убывающая
    MoonPic = pgm_get_far_address(Full_Moon);
  }
  if ((AG > 20.30) && (AG <= 23.99)) {
    LP = print_text[118];  // ~62.5-37.5% 3 Четверть
    MoonPic = pgm_get_far_address(Full_Moon);
  }
  if ((AG > 23.99) && (AG <= 27.68)) {
    LP = print_text[117];  // ~37.5-12.5% Убывающая
    MoonPic = pgm_get_far_address(Full_Moon);
  }
  if ((AG >= 27.68) && (AG <= LC)) {
    LP = print_text[112];  // ~12.5-0% Новолуние
    MoonPic = pgm_get_far_address(First_Quarter);
  }

  lunar_perc = phase * 100; return phase;
}

double MyNormalize(double v) {
  v = v - floor(v);
  if (v < 0) v = v + 1; return v;
}

/********************************* MISC. FUNCTIONS ************************************/
void clearFscreen() {  // очистить полностью весь экран
  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRect(1, 0, 318, 226);
}

void clearScreen() {  // очистить экран (кроме верхнего и нижнего бара)
  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRect(1, 13, 318, 226);
}

void printButGreen(char* buton) {   // рамка вкл в меню таймеров (вверху)
  myGLCD.setFont(RusFont2);  // 3
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(8, 17, 142, 34);  // белая кайма вокруг зелёной ON
  myGLCD.setColor(70, 200, 0);           // цвет зеленый для ON
  myGLCD.fillRoundRect(9, 18, 141, 33);  // размер кнопки ON
  myGLCD.setColor(0, 0, 0);              // текст черный
  myGLCD.setBackColor(70, 200, 0);
  myGLCD.print(buton, 16, 21);
}  // текст  (buton, 16, 23);

void printButRed(char* butoff) {   // рамка выкл в меню таймеров (вверху)
  myGLCD.setFont(RusFont2);  // 3
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(173, 17, 312, 34); // белая кайма вокруг красной OFF
  myGLCD.setColor(255, 0, 0);             // цвет красный для OFF
  myGLCD.fillRoundRect(174, 18, 311, 33); // размер кнопки OFF
  myGLCD.setColor(0, 0, 0);               // текст черный
  myGLCD.setBackColor(255, 0, 0);
  myGLCD.print(butoff, 179, 21);
}  // текст  (butoff, 179, 23);

void printPres(char* pres) { // preset п1-п4
  myGLCD.setColor(0, 0, 255);
  myGLCD.drawRect(271, 88, 313, 103);
  myGLCD.setFont(RusFont6);
  myGLCD.setColor(0, 255, 0);
  myGLCD.setBackColor( 0, 0, 0);
  myGLCD.print(pres, 281, 90);
}

void printPresoff(char* presoff) { // preset off
  myGLCD.setColor(68, 68, 68);
  myGLCD.setBackColor(0, 0, 0);
  myGLCD.drawRect(271, 88, 313, 103);
  myGLCD.setFont(RusFont6);
  myGLCD.print(presoff, 277, 90);
}

void printHeader() {     // верхний баннер (желтая рамка)
  myGLCD.setFont(RusFont3);       // шрифт русский
  myGLCD.setColor(255, 255, 0);   // цвет желтый  (255, 255, 0);
  myGLCD.setBackColor(255, 255, 0);
  myGLCD.fillRect (1, 0, 318, 13);
  myGLCD.setColor(255, 0, 0);      // цвет красный
  myGLCD.drawRect(1, 0, 318, 13);  // рамка
  myGLCD.setColor(0, 0, 0);           // шрифт текста черный
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Header_Text_table[PrintStringIndex])));
  myGLCD.print(buffer, CENTER, 3 );
} // текст 4

void printFramework() { // timer
  myGLCD.setColor(95, 95, 95);              // цвет серый
  myGLCD.drawRoundRect(21, 93, 65, 129);    // кайма вокруг часов включения
  myGLCD.drawRoundRect(89, 93, 133, 129);   // кайма вокруг минут
  myGLCD.drawRoundRect(187, 93, 230, 129);  // кайма вокруг часов выключения
  myGLCD.drawRoundRect(256, 93, 300, 129);  // кайма вокруг минут
  myGLCD.drawRoundRect(10, 40, 142, 183);   // белая кайма вокруг времени включения
  myGLCD.drawRoundRect(8, 38, 144, 185);    // вторая кайма
  myGLCD.drawRoundRect(175, 40, 310, 183);  // кайма вокруг времени выключения
  myGLCD.drawRoundRect(173, 38, 312, 185);  // вторая кайма
  myGLCD.drawRoundRect(149, 38, 168, 66);
}  // кайма вокруг номера канала

void printPicture() { // картики стрелок таймеров
  myGLCD.drawBitmap(20, 44, 48, 31, up, 1);    // cтрелка вверх часы включения
  myGLCD.drawBitmap(20, 149, 48, 31, down, 1); // cтрелка вниз часы включения
  myGLCD.drawBitmap(89, 44, 48, 31, up, 1);    // cтрелка вверх часы включения
  myGLCD.drawBitmap(89, 149, 48, 31, down, 1); // cтрелка вниз часы включения

  myGLCD.drawBitmap(185, 44, 48, 31, up, 1);     // cтрелка вверх часы выключения
  myGLCD.drawBitmap(185, 149, 48, 31, down, 1);  // cтрелка вниз часы выключения
  myGLCD.drawBitmap(255, 44, 48, 31, up, 1);     // cтрелка вверх часы выключения
  myGLCD.drawBitmap(255, 149, 48, 31, down, 1);
} // cтрелка вниз часы выключения

void printTimernumber(char* tnumber) { // обозначить номер таймера
  setFont(LARGE, 255, 255, 255, 0, 0, 0);
  myGLCD.print(print_text[56], 70, 103);   // :
  myGLCD.print(print_text[56], 236, 103);  // :
  myGLCD.setFont(DotMatrix_M_Num); // шрифт DotMatrix
  myGLCD.setColor(88, 255, 113);   // цвет зелёный
  myGLCD.setBackColor(0, 0, 0);
  myGLCD.print(tnumber, 151, 41);
}

void setFont(boolean font, byte cr, byte cg, byte cb, byte br, byte bg, byte bb) {
  myGLCD.setBackColor(br, bg, bb);                // font background black
  myGLCD.setColor(cr, cg, cb);                    // font color white
  if (font == LARGE) myGLCD.setFont(BigRusFont); else   // font size LARGE
    if (font == SMALL) myGLCD.setFont(RusFont6);
}      // маленький фонт

void waitForTouchRelease() {
  while (myTouch.dataAvailable() == true)   // Wait for release
    myTouch.read();
}

int LedToPercent (int Led_out, int resolution) { // returns LED output in % with rounding to the nearest whole number

  int result;
  float Temp_in = Led_out;
  float Temp = 100 * Temp_in / resolution;
  result  = map(Led_out, 0, resolution, 0, 100);
  if ((Temp - result) >= 0.5) {
    result += 1;
  };      // rounding to the nearest whole number
  return result;
}

void SmallLedBarGraph(int Led_out, int resolution, byte shift_X, byte cR, byte cG, byte cB) {
  int bar = map(Led_out, 0, resolution, 83, 33);   // Top end bar - Y=31, Bot end bar Y=131
  if (bar < 33 ) {
    bar = 33;
  }
  if (bar > 83) {
    bar = 83;
  }
  myGLCD.setColor(0, 0, 0);                     // V -
  myGLCD.fillRect(12 + shift_X, bar, 12 + 18 + shift_X, 33); // hide end of last bar (Top end bar)
  myGLCD.setColor(cR, cG, cB); //  ^ - ширина бара
  myGLCD.drawRect(12 + shift_X, 83, 12 + 18 + shift_X, 83); // %bar place holder
  myGLCD.fillRect(12 + shift_X, 83, 12 + 18 + shift_X, bar);
} // percentage bar
//                 ^ -

void drawBarGraph() {                       // графика уровней каналов в главном экране

  setFont(SMALL, 255, 255, 255, 0, 0, 0);
  myGLCD.drawRect(3, 86, 164, 86);       // x-line  (30, 148, 164, 148);
  for (int i = 0; i < 5; i++) {
    myGLCD.drawLine(2, (i * 11) + 36, 6, (i * 11) + 36); // tick-marks big
  }
  myGLCD.setColor(190, 190, 190);
  for (int i = 0; i < 4; i++) {
    myGLCD.drawLine(2, (i * 11) + 42, 4, (i * 11) + 42); // tick-marks small
  }
  // (пунктир)
  myGLCD.setColor(90, 90, 90); //               V - 55
  for (int i = 36; i < 86; i++) {
    myGLCD.drawPixel(10, i);
    i = i + 3;
  }
  //           ^-верх^ - размер пунктира
  for (int i = 36; i < 86; i++) {
    myGLCD.drawPixel(32, i);
    i = i + 3;
  }
  for (int i = 36; i < 86; i++) {
    myGLCD.drawPixel(54, i);
    i = i + 3;
  }
  for (int i = 36; i < 86; i++) {
    myGLCD.drawPixel(76, i);
    i = i + 3;
  }
  for (int i = 36; i < 86; i++) {
    myGLCD.drawPixel(98, i);
    i = i + 3;
  }
  for (int i = 36; i < 86; i++) {
    myGLCD.drawPixel(120, i);
    i = i + 3;
  }
  for (int i = 36; i < 86; i++) {
    myGLCD.drawPixel(142, i);
    i = i + 3;
  }
  for (int i = 36; i < 86; i++) {
    myGLCD.drawPixel(164, i);
    i = i + 3;
  }

}
//==================== Слайдерное ручное управление ======================
void drawSliderBarGraph() {

  setFont(SMALL, 255, 255, 255, 0, 0, 0); // шрифт белый, фон черный
  myGLCD.drawRect(30, 173, 315, 173);     // x-line
  myGLCD.drawRect(30, 173, 30, 44);       // y-line
  myGLCD.setColor(190, 190, 190);

  for (int i = 0; i < 10; i++) {          // tick-marks шкала процентов
    myGLCD.drawLine(31, (i * 13) + 44, 37, (i * 13) + 44);
  }
  for (int i = 0; i < 10; i++) {          // вторая шкала
    myGLCD.drawLine(31, (i * 13) + 51, 34, (i * 13) + 51);
  }

  myGLCD.setFont(RusFont1); // нарисовать шкалу процентов
  myGLCD.print(print_text[46], 5, 41);    // 100
  myGLCD.print(print_text[45], 12, 54);   // 90
  myGLCD.print(print_text[44], 12, 67);   // 80
  myGLCD.print(print_text[43], 12, 80);   // 70
  myGLCD.print(print_text[42], 12, 93);   // 60
  myGLCD.print(print_text[41], 12, 106);  // 50
  myGLCD.print(print_text[40], 12, 119);  // 40
  myGLCD.print(print_text[39], 12, 132);  // 30
  myGLCD.print(print_text[38], 12, 145);  // 20
  myGLCD.print(print_text[37], 12, 158);  // 10
  myGLCD.print(print_text[187], 20, 171); // 0

  myGLCD.setColor(180, 180, 180);
  for (int i = 0; i < 10; i++) {                 // горизонтальные пунктирные линии шкалы
    for (int k = 46; k < 311; k++) {
      myGLCD.drawPixel(k, (i * 13) + 44);
      k = k + 2;
    }
  }
  //------------ 39    308
  myGLCD.setFont(SmallFont);
  myGLCD.setColor(rgbCh0[0], rgbCh0[1], rgbCh0[2]);     // цвет белый
  myGLCD.drawRect(49, 44, 79, 172);   // БЕЛЫЙ %bar place holder
  myGLCD.setColor(rgbCh1[0], rgbCh1[1], rgbCh1[2]);     // цвет голубой
  myGLCD.drawRect(87, 44, 117, 172);  // ГОЛУБОЙ %bar place holder
  myGLCD.setColor(rgbCh2[0], rgbCh2[1], rgbCh2[2]);       // цвет синий
  myGLCD.drawRect(125, 44, 155, 172); // РОЯЛЬ %bar place holder
  myGLCD.setColor(rgbCh3[0], rgbCh3[1], rgbCh3[2]);         // цвет красный
  myGLCD.drawRect(163, 44, 193, 172); // КРАСНЫЙ %bar place holder;
  myGLCD.setColor(rgbCh4[0], rgbCh4[1], rgbCh4[2]);     // цвет фиолетовый
  myGLCD.drawRect(201, 44, 231, 172); // UV %bar place holder
  myGLCD.setColor(rgbCh5[0], rgbCh5[1], rgbCh5[2]);      // цвет оранжевый
  myGLCD.drawRect(239, 44, 269, 172); // ОРАНЖЕВЫЙ %bar place holder
  myGLCD.setColor(rgbCh6[0], rgbCh6[1], rgbCh6[2]);         // цвет зеленый
  myGLCD.drawRect(277, 44, 307, 172); // ЗЕЛЕНЫЙ %bar place holder

  myGLCD.drawBitmap(10, 190, 24, 24, clos, 1); // картинка красный крестик (выход) (7, 187, 24, 24, clos, 1);

}

//===========================================================================
void SliderBars() { // ========= Слайдер ручной настройки яркости =========
  int TempY;
  TempY = map(y, 172, 44, 0, 100); tSlide = TempY; // 0-255

  myGLCD.setColor(sbR, sbG, sbB);                      // Slider Bar Color
  myGLCD.fillRect(sbX1, y, sbX2, 172);                 // draw the bar where you touch
  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRect(sbX1 + 1, y, sbX2 - 1, 45);          // hide the bar
  myGLCD.setColor(sbR, sbG, sbB);                      // Slider Bar Color
  myGLCD.drawLine(sbX1, 44, sbX2, 44);                 // fills in the top of bar
  setFont(SMALL, sbR, sbG, sbB, 0, 0, 0);

  if (y <= 44) {
    y = 44;
  }
  if (y >= 172) {
    y = 172;
  }
  if (TempY >= 100) {
    TempY = 100;
  }

  if (TempY <= 0) {
    TempY = 0;
  } yStore = TempY;
  if (TempY < 10) {
    myGLCD.print(print_text[188], sbX1 + 5, 186); // 00
    myGLCD.printNumI(TempY, sbX1 + 21, 186);
  }

  if ((TempY >= 10) && (TempY < 100)) {
    myGLCD.print(print_text[187], sbX1 + 5, 186);           // 0
    myGLCD.printNumI(TempY, sbX1 + 13, 186);
  }
  if (TempY >= 100) {
    myGLCD.printNumI(TempY, sbX1 + 5, 186);
  }

  for (int i = 0; i < 7; i++) {
    if ((x >= (i * 38) + 39) && (x <= (i * 38) + 69)) {
      tled[k] = TempY;  // 49 79
      if (i == 0) {
        yWHT = TempY;
        wwtcol_out = yWHT;
      }
      if (i == 1) {
        yBLU = TempY;
        cwtcol_out = yBLU;
      }
      if (i == 2) {
        yRBL = TempY;
        rbcol_out = yRBL;
      }
      if (i == 3) {
        yRED = TempY;
        rcol_out = yRED;
      }
      if (i == 4) {
        yUVL = TempY;
        uvcol_out = yUVL;
      }
      if (i == 5) {
        ySMP = TempY;
        ocol_out = ySMP;
      }
      if (i == 6) {
        yGRN = TempY;
        grcol_out = yGRN;
      }
    }
  }
  LED_levelo_output();
}

void UpDnButtonSlide() { // ==== слайдер ручного управления (кнопки Вверх и Вниз)
  int yTemp;
  if (yWHT >= 100) {
    yWHT = 100;  // white  255
  } if (yWHT <= 0) {
    yWHT = 0;
  }
  if (yBLU >= 100) {
    yBLU = 100;
  } if (yBLU <= 0) {
    yBLU = 0;
  }
  if (yRBL >= 100) {
    yRBL = 100;
  } if (yRBL <= 0) {
    yRBL = 0;
  }
  if (yRED >= 100) {
    yRED = 100;
  } if (yRED <= 0) {
    yRED = 0;
  }
  if (yUVL >= 100) {
    yUVL = 100;
  } if (yUVL <= 0) {
    yUVL = 0;
  }
  if (ySMP >= 100) {
    ySMP = 100;
  } if (ySMP <= 0) {
    ySMP = 0;
  }
  if (yGRN >= 100) {
    yGRN = 100;  // green
  } if (yGRN <= 0) {
    yGRN = 0;
  }

  for (int i = 0; i < 7; i++) {
    if ((x >= (i * 38) + 49) && (x <= (i * 38) + 79)) {
      if (i == 0) {
        yStore = yWHT;  // запомнить текущую яркость
        wwtcol_out = yWHT;
      }
      if (i == 1) {
        yStore = yBLU;
        cwtcol_out = yBLU;
      }
      if (i == 2) {
        yStore = yRBL;
        rbcol_out = yRBL;
      }
      if (i == 3) {
        yStore = yRED;
        rcol_out = yRED;
      }
      if (i == 4) {
        yStore = yUVL;
        uvcol_out = yUVL;
      }
      if (i == 5) {
        yStore = ySMP;
        ocol_out = ySMP;
      }
      if (i == 6) {
        yStore = yGRN;  // green
        grcol_out = yGRN;
      }
    }
  }

  if (yStore >= 100) {
    yStore = 100;
  }
  if (yStore <= 0) {
    yStore = 0;
  }
  setFont(SMALL, sbR, sbG, sbB, 0, 0, 0);
  myGLCD.print(F("    "), sbX1 + 5, 186);
  if (yStore < 10) {
    myGLCD.print(print_text[188], sbX1 + 5, 186); //  00
    myGLCD.printNumI(yStore, sbX1 + 21, 186);
  }

  if ((yStore >= 10) && (yStore < 100)) {
    myGLCD.print(print_text[187], sbX1 + 5, 186);    // 0
    myGLCD.printNumI(yStore, sbX1 + 13, 186);
  }
  if (yStore >= 100) {
    myGLCD.printNumI(yStore, sbX1 + 5, 186);
  }

  yTemp = map(yStore, 0, 100, 172, 44);
  myGLCD.setColor(sbR, sbG, sbB);            // Bar Color
  myGLCD.fillRect(sbX1, yTemp, sbX2, 172);   // draw the bar from where it was last touched
  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRect(sbX1 + 1, yTemp, sbX2 - 1, 45); // hide the bar
  myGLCD.setColor(sbR, sbG, sbB);            // Bar Color
  myGLCD.drawLine(sbX1, 44, sbX2, 44);       // fills in the top of bar
  LED_levelo_output();
}

//========================= Слайдер настройки каналов яркости OLEG ========================
int SliderBarsForChange(int TopSldY, int BopSldY, int y, int sbR, int sbG, int sbB, int sbX1, int sbX2) {
  //  Slider bar vertical size: TopSldY/BotSldY, Y - value, sbR/sbG/sbB - colour
  int TempY;					   // sbX1/sbX2 - horisontal size

  if (y <= TopSldY) {
    y = TopSldY;
  }
  if (y >= BotSldY) {
    y = BotSldY;
  }

  if (SliderSwitch == true) {			     // for slider update after pressing up/down button
    if (tSlide <= 0) {
      tSlide = 0; // and for first time open slider windows
    }
    if (tSlide >= 100) {
      tSlide = 100;
    }
    y = map(tSlide, 0, 100, BotSldY, TopSldY);     // mapping 0-100 counter to slider size and draw
    TempY = tSlide;
  }			     // for display value in %

  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRect(sbX1 + 1, y, sbX2 - 1, TopSldY);   // hide the bar (for white bar rectangle)
  //myGLCD.fillRect(sbX1+1, y, sbX2-1, TopSldY+1);     // hide the bar (for color bar rectangle)
  myGLCD.setColor(sbR, sbG, sbB);                    // Slider Bar Color
  myGLCD.fillRect(sbX1 + 1, y, sbX2 - 1, BotSldY);   // draw the bar where you touch

  if (SliderSwitch == false) {			     // for TOUCH function, convert touch coordinate to 0-100%
    TempY = map(y, BotSldY, TopSldY, 0, 100);   // mapping slider size to 0-100%

    if (TempY >= 100) {
      TempY = 100; // slider change from 0% to 100%
    }
    if (TempY <= 0) {
      TempY = 0;
    }
  }

  myGLCD.setBackColor(0, 0, 0);
  myGLCD.setFont(SmallFont);       // set printed text colour
  if (TempY < 10) {
    myGLCD.print("  ", sbX1 + 0, BotSldY + 2);
    myGLCD.printNumI(TempY, sbX1 + 16, BotSldY + 2);
    myGLCD.print(print_text[142], sbX1 + 24, BotSldY + 2);
  } // %

  if ((TempY >= 10) && (TempY < 100)) {
    myGLCD.print(F("  "), sbX1 + 0, BotSldY + 2);
    myGLCD.printNumI(TempY, sbX1 + 8, BotSldY + 2);
    myGLCD.print(print_text[142], sbX1 + 24, BotSldY + 2);
  } // %

  if (TempY >= 100) {
    myGLCD.printNumI(TempY, sbX1 + 0, BotSldY + 2);
    myGLCD.print(print_text[142], sbX1 + 24, BotSldY + 2);
  }  // %
  tSlide = TempY;				      // temporary store previos value
  return TempY;
}

//---------------------------------------------СКРИНСЕЙВЕР-----------------------------------------
void TimeSaver(boolean refreshAll = false) {                                      // хранитель экрана

  if ((RTC.hour == 0) && (RTC.minute == 0) && (RTC.second < 6)) {
    clearFscreen(); // Очистить экран в полночь
  }

  myGLCD.setColor(127, 255, 212);                                            // ЦВЕТ ЧАСОВ НА СКРИНСЕЙВЕРЕ
  myGLCD.setBackColor(0, 0, 0);
  myGLCD.setFont(SevenSegNumFont); // большой шрифт

  myGLCD.printNumI(RTC.hour, 80, 65, 2, '0');       // Часы
  myGLCD.fillRoundRect(156, 75, 164, 83);           // Точка
  myGLCD.fillRoundRect(156, 93, 164, 101);          // Точка
  myGLCD.printNumI(RTC.minute, 175, 65, 2, '0');    // Минуты

  if (setScreensaverDOWonOff == 1) {          // Date and Date

    // RTC.day - дата, RTC.month  - месяц, RTC.year - год, RTC.dow  - день недели
    // Отображать день недели
    myGLCD.setFont(RusFont3);  // Выбор шрифта
    myGLCD.setColor(127, 255, 212);
    myGLCD.setBackColor(0, 0, 0);

    if (RTC.dow == 1) {
      myGLCD.print(print_text[156], 80, 135); // Понедельник / Monday
    }
    if (RTC.dow == 2) {
      myGLCD.print(print_text[157], 95, 135); // Вторник / Tuesday
    }
    if (RTC.dow == 3) {
      myGLCD.print(print_text[158], 95, 135); // Среда / Wednesday
    }
    if (RTC.dow == 4) {
      myGLCD.print(print_text[159], 95, 135); // Четверг / Thursday
    }
    if (RTC.dow == 5) {
      myGLCD.print(print_text[160], 95, 135); // Пятница / Friday
    }
    if (RTC.dow == 6) {
      myGLCD.print(print_text[161], 95, 135); // Суббота / Saturday
    }
    if (RTC.dow == 0) {
      myGLCD.setColor(255, 0, 0);  // Воскресенье / Sunday
      myGLCD.print(print_text[162], 80, 135);
    }

    // Позиция отображения даты, в зависимости от длинны имени дня недели
    if (RTC.dow == 1 || RTC.dow == 0 ) {
      xdate = 83; // позиция для Понедельник, Воскресенье.
    }
    if (RTC.dow == 2 || RTC.dow == 4 || RTC.dow == 5 || RTC.dow == 6 ) {
      xdate = 73; // позиция для Вторник, Четверг, Пятница, Суббота.
    }
    if (RTC.dow == 3 ) {
      xdate = 59; // позиция для Среда.
    }

    // Отображать дату
    //setFont(SMALL, 160, 255, 255, 30, 30, 30);
    myGLCD.setColor(127, 255, 212);
    myGLCD.setFont(BigRusFont);  // Выбор шрифта
    myGLCD.printNumI(RTC.day, xdate + 91, 128);

    // Отображать месяц   // позиция месяца xdate+24
    myGLCD.setFont(RusFont3);  // Выбор шрифта
    if (RTC.month == 1) {
      myGLCD.print(print_text[163], xdate + 130, 135); // Января / January
    }
    if (RTC.month == 2) {
      myGLCD.print(print_text[164], xdate + 130, 135); // Февраля / February
    }
    if (RTC.month == 3) {
      myGLCD.print(print_text[165], xdate + 130, 135); // Марта / March
    }
    if (RTC.month == 4) {
      myGLCD.print(print_text[166], xdate + 130, 135); // Апреля / Avril
    }
    if (RTC.month == 5) {
      myGLCD.print(print_text[167], xdate + 130, 135); // Мая / May
    }
    if (RTC.month == 6) {
      myGLCD.print(print_text[168], xdate + 130, 135); // Июня / June
    }
    if (RTC.month == 7) {
      myGLCD.print(print_text[169], xdate + 130, 135); // Июля / July
    }
    if (RTC.month == 8) {
      myGLCD.print(print_text[170], xdate + 130, 135); // Августа / August
    }
    if (RTC.month == 9) {
      myGLCD.print(print_text[171], xdate + 130, 135); // Сентября / September
    }
    if (RTC.month == 10) {
      myGLCD.print(print_text[172], xdate + 130, 135); // Октября / October
    }
    if (RTC.month == 11) {
      myGLCD.print(print_text[173], xdate + 130, 135); // Ноября / November
    }
    if (RTC.month == 12) {
      myGLCD.print(print_text[174], xdate + 130, 135); // Декабря / December
    }
  }

  // Отображать температуру
  //setFont(SMALL, 160, 255, 255, 30, 30, 30);
  myGLCD.setFont(RusFont3);  // Выбор шрифта
  myGLCD.print(print_text[214], 58, 170);
  myGLCD.setFont(BigRusFont);
  if (tempW > 10 && tempW < 50 ) {
    myGLCD.printNumF(tempW, 1, 193, 163);
  }
  else myGLCD.print("**.*", 193, 163);
  myGLCD.setFont(SmallFont);
  myGLCD.print("o", 263, 161);
  myGLCD.print("C", 270, 167);
}

//********************************************НАСТРОЙКА СКРИНСЕЙВЕРА********************************************************
void screenSaver() {        // Make the Screen Go Blank after so long
  setScreenSaverTimer = setSSmintues * 12;

  if ((setScreensaverOnOff == 1) && (tempAlarmflag == false))
  { if (screenSaverCounter <= setScreenSaverTimer) {
      screenSaverCounter++;
    }

    if (screenSaverCounter == setScreenSaverTimer)
    { dispScreen = 55;
      myGLCD.clrScr();
      SCREEN_RETURN = false;
    }

    if (setClockOrBlank == 0) { // пустой экран / часы

      if (setScreensaverTupe == 0) {       // цифровые часы
        if (screenSaverCounter > setScreenSaverTimer) {
          dispScreen = 55;  // запустить TimeSaver (циф. часы)
          TimeSaver(true);
        }
      }

      if (setScreensaverTupe == 1) {     // аналоговые часы
        if (screenSaverCounter > setScreenSaverTimer) {
          aclock = 1;  // запустить ATimeSaver (аналог. часы)
          dispScreen = 55;
          ATimeSaver();
        }
      }
    }
  }
}

/********************************** НАСТРОЙКА ХРАНИТЕЛЯ ЭКРАНА *************************************/
void ScreensaverSelect() {
  if (setClockOrBlank == 0) {                   // Choose Screensaver Buttons
    myGLCD.setColor(0, 0, 255);
    myGLCD.fillRoundRect(185, 20, 235, 40);
    Menu_Text(4, 191, 24, 2, 5 );           // ПУСТО
    myGLCD.setColor(0, 255, 0);
    myGLCD.fillRoundRect(255, 20, 305, 40);
    Menu_Text(5, 264, 24, 2, 7 );            // ЧАСЫ
    myGLCD.setColor(64, 64, 64);
    myGLCD.drawLine(0, 76, 319, 76);
    Menu_Text(6, 232, 84, 2, 2 );           // тип экрана
    Menu_Text(7, 32, 57, 1, 2 );            // показывать дату
  } else {
    myGLCD.setColor(0, 255, 0);
    myGLCD.fillRoundRect(185, 20, 235, 40);
    Menu_Text(4, 191, 24, 2, 7 );           //  ПУСТО
    myGLCD.setColor(0, 0, 255);
    myGLCD.fillRoundRect(255, 20, 305, 40);
    Menu_Text(5, 264, 24, 2, 5 );            // ЧАСЫ
    myGLCD.setColor(0, 0, 0);
    myGLCD.fillRect(1, 47, 318, 77);
  }
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(185, 20, 235, 40);
  myGLCD.drawRoundRect(255, 20, 305, 40);

  if (setClockOrBlank == 0) {
    if (setScreensaverDOWonOff == 0) {      // Show Date on Screensaver

      printButton(print_text[36], ClocBlno[0], ClocBlno[1], ClocBlno[2], ClocBlno[3], SMALL, GREEN_BAC); // green  YES
      printButton(print_text[35], ClocBlyes[0], ClocBlyes[1], ClocBlyes[2], ClocBlyes[3], SMALL);  // blue  NO
    } else {
      printButton(print_text[36], ClocBlno[0], ClocBlno[1], ClocBlno[2], ClocBlno[3], SMALL);       // blue  YES
      printButton(print_text[35], ClocBlyes[0], ClocBlyes[1], ClocBlyes[2], ClocBlyes[3], SMALL, GREEN_BAC);
    }
  } // green  NO

  if (setClockOrBlank == 0) { // выбор типа хранителя экрана аналоговые / цифровые часы
    if (setScreensaverTupe == 1) {
      myGLCD.setColor(0, 0, 255);                // синий
      myGLCD.fillRoundRect(245, 132, 301, 152);  // размер кнопки А.ЧАСЫ
      Menu_Text(11, 250, 137, 2, 5 );            // А.ЧАСЫ
      myGLCD.setColor(0, 255, 0);                // зелёный
      myGLCD.fillRoundRect(245, 100, 301, 119);  // размер кнопки Ц.ЧАСЫ
      Menu_Text(10, 250, 104, 2, 7 );            // Ц.ЧАСЫ

    } else {
      myGLCD.setFont(RusFont2);                 // русский фонт
      myGLCD.setColor(0, 255, 0);               // зелёный
      myGLCD.fillRoundRect(245, 132, 301, 152); // размер кнопки А.ЧАСЫ
      Menu_Text(11, 250, 137, 2, 7 );           // аналоговые часы  А.ЧАСЫ
      myGLCD.setColor(0, 0, 255);                // синий
      myGLCD.fillRoundRect(245, 100, 301, 119);  // размер кнопки Ц.ЧАСЫ
      Menu_Text(10, 250, 104, 2, 5);
    }           // цифровые часы  Ц.ЧАСЫ
    myGLCD.setColor(255, 255, 255);
    myGLCD.drawRoundRect(245, 132, 301, 152);   // белая рамка вокруг кнопок А.ЧАСЫ
    myGLCD.drawRoundRect(245, 100, 301, 119);
  }
} // белая рамка вокруг кнопок Ц.ЧАСЫ

/*********************** ГЛАВНЫЙ ЭКРАН ****************** MAIN SCREEN *************************** dispScreen = 0 */                           //   ГЛАВНЫЙ ЭКРАН
void mainScreen(boolean refreshAll = false) {
  int ledLevel, bar;
  String oldval, deg;

  char buffer_Led_Out[15];
  TimeDateBar(true);
  titledate(true); // дата в нижней строке
  oldval = day;                               // refresh day if different
  day = String(RTC.day);

  if ((oldval != day) || refreshAll) {
    myGLCD.setColor(92, 92, 92);
    myGLCD.drawRect(4, 166, 110, 178);        // таймеры
    myGLCD.drawRect(4, 178, 110, 189);
    myGLCD.drawRect(4, 189, 110, 200);
    myGLCD.drawRect(4, 200, 110, 211);
    myGLCD.drawRect(4, 211, 110, 211);
    myGLCD.setColor(0, 153, 153);            // Draw Borders & Dividers
    myGLCD.drawRect(0, 14, 319, 226);        // Outside Border
    myGLCD.drawRect(168, 14, 170, 226);      // Vertical Divider
    myGLCD.drawRect(114, 158, 164, 222);     // вокруг авто-долива
    myGLCD.drawRect(4, 158, 110, 222);
    myGLCD.drawRect(265, 4, 265, 107);       // вертикальная линия справа от луны
    myGLCD.drawRect(265, 39, 319, 39);       // Горизонтальная линия над фильтром
    myGLCD.drawRect(4, 95, 110, 148);        // рамка вокруг дозаторов
    myGLCD.drawRect(114, 95, 164, 138);      // рамка вокруг PH
    myGLCD.drawRect(265, 73, 319, 110);      // вокруг пресетов
    myGLCD.drawRect(218, 110, 303, 169);     // рамка вокруг текущей и максимальной температуры
    myGLCD.drawRect(264, 131, 303, 169);     // рамка вокруг максимальной температуры
    myGLCD.drawRect(170, 131, 319, 169);     // рамка вокруг всей температуры
    myGLCD.drawRect(175, 175, 314, 222);     // МОНИТОР ТРЕВОГИ
    myGLCD.drawRect(170, 108, 319, 110);     // горизонтальная линия (под луной)
    myGLCD.setColor(64, 64, 64);
    myGLCD.fillRect(0, 0, 319, 14);           // Top Bar
    myGLCD.setColor(0, 0, 0);
    myGLCD.drawLine(159, 126, 161, 126);      // Horizontal Divider Separator
    myGLCD.setFont(RusFont3);
    myGLCD.setColor(255, 255, 0);
    myGLCD.setBackColor(64, 64, 64);

    //************* SW version TEXT *************************************
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[22])));
    myGLCD.print(buffer, 30, 3);           // УПРАВЛЕНИЕ АКВАРИУМОМ
    myGLCD.setFont(SmallFont);               // ENG font
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[185])));
    myGLCD.print(buffer, 210, 0);          // software version !!!!
    //*******************************************************************

    Menu_Text(23, 13, 18, 1, 8);          // ЯРКОСТЬ КАНАЛОВ  // цвет бирюзовый
    if (DimmL == 0) {
      myGLCD.setFont(SmallFont);
      myGLCD.printNumI(100, 135, 15);
      myGLCD.print(print_text[142], 159, 15);
    }   // 100 %
    Menu_Text(72, 24, 92, 1, 8);          //ДОЗАТОРЫ
    myGLCD.setColor(0, 204, 204);         // цвет бирюзовый
    myGLCD.drawRoundRect(10, 102, 55, 122);
    myGLCD.drawRoundRect(60, 102, 105, 122);
    myGLCD.drawRoundRect(10, 125, 55, 145);
    myGLCD.drawRoundRect(60, 125, 105, 145);

    /*********************** СОСТОЯНИЕ ДОЗАТОРОВ *******************************/
    if (DOZTime1 == 1) {
      setFont(SMALL, 255, 255, 0, 0, 0, 0);
      myGLCD.setFont(RusFont1);
      myGLCD.printNumI(dozPump1H, 15, 109, 2, '0');
      myGLCD.print(":", 29, 109);
      myGLCD.printNumI(dozPump1M, 36, 109, 2, '0');
    }

    if (DOZTime1 == 0) {
      Menu_Text(184, 15, 109, 1, 8);
    }

    if (DOZTime2 == 1) {
      setFont(SMALL, 255, 255, 0, 0, 0, 0);
      myGLCD.setFont(RusFont1);
      myGLCD.printNumI(dozPump2H, 65, 109, 2, '0');
      myGLCD.print(":", 79, 109);
      myGLCD.printNumI(dozPump2M, 86, 109, 2, '0');
    }

    if (DOZTime2 == 0) {
      Menu_Text(184, 65, 109, 1, 8);
    }

    if (DOZTime3 == 1) {
      setFont(SMALL, 255, 255, 0, 0, 0, 0);
      myGLCD.setFont(RusFont1);
      myGLCD.printNumI(dozPump3H, 15, 131, 2, '0');
      myGLCD.print(":", 29, 131);
      myGLCD.printNumI(dozPump3M, 36, 131, 2, '0');
    }

    if (DOZTime3 == 0) {
      Menu_Text(184, 15, 131, 1, 8);
    }

    if (DOZTime4 == 1) {
      setFont(SMALL, 255, 255, 0, 0, 0, 0);
      myGLCD.setFont(RusFont1);
      myGLCD.printNumI(dozPump4H, 65, 131, 2, '0');
      myGLCD.print(":", 79, 131);
      myGLCD.printNumI(dozPump4M, 86, 131, 2, '0');
    }

    if (DOZTime4 == 0) {
      Menu_Text(184, 65, 131, 1, 8);
    }

    /*********************** СОСТОЯНИЕ РН СЕНСОРА *******************************/
    Menu_Text(82, 132, 92, 1, 8);   //PH
    /*        myGLCD.setFont(BigRusFont);
      if (dispScreen==0 && screenSaverCounter<setScreenSaverTimer && avgMeasuredPH > 3 && avgMeasuredPH < 9,9){
        //   myGLCD.setFont(BigRusFont);
           myGLCD.printNumF(avgMeasuredPH,1, 116, 110);
      }
      else{ if (dispScreen==0 && screenSaverCounter<setScreenSaverTimer){
      // myGLCD.setFont(BigRusFont);
      myGLCD.print("*.*", 116, 110);}
      }
    */
    /*********************** СОСТОЯНИЕ ДАТЧИКА УРОВНЯ ВОДЫ *******************************/

    Menu_Text(76, 112, 145, 1, 8);           //УРОВЕНЬ
    Menu_Text(73, 125, 155, 1, 8);           //ВОДЫ
    myGLCD.setColor(120, 120, 120);          // цвет
    myGLCD.drawRoundRect(120, 170, 158, 178);
    myGLCD.drawRoundRect(120, 180, 158, 188);
    myGLCD.drawRoundRect(120, 190, 158, 198);
    myGLCD.drawRoundRect(120, 203, 158, 218);
    //   Menu_Text(77, 125, 207, 1, 8);            //СЛИВ

    /********************************** НАДПИСИ ****************************************/

    Menu_Text(100, 273, 42, 1, 4);            // ПАУЗА
    Menu_Text(101, 277, 56, 1, 4);            // КОРМ.
    Menu_Text(24, 182, 18, 1, 8);             // ФАЗA ЛУНЫ
    Menu_Text(25, 229, 113, 1, 2);            // ТЕМПЕРАТ.
    Menu_Text(13, 186, 173, 1, 3);            // МОНИТОР СОБЫТИЙ
    Menu_Text(12, 178, 115, 6, 8);            // Датч.  (buffer, 178, 122);

    myGLCD.setFont(RusFont1);                 // font
    myGLCD.setColor(0, 176, 114);             // зел. шрифт берюзовый (32, 255, 255);
    myGLCD.print(print_text[146], 221, 122);  // Текущ
    myGLCD.setColor(255, 151, 48);            // красн (морковный)
    myGLCD.print(print_text[145], 269, 122);  // Макс

    /*********************************** ТАЙМЕРЫ ***************************************/

    Menu_Text(169, 32, 153, 1, 8);    // Timer
    Menu_Text(151, 8, 168, 1, 2);     // 1
    Menu_Text(152, 8, 180, 1, 2);     // 2
    Menu_Text(153, 8, 191, 1, 2);     // 3
    Menu_Text(154, 8, 202, 1, 2);     // 4
    Menu_Text(155, 8, 213, 1, 2);     // 5

    float lunarCycle = moonPhase(RTC.year, RTC.month, RTC.day); // get a value for the lunar cycle
    myGLCD.drawBitmap(194, 29, 53, 53, First_Quarter, 1);  // new  картинка луны
    myGLCD.drawBitmap(271, 76, 45, 9, preset, 1);    // картинка  пресеты

    myGLCD.setFont(RusFont1);
    myGLCD.setColor(210, 210, 210);
    myGLCD.setBackColor(0, 0, 0);

    if ((lunarCycle * 100) < 1) {
      myGLCD.print(F(" 0.0"), 216, 96); // Print % of Full to LCD (94-для шрифта 3)
    }
    else {
      myGLCD.printNumF(lunarCycle * 100, 1, 216, 96);
    }
    myGLCD.print(print_text[142], 251, 96);  // % знак процентов

    myGLCD.setColor(156, 156, 156);
    myGLCD.setBackColor(0, 0, 0);
    myGLCD.print(print_text[89], 178, 96); // Луна
    char bufferLP[16];
    LP.toCharArray(bufferLP, 16);
    myGLCD.print(bufferLP, 179, 86);       // отображение текущей фазы луны

  }
  // бары уровней яркости LED
  if ((whiteLed != wwtled_out) || refreshAll) {      // refresh red led display
    whiteLed = wwtled_out;
    ledLevel = LedToPercent(wwtled_out, MAX_LINEAR_LEVEL);
    oldval.toCharArray(buffer_Led_Out, 15);
    SmallLedBarGraph(wwtled_out, MAX_LINEAR_LEVEL, 0, rgbCh0[0], rgbCh0[1], rgbCh0[2]); // led_out, resoluton, Xshift, bar color
  } // имя канала + уровень

  if ((blueLed != cwtled_out) || refreshAll) {      // refresh red led display
    blueLed = cwtled_out;
    ledLevel = LedToPercent(cwtled_out, MAX_LINEAR_LEVEL);
    oldval.toCharArray(buffer_Led_Out, 15);
    SmallLedBarGraph(cwtled_out, MAX_LINEAR_LEVEL, 22, rgbCh1[0], rgbCh1[1], rgbCh1[2]); // led_out, resoluton, Xshift, bar color
    //                                       ^ - расстояние между барами 22
  }

  if ((rblueLed != rbled_out) || refreshAll) {      // refresh red led display
    rblueLed = rbled_out;
    ledLevel = LedToPercent(rbled_out, MAX_LINEAR_LEVEL);
    oldval.toCharArray(buffer_Led_Out, 15);
    SmallLedBarGraph(rbled_out, MAX_LINEAR_LEVEL, 44, rgbCh2[0], rgbCh2[1], rgbCh2[2]); // led_out, resoluton, Xshift, bar color
  }

  if ((redLed != rled_out) || refreshAll) {      // refresh red led display
    redLed = rled_out;
    ledLevel = LedToPercent(rled_out, MAX_LINEAR_LEVEL);
    oldval.toCharArray(buffer_Led_Out, 15);
    SmallLedBarGraph(rled_out, MAX_LINEAR_LEVEL, 66, rgbCh3[0], rgbCh3[1], rgbCh3[2]); // led_out, resoluton, Xshift, bar color
  }

  if ((uvLed != uvled_out) || refreshAll) {      // refresh red led display
    uvLed = uvled_out;
    ledLevel = LedToPercent(uvled_out, MAX_LINEAR_LEVEL);
    oldval.toCharArray(buffer_Led_Out, 15);
    SmallLedBarGraph(uvled_out, MAX_LINEAR_LEVEL, 88, rgbCh4[0], rgbCh4[1], rgbCh4[2]); // led_out, resoluton, Xshift, bar color
  }

  if ((orLed != oLed_out) || refreshAll) {      // refresh red led display
    orLed = oLed_out;
    ledLevel = LedToPercent(oLed_out, MAX_LINEAR_LEVEL);
    oldval.toCharArray(buffer_Led_Out, 15);
    SmallLedBarGraph(oLed_out, MAX_LINEAR_LEVEL, 110, rgbCh5[0], rgbCh5[1], rgbCh5[2]); // led_out, resoluton, Xshift, bar color
  }

  if ((grLed != gled_out) || refreshAll) {       // refresh red led display
    grLed = gled_out;
    ledLevel = LedToPercent(gled_out, MAX_LINEAR_LEVEL);
    oldval.toCharArray(buffer_Led_Out, 15);
    SmallLedBarGraph(gled_out, MAX_LINEAR_LEVEL, 132, rgbCh6[0], rgbCh6[1], rgbCh6[2]);   // led_out, resoluton, Xshift, bar color
  }

  if ((whiteLed = wwtled_out) || (blueLed = cwtled_out) || (rblueLed = rbled_out) || (redLed = rled_out)
      || (uvLed = uvled_out) || (orLed = oLed_out) || (grLed = gled_out) || refreshAll) {
    drawBarGraph(); // refresh all static led bar
  }

  // ----------------------------------ТЕМПЕРАТУРА В ГЛАВНОМ ЭКРАНЕ -----------------------------------
  if (refreshAll ) {
    if (counterB1 == 1 || counterB2 == 1 || counterB3 == 1) {
      myGLCD.setFont(RusFont1);
      myGLCD.setColor(78, 210, 0);
      myGLCD.setBackColor(0, 0, 0);
      //setFont(RUS1, 78, 210, 0, 0, 0, 0);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[27])));
      myGLCD.print(buffer, 176, 135);      // ВОДА  (Темп.Воды)
      setFont(SMALL, 200, 200, 200, 0, 0, 0);
      myGLCD.drawCircle(307, 135, 1);     //  значек цельсия (темп воды)
      myGLCD.print(print_text[57], 311, 133); // C темп воды
      setFont(SMALL, 255, 151, 48, 0, 0, 0);
      myGLCD.printNumF(MaxTempW, 1, 270 , 133);     // отображение максимальной температуры воды
    }
    if (counterB1 == 2 || counterB2 == 2 || counterB3 == 2) {
      myGLCD.setFont(RusFont1);
      myGLCD.setColor(78, 210, 0);
      myGLCD.setBackColor(0, 0, 0);
      //setFont(RUS1, 78, 210, 0, 0, 0, 0);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[28])));
      myGLCD.print(buffer, 176, 147);     // РАД 1
      setFont(SMALL, 200, 200, 200, 0, 0, 0);
      myGLCD.drawCircle(307, 147, 1);     // значек цельсия (темп рад)
      myGLCD.print(print_text[57], 311, 145);  // C  рад 1
      setFont(SMALL, 255, 151, 48, 0, 0, 0);  // красный шрифт для макс.
      myGLCD.printNumF( MaxTempH1, 1, 270, 145);

    }

    if (counterB1 == 3 || counterB2 == 3 || counterB3 == 3) {
      myGLCD.setFont(RusFont1);
      myGLCD.setColor(78, 210, 0);
      myGLCD.setBackColor(0, 0, 0);
      //setFont(RUS1, 78, 210, 0, 0, 0, 0);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[29])));
      myGLCD.print(buffer, 176, 159);      // РАД 2
      setFont(SMALL, 200, 200, 200, 0, 0, 0);
      myGLCD.drawCircle(307, 159, 1);     // значек цельсия (темп самп)
      myGLCD.print(print_text[57], 311, 157);  // C для рад 2
      setFont(SMALL, 255, 151, 48, 0, 0, 0);  // красный шрифт для макс.
      myGLCD.printNumF( MaxTempH2, 1, 270, 157);

    }

    calculateStartTime();
  }

  myGLCD.setColor(0, 0, 0);       // clear cooler / heater & alarm notices
  if (tempCoolflag == false && tempHeatflag == false) {
    myGLCD.fillRect(190, 184, 303, 201); // очистка баннера нагреватель холодильник(196, 184, 295, 201);
  }
  if (tempAlarmflag == false) {
    myGLCD.fillRect(185, 202, 305, 220); // очистка баннера alarm
  }

  // ------- Отображение температуры Воды ---------------------
  if (counterB1 == 1 || counterB2 == 1 || counterB3 == 1) {
    if (tempW == -127 || tempW == -196 ) {   // range in deg C no matter what
      setFont(SMALL, 255, 0, 0, 0, 0, 0);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[30])));
      myGLCD.print(buffer, 227, 133);     // Err.  sensor disconnected
      tempCoolflag = false;
      tempHeatflag = false;
    } else {

      if (tempCoolflag == true) {                 // Water temperature too HIGH
        myGLCD.setFont(RusFont3);
        myGLCD.setColor(16, 63, 255);
        myGLCD.setBackColor(0, 0, 0);
        //setFont(RUS3, 16, 63, 255, 0, 0, 0);
        myGLCD.drawRect(198, 186, 293, 200);
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[31])));
        myGLCD.print(buffer, 208, 190);
      }      // ХОЛОДИЛЬНИК
      else if (tempHeatflag == true) {             // Water temperature too LOW
        myGLCD.setFont(RusFont3);
        myGLCD.setColor(255, 24, 127);
        myGLCD.setBackColor(0, 0, 0);
        //setFont(RUS3, 255, 24, 127, 0, 0, 0);
        myGLCD.drawRect(198, 186, 293, 200);
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[32])));
        myGLCD.print(buffer, 203, 190);
      }    // НАГРЕВАТЕЛЬ

      setFont(SMALL, 0, 255, 0, 0, 0, 0);
      //          myGLCD.printNumF( tempW, 1, 227, 133+ShiftDrawY); // отображение темп в аквариуме
      myGLCD.printNumF( tempW, 1, 227, 133); // отображение темп в аквариуме

      setFont(SMALL, 200, 200, 200, 0, 0, 0);
      myGLCD.drawCircle(307, 135, 1);     // значек цельсия (темп воды)
      myGLCD.print(print_text[57], 311, 133);  // C для воды в аквариуме

      if (tempAlarmflag == true) {
        // setFont(LARGE, 255, 0, 0, 0, 0, 0);
        myGLCD.setFont(RusFont3);
        myGLCD.setColor(255, 76, 0);  // красный
        myGLCD.setBackColor(0, 0, 0);
        strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[33])));
        myGLCD.print(buffer, 192, 210);
      }
    }
  }   // ALARM!!

  //-------- Отображение температуры радиатора Датчик 1 -----------
  if (counterB1 == 2 || counterB2 == 2 || counterB3 == 2) {
    if (tempH1 == -127 || tempH1 == -196) {
      setFont(SMALL, 255, 0, 0, 0, 0, 0);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[30]))); // sensor disconnected
      myGLCD.print(buffer, 227, 145);     // Err.  sensor disconnected
    } else {
      setFont(SMALL, 0, 255, 0, 0, 0, 0);
      myGLCD.printNumF( tempH1, 1, 227, 145);  // Heatsink1 temperature (No Flags)
      setFont(SMALL, 200, 200, 200, 0, 0, 0);
      myGLCD.drawCircle(307, 147, 1);         // значек цельсия  (темп рад)
      myGLCD.print(print_text[57], 311, 145);
    }
  } // C для рад 1

  //-------- Отображение температуры радиатора Датчик 2 -----------
  if (counterB1 == 3 || counterB2 == 3 || counterB3 == 3) {
    if (tempH2 == -127 || tempH2 == -196) {
      setFont(SMALL, 255, 0, 0, 0, 0, 0);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[30]))); // sensor disconnected
      myGLCD.print(buffer, 227, 157);    // Err.  sensor disconnected
    } else {
      setFont(SMALL, 0, 255, 0, 0, 0, 0);
      myGLCD.printNumF( tempH2, 1, 227, 157); // Heatsink2 temperature (No Flags)
      setFont(SMALL, 200, 200, 200, 0, 0, 0);
      myGLCD.drawCircle(307, 159, 1);       // значек цельсия  (темп самп)
      myGLCD.print(print_text[57], 311, 157);
    }
  }
}  // C для рад 2

//************************************ОКОНЧАНИЕ ГЛАВНОГО ЭКРАНА**************************************//
void screenReturn() {  // Авто-возврат в главный экран
  if (dispScreen != 55) {
    setReturnTimer = setScreenSaverTimer * .75;  // время через которое возврат в главный экран
    if (SCREEN_RETURN == true) {                   // 75% от времени хранителя экрана
      if (dispScreen != 0) {
        if (myTouch.dataAvailable()) {
          processMyTouch();
        } else {
          returnTimer++;
        }
        if (returnTimer > setReturnTimer) {
          returnTimer = 0;
          LEDtestTick = false; colorLEDtest = false; ReadFromEEPROM();
          dispScreen = 0; clearScreen(); mainScreen(true);
        }
      }
    }
  }
}

/*********************** ЭКРАН ГЛАВНОГО МЕНЮ *********************************************** dispScreen = 1 */
void menuScreen() {

  PrintStringIndex = 0; printHeader (); // ГЛАВНОЕ МЕНЮ

  myGLCD.setColor(64, 64, 64);
  myGLCD.drawRect(0, 14, 319, 225);   // вокруг всего экрана
  myGLCD.drawRect(0, 194, 319, 196);  // внизу (под кнопками) 194, 196
  myGLCD.setColor(110, 110, 110);
  myGLCD.drawRoundRect(9, 201, 193, 215); // ГРАФИК ТЕМПЕРАТУР (закругленные края)
  printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);   // ОТМЕНА

  printButton("", tanD[0], tanD[1], tanD[2], tanD[3]);            // УСТАНОВКА ВРЕМЕНИ И ДАТЫ
  printButton("", temC[0], temC[1], temC[2], temC[3]);            // УСТАНОВКА ТЕМПЕРАТУРЫ
  printButton("", feed[0], feed[1], feed[2], feed[3]);            // АВТОКОРМУШКА
  printButton("", Pumpset[0], Pumpset[1], Pumpset[2], Pumpset[3]);// Дозатор УДО
  printButton("", gSet[0], gSet[1], gSet[2], gSet[3]);            // ОСНОВНЫЕ НАСТРОЙКИ
  printButton("", tesT[0], tesT[1], tesT[2], tesT[3]);            // АВТО ТЕСТ
  printButton("", tesT2[0], tesT2[1], tesT2[2], tesT2[3]);        // Тест 2 (с графиками)
  printButton("", ledChM[0], ledChM[1], ledChM[2], ledChM[3]);    // Настройка каналов по цветам
  printButton("", Sector[0], Sector[1], Sector[2], Sector[3]);    // Настройка по секторам времени
  printButton("", Preset[0], Preset[1], Preset[2], Preset[3]);    // Запись присетов
  printButton("", timday[0], timday[1], timday[2], timday[3]);    // Суточные Таймеры
  printButton("", PHset[0], PHset[1], PHset[2], PHset[3]);        // PH
  printButton("", logW[0], logW[1], logW[2], logW[3]);            // график темп. для воды
  printButton("", logH[0], logH[1], logH[2], logH[3]);            // график темп. радиаторы

  myGLCD.setFont(RusFont1);
  myGLCD.setColor(190, 190, 190);
  myGLCD.setBackColor(0, 0, 0);
  myGLCD.print(print_text[121], 34, 198);   // ГРАФИК ТЕМПЕРАТУР

  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(0, 0, 255);
  myGLCD.print(print_text[122], 20, 29);    // УСТАНОВКА
  myGLCD.print(print_text[123], 28, 42);    // ВРЕМЕНИ
  myGLCD.print(print_text[130], 143, 29);   // АВТО
  myGLCD.print(print_text[131], 142, 41);   // ТЕСТ
  myGLCD.print(print_text[132], 242, 29);   // ГРАФИК
  myGLCD.print(print_text[133], 238, 41);   // КАНАЛОВ
  myGLCD.print(print_text[122], 20, 73);    // УСТАНОВКА
  myGLCD.print(print_text[124], 12, 87);    // ТЕМПЕРАТУРЫ
  myGLCD.print(print_text[217], 138, 72);   // ПОМПЫ
  myGLCD.print(print_text[218], 130, 86);   // ТЕЧЕНИЯ
  myGLCD.print(print_text[126], 238, 118);  // ДОЗАТОР
  myGLCD.print(print_text[141], 222, 131);  // УГЛЕКИСЛОТЫ
  myGLCD.print(print_text[136], 36, 118);   // ВРЕМЯ
  myGLCD.print(print_text[216], 19, 131);   // КОРМЛЕНИЯ
  myGLCD.print(print_text[135], 135, 118);  // CЕКТОР
  myGLCD.print(print_text[123], 131, 131);  // ВРЕМЕНИ
  myGLCD.print(print_text[137], 240, 72);   // ЗАПИСЬ
  myGLCD.print(print_text[138], 232, 86);   // ПРЕСЕТОВ
  myGLCD.print(print_text[128], 25, 160);   // ОСНОВНЫЕ
  myGLCD.print(print_text[129], 20, 174);   // НАСТРОЙКИ
  myGLCD.print(print_text[126], 238, 160);  // ДОЗАТОР
  myGLCD.print(print_text[127], 238, 174);  // УДОБРЕНИЙ
  myGLCD.print(print_text[139], 128, 160);  // СУТОЧНЫЕ
  myGLCD.print(print_text[140], 132, 174);  // ТАЙМЕРЫ
  myGLCD.setFont(RusFont1);
  myGLCD.print(print_text[119], 25, 212);   // АКВАР.
  myGLCD.print(print_text[120], 115, 212);  // РАДИАТ
}

/************** ЭКРАН УСТАНОВКИ ВРЕМЕНИ ************************************************** dispScreen = 2 */
void clockScreen(boolean refreshAll = true) {
  if (refreshAll) {
    rtcSetMin = RTC.minute; rtcSetHr = RTC.hour;
    rtcSetDy = RTC.day; rtcSetMon = RTC.month; rtcSetYr = RTC.year;

    PrintStringIndex = 1; printHeader (); // УСТАНОВКА ВРЕМЕНИ И ДАТЫ

    myGLCD.setColor(64, 64, 64);                   // Draw Dividers in Grey
    myGLCD.drawRect(0, 196, 319, 194);             // Bottom Horizontal Divider
    myGLCD.drawLine(0, 105, 319, 105);             // Middle Horizontal Divider
    myGLCD.drawRoundRect(9, 35, 95, 90);        // рамка вокруг текста (ВРЕМЯ)

    PrintBSC3b();

    drawUpButtonSlide(houU[0], houU[1]);           // hour up
    drawUpButtonSlide(minU[0], minU[1]);           // min up
    drawDownButtonSlide(houD[0], houD[1]);         // hour down
    drawDownButtonSlide(minD[0], minD[1]);         // min down
    drawUpButtonSlide(dayU[0], dayU[1]);           // day up
    drawUpButtonSlide(monU[0], monU[1]);           // month up
    drawUpButtonSlide(yeaU[0], yeaU[1]);           // year up
    drawDownButtonSlide(dayD[0], dayD[1]);         // day down
    drawDownButtonSlide(monD[0], monD[1]);         // month down
    drawDownButtonSlide(yeaD[0], yeaD[1]);
  }       // year down

  timeDispH = rtcSetHr; timeDispM = rtcSetMin;
  xTimeH = 107; yTime = 52; xColon = xTimeH + 42;
  xTimeM10 = xTimeH + 70; xTimeM1 = xTimeH + 86;
  timeChange();

  myGLCD.setBackColor(0, 0, 0);
  myGLCD.print(print_text[155], 149, 142);    // | /
  myGLCD.print(print_text[155], 219, 142);    // | /

  // установка числа, месяца, года     DD/MM/YYYY Format
  Menu_Text(34, 5, 158, 6, 4 );    // (ДД/MM/ГГГГ)  День / Месяц / Год
  Menu_Text(0, 34, 43, 6, 8 );       //   ВРЕМЯ:
  Menu_Text(3, 17, 55, 6, 8 );       // в формате (время)
  Menu_Text(3, 17, 144, 6, 8 );      // в формате (дата)
  Menu_Text(1, 36, 132, 6, 8 );      //   ДАТА:
  setFont(LARGE, 16, 255, 27, 0, 0, 0);   // зел. шрифт
  if ((rtcSetDy >= 0) && (rtcSetDy <= 9)) {           // Set DAY
    myGLCD.print(print_text[187], 107, 142);  // 0
    myGLCD.printNumI(rtcSetDy, 123, 142);
  }
  else {
    myGLCD.printNumI(rtcSetDy, 107, 142);
  }

  if ((rtcSetMon >= 0) && (rtcSetMon <= 9)) {         // Set MONTH
    myGLCD.print(print_text[187], 177, 142);  // 0
    myGLCD.printNumI(rtcSetMon, 193, 142);
  }
  else {
    myGLCD.printNumI(rtcSetMon, 177, 142);
  }

  myGLCD.printNumI(rtcSetYr, 247, 142);
}    // Set YEAR

void timeChange() {                              // установка времени
  Menu_Text(166, 32, yTime + 17, 6, 4 );     // (24HR)
  timeCorrectFormat();
}

void timeCorrectFormat() {
  setFont(LARGE, 16, 255, 27, 0, 0, 0);   // зел. шрифт
  myGLCD.print(print_text[56], xColon, yTime);        // :

  if ((timeDispH >= 0) && (timeDispH <= 9)) {             // Set HOUR
    myGLCD.print(print_text[187], xTimeH, yTime);  // 0
    myGLCD.printNumI(timeDispH, xTimeH + 16, yTime);
  }
  else {
    myGLCD.printNumI(timeDispH, xTimeH, yTime);
  }

  if ((timeDispM >= 0) && (timeDispM <= 9)) {              // Set MINUTES
    myGLCD.print(print_text[187], xTimeM10, yTime);  // 0
    myGLCD.printNumI(timeDispM, xTimeM1, yTime);
  }
  else {
    myGLCD.printNumI(timeDispM, xTimeM10, yTime);
  }
}

/************** ЭКРАН УСТАНОВОК ТЕМПЕРАТУРЫ ******************************************************* dispScreen = 3 */
void tempScreen(boolean refreshAll = false) {
  if (refreshAll) {

    // Текущая температура (монитор)
    myGLCD.setFont(RusFont1);
    myGLCD.setColor(88, 255, 238);
    myGLCD.setBackColor(0, 0, 0);
    myGLCD.print(print_text[146], 275, 22);  // Текущ
    myGLCD.print(print_text[145], 280, 57);  // Макс
    myGLCD.setColor(200, 200, 200);
    myGLCD.drawCircle(311, 37, 1);           // значек цельсия (темп воды)
    myGLCD.drawCircle(311, 72, 1);           // значек цельсия (темп воды) max
    setFont(SMALL, 0, 255, 0, 0, 0, 0);
    myGLCD.printNumF( tempW, 1, 278, 36);    // отображение темп в аквариуме
    setFont(SMALL, 255, 0, 0, 0, 0, 0);
    myGLCD.printNumF(MaxTempW, 1, 278 , 71); // отображение максимальной температуры воды

    if (setTempC == 0) {
      setTempC = 26.1;  // change to 26.1 deg C
    }
    if (offTempC == 0) {
      offTempC = 2.0;  // change to 1 deg C
    }
    temp2beS = setTempC;               // отображение заданной температуры
    temp2beO = offTempC;               // гистерезис теипературы
    temp2beA = alarmTempC;             // гистерезис тревоги

    PrintStringIndex = 2; printHeader ();     // УСТАНОВКА ТЕМПЕРАТУРЫ ВОДЫ

    myGLCD.setColor(0, 0, 255);                // синий цвет (royal)
    myGLCD.drawRoundRect(51, 24, 269, 187);    // нарисовать синюю рамку
    myGLCD.setColor(64, 64, 64);
    myGLCD.drawRoundRect(273, 33, 315, 50);    // рамка вокруг текущ. температуры
    myGLCD.drawRoundRect(273, 68, 315, 85);    // рамка вокруг max
    myGLCD.drawRoundRect(273, 98, 315, 180);   // рамка внизу
    myGLCD.setColor(140, 140, 140);            // серый цвет

    //======= лог за сутки
    myGLCD.setColor(255, 255, 255);
    myGLCD.drawRoundRect(13, 83, 37, 127);       // нарисовать рамку log
    myGLCD.setColor(40, 244, 255);
    myGLCD.setBackColor(0, 0, 0);
    myGLCD.print(F("L"), 22, 90);           // Л
    myGLCD.print(F("O"), 22, 102);          // О
    myGLCD.print(F("G"), 22, 114);          // Г

    myGLCD.setColor(120, 120, 120);              // серый цвет
    myGLCD.drawLine(24, 78, 24, 30);             // линия от кнопки лог вверх
    myGLCD.drawLine(24, 133, 24, 182);           // линия от кнопки лог вниз
    myGLCD.drawLine(26, 78, 26, 30);             // вторая линия от кнопки лог вверх
    myGLCD.drawLine(26, 133, 26, 182);           // вторая линия от кнопки лог вниз

    myGLCD.setColor(64, 64, 64);                        // Draw Dividers in Grey
    myGLCD.drawRect(0, 196, 319, 194);                  // Bottom Horizontal Divider
    PrintBSC3b();

    Menu_Text(37, 71, 36, 1, 2 );         // заданная температура
    Menu_Text(38, CENTER, 87, 1, 2 );     // Гистерезис Температуры
    Menu_Text(39, CENTER, 137, 1, 3 );    // Гистерезис Тревоги
    setFont(SMALL, 255, 255, 255, 0, 0, 0);
    myGLCD.drawCircle(239, 35, 1);             // знак цельсия
    myGLCD.print(print_text[57], 244, 33);     // C
    if (tempW == -127 || tempW == -196) {          // sensor disconnected
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(270, 21, 315, 95);
    }

    temp2beS = PlusMinusCountF (false, false, temM[0], temM[1], 150, 10, 40, 0.1, temp2beS);           // desired temperature
    if (PlsMnsPress == true) {
      PlsMnsPress = false;
    }
    temp2beO = PlusMinusCountF (false, false, temM[0], offM[1], 150, 0.2, 5, 0.1, temp2beO);           // temperature accurancy
    if (PlsMnsPress == true) {
      PlsMnsPress = false;
    }
    temp2beA = PlusMinusCountF (false, false, temM[0], SoundAlarmTm[1], 150, 1, 10, 0.1, temp2beA);    // alarm temperature
    if (PlsMnsPress == true) {
      PlsMnsPress = false;
    }

  }
}

/***************************ТЕСТ МАССИВА ОСВЕЩЕНИЯ******************************************* dispScreen = 5 */
void testArrayScreen(boolean refreshAll = false) {
  if (refreshAll) {

    PrintStringIndex = 3; printHeader (); // АВТОМАТИЧЕСКОЕ ТЕСТИРОВАНИЕ КАНАЛОВ
    myGLCD.fillRoundRect (1, 15, 318, 37); // очистка баннера "Test in Progress"
    myGLCD.setColor(64, 64, 64);            // Draw Dividers in Grey
    myGLCD.drawRect(0, 196, 319, 194);      // Bottom Horizontal Divider
    printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
    printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);

    printButton ("", stsT[0], stsT[1], stsT[2], stsT[3], true);      // start/stop
    printButton (print_text[58], tenM[0], tenM[1], tenM[2], tenM[3], true);  // -10s
    printButton (print_text[59], tenP[0], tenP[1], tenP[2], tenP[3], true);  // +10s
    myGLCD.print(print_text[96], stsT[0] + 6, stsT[1] + 15);  // START
    myGLCD.print(print_text[154], stsT[0] + 15, stsT[1] + 40); // TEST
  } else {
    min_cnt = 560;                 // начало теста с 9:20
    myGLCD.setColor(0, 0, 0);
    myGLCD.fillRect (1, 37, 318, 99);        // clear test results if any
    myGLCD.fillRect (1, 187, 318, 227);      // clear the "Back" and "Cancel" Buttons
    myGLCD.setColor(130, 130, 130);          // цвет серый
    myGLCD.drawRoundRect (6, 41, 98, 105);   // рамка вокруг времени
    myGLCD.setColor(110, 110, 110);
    myGLCD.drawRoundRect (119, 39, 311, 53); // рамка вокруг УРОВНИ ВЫХОДОВ (0-255)

    myGLCD.setColor(0, 0, 255); // синяя кнопка тест выкл.
    myGLCD.fillRect(stsT[0] + 5, stsT[1] + 5, stsT[2] - 5, stsT[3] - 40); // clear 'start'
    setFont(LARGE, 255, 0, 0, 0, 0, 255);
    myGLCD.setFont(BigRusFont);
    myGLCD.print(print_text[4], stsT[0] + 15, stsT[1] + 15); // STOP

    Menu_Text(40, 239, 166, 1, 8 );       // Время
    Menu_Text(41, 235, 175, 1, 8 );       // Вперед
    Menu_Text(40, 37, 166, 1, 8 );        // Время
    Menu_Text(42, 37, 175, 1, 8 );        // Назад

    myGLCD.drawRoundRect (stsT[0], stsT[1], stsT[2], stsT[3]); // red button during test

    Menu_Text(43, CENTER, 17, 0, 8 );     // Тест в процессе
    Menu_Text(44, 13, 49, 0, 2 );         // ВРЕМЯ
    Menu_Text(45, 125, 41, 2, 2 );        // УРОВНИ ВЫХОДОВ

    while (LEDtestTick) {                              // test LED and speed up time
      unsigned long currentMillis = millis();
      if (myTouch.dataAvailable()) {
        processMyTouch();
      }
      if (currentMillis - previousMillisLED > 800) {   // change time every 0.5s
        previousMillisLED = currentMillis; min_cnt++;
        int hours = min_cnt / 60;
        int minut = min_cnt % 60;

        // Test LED
        setFont(LARGE, 255, 255, 255, 0, 0, 0);
        myGLCD.print(print_text[56], 44, 78);     // :
        myGLCD.setFont(DotMatrix_M_Num);  // Выбор шрифта

        myGLCD.printNumI(hours, 12, 75, 2, '0');
        myGLCD.printNumI(minut, 60, 75, 2, '0');

        currentTime = millis();
        if (currentTime >= (loopTime + 250)) {      // сравниваем текущий таймер с переменной loopTime + 1 секунда
          loopTime = currentTime;
        }           // в loopTime записываем новое значение
        // -----------
        myGLCD.setFont(RusFont1);
        myGLCD.setColor(rgbCh0[0], rgbCh0[1], rgbCh0[2]);
        myGLCD.setBackColor(0, 0, 0);
        String wwtled = print_text[104] + String(wwtled_out) + print_text[153] + " ";  // БЕЛЫЙ :
        char bufferW[11]; wwtled.toCharArray(bufferW, 11);
        myGLCD.print(bufferW, 133, 57);
        //------------
        myGLCD.setColor(rgbCh1[0], rgbCh1[1], rgbCh1[2]);
        myGLCD.setBackColor(0, 0, 0);
        String swtled = print_text[105] + String(cwtled_out) + print_text[153] + " ";  // ГОЛУБОЙ
        char bufferB[11]; swtled.toCharArray(bufferB, 11);
        myGLCD.print(bufferB, 133, 67);
        //------------
        myGLCD.setColor(rgbCh2[0], rgbCh2[1], rgbCh2[2]);
        myGLCD.setBackColor(0, 0, 0); //                 v -два пробела
        String rswtled = print_text[106] + String(rbled_out) + print_text[153] + " "; // СИНИЙ
        char bufferRB[11]; rswtled.toCharArray(bufferRB, 11);
        myGLCD.print(bufferRB, 133, 77);
        //------------
        myGLCD.setColor(rgbCh3[0], rgbCh3[1], rgbCh3[2]);
        myGLCD.setBackColor(0, 0, 0);
        String rled = print_text[107] + String(rled_out) + print_text[153] + " ";  // КРАСНЫЙ
        char bufferR[11]; rled.toCharArray(bufferR, 11);
        myGLCD.print(bufferR, 225, 57);
        //------------
        myGLCD.setColor(rgbCh4[0], rgbCh4[1], rgbCh4[2]);
        myGLCD.setBackColor(0, 0, 0);
        String uvled = print_text[108] + String(uvled_out) + print_text[153] + " "; // УЛЬТРА
        char bufferUV[11]; uvled.toCharArray(bufferUV, 11);
        myGLCD.print(bufferUV, 225, 67);
        //------------
        myGLCD.setColor(rgbCh5[0], rgbCh5[1], rgbCh5[2]);
        myGLCD.setBackColor(0, 0, 0);
        String oLed = print_text[109] + String(oLed_out) + print_text[153] + " "; // ОРАНЖ.
        char bufferS[11]; oLed.toCharArray(bufferS, 11);
        myGLCD.print(bufferS, 225, 77);
        //------------
        myGLCD.setColor(rgbCh6[0], rgbCh6[1], rgbCh6[2]);
        myGLCD.setBackColor(0, 0, 0);
        String gled = print_text[110] + String(gled_out) + print_text[153] + " ";  // ЗЕЛЕНЫЙ:
        char bufferGR[11]; gled.toCharArray(bufferGR, 11);
        myGLCD.print(bufferGR, 225, 87);
        LED_levelo_output(); checkTempC();
      }
    }
  }
}    // TimeDateBar();

/*********************************************** ТЕСТ ОСВЕЩЕНИЯ ПОКАНАЛЬНО *************************************************** dispScreen = 6 */
void testIndLedScreen() {

  PrintStringIndex = 4; printHeader (); // РУЧНОЙ ТЕСТ В ДИАПАЗОНЕ 0-2000

  setFont(SMALL, 255, 255, 255, 0, 0, 0);
  myGLCD.setColor(rgbCh0[0], rgbCh0[1], rgbCh0[2]);
  myGLCD.print(print_text[184], 54, 176);   //  WHT
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[46])));

  myGLCD.setColor(rgbCh1[0], rgbCh1[1], rgbCh1[2]);
  myGLCD.print(print_text[183], 92, 176);    // BLU
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[46])));

  myGLCD.setColor(rgbCh2[0], rgbCh2[1], rgbCh2[2]);
  myGLCD.print(print_text[182], 130, 176);   // RBL
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[46])));

  myGLCD.setColor(rgbCh3[0], rgbCh3[1], rgbCh3[2]);
  myGLCD.print(print_text[181], 168, 176);   // RED
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[46])));

  myGLCD.setColor(rgbCh4[0], rgbCh4[1], rgbCh4[2]);
  myGLCD.print(print_text[180], 206, 176);   // UVL
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[46])));

  myGLCD.setColor(rgbCh5[0], rgbCh5[1], rgbCh5[2]);     // initial Prints
  myGLCD.print(print_text[179], 244, 176);   // ORG
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[46])));

  myGLCD.setColor(rgbCh6[0], rgbCh6[1], rgbCh6[2]);
  myGLCD.print(print_text[178], 282, 176);   // GRN
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[46])));

  for (int b = 0; b < 7; b++) {
    drawUpButtonSlide((b * 38) + 49, 17);
  }
  for (int b = 0; b < 7; b++) {
    drawDownButtonSlide((b * 38) + 49, 200);
  }
  drawSliderBarGraph();

  for (int i = 0; i < 7; i++) {
    if (i == 0) {
      sbR = rgbCh0[0];  // WHITE
      sbG = rgbCh0[1];
      sbB = rgbCh0[2];
      sbX1 = 49;
      sbX2 = 79;
      yWHT;
      x = 54;
      y = 25;
      UpDnButtonSlide();
    }
    if (i == 1) {
      sbR = rgbCh1[0];  // BLUE
      sbG = rgbCh1[1];
      sbB = rgbCh1[2];
      sbX1 = 87;
      sbX2 = 117;
      yBLU;
      x = 92;
      y = 25;
      UpDnButtonSlide();
    }
    if (i == 2) {
      sbR = rgbCh2[0];  // ROYAL BLUE
      sbG = rgbCh2[1];
      sbB = rgbCh2[2];
      sbX1 = 125;
      sbX2 = 155;
      yRBL;
      x = 130;
      y = 25;
      UpDnButtonSlide();
    }
    if (i == 3) {
      sbR = rgbCh3[0];  // RED
      sbG = rgbCh3[1];
      sbB = rgbCh3[2];
      sbX1 = 163;
      sbX2 = 193;
      yRED;
      x = 168;
      y = 25;
      UpDnButtonSlide();
    }
    if (i == 4) {
      sbR = rgbCh4[0];  // UV
      sbG = rgbCh4[1];
      sbB = rgbCh4[2];
      sbX1 = 201;
      sbX2 = 231;
      yUVL;
      x = 206;
      y = 25;
      UpDnButtonSlide();
    }
    if (i == 5) {
      sbR = rgbCh5[0];  // Orange
      sbG = rgbCh5[1];
      sbB = rgbCh5[2];
      sbX1 = 239;
      sbX2 = 269;
      ySMP;
      x = 244;
      y = 25;
      UpDnButtonSlide();
    }
    if (i == 6) {
      sbR = rgbCh6[0];  // Green
      sbG = rgbCh6[1];
      sbB = rgbCh6[2];
      sbX1 = 277;
      sbX2 = 307;
      yGRN;
      x = 282;
      y = 25;
      UpDnButtonSlide();
    }
  }
}

/***************************************   ПРИСВОЕНИЕ ЦВЕТА КАНАЛАМ **************************************************/
void RGBTune() {
  PrintStringIndex = 5; printHeader (); // УСТАНОВКА ЦВЕТА КАНАЛОВ

  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect ((15) - 2, (45) - 2, (180) + 2, (75) + 2);

  if (rgbCh0[0] + rgbCh0[1] + rgbCh0[2] < 100) {
    myGLCD.setColor(200, 200, 200);
  }
  else {
    myGLCD.setColor(rgbCh0[0], rgbCh0[1], rgbCh0[2]);
  }
  myGLCD.drawRoundRect ((RGBch1[0]) - 2, (RGBch1[1]) - 2, (RGBch1[2]) + 2, (RGBch1[3]) + 2);

  if (rgbCh1[0] + rgbCh1[1] + rgbCh1[2] < 100) {
    myGLCD.setColor(200, 200, 200);
  }
  else {
    myGLCD.setColor(rgbCh1[0], rgbCh1[1], rgbCh1[2]);
  }
  myGLCD.drawRoundRect ((RGBch2[0]) - 2, (RGBch2[1]) - 2, (RGBch2[2]) + 2, (RGBch2[3]) + 2);

  if (rgbCh2[0] + rgbCh2[1] + rgbCh2[2] < 100) {
    myGLCD.setColor(200, 200, 200);
  }
  else {
    myGLCD.setColor(rgbCh2[0], rgbCh2[1], rgbCh2[2]);
  }
  myGLCD.drawRoundRect ((RGBch3[0]) - 2, (RGBch3[1]) - 2, (RGBch3[2]) + 2, (RGBch3[3]) + 2);

  if (rgbCh3[0] + rgbCh3[1] + rgbCh3[2] < 100) {
    myGLCD.setColor(200, 200, 200);
  }
  else {
    myGLCD.setColor(rgbCh3[0], rgbCh3[1], rgbCh3[2]);
  }
  myGLCD.drawRoundRect ((RGBch4[0]) - 2, (RGBch4[1]) - 2, (RGBch4[2]) + 2, (RGBch4[3]) + 2);

  if (rgbCh4[0] + rgbCh4[1] + rgbCh4[2] < 100) {
    myGLCD.setColor(200, 200, 200);
  }
  else {
    myGLCD.setColor(rgbCh4[0], rgbCh4[1], rgbCh4[2]);
  }
  myGLCD.drawRoundRect ((RGBch5[0]) - 2, (RGBch5[1]) - 2, (RGBch5[2]) + 2, (RGBch5[3]) + 2);

  if (rgbCh5[0] + rgbCh5[1] + rgbCh5[2] < 100) {
    myGLCD.setColor(200, 200, 200);
  }
  else {
    myGLCD.setColor(rgbCh5[0], rgbCh5[1], rgbCh5[2]);
  }
  myGLCD.drawRoundRect ((RGBch6[0]) - 2, (RGBch6[1]) - 2, (RGBch6[2]) + 2, (RGBch6[3]) + 2);

  if (rgbCh6[0] + rgbCh6[1] + rgbCh6[2] < 100) {
    myGLCD.setColor(200, 200, 200);
  }
  else {
    myGLCD.setColor(rgbCh6[0], rgbCh6[1], rgbCh6[2]);
  }
  myGLCD.drawRoundRect ((RGBch7[0]) - 2, (RGBch7[1]) - 2, (RGBch7[2]) + 2, (RGBch7[3]) + 2);


  printButton("", MOONch[0], MOONch[1], MOONch[2], MOONch[3]);    // КНОПКА ЛУНЫ
  myGLCD.setFont(RusFont1);
  myGLCD.print(print_text[89], MOONch[0] + 22, MOONch[1] + 12);              //  ЛУНА

  TopSldY = 53; BotSldY = TopSldY + (100);

  for (byte b = 5; b < 8; b++) {   // UP Buttons
    drawUpButtonSlide(((b * 35) + 21), TopSldY - (26));
  }
  for (byte b = 5; b < 8; b++) {   // DOWN Buttons
    drawDownButtonSlide(((b * 35) + 21), TopSldY + (115));
  }

  for (byte i = 5; i < 8; i++) {
    sbX1 = ((i * 35) + 21); sbX2 = ((i * 35) + 51); // draw white bar outline rectangle (byte i=0; i<8; i++)sbX1=(i*35)+4; sbX2=(i*35)+34;
    setFont(LARGE, 255, 255, 255, 0, 0, 0);
    myGLCD.drawRect(sbX1, TopSldY - 1, sbX2, BotSldY);
  }

  for (byte i = 5; i < 8; i++) {
    sbX1 = ((i * 35) + 21); sbX2 = ((i * 35) + 51); // print Slider Bar current values
    SliderSwitch  = true;

    if (i == 5) {
      sbR = 255; sbG = 0; sbB = 0; tSlide =  temp_R_color;	 // R colour(Красный)
      setFont(LARGE, sbR, sbG, sbB, 0, 0, 0);
      R_color = SliderBarsForChange(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }

    if (i == 6) {
      sbR = 0; sbG = 255; sbB = 0; tSlide =  temp_G_color;	 // G colour (Зеленый)
      setFont(LARGE, sbR, sbG, sbB, 0, 0, 0);
      G_color = SliderBarsForChange(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }

    if (i == 7) {
      sbR = 0; sbG = 0; sbB = 255; tSlide =  temp_B_color;	 // B colour (Синий)
      setFont(LARGE, sbR, sbG, sbB, 0, 0, 0);
      B_color = SliderBarsForChange(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }
  }

  myGLCD.setColor(temp_R_color, temp_G_color, temp_B_color);
  myGLCD.fillRoundRect (15, 45, 180, 75);

  PrintBSC3b();

}

/*********************************** ЗНАЧЕНИЯ ЯРКОСТИ ЛУНЫ********************************* dispScreen = 8 */
void ledValuesScreen() {

  int a;

  // MOON
  tMinI = MinI; tMaxI = MaxI;
  PrintStringIndex = 6;  printHeader (); // УСТАНОВКА ЯРКОСТИ ЛУНЫ

  Menu_Text(63, 36, 20, 2, 2 );      // Минимальная
  Menu_Text(65, 52, 32, 2, 2 );      //   Яркость
  myGLCD.drawBitmap(52, 47, 53, 53, First_Quarter, 1);
  myGLCD.print(print_text[176], 46, 122);       // (0--100)(48, 122) 255  0...100 %"
  myGLCD.print(print_text[60], 20, 177);  // -1
  myGLCD.print(print_text[61], 124, 177); // +5
  Menu_Text(64, 195, 20, 2, 2 );       // Максимальная
  Menu_Text(65, 214, 32, 2, 2 );       //   Яркость
  myGLCD.drawBitmap(215, 48, 53, 53, Full_Moon, 1);

  myGLCD.print(print_text[176], 207, 122);   // (0--100)(209, 122) 255  0...100 %"
  myGLCD.print(print_text[60], 182, 177);    // -1
  myGLCD.print(print_text[61], 286, 177);    // +5

  Menu_Text(66, 39, 108, 1, 2 ); // НОВАЯ ЛУНА
  Menu_Text(67, 196, 108, 1, 2 ); // ПОЛНАЯ ЛУНА

  setFont(LARGE, 255, 255, 255, 0, 0, 0);
  myGLCD.print(print_text[111], 55, 152);  // (три пробела)
  if (tMinI <= 9) {
    myGLCD.printNumI(tMinI, 71, 152);
  }
  if ((tMinI >= 10) && (tMinI <= 99)) {
    myGLCD.printNumI(tMinI, 63, 152);
  }
  if (tMinI >= 100) {
    myGLCD.printNumI(tMinI, 55, 152);
  }

  myGLCD.print(print_text[111], 217, 152); // (три пробела)
  if (tMaxI <= 9) {
    myGLCD.printNumI(tMaxI, 233, 152);
  }
  if ((tMaxI >= 10) && (tMaxI <= 99)) {
    myGLCD.printNumI(tMaxI, 225, 152);
  }
  if (tMaxI >= 100) {
    myGLCD.printNumI(tMaxI, 217, 152);
  }

  printButton(print_text[28], MINiM[0], MINiM[1], MINiM[2], MINiM[3], true);  // Minimum Illum. minus
  printButton(print_text[27], MINiP[0], MINiP[1], MINiP[2], MINiP[3], true);  // Minimum Illum. plus
  printButton(print_text[28], MAXiM[0], MAXiM[1], MAXiM[2], MAXiM[3], true);  // Max Illum. minus
  printButton(print_text[27], MAXiP[0], MAXiP[1], MAXiP[2], MAXiP[3], true);  // Max Illum. plus

  myGLCD.setColor(64, 64, 64);      // серый
  myGLCD.drawRect(158, 14, 160, 194);
  myGLCD.drawRect(0, 196, 319, 194);
  myGLCD.drawRect(54, 147, 104, 173);  // Min
  myGLCD.drawRect(216, 147, 266, 173); // Max

  myGLCD.setColor(0, 0, 0);
  myGLCD.drawLine(159, 193, 159, 195);
  PrintBSC3b();
}

//**************************************КОРМУШКА ВКЛ ВЫКЛ*************************************** dispScreen = ? ****//
void feedingTimeOnOff()
{
  if ((feedTime == 1) && (FEEDTime1 == 1))
  { myGLCD.setColor(0, 255, 0);
    myGLCD.fillRoundRect(70, 150, 250, 170);
    setFont(SMALL, 0, 0, 0, 0, 255, 0);
    myGLCD.setFont(RusFont3);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
    myGLCD.print(buffer, 94, 157);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[128])));
    myGLCD.print(buffer, 94 + 120, 157);
  }

  if ((feedTime == 1) && (FEEDTime1 == 0))
  { myGLCD.setColor(255, 0, 0);
    myGLCD.fillRoundRect(70, 150, 250, 170);
    setFont(SMALL, 255, 255, 255, 255, 0, 0);
    myGLCD.setFont(RusFont3);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
    myGLCD.print(buffer, 90, 157);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184])));
    myGLCD.print(buffer, 90 + 120, 157);
  }

  if ((feedTime == 2) && (FEEDTime2 == 1))
  { myGLCD.setColor(0, 255, 0);
    myGLCD.fillRoundRect(70, 150, 250, 170);
    setFont(SMALL, 0, 0, 0, 0, 255, 0);
    myGLCD.setFont(RusFont3);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
    myGLCD.print(buffer, 94, 157);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[128])));
    myGLCD.print(buffer, 94 + 120, 157);
  }

  if ((feedTime == 2) && (FEEDTime2 == 0))
  { myGLCD.setColor(255, 0, 0);
    myGLCD.fillRoundRect(70, 150, 250, 170);
    setFont(SMALL, 255, 255, 255, 255, 0, 0);
    myGLCD.setFont(RusFont3);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
    myGLCD.print(buffer, 90, 157);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184])));
    myGLCD.print(buffer, 90 + 120, 157);
  }

  if ((feedTime == 3) && (FEEDTime3 == 1))
  { myGLCD.setColor(0, 255, 0);
    myGLCD.fillRoundRect(70, 150, 250, 170);
    setFont(SMALL, 0, 0, 0, 0, 255, 0);
    myGLCD.setFont(RusFont3);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
    myGLCD.print(buffer, 94, 157);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[128])));
    myGLCD.print(buffer, 94 + 120, 157);
  }

  if ((feedTime == 3) && (FEEDTime3 == 0))
  { myGLCD.setColor(255, 0, 0);
    myGLCD.fillRoundRect(70, 150, 250, 170);
    setFont(SMALL, 255, 255, 255, 255, 0, 0);
    myGLCD.setFont(RusFont3);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
    myGLCD.print(buffer, 90, 157);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184])));
    myGLCD.print(buffer, 90 + 120, 157);
  }

  if ((feedTime == 4) && (FEEDTime4 == 1))
  { myGLCD.setColor(0, 255, 0);
    myGLCD.fillRoundRect(70, 150, 250, 170);
    setFont(SMALL, 0, 0, 0, 0, 255, 0);
    myGLCD.setFont(RusFont3);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
    myGLCD.print(buffer, 94, 157);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[128])));
    myGLCD.print(buffer, 94 + 120, 157);
  }

  if ((feedTime == 4) && (FEEDTime4 == 0))
  { myGLCD.setColor(255, 0, 0);
    myGLCD.fillRoundRect(70, 150, 250, 170);
    setFont(SMALL, 255, 255, 255, 255, 0, 0);
    myGLCD.setFont(RusFont3);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
    myGLCD.print(buffer, 90, 157);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184])));
    myGLCD.print(buffer, 90 + 120, 157);
  }

  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(70, 150, 250, 170);
}

/******** AUTOMATIC FEEDER SCREEN ************* dispScreen = 38 **********************/
void autoFeederScreen()
{
  PrintStringIndex = 27;
  printHeader ();

  myGLCD.setColor(64, 64, 64);
  myGLCD.drawRect(0, 196, 319, 194);
  printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);        // НАЗАД
  printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);        // ОТМЕНА
  myGLCD.setColor(0, 192, 192);
  myGLCD.drawRect(159, 194, 161, 121);
  myGLCD.drawRoundRect(78, 87, 242, 121);
  myGLCD.drawRoundRect(80, 89, 240, 119);
  myGLCD.drawRect(0, 103, 78, 105);
  myGLCD.drawRect(242, 103, 319, 105);
  myGLCD.drawLine(159, 87, 159, 14);
  myGLCD.drawLine(161, 87, 161, 14);
  myGLCD.setColor(0, 0, 0);
  myGLCD.drawLine(160, 195, 160, 193);
  myGLCD.drawLine(160, 122, 160, 120);
  myGLCD.drawLine(77, 104, 79, 104);
  myGLCD.drawLine(241, 104, 243, 104);
  myGLCD.drawLine(160, 88, 160, 86);
  myGLCD.setColor(153, 0, 102);
  myGLCD.fillRoundRect(85, 94, 235, 114);           //Feed Fish Now Button
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(85, 94, 235, 114);
  setFont(SMALL, 255, 255, 255, 153, 0, 102);
  myGLCD.setFont(RusFont3);
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[180])));
  myGLCD.print(buffer, 97, 101);

  if (FEEDTime1 == 0)                               //Feeding Time 1 Button
  { myGLCD.setColor(255, 0, 0);
    myGLCD.fillRoundRect(5, 20, 155, 40);
    setFont(SMALL, 255, 255, 255, 255, 0, 0);
    myGLCD.setFont(RusFont3);
    myGLCD.print("1", 30, 27);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
    myGLCD.print(buffer, 60, 27);
    setFont(SMALL, 255, 0, 0, 0, 0, 0);
    myGLCD.setFont(RusFont2);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[182])));
    myGLCD.print(buffer, 62, 52);
    myGLCD.setFont(RusFont2);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[183])));
    myGLCD.print(buffer, 24, 65);
  }
  else
  { myGLCD.setColor(0, 255, 0);
    myGLCD.fillRoundRect(5, 20, 155, 40);
    setFont(SMALL, 0, 0, 0, 0, 255, 0);
    myGLCD.setFont(RusFont3);
    myGLCD.print("1", 30, 27);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
    myGLCD.print(buffer, 60, 27);
    timeDispH = feedFish1H; timeDispM = feedFish1M;
    if (setTimeFormat == 0) {
      xTimeH = 40;
    }
    if (setTimeFormat == 1) {
      xTimeH = 16;
    }
    if ((timeDispH >= 0) && (timeDispH <= 11)) {
      AM_PM = 1;
    }
    else {
      AM_PM = 2;
    }
    yTime = 56; xColon = xTimeH + 32;
    xTimeM10 = xTimeH + 48; xTimeM1 = xTimeH + 64; xTimeAMPM = xTimeH + 96;
    timeCorrectFormat();
  }

  if (FEEDTime2 == 0)                               //Feeding Time 2 Button
  { myGLCD.setColor(255, 0, 0);
    myGLCD.fillRoundRect(165, 20, 315, 40);
    setFont(SMALL, 255, 255, 255, 255, 0, 0);
    myGLCD.setFont(RusFont3);
    myGLCD.print("2", 194, 27);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
    myGLCD.print(buffer, 224, 27);
    setFont(SMALL, 255, 0, 0, 0, 0, 0);
    myGLCD.setFont(RusFont2);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[182])));
    myGLCD.print(buffer, 224, 52);
    myGLCD.setFont(RusFont2);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[183])));
    myGLCD.print(buffer, 184, 65);
  }

  else
  { myGLCD.setColor(0, 255, 0);
    myGLCD.fillRoundRect(165, 20, 315, 40);
    setFont(SMALL, 0, 0, 0, 0, 255, 0);
    myGLCD.setFont(RusFont3);
    myGLCD.print("2", 194, 27);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
    myGLCD.print(buffer, 224, 27);
    timeDispH = feedFish2H; timeDispM = feedFish2M;
    if (setTimeFormat == 0) {
      xTimeH = 200;
    }
    if (setTimeFormat == 1) {
      xTimeH = 176;
    }
    if ((timeDispH >= 0) && (timeDispH <= 11)) {
      AM_PM = 1;
    }
    else {
      AM_PM = 2;
    }
    yTime = 56; xColon = xTimeH + 32;
    xTimeM10 = xTimeH + 48; xTimeM1 = xTimeH + 64; xTimeAMPM = xTimeH + 96;
    timeCorrectFormat();
  }

  if (FEEDTime3 == 0)                               //Feeding Time 3 Button
  { myGLCD.setColor(255, 0, 0);
    myGLCD.fillRoundRect(5, 168, 155, 188);
    setFont(SMALL, 255, 255, 255, 255, 0, 0);
    myGLCD.setFont(RusFont3);
    myGLCD.print("3", 30, 175);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
    myGLCD.print(buffer, 60, 175);
    setFont(SMALL, 255, 0, 0, 0, 0, 0);
    myGLCD.setFont(RusFont2);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[182])));
    myGLCD.print(buffer, 62, 133);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[183])));
    myGLCD.print(buffer, 24, 146);
  }
  else
  { myGLCD.setColor(0, 255, 0);
    myGLCD.fillRoundRect(5, 168, 155, 188);
    setFont(SMALL, 0, 0, 0, 0, 255, 0);
    myGLCD.setFont(RusFont3);
    myGLCD.print("3", 30, 175);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
    myGLCD.print(buffer, 60, 175);
    timeDispH = feedFish3H; timeDispM = feedFish3M;
    if (setTimeFormat == 0) {
      xTimeH = 40;
    }
    if (setTimeFormat == 1) {
      xTimeH = 16;
    }
    if ((timeDispH >= 0) && (timeDispH <= 11)) {
      AM_PM = 1;
    }
    else {
      AM_PM = 2;
    }
    yTime = 137; xColon = xTimeH + 32;
    xTimeM10 = xTimeH + 48; xTimeM1 = xTimeH + 64; xTimeAMPM = xTimeH + 96;
    timeCorrectFormat();
  }

  if (FEEDTime4 == 0)                               //Feeding Time 4 Button
  { myGLCD.setColor(255, 0, 0);
    myGLCD.fillRoundRect(165, 168, 315, 188);
    setFont(SMALL, 255, 255, 255, 255, 0, 0);
    myGLCD.setFont(RusFont3);
    myGLCD.print("4", 194, 175);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
    myGLCD.print(buffer, 224, 175);
    setFont(SMALL, 255, 0, 0, 0, 0, 0);
    myGLCD.setFont(RusFont2);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[182])));
    myGLCD.print(buffer, 224, 133);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[183])));
    myGLCD.print(buffer, 184, 146);
  }
  else
  { myGLCD.setColor(0, 255, 0);
    myGLCD.fillRoundRect(165, 168, 315, 188);
    setFont(SMALL, 0, 0, 0, 0, 255, 0);
    myGLCD.setFont(RusFont3);
    myGLCD.print("4", 194, 175);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
    myGLCD.print(buffer, 224, 175);
    timeDispH = feedFish4H; timeDispM = feedFish4M;
    if (setTimeFormat == 0) {
      xTimeH = 200;
    }
    if (setTimeFormat == 1) {
      xTimeH = 176;
    }
    if ((timeDispH >= 0) && (timeDispH <= 11)) {
      AM_PM = 1;
    }
    else {
      AM_PM = 2;
    }
    yTime = 137; xColon = xTimeH + 32;
    xTimeM10 = xTimeH + 48; xTimeM1 = xTimeH + 64; xTimeAMPM = xTimeH + 96;
    timeCorrectFormat();
  }

  myGLCD.setColor(255, 255, 255);
  for (int x = 0; x < 2; x++)
  { for (int y = 0; y < 2; y++)
    {
      myGLCD.drawRoundRect((x * 160) + 5, (y * 148) + 20, (x * 160) + 155, (y * 148) + 40);
    }
  }
}

/******************************************* ВЫХОД АВТОКОРМУШКИ ***************************************************/
void feedingTimeOutput() {
  if ((FEEDTime1 == 1) && (feedFish1H == RTC.hour) && (feedFish1M == RTC.minute) && (RTC.second >= 0 && RTC.second < 5)) {
    fiveTillBackOn1 = 0; FeedWaveCtrl_1 = true;
    digitalWrite(autoFeeder, HIGH);
  }
  else {
    if ((FEEDTime2 == 1) && (feedFish2H == RTC.hour) && (feedFish2M == RTC.minute) && (RTC.second >= 0 && RTC.second < 5)) {
      fiveTillBackOn2 = 0; FeedWaveCtrl_2 = true;
      digitalWrite(autoFeeder, HIGH);
    }
    else {
      if ((FEEDTime3 == 1) && (feedFish3H == RTC.hour) && (feedFish3M == RTC.minute) && (RTC.second >= 0 && RTC.second < 5)) {
        fiveTillBackOn3 = 0; FeedWaveCtrl_3 = true;
        digitalWrite(autoFeeder, HIGH);
      }
      else {
        if ((FEEDTime4 == 1) && (feedFish4H == RTC.hour) && (feedFish4M == RTC.minute) && (RTC.second >= 0 && RTC.second < 5)) {
          fiveTillBackOn4 = 0; FeedWaveCtrl_4 = true;
          digitalWrite(autoFeeder, HIGH);
        }
        else {
          if (ManFeed == 1) {
            fiveTillBackOn5 = 0; FeedWaveCtrl_5 = true;
            digitalWrite(autoFeeder, HIGH); ManFeed = 0;
            if (dispScreen == 38) {
              myGLCD.setColor(153, 0, 102);
              myGLCD.fillRoundRect(85, 94, 235, 114);
              myGLCD.setColor(255, 255, 255);
              myGLCD.drawRoundRect(85, 94, 235, 114);
              setFont(SMALL, 255, 255, 255, 153, 0, 102);
              myGLCD.setFont(RusFont3);
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[180])));
              myGLCD.print(buffer, 97, 101);
            }
          }
          else {
            digitalWrite(autoFeeder, LOW);
          }
        }
      }
    }
  }

  if (FeedWaveCtrl_0 == true) {
    fiveTillBackOn0++;
    if (dispScreen == 0) {
      CountDown(fiveTillBackOn0);
    }
    if (fiveTillBackOn0 > Tpause)                     //120 is 10 minutes (120/12=10)
    { FeedWaveCtrl_0 = false;
      if (dispScreen == 0) {
        myGLCD.setColor(0, 0, 0);
        myGLCD.fillRect(273, 56, 310, 70);
      }
    }
  }

  if (FeedWaveCtrl_1 == true) {
    fiveTillBackOn1++;
    if (dispScreen == 0) {
      CountDown(fiveTillBackOn1);
    }
    if (fiveTillBackOn1 > Tpause)                     //120 is 10 minutes (120/12=10)
    { FeedWaveCtrl_1 = false;
      if (dispScreen == 0) {
        myGLCD.setColor(0, 0, 0);
        myGLCD.fillRect(273, 56, 310, 70);
      }
    }
  }

  if (FeedWaveCtrl_2 == true) {
    fiveTillBackOn2++;
    if (dispScreen == 0) {
      CountDown(fiveTillBackOn2);
    }
    if (fiveTillBackOn2 > Tpause)                     //120 is 10 minutes (120/12=10)
    { FeedWaveCtrl_2 = false;
      if (dispScreen == 0 && screenSaverCounter < setScreenSaverTimer) {
        myGLCD.setColor(0, 0, 0);
        myGLCD.fillRect(273, 56, 310, 70);
      }
    }
  }

  if (FeedWaveCtrl_3 == true) {
    fiveTillBackOn3++;
    if (dispScreen == 0) {
      CountDown(fiveTillBackOn3);
    }
    if (fiveTillBackOn3 > Tpause)                     //120 is 10 minutes (120/12=10)
    { FeedWaveCtrl_3 = false;
      if (dispScreen == 0) {
        myGLCD.setColor(0, 0, 0);
        myGLCD.fillRect(273, 56, 310, 70);
      }
    }
  }

  if (FeedWaveCtrl_4 == true) {
    fiveTillBackOn4++;
    if (dispScreen == 0) {
      CountDown(fiveTillBackOn4);
    }
    if (fiveTillBackOn4 > Tpause)                     //120 is 10 minutes (120/12=10)
    { FeedWaveCtrl_4 = false;
      if (dispScreen == 0) {
        myGLCD.setColor(0, 0, 0);
        myGLCD.fillRect(273, 56, 310, 70);
      }
    }
  }

  if (FeedWaveCtrl_5 == true) {
    fiveTillBackOn5++;
    if (dispScreen == 0) {
      CountDown(fiveTillBackOn5);
    }
    if (fiveTillBackOn5 > Tpause)                     //120 is 10 minutes (120/12=10)
    { FeedWaveCtrl_5 = false;
      if (dispScreen == 0) {
        myGLCD.setColor(0, 0, 0);
        myGLCD.fillRect(273, 56, 310, 70);
      }
    }
  }

  if ((FeedWaveCtrl_0 == false) && (FeedWaveCtrl_1 == false) && (FeedWaveCtrl_2 == false) &&
      (FeedWaveCtrl_3 == false) && (FeedWaveCtrl_4 == false) && (FeedWaveCtrl_5 == false)) {
    setAutoStop = 0;
    if (dispScreen == 0) {
      Menu_Text(101, 277, 56, 1, 4);
    }
  }
  else {
    setAutoStop = 1;
  }


}

/*********************** END OF AUTOMATIC FEEDER SETTINGS SCREEN **********************/
/***** SET AUTOMATIC FEEDER TIMES SCREEN ********** dispScreen = 39 *******************/
void setFeederTimesScreen(boolean refreshAll = true)
{
  if (feedTime == 1)
  { //printHeader("Set Feeding Time 1"); }
    PrintStringIndex = 27;
    printHeader ();
  }
  if (feedTime == 2)
  { //printHeader("Set Feeding Time 2");}
    PrintStringIndex = 27;
    printHeader ();
  }
  if (feedTime == 3)
  { //printHeader("Set Feeding Time 3");}
    PrintStringIndex = 27;
    printHeader ();
  }
  if (feedTime == 4)
  { //printHeader("Set Feeding Time 4");}
    PrintStringIndex = 27;
    printHeader ();
  }

  if (refreshAll)
  {
    //   rtcSetMin=RTC.minute; rtcSetHr=RTC.hour;

    if (feedTime == 1)
    {
      rtcSetMin = feedFish1M;
      rtcSetHr = feedFish1H;
    }
    if (feedTime == 2)
    {
      rtcSetMin = feedFish2M;
      rtcSetHr = feedFish2H;
    }
    if (feedTime == 3)
    {
      rtcSetMin = feedFish3M;
      rtcSetHr = feedFish3H;
    }
    if (feedTime == 4)
    {
      rtcSetMin = feedFish4M;
      rtcSetHr = feedFish4H;
    }

    myGLCD.setColor(64, 64, 64);
    myGLCD.drawRect(0, 196, 319, 194);

    PrintBSC3b();

    feedingTimeOnOff();

    drawUpButtonSlide(houP[0], houP[1]);                //hour up
    drawUpButtonSlide(minP[0], minP[1]);                //min up
    drawDownButtonSlide(houM[0], houM[1]);              //hour down
    drawDownButtonSlide(minM[0], minM[1]);              //min down
    if (setTimeFormat == 1)
    { drawUpButtonSlide(ampmP[0], ampmP[1]);          //AM/PM up
      drawDownButtonSlide(ampmM[0], ampmM[1]);
    }       //AM/PM down
  }

  timeDispH = rtcSetHr; timeDispM = rtcSetMin;
  xTimeH = 107; yTime = 68;  xColon = xTimeH + 42;
  xTimeM10 = xTimeH + 70; xTimeM1 = xTimeH + 86; xTimeAMPM = xTimeH + 155;
  timeChange();
}

/********************** END OF SET AUTOMATIC FEEDER TIMES SCREEN **********************/
//======================== Помпы течения ===========================
/*1 - Pulse - WaveMaker;
  2 - Ramp - Разгон от минимума до максимума, а затем наоборот;
  3 - ReefCrazy - Имитирует reefCrest от Vortech;
  4 - Pump ON - Помпы постоянно включены
  5 - Pulse -  Пульсация
  6 - Pump OFF - Помпы постоянно выключены  */
void activMode() { // индикация выбранного режима работы помп
  setFont(LARGE, 0, 255, 0, 0, 0, 0);   // зеленый шрифт режимов
  switch (ModeSel) {
    case 1: // индикация режима 1
      //  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[115])));
      myGLCD.print("Pulse", 15, 58); // "Pulse    "  помпы работают по очереди, разгон от минимума до максимума
      break;
    case 2: // индикация режима 2
      //  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[116])));
      myGLCD.print("Rampe", 15, 58); // "Ramp    "  помпы работают одновременно, разгон от минимума до максимума
      break;
    case 3: // индикация режима 3
      //  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[117])));
      myGLCD.print("ReefCrazy" , 15, 58); // "ReefCrazy"   Имитирует шторм
      break;
    case 4: // индикация режима 4
      //  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[118])));
      myGLCD.print("Pump ON 50%", 15, 58);  // "Pump ON  "  Помпы включены на 100%
      break;
    case 5: // индикация режима 5
      //   strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[119])));
      myGLCD.print("Pulse", 15, 58);  // "Pulse    "
      break;
    case 6: // индикация режима 6
      //    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[120])));
      myGLCD.print("Pump OFF", 15, 58);  // "Pump OFF "  Помпы выключены
      break;
    case 7: // индикация режима 6
      //   strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[121])));
      myGLCD.print("FullPower", 15, 58);  // "FullPower"
      break;
  }
}

// бары уровней помп
void GraphPWMPump() {
  int bar = map(Pump1PWM + maxP1, 0, 255, 185, 77);
  if (bar < 77 ) {
    bar = 77; // левый бар
  }
  if (bar > 185) {
    bar = 185;
  }
  int bar2 = map(Pump2PWM + maxP2, 0, 255, 185, 77);
  if (bar2 < 77 ) {
    bar = 77; // правый бар
  }
  if (bar2 > 185) {
    bar = 185;
  }

  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRect(251, bar, 272, 77);         // гашение столбика канала 1
  myGLCD.fillRect(280, bar2, 301, 77);        // гашение столбика канала 2

  myGLCD.setColor(0, 0, 255);                 // цвет бара синий
  myGLCD.setBackColor(0, 0, 0);
  myGLCD.fillRect(251, 185, 272, bar);        // размер столбика яркости bar 1
  myGLCD.fillRect(280, 185, 301, bar2);       // размер столбика яркости bar 2

  setFont(SMALL, 255, 255, 0, 0, 0, 0);
  Pump1PWM = map(Pump1PWM + maxP1, 0, 255, 0, 100); // отображать уровень выхода помпы 1
  Pump2PWM = map(Pump2PWM + maxP2, 0, 255, 0, 100);
  myGLCD.printNumI(Pump1PWM, 256, 60);
  myGLCD.printNumI(Pump2PWM, 285, 60);
}

//==================== Режимы для помп
void modePump1(byte value, byte starttime, byte startvalue) { // помпы работают по очереди (левая, затем правая)----- mode 1
  times = millis();
  value = 128 + 127 * cos(2 * PI / periode * (times - (starttime + startvalue)));
  Pump1 = 255 - value;
  Pump2 = value;
}

void modePump2(byte starttime) {   // помпы работают одновременно ---------- mode 2
  times = millis();
  value = 128 + 127 * cos(2 * PI / periode * (times - (starttime + startvalue)));
  Pump1 = value;
  Pump2 = value;
}

void modePump3(long starttime) {  // --------------------------------------- mode 3
  switch (cmode) {
    case 0: times = millis();
      value = 128 + 127 * cos(2 * PI / periode * (times - (starttime + startvalue)));
      Pump1 = 100;
      Pump2 = 0;
      if (value == 2) {
        cmode = 1; // при достижении уровня 1 - перейти к case 1
      }
      break;

    case 1:
      times = millis();
      value = 128 + 127 * cos(2 * PI / periode * (times - (starttime + startvalue)));
      Pump1 = 100 - value; // 255
      Pump2 = 0;
      if (value == 1) {
        cmode = 2; // при достижении значения уровня 254 - перейти к case 1
      }
      break;

    case 2: times = millis();
      value = 128 + 127 * cos(2 * PI / periode * (times - (starttime + startvalue)));
      Pump1 = 127; // 255
      Pump2 = 127; // 255-value;
      if (value == 1) {
        cmode = 3;
      }
      break;

    case 3: times = millis();
      value = 128 + 127 * cos(2 * PI / periode * (times - (starttime + startvalue)));
      Pump1 = 0;
      Pump2 = 255 - value;
      if (value == 2) {
        cmode = 4;
      }
      break;

    case 4: times = millis();
      value = 128 + 127 * cos(2 * PI / periode * (times - (starttime + startvalue)));
      Pump1 = 255;
      Pump2 = 0;
      if (value == 254) {
        cmode = 5;
      }
      break;

    case 5: times = millis();
      value = 128 + 127 * cos(2 * PI / periode * (times - (starttime + startvalue)));
      Pump1 = 255;
      Pump2 = 255 - value;
      if (value == 254) {
        cmode = 6;
      }
      break;

    case 6: times = millis();
      value = 128 + 127 * cos(2 * PI / periode * (times - (starttime + startvalue)));
      Pump1 = 255 - value;
      Pump2 = 255;
      if (value == 254) {
        cmode = 7;
      }
      break;

    case 7: times = millis();
      value = 128 + 127 * cos(2 * PI / periode * (times - (starttime + startvalue)));
      Pump1 = 127;  // 255
      Pump2 = 127;  // 255
      if (value == 126) {
        cmode = 0;
      }
      break;
  }
}

void modePump5() {   // Низкая скорость
  times = millis();
  value = 128 + 127 * cos(2 * PI / periode * (times - (starttime + startvalue)));
  Pump1 = 127;
  Pump2 = 127;
}

void pumpPWM() {  // работа помп в цикле

  if (ModeSel == 1) {
    modePump1(value, starttime, startvalue);  // режим 1 (помпы работают по очереди)
  }
  if (ModeSel == 2) {
    modePump2(starttime);  // режим 2 (помпы работают одновременно)
  }
  if (ModeSel == 3) {
    modePump3(starttime);
  }

  if (ModeSel == 4) {
    value = 128;            // помпы работают на половину мощности
    Pump1 = value;
    Pump2 = value;
  }

  if (ModeSel == 5) {
    modePump5();
  }

  // Остановить помпы  PumpOFF --------------------------------------- Режим 6
  if (ModeSel == 6) {
    value = 0;
    Pump1 = value;    // отключить помпы
    Pump2 = value;
  }
  // Помпы работают на всю мощность ---------------------------------- Режим 7
  if (ModeSel == 7) {
    value = 255;
    Pump1 = value;
    Pump2 = value;
  }
} // Les pompes fonctionnent à pleine puissance

void P1(byte MaxPower, byte MinPower) {
  Pump1PWM = map(Pump1, 0, 100, MinPower, MaxPower);
  analogWrite(PWMPinA, (Pump1PWM + maxP1) * 8);
}   // commande la pompe A dans la boucle (* 8 11 bits PWM pin 8)

void P2(byte MaxPower, byte MinPower) {
  Pump2PWM = map(Pump2, 0, 100, MinPower, MaxPower);
  analogWrite(PWMPinB, Pump2PWM + maxP2);
} // contrôle de la pompe B dans le cycle

/************ PWM Punp ***************************************************************** dispScreen = 10 */
void WavePWMScreen() {
  PrintStringIndex = 28; printHeader (); // баннер помпы в режиме ШИМ

  myGLCD.setColor(64, 64, 64);     // Draw Dividers in Grey
  myGLCD.drawRect(0, 196, 319, 194);   // Bottom Horizontal Divider
  myGLCD.drawRect(9, 53, 235, 78);     // Cadre autour du nom du mode sélectionné
  myGLCD.drawRect(86, 100, 148, 180);  // рамка в центре (72, 98, 125, 175)
  myGLCD.drawRect(154, 100, 216, 180); // рамка в центре 2

  myGLCD.drawRect(3, 82, 75, 190);   // рамка вокруг скорость

  setFont(SMALL, 190, 190, 190, 0, 0, 0);
  myGLCD.drawRect(242, 187, 309, 187);      // x-line (горизонтальная)(243, 183, 292, 183); короткая
  myGLCD.drawRect(240, 190, 311, 190);      // горизонтальная в самом низу
  myGLCD.drawRect(245, 187, 245, 77);       // y-line (вертикальная 1)
  myGLCD.drawRect(306, 187, 306, 77);       // y-line (вертикальная 2)

  for (int i = 0; i < 11; i++) {
    myGLCD.drawLine(242, (i * 11) + 77, 248, (i * 11) + 77); // левая шкала
  }
  for (int i = 0; i < 11; i++) {
    myGLCD.drawLine(303, (i * 11) + 77, 309, (i * 11) + 77); // правая шкала
  }

  // (пунктир)
  myGLCD.setColor(150, 150, 150);
  for (int i = 77; i < 210; i++) {
    myGLCD.drawPixel(276, i);
    i = i + 10;
  }

  if (ModeSel == 1) {
    printButton("1", ModePump1[0], ModePump1[1], ModePump1[2], ModePump1[3], LARGE, GREEN_BAC);
  }
  else {
    printButton104("1", ModePump1[0], ModePump1[1], ModePump1[2], ModePump1[3], LARGE);
  }
  if (ModeSel == 2) {
    printButton("2", ModePump2[0], ModePump2[1], ModePump2[2], ModePump2[3], LARGE, GREEN_BAC);
  }
  else {
    printButton104("2", ModePump2[0], ModePump2[1], ModePump2[2], ModePump2[3], LARGE);
  }
  if (ModeSel == 3) {
    printButton("3", ModePump3[0], ModePump3[1], ModePump3[2], ModePump3[3], LARGE, GREEN_BAC);
  }
  else {
    printButton104("3", ModePump3[0], ModePump3[1], ModePump3[2], ModePump3[3], LARGE);
  }
  if (ModeSel == 4) {
    printButton("4", ModePump4[0], ModePump4[1], ModePump4[2], ModePump4[3], LARGE, GREEN_BAC);
  }
  else {
    printButton104("4", ModePump4[0], ModePump4[1], ModePump4[2], ModePump4[3], LARGE);
  }
  if (ModeSel == 5) {
    printButton("5", ModePump5[0], ModePump5[1], ModePump5[2], ModePump5[3], LARGE, GREEN_BAC);
  }
  else {
    printButton104("5", ModePump5[0], ModePump5[1], ModePump5[2], ModePump5[3], LARGE); // low speed
  }
  //                                  V - "OFF"
  if (ModeSel == 6) {
    printButton104("OFF", ModePump6[0], ModePump6[1], ModePump6[2], ModePump6[3], SMALL);
  }
  else {
    printButton("OFF", ModePump6[0], ModePump6[1], ModePump6[2], ModePump6[3], SMALL); // выкл помпы
  }
  if (ModeSel == 7) {
    printButton("MAX", ModePump7[0], ModePump7[1], ModePump7[2], ModePump7[3], SMALL, GREEN_BAC);
  }
  else {
    printButton("MAX", ModePump7[0], ModePump7[1], ModePump7[2], ModePump7[3], SMALL);  // помпы вкл на 100%
  }

  myGLCD.setFont(RusFont6);
  myGLCD.setColor(88, 255, 238); // бирюзовый шрифт
  myGLCD.setBackColor(0, 0, 0); // 240,84,0 красный
  myGLCD.print(print_text[187], 232, 179); // 0 (ноль)
  myGLCD.print(print_text[41], 225, 127);  // 50

  myGLCD.print("Vitesse|", 9, 83);         // СКОРОСТЬ

  myGLCD.setFont(RusFont1);
  myGLCD.setColor(0, 0, 255);

  myGLCD.setColor(255, 255, 255);
  myGLCD.print(F("Etat"), 236 , 15); // состояние

  setFont(SMALL, 255, 255, 255, 0, 0, 0);
  myGLCD.setFont(SmallFont);
  myGLCD.print("MIN", 90 , 86);  //  MIN
  myGLCD.print("MAX", 121 , 86); //  MAX
  myGLCD.print("MIN", 158 , 86); //  MIN
  myGLCD.print("MAX", 189 , 86); //  MAX

  drawUpButtonSlide(SpeedPplus[0], SpeedPplus[1]);      // cтрелка вверх скорость плюс
  drawDownButtonSlide(SpeedPminus[0], SpeedPminus[1]);  // стрелка вниз скорость минус

  printButton(print_text[27], MaxPlus1[0], MaxPlus1[1], MaxPlus1[2], MaxPlus1[3], SMALL);     // +  1
  printButton(print_text[28], MaxMinus1[0], MaxMinus1[1], MaxMinus1[2], MaxMinus1[3], SMALL); // -

  printButton(print_text[27], MinPlus1[0], MinPlus1[1], MinPlus1[2], MinPlus1[3], SMALL);
  printButton(print_text[28], MinMinus1[0], MinMinus1[1], MinMinus1[2], MinMinus1[3], SMALL);

  printButton(print_text[27], MaxPlus2[0], MaxPlus2[1], MaxPlus2[2], MaxPlus2[3], SMALL);   // 2
  printButton(print_text[28], MaxMinus2[0], MaxMinus2[1], MaxMinus2[2], MaxMinus2[3], SMALL);

  printButton(print_text[27], MinPlus2[0], MinPlus2[1], MinPlus2[2], MinPlus2[3], SMALL);
  printButton(print_text[28], MinMinus2[0], MinMinus2[1], MinMinus2[2], MinMinus2[3], SMALL);

  printButton("", SpeedMax[0], SpeedMax[1], SpeedMax[2], SpeedMax[3], SMALL); // перейти к минимальной скорости
  printButton("", SpeedMin[0], SpeedMin[1], SpeedMin[2], SpeedMin[3], SMALL);

  setFont(SMALL, 255, 255, 255, 0, 0, 255);
  myGLCD.setFont(SmallFont);
  myGLCD.print(F("MAX"), 8, 132, 270); // MAX повернуть текст на 90 градусов
  myGLCD.print(F("MIN"), 8, 173, 270); // MIN

  PrintBSC3b();

  setFont(LARGE, 0, 255, 0, 0, 0, 0);   // зеленый шрифт режимов
  PumpSpeed();
  activMode();
}   // индикация работы выбранного режима

void PumpSpeed() {
  setFont(SMALL, 255, 255, 255, 0, 0, 0);
  myGLCD.printNumI(maxP1, 95, 134);   // отображение максимальной скорости помпа1
  myGLCD.printNumI(minP1, 125, 134);  // отображение минимальной скорости помпа2

  myGLCD.printNumI(maxP2, 163, 134);  // отображение максимальной скорости помпа1
  myGLCD.printNumI(minP2, 193, 134);  // отображение минимальной скорости помпа2

  SpeedPump = periode;
  SpeedPump = map(periode, 32608, 10, 1, 99);  // конвертировать большие цифры > 0-100 %
  if ((SpeedPump <= 99) && (SpeedPump >= 10)) {
    myGLCD.setColor(0, 0, 0);  // очистка обласити при смене цмфры
    myGLCD.fillRoundRect(30, 125, 70, 147);
    setFont(LARGE, 192, 192, 255, 0, 0, 0);
    myGLCD.printNumI(SpeedPump, 40, 133);
  }
  if (SpeedPump <= 9) {
    myGLCD.setColor(0, 0, 0);  // очистка обласити при смене цмфры
    myGLCD.fillRoundRect(30, 125, 70, 147);
    setFont(LARGE, 192, 192, 255, 0, 0, 0);
    myGLCD.printNumI(SpeedPump, 47, 133);
  }
}

/************************************* ГЛАВНЫЕ НАСТРОЙКИ, СТР.1******************************** dispScreen = 14 */
void generalSettingsScreen_1() {

  PrintStringIndex = 10; printHeader (); // ГЛАВНЫЕ НАСТРОЙКИ, СТР.1

  myGLCD.setColor(64, 64, 64);
  myGLCD.drawRect(0, 196, 319, 194); // Bottom Horizontal Divider
  myGLCD.drawLine(0, 47, 319, 47);   // разделительная линия (поиск датч / бекап)
  myGLCD.drawLine(0, 81, 319, 81);
  myGLCD.drawLine(0, 115, 319, 115); // разделительная линия (сброс / установ)
  myGLCD.drawLine(0, 151, 319, 151); // разделительная линия (установ / устан. темп.)

  // кнопки
  myGLCD.setColor(0, 0, 255);   // синий цвет
  myGLCD.fillRoundRect(195, 20, 295, 40);       // detect
  myGLCD.fillRoundRect(205, 54, 285, 74);       // backup
  myGLCD.fillRoundRect(205, 88, 285, 108);      // сброс
  // рамки
  myGLCD.setColor(255, 255, 255);  // шрифт белый
  myGLCD.drawRoundRect(195, 20, 295, 40);       // detect
  myGLCD.drawRoundRect(205, 54, 285, 74);       // backup
  myGLCD.drawRoundRect(205, 88, 285, 108);      // сброс

  printButtonRUS(print_text[2], backGS[0], backGS[1], backGS[2], backGS[3], SMALL);
  printButtonRUS(print_text[17], nextGS[0], nextGS[1], nextGS[2], nextGS[3], SMALL);
  printButtonRUS(print_text[3], prSAVEgs[0], prSAVEgs[1], prSAVEgs[2], prSAVEgs[3], SMALL);
  printButtonRUS(print_text[1], canCgs[0], canCgs[1], canCgs[2], canCgs[3], SMALL);

  myGLCD.setFont(RusFont6); // 1
  myGLCD.setColor(0, 255, 0);
  myGLCD.setBackColor(0, 0, 0);

  myGLCD.print(print_text[211], 12, 24);   // ПОИСК ДАТЧИКОВ ТЕМПЕР.
  myGLCD.print(print_text[210], 16, 58);   // CОХР. НАСТР. НА КАРТУ
  myGLCD.print(print_text[209], 31, 92);   // СБРОСИТЬ НАСТРОЙКИ
  Menu_Text(133, 36, 127, 6, 2 );          // ПОДСВЕТКА ЭКРАНА
  Menu_Text(134, 17, 165, 6, 2 );          // ТЕМП. ВКЛ. ВЕНТИЛЯТОРА

  myGLCD.setFont(RusFont2);
  myGLCD.setColor(255, 255, 255); // шрифт белый
  myGLCD.setBackColor(0, 0, 255);

  myGLCD.print(print_text[195], 206, 25);    // ПОИСК ДАТЧ.
  myGLCD.print(print_text[194], 225, 59);    // БЕКАП
  myGLCD.print(print_text[193], 224, 93);    // СБРОС
  genSetSelect_1();

  bitClear(GlobalStatus1Byte, 0);
}    // reset bit for Y/N button

/*********************************** ГЛАВНЫЕ НАСТРОЙКИ, СТР.2 ************************ dispScreen = 15 */
void generalSettingsScreen_2() {

  PrintStringIndex = 11; printHeader (); // ГЛАВНЫЕ НАСТРОЙКИ, СТР.2

  myGLCD.setColor(64, 64, 64);     // серый цвет
  myGLCD.drawLine(0, 88, 319, 88);
  myGLCD.drawLine(0, 162, 319, 162);
  myGLCD.drawRect(0, 196, 319, 194); // двойная линия над кнопками (back next)

  printButtonRUS(print_text[2], backGS[0], backGS[1], backGS[2], backGS[3], SMALL);
  printButtonRUS(print_text[17], nextGS[0], nextGS[1], nextGS[2], nextGS[3], SMALL);
  printButtonRUS(print_text[3], prSAVEgs[0], prSAVEgs[1], prSAVEgs[2], prSAVEgs[3], SMALL);
  printButtonRUS(print_text[1], canCgs[0], canCgs[1], canCgs[2], canCgs[3], SMALL);


  Menu_Text(135, 15, 38, 6, 2 ); //    АВТО-УМЕНЬШ. ЯРКОСТИ
  Menu_Text(136, 43, 53, 6, 2 ); //    ПРИ ПЕРЕГРЕВЕ
  Menu_Text(137, 53, 113, 6, 2 ); //    УСТАНОВКИ
  Menu_Text(138, 27, 128, 6, 2 ); //    ХРАНИТЕЛЯ ЭКРАНА
  Menu_Text(139, 18, 173, 6, 2 ); //    ОГРАНИЧЕНИЕ МОЩНОСТИ

  genSetSelect_2();
}

/************************* ГЛАВНЫЕ НАСТРОЙКИ, СТР.3 ******************************* dispScreen = ? */
void generalSettingsScreen_3() {

  PrintStringIndex = 12; printHeader (); // ГЛАВНЫЕ НАСТРОЙКИ, СТР.3

  myGLCD.setColor(64, 64, 64);     // серый цвет
  myGLCD.drawLine(0, 83, 319, 83);
  myGLCD.drawLine(0, 162, 319, 162);
  myGLCD.drawRect(0, 196, 319, 194);

  Menu_Text(90, 15, 38, 6, 2 ); //    ПРИСВОЕНИЕ ЦВЕТА
  Menu_Text(91, 53, 53, 6, 2 ); //        КАНАЛАМ
  Menu_Text(92, 35, 113, 6, 2 ); //     РЕЖИМ РАБОТЫ
  Menu_Text(93, 45, 128, 6, 2 ); //      ОХЛАЖДЕНИЯ


  PrintBSC3b();
  genSetSelect_3();
}

//----------------------- главные настройки стр. 1 -----------------------------
void genSetSelect_1() {

  myGLCD.setColor(0, 0, 255);                // Change Fan Startup Temps Button
  myGLCD.fillRoundRect(195, 159, 295, 179);  // кнопка УСТАН. ТЕМП
  myGLCD.fillRoundRect(205, 123, 285, 143);  // кнопка ЯРК. ЭКРАНА подсветка экрана
  Menu_Text(18, 205, 164, 2, 5 );            // УСТАН.ТЕМП.
  myGLCD.print(print_text[122], 211, 128);   // УСТАНОВ. ( яркость подсветки)
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(195, 159, 295, 179);   // белая рамка вокруг кнопки установ. температуры
  myGLCD.drawRoundRect(205, 123, 285, 143);
} // УСТАНОВ. ( яркость подсветки)

//------------------------- главные настройки стр.2 -----------------------------
void genSetSelect_2() {

  if (setDimLEDsOnOff == 1) {             // Dim LEDs Temp Buttons

    printButton(print_text[212], DledOn[0], DledOn[1], DledOn[2], DledOn[3], SMALL, GREEN_BAC); // green  ON
    printButton(print_text[213], DledOff[0], DledOff[1], DledOff[2], DledOff[3], SMALL);    // blue  OFF
  } else {
    printButton(print_text[212], DledOn[0], DledOn[1], DledOn[2], DledOn[3], SMALL);         // blue  ON
    printButton104(print_text[213], DledOff[0], DledOff[1], DledOff[2], DledOff[3], SMALL);
  } // red  OFF

  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect(185, 55, 305, 75);
  Menu_Text(127, 192, 60, 2, 5 ); // УСТАНОВ. ТЕМП.

  if (setScreensaverOnOff == 1) {     // Хранитель экрана Screensaver

    printButton(print_text[212], SsavOn[0], SsavOn[1], SsavOn[2], SsavOn[3], SMALL, GREEN_BAC); // green ON
    printButton(print_text[213], SsavOff[0], SsavOff[1], SsavOff[2], SsavOff[3], SMALL);  // blue OFF
  } else {
    printButton(print_text[212], SsavOn[0], SsavOn[1], SsavOn[2], SsavOn[3], SMALL);         // blue ON
    printButton104(print_text[213], SsavOff[0], SsavOff[1], SsavOff[2], SsavOff[3], SMALL);
  } // red OFF

  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect(185, 129, 305, 149);
  Menu_Text(21, 210, 134, 2, 5 );   // НАСТРОИТЬ

  if (DimmL == 1) {                       // Dimm  ОГРАНИЧЕНИЕ МОЩНОСТИ (РУЧНОЕ ДИММИРОВАНИЕ)
    printButton(print_text[212], DimmLOn[0], DimmLOn[1], DimmLOn[2], DimmLOn[3], SMALL, GREEN_BAC); // green  ON
    printButton(print_text[213], DimmLOff[0], DimmLOff[1], DimmLOff[2], DimmLOff[3], SMALL);     // blue OFF
  } else {
    printButton(print_text[212], DimmLOn[0], DimmLOn[1], DimmLOn[2], DimmLOn[3], SMALL);         // blue  ON
    printButton104(print_text[213], DimmLOff[0], DimmLOff[1], DimmLOff[2], DimmLOff[3], SMALL);
  } // red OFF

  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(185, 129, 305, 149);
  myGLCD.drawRoundRect(185, 55, 305, 75);
}

//------------------------- главные настройки стр.3 -----------------------------
void genSetSelect_3() {

  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect(185, 35, 305, 55);
  Menu_Text(21, 210, 40, 2, 5 );         // НАСТРОИТЬ

  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect(185, 115, 305, 135);
  Menu_Text(21, 210, 120, 2, 5 );        // НАСТРОИТЬ

  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(185, 35, 305, 55);
  myGLCD.drawRoundRect(185, 115, 305, 135);

}
//*********************** ОБОРОТЫ ВЕНТИЛЯТОРА ОЖЛАЖДЕНИЯ************************************
void coolfanPWM() {

  PrintStringIndex = 7; printHeader (); // РЕЖИМ РАБОТЫ ОХЛАЖДЕНИЯ

  myGLCD.setColor(64, 64, 64);     // серый цвет
  myGLCD.drawLine(0, 100, 319, 100);
  myGLCD.drawLine(0, 180, 319, 180);
  myGLCD.drawRect(0, 196, 319, 194);

  Menu_Text(58, 30, 25, 3, 2 );          //
  Menu_Text(59, 26, 110, 3, 2 );         //
  Menu_Text(60, 23, 45, 3, 2 );          //
  Menu_Text(61, 180, 45, 3, 2 );         //

  CoolFanMIN = PlusMinusCountI (false, false, temM[0] - 50, temM[1] + 15, 95, 0, 100, 1, CoolFanMIN);               //
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }
  CoolFanPWM = PlusMinusCountI (false, false, temM[0] - 50, SoundAlarmTm[1] - 20, 95, 0, 100, 1, CoolFanPWM);       //
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }
  HeatsinkLoPWM = PlusMinusCountI (false, false, temM[0] + 105, temM[1] + 15, 95, 0, 100, 1, HeatsinkLoPWM);        //
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }
  HeatsinkHiPWM = PlusMinusCountI (false, false, temM[0] + 105, SoundAlarmTm[1] - 20, 95, 0, 100, 1, HeatsinkHiPWM); //
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }
  PrintBSC3b();
}

/**************************** УСТАНОВКА ТЕМПЕРАТУРЫ ВКЛ. ВЕНТИЛЯТОРОВ ******************* dispScreen = 16 */
void ChangeFanTempsScreen(boolean refreshAll = false) {
  //  String deg;

  if (refreshAll) {
    if (setTempToBeginHeatsink1FanC == 0) {
      setTempToBeginHeatsink1FanC = 29.0; // change to 29 deg C
    }
    if (setTempToBeginHeatsink2FanC == 0) {
      setTempToBeginHeatsink2FanC = 29.0; // change to 29 deg C
    }
    temp2beHFan = setTempToBeginHeatsink1FanC; temp2beSFan = setTempToBeginHeatsink2FanC;

    PrintStringIndex = 14; printHeader (); // УСТАНОВКА ТЕМПЕРАТУРЫ ВКЛ. ВЕНТИЛЯТОРОВ

    myGLCD.setColor(64, 64, 64);
    myGLCD.drawRect(0, 103, 319, 105);
    myGLCD.setColor(0, 123, 176);        // синий
    myGLCD.drawRect(123, 56, 197, 80);   // рамка вокруг вентелятор охлаждения радиатора-1
    myGLCD.drawRect(123, 147, 197, 171); // рамка вокруг вентелятор охлаждения радиатора-2
    myGLCD.setColor(110, 110, 110);      // цвет серый
    myGLCD.drawRect(59, 34, 260, 100);   // большая рамка вокруг температуры включения 1
    myGLCD.drawRect(59, 125, 260, 191);  // большая рамка вокруг температуры включения 2
    Menu_Text(140, CENTER, 20, 1, 2 );   // ВЕНТИЛЯТОР ОХЛАЖДЕНИЯ РАДИАТОРА-1
    Menu_Text(141, CENTER, 112, 1, 2 );  // ВЕНТИЛЯТОР ОХЛАЖДЕНИЯ РАДИАТОРА-2
    Menu_Text(142, 66, 39, 1, 2 );       // д1 ТЕМПЕРАТУРА ВКЛЮЧЕНИЯ
    Menu_Text(142, 66, 130, 1, 2 );      // д2 ТЕМПЕРАТУРА ВКЛЮЧЕНИЯ

    myGLCD.print(print_text[146], 270, 39);   // Текущ  для датчика радиатора 1
    myGLCD.print(print_text[145], 275, 69);   // Макс  для датчика радиатора 1
    myGLCD.print(print_text[146], 270, 130);  // Текущ  для датчика радиатора 2
    myGLCD.print(print_text[145], 275, 160);  // Макс  для датчика радиатора 2

    setFont(SMALL, 255, 255, 255, 0, 0, 0);
    myGLCD.drawCircle(239, 38, 1);      // радиатор д-1
    myGLCD.print(print_text[57], 244, 36);
    myGLCD.drawCircle(239, 129, 1);     // радиатор д-2
    myGLCD.print(print_text[57], 244, 127);
    Menu_Text(144, CENTER, 86, 1, 2 ); // (25-50)
    Menu_Text(144, CENTER, 177, 1, 2 ); // (25-50)

    printButton(print_text[28], HoodFanTm[0], HoodFanTm[1], HoodFanTm[2], HoodFanTm[3], LARGE);
    printButton(print_text[27], HoodFanTp[0], HoodFanTp[1], HoodFanTp[2], HoodFanTp[3], LARGE);
    printButton(print_text[28], SumpFanTm[0], SumpFanTm[1], SumpFanTm[2], SumpFanTm[3], LARGE);
    printButton(print_text[27], SumpFanTp[0], SumpFanTp[1], SumpFanTp[2], SumpFanTp[3], LARGE);

    printButtonRUS(print_text[152], SalaRm[0], SalaRm[1], SalaRm[2], SalaRm[3], SMALL); // sound alarm

    myGLCD.setColor(64, 64, 64);
    myGLCD.drawRect(0, 196, 319, 194);
    myGLCD.drawRoundRect(268, 49, 311, 65);    // рамка вокруг текущ. температуры радиатора 1
    myGLCD.drawRoundRect(268, 78, 311, 95);    // рамка вокруг max радиатора 1
    myGLCD.drawRoundRect(268, 140, 311, 156);  // рамка вокруг текущ. температуры радиатора 2
    myGLCD.drawRoundRect(268, 169, 311, 186);  // рамка вокруг max радиатора 2

    PrintBSC3b();
  }

  setFont(LARGE, 255, 255, 255, 0, 0, 0);
  myGLCD.printNumF(temp2beHFan, 1, CENTER, 61);  // Fan 1
  myGLCD.printNumF(temp2beSFan, 1, CENTER, 152); // Fan 2

  if (tempH1 == -127 || tempH1 == -196) {  // sensor disconnected
    myGLCD.setColor(0, 0, 0); myGLCD.fillRect(265, 34, 311, 100);
  } else {
    setFont(SMALL, 0, 255, 0, 0, 0, 0);  // зеленый шрифт для текущ.
    myGLCD.printNumF( tempH1, 1, 275, 51);
  }      // отображение текущ. температуры радиатора 1

  if (tempH2 == -127 || tempH2 == -196) {  // sensor disconnected
    myGLCD.setColor(0, 0, 0); myGLCD.fillRect(265, 125, 311, 191);
  } else {
    setFont(SMALL, 0, 255, 0, 0, 0, 0);  // зеленый шрифт для текущ.
    myGLCD.printNumF( tempH2, 1, 275, 142);
  }     // отображение текущ. температуры радиатора 2

  if (MaxTempH1 == 0.0) {  // sensor disconnected
    myGLCD.setColor(0, 0, 0); myGLCD.print(print_text[153], 275, 81); // (два пробела)
  } else {
    setFont(SMALL, 255, 0, 0, 0, 0, 0);  // красный шрифт для макс.
    myGLCD.printNumF( MaxTempH1, 1, 275, 81);
  }  // отображение max температуры радиатора 1

  if (MaxTempH2 == 0.0) { // sensor disconnected
    myGLCD.setColor(0, 0, 0); myGLCD.print(print_text[153], 275, 172); // (два пробела)
  } else {
    setFont(SMALL, 255, 0, 0, 0, 0, 0);    // красный шрифт для макс.
    myGLCD.printNumF( MaxTempH2, 1, 275, 172);
  }
}  // отображение max температуры радиатора 2

void SoundAlarm() {
  myGLCD.setColor(255, 240, 255);
  myGLCD.fillRect(50, 70, 269, 218);
  myGLCD.setColor(0, 0, 255);
  myGLCD.drawRoundRect(50 + 2, 70 + 2, 269 - 2, 218 - 2);
  myGLCD.setColor(255, 0, 0); // red
  myGLCD.drawRoundRect(214, 116, 258, 152);
  myGLCD.drawRoundRect(62, 116, 106, 152);
  Menu_Text(167, CENTER, 85, 1, 6 );        // УСТАНОВКА ЗВУКОВОЙ ТРЕВОГИ (при достижении заданной температуры)
  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRoundRect(117, 104, 203, 165);
  myGLCD.setColor(0, 0, 255);
  myGLCD.drawRoundRect(115, 102, 205, 167);

  printButton(print_text[28], SoundATm[0], SoundATm[1], SoundATm[2], SoundATm[3], LARGE); // звуковая тревога +
  printButton(print_text[27], SoundATp[0], SoundATp[1], SoundATp[2], SoundATp[3], LARGE); // звуковая тревога -

  myGLCD.setColor(0, 255, 0);
  myGLCD.fillRoundRect(95, 185, 135, 205);


  Menu_Text(17, 107, 190, 1, 7 );        // OK
  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect(175, 185, 240, 205);
  setFont(SMALL, 255, 255, 255, 0, 0, 255);
  myGLCD.setFont(RusFont1);
  myGLCD.print(print_text[1], 175 + 9, 185 + 7); // CANCEL

  myGLCD.setColor(0, 0, 0);
  myGLCD.drawRoundRect(95, 185, 135, 205);
  myGLCD.drawRoundRect(175, 185, 240, 205);
  setSalarm();
}

void setSalarm() {
  if (setTempToSoundAlarmC == 255) {
    Menu_Text(26, 122, 125, 0, 3 );                         //  ( OFF alarm )
  } else {
    setFont(LARGE, 255, 255, 255, 0, 0, 0);
    myGLCD.printNumF(setTempToSoundAlarmC, 1, 129, 125);
  }
} // display sound alarm temp

/******** УМЕНЬШЕНИЕ ЯРКОСТИ ПРИ ПЕРЕГРЕВЕ ******************************************************** dispScreen = 17 */
void DimLEDsAtTempScreen() {

  PrintStringIndex = 15; printHeader (); // УМЕНЬШЕНИЕ ЯРКОСТИ ПРИ ПЕРЕГРЕВЕ

  Menu_Text(145,  6, 52, 6, 2 );     // ПРИ НАГРЕВЕ РАДИАТОРА
  Menu_Text(146, 35, 67, 6, 2 );     //    ДО ТЕМПЕРАТУРЫ:
  Menu_Text(147, 21, 131, 6, 2 );    //   УМЕНЬШАТЬ ЯРКОСТЬ
  Menu_Text(114, 33, 145, 6, 2 );    //    CВЕТОДИОДОВ ДО:

  setFont(SMALL, 0, 255, 0, 0, 0, 0);
  myGLCD.drawCircle(273, 61, 1);
  myGLCD.print(print_text[57], 278, 59);
  myGLCD.setFont(SmallFont);  // Выбор шрифта
  myGLCD.print(print_text[142], 273, 140); // % (проценты)
  myGLCD.setColor(64, 64, 64);
  myGLCD.drawRoundRect(176, 31, 265, 96);   // рамка вокруг кнопок регулировки
  myGLCD.drawRoundRect(176, 112, 265, 177);
  drawUpButton(235, 36);
  drawDownButton(235, 66);
  drawUpButton(235, 117);
  drawDownButton(235, 147);

  TempLEDsDimTemp = setLEDsDimTempC;
  TempLEDsDimPercent = setLEDsDimPercent;

  setFont(LARGE, 255, 108, 72, 0, 0, 0);
  if (TempLEDsDimTemp >= 100) {
    myGLCD.printNumI(TempLEDsDimTemp, 181, 55);
  }
  if ((TempLEDsDimTemp <= 99) && (TempLEDsDimTemp >= 10)) {
    myGLCD.printNumI(TempLEDsDimTemp, 189, 55);
  }
  if (TempLEDsDimTemp <= 9) {
    myGLCD.printNumI(TempLEDsDimTemp, 198, 55);
  }

  setFont(LARGE, 255, 255, 255, 0, 0, 0);
  if (TempLEDsDimPercent >= 100) {
    myGLCD.printNumI(TempLEDsDimPercent, 181, 136);
  }
  if ((TempLEDsDimPercent <= 99) && (TempLEDsDimPercent >= 10)) {
    myGLCD.printNumI(TempLEDsDimPercent, 189, 136);
  }
  if (TempLEDsDimPercent <= 9) {
    myGLCD.printNumI(TempLEDsDimPercent, 198, 136);
  }

  myGLCD.setColor(64, 64, 64);
  myGLCD.drawRect(0, 196, 319, 194);
  PrintBSC3b();
}

/***** УСТАНОВКИ ХРАНИТЕЛЯ ЭКРАНА *************************************************** dispScreen = 18 */
void ScreensaverSettingsScreen() {

  PrintStringIndex = 16; printHeader (); // УСТАНОВКИ ХРАНИТЕЛЯ ЭКРАНА

  myGLCD.setColor(64, 64, 64);
  myGLCD.drawLine(0, 45, 319, 45);

  Menu_Text(148, 18, 26, 1, 2 );     // ЧАСЫ / ПУСТОЙ ЭКРАН
  Menu_Text(149, 8, 114, 1, 2 );     // активация
  Menu_Text(150, 23, 125, 1, 2 );    // после
  Menu_Text(132, 174, 119, 1, 2 );   // минут
  myGLCD.setColor(64, 64, 64);
  myGLCD.drawRoundRect(84, 84, 167, 165); // рамка вокруг установок времени скринсейва

  drawUpButton(135, 92);     // кнопка вверх установки времени хранителя экрана
  drawDownButton(135, 132);  // кнопка вниз установки времени хранителя экрана

  myGLCD.setFont(DotMatrix_M_Num);  // Выбор шрифта
  myGLCD.setColor(80, 255, 246);    // цвет голубой
  myGLCD.setBackColor(0, 0, 0);     // цвет фона черный
  TempSSminutes = setSSmintues;
  if (TempSSminutes >= 10) {
    myGLCD.printNumI(TempSSminutes, 87, 112);
  }
  else {
    myGLCD.printNumI(TempSSminutes, 102, 112);
  }

  myGLCD.setColor(64, 64, 64);
  myGLCD.drawRect(0, 196, 319, 194); // горизонтальные линии над кнопкой
  myGLCD.drawRect(0, 176, 319, 174); // горизонтальные линии под A.clock
  PrintBSC3b();
  ScreensaverSelect();
}

/************** СУТОЧНЫЕ ТАЙМЕРЫ ************************************************************** dispScreen = 19 */
void TimerScreen() {

  PrintStringIndex = 17; printHeader (); // СУТОЧНЫЕ ТАЙМЕРЫ

  myGLCD.setColor(0, 0, 255);     // цвет синий
  for (int x = 0; x < 5; x++) {
    myGLCD.fillRoundRect((x * 64) + 7, 22, (x * 64) + 56, 45); // имена таймеров
  }
  myGLCD.setColor(255, 255, 255); // цвет белый
  for (int x = 0; x < 5; x++) {
    myGLCD.drawRoundRect((x * 64) + 7, 22, (x * 64) + 56, 45); // белые рамки вокруг таймеров
  }

  // кнопки ON
  myGLCD.setColor(70, 200, 0);     // цвет зеленый
  for (int x = 0; x < 5; x++) {
    myGLCD.fillRoundRect((x * 64) + 7, 51, (x * 64) + 56, 79); // нарисовать кнопки ON
  }
  myGLCD.setColor(255, 255, 255);  // цвет белый
  for (int x = 0; x < 5; x++) {
    myGLCD.drawRoundRect((x * 64) + 7, 51, (x * 64) + 56, 79); // белые рамки вокруг ON
  }

  // размер рамки времени OFF
  myGLCD.setColor(255, 255, 255);  // цвет белый
  for (int x = 0; x < 5; x++) {
    myGLCD.drawRoundRect((x * 64) + 7, 81, (x * 64) + 56, 113); // рамки вокруг времени ON
  }

  // кнопки OFF
  myGLCD.setColor(255, 0, 0);
  for (int x = 0; x < 5; x++) {
    myGLCD.fillRoundRect((x * 64) + 7, 122, (x * 64) + 56, 150); // нарисовать кнопки OFF
  }
  myGLCD.setColor(255, 255, 255);   // цвет белый
  for (int x = 0; x < 5; x++) {
    myGLCD.drawRoundRect((x * 64) + 7, 122, (x * 64) + 56, 150); // белые рамки вокруг OFF
  }
  myGLCD.setColor(255, 255, 255);   // цвет белый
  for (int x = 0; x < 5; x++) {
    myGLCD.drawRoundRect((x * 64) + 7, 152, (x * 64) + 56, 184); // размер рамки таймера OFF
  }

  // рамка вокруг кнопок таймеров
  myGLCD.setColor(130, 130, 130);  // цвет серый
  for (int x = 0; x < 5; x++) {
    myGLCD.drawRoundRect((x * 64) + 5, 20, (x * 64) + 58, 186);
  }

  setFont(SMALL, 32, 255, 255, 0, 0 , 0);              // цвет шрифта бирюзовый, фон черный

  // показать время включения таймера 1 в окне ON
  myGLCD.printNumI(on1 / 60, 13, 91, 2, '0');          // часы
  myGLCD.print(print_text[56], 29, 90);                // :
  myGLCD.printNumI(on1 - ((on1 / 60) * 60), 37, 91, 2, '0'); // минуты

  // показать время включения таймера 2 в окне ON
  myGLCD.printNumI(on2 / 60, 77, 91, 2, '0');           // часы
  myGLCD.print(print_text[56], 93, 90);                 // :
  myGLCD.printNumI(on2 - ((on2 / 60) * 60), 101, 91, 2, '0'); // минуты

  // показать время включения таймера 3 в окне ON
  myGLCD.printNumI(on3 / 60, 141, 91, 2, '0');          // часы
  myGLCD.print(print_text[56], 157, 90);                // :
  myGLCD.printNumI(on3 - ((on3 / 60) * 60), 165, 91, 2, '0'); // минуты

  // показать время включения таймера 4 в окне ON
  myGLCD.printNumI(on4 / 60, 205, 91, 2, '0');          // часы
  myGLCD.print(print_text[56], 221, 90);                // :
  myGLCD.printNumI(on4 - ((on4 / 60) * 60), 229, 91, 2, '0'); // минуты

  // показать время включения таймера 5 в окне ON
  myGLCD.printNumI(on5 / 60, 269, 91, 2, '0');          // часы
  myGLCD.print(print_text[56], 285, 90);                // :
  myGLCD.printNumI(on5 - ((on5 / 60) * 60), 293, 91, 2, '0'); // минуты

  // показать время выключения таймера 1 в окне OFF
  myGLCD.printNumI(off1 / 60, 13, 162, 2, '0');          // часы
  myGLCD.print(print_text[56], 29, 161);                 // :
  myGLCD.printNumI(off1 - ((off1 / 60) * 60), 37, 162, 2, '0'); // минуты

  // показать время выключения таймера 2 в окне OFF
  myGLCD.printNumI(off2 / 60, 77, 162, 2, '0');           // часы
  myGLCD.print(print_text[56], 93, 161);                  // :
  myGLCD.printNumI(off2 - ((off2 / 60) * 60), 101, 162, 2, '0'); // минуты

  // показать время выключения таймера 3 в окне OFF
  myGLCD.printNumI(off3 / 60, 141, 162, 2, '0');          // часы
  myGLCD.print(print_text[56], 157, 161);                 // :
  myGLCD.printNumI(off3 - ((off3 / 60) * 60), 165, 162, 2, '0'); // минуты

  // показать время выключения таймера 4 в окне OFF
  myGLCD.printNumI(off4 / 60, 205, 162, 2, '0');          // часы
  myGLCD.print(print_text[56], 221, 161);                 // :
  myGLCD.printNumI(off4 - ((off4 / 60) * 60), 229, 162, 2, '0'); // минуты

  // показать время выключения таймера 5 в окне OFF
  myGLCD.printNumI(off5 / 60, 269, 162, 2, '0');          // часы
  myGLCD.print(print_text[56], 285, 161);                 // :
  myGLCD.printNumI(off5 - ((off5 / 60) * 60), 293, 162, 2, '0'); // минуты

  setFont(SMALL, 255, 255, 255, 0, 0 , 255); // цвет шрифта белый, цвет вокруг текста названия таймера синий
  myGLCD.setFont(RusFont1);
#ifdef freshwater
  myGLCD.print(print_text[62], 16, 30);  // 1
  myGLCD.print(print_text[63], 82, 30);  // 2
  myGLCD.print(print_text[64], 136, 30); // 3
  myGLCD.print(print_text[65], 213, 30); // 4
  myGLCD.print(print_text[66], 271 , 30); // 5
#endif
#ifdef seawater
  myGLCD.setFont(BigRusFont);         // выбрать шрифт
  myGLCD.print(print_text[62], 8, 26);   // 1
  myGLCD.print(print_text[63], 72, 26);  // 2
  myGLCD.print(print_text[64], 136, 26); // 3
  myGLCD.print(print_text[65], 200, 26); // 4
  myGLCD.print(print_text[66], 264, 26); // 5
#endif
  myGLCD.setFont(BigRusFont);         // выбрать шрифт
  myGLCD.setBackColor(70, 200, 0); // цвет вокруг текста ON зеленый
  myGLCD.print(print_text[212], 8, 57);   // ON
  myGLCD.print(print_text[212], 72, 57);   // ON
  myGLCD.print(print_text[212], 136, 57);  // ON
  myGLCD.print(print_text[212], 200, 57);  // ON
  myGLCD.print(print_text[212], 264, 57);  // ON

  myGLCD.setBackColor(255, 0, 0);   // цвет вокруг текста OFF красный
  myGLCD.print(print_text[213], 8, 129);    // OFF
  myGLCD.print(print_text[213], 72, 129);   // OFF
  myGLCD.print(print_text[213], 136, 129);  // OFF
  myGLCD.print(print_text[213], 200, 129);  // OFF
  myGLCD.print(print_text[213], 264, 129);  // OFF

  myGLCD.setColor(64, 64, 64);      // цвет серый
  myGLCD.drawRect(0, 195, 319, 193);
  printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
  printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);
}

/********************************УСТАНОВКА ВРЕМЕНИ ТАЙМЕРА 1********************************************* Timer 1 dispScreen = 20 */
void light1set() {
  PrintStringIndex = 18; printHeader (); TimerSetPic(); // УСТАНОВКИ ТАЙМЕРА 1
  printTimernumber(print_text[47]); // номер таймера  (1)
  myGLCD.setColor(0, 255, 255); timer1Change();
}  // цвет бирюзовый

void timer1Change() {
  printFont();
  myGLCD.printNumI(on1 / 60, 27, 100, 2, '0');            // время включения часы
  myGLCD.printNumI(on1 - ((on1 / 60) * 60), 95, 100, 2, '0'); // время включения минуты
  myGLCD.printNumI(off1 / 60, 193, 100, 2, '0');          // время выключения часы
  myGLCD.printNumI(off1 - ((off1 / 60) * 60), 262, 100, 2, '0');
} // время выключения минуты


/************************УСТАНОВКА ВРЕМЕНИ ТАЙМЕРА 2***************************************** Timer 2 dispScreen = 21 */
void light2set() {
  PrintStringIndex = 19; printHeader (); TimerSetPic(); // УСТАНОВКИ ТАЙМЕРА 2
  printTimernumber(print_text[48]);  // номер таймера (2)
  myGLCD.setColor(0, 255, 255);  timer2Change();
} // цвет бирюзовый

void timer2Change() {
  printFont();
  myGLCD.printNumI(on2 / 60, 27, 100, 2, '0');            // время включения часы
  myGLCD.printNumI(on2 - ((on2 / 60) * 60), 95, 100, 2, '0'); // время включения минуты
  myGLCD.printNumI(off2 / 60, 193, 100, 2, '0');          // время выключения часы
  myGLCD.printNumI(off2 - ((off2 / 60) * 60), 262, 100, 2, '0');
} // время выключения минуты

/********************************УСТАНОВКА ВРЕМЕНИ ТАЙМЕРА 3***************************** Timer 3 dispScreen = 22 */
void light3set() {
  PrintStringIndex = 20; printHeader (); TimerSetPic(); // УСТАНОВКИ ТАЙМЕРА 3
  printTimernumber(print_text[49]);                   // номер таймера (3)
  myGLCD.setColor(0, 255, 255);  timer3Change();
}    // цвет бирюзовый

void timer3Change() {
  printFont();
  myGLCD.printNumI(on3 / 60, 27, 100, 2, '0');            // время включения часы
  myGLCD.printNumI(on3 - ((on3 / 60) * 60), 95, 100, 2, '0'); // время включения минуты
  myGLCD.printNumI(off3 / 60, 193, 100, 2, '0');          // время выключения часы
  myGLCD.printNumI(off3 - ((off3 / 60) * 60), 262, 100, 2, '0');
}// время выключения минуты


/*********************************УСТАНОВКА ВРЕМЕНИ ТАЙМЕРА 4******************************* Timer 4 dispScreen = 23 */
void light4set() {
  PrintStringIndex = 21; printHeader (); TimerSetPic(); // УСТАНОВКИ ТАЙМЕРА 4
  printTimernumber(print_text[50]);                   // номер таймера  (4)
  myGLCD.setColor(0, 255, 255);   timer4Change();
}   // цвет бирюзовый

void timer4Change() {
  printFont();
  myGLCD.printNumI(on4 / 60, 27, 100, 2, '0');            // время включения часы
  myGLCD.printNumI(on4 - ((on4 / 60) * 60), 95, 100, 2, '0'); // время включения минуты
  myGLCD.printNumI(off4 / 60, 193, 100, 2, '0');          // время выключения часы
  myGLCD.printNumI(off4 - ((off4 / 60) * 60), 262, 100, 2, '0');
}// время выключения минуты

/*******************************УСТАНОВКА ВРЕМЕНИ ТАЙМЕРА 5*********************************** Timer 5  dispScreen = 24 */
void light5set() {
  PrintStringIndex = 22; printHeader (); TimerSetPic(); // УСТАНОВКИ ТАЙМЕРА 5
  printTimernumber(print_text[51]);                  // номер таймера  (5)
  myGLCD.setColor(0, 255, 255);   timer5Change();
}  // цвет бирюзовый

void timer5Change() {
  printFont();
  myGLCD.printNumI(on5 / 60, 27, 100, 2, '0');            // время включения часы
  myGLCD.printNumI(on5 - ((on5 / 60) * 60), 95, 100, 2, '0'); // время включения минуты
  myGLCD.printNumI(off5 / 60, 193, 100, 2, '0');          // время выключения часы
  myGLCD.printNumI(off5 - ((off5 / 60) * 60), 262, 100, 2, '0');
}// время выключения минуты

// ------------- Срабатывание таймеров
void light() {
# ifdef freshwater
  // 1 - канал таймера АЭРАЦИИ
  if (timer1Status < 2 && prog1 == 0) {
    if (on1 < off1) {
      if (timer >= on1 && timer < off1 && setAutoStop != 1) {
        timer1Status = 1;
      } else {
        timer1Status = 0;
      }
    }
    if (on1 > off1) {
      if (timer < on1 && timer >= off1) {
        timer1Status = 0;
      } else if (setAutoStop != 1) {
        timer1Status = 1;
      } else {
        timer1Status = 0;
      }
    }
  }
  // 2 - канал таймера  CO2
  if (timer2Status < 2 && prog2 == 0) {
    if (on2 < off2) {
      if (timer >= on2 && timer < off2 && co == 1) {
        timer2Status = 1;
      } else {
        timer2Status = 0;
      }
    }
    if (on2 > off2) {
      if (timer < on2 && timer >= off2) {
        timer2Status = 0;
      } else if (co == 1) {
        timer2Status = 1;
      }
    }
  }
  // 3 - канал таймера ФИЛЬТРА
  if (timer3Status < 2 && prog3 == 0) {
    if (on3 < off3) {
      if (timer >= on3 && timer < off3 && setAutoStop != 1) {
        timer3Status = 1;
      } else {
        timer3Status = 0;
      }
    }
    if (on3 > off3) {
      if (timer < on3 && timer >= off3) {
        timer3Status = 0;
      } else if (setAutoStop != 1) {
        timer3Status = 1;
      } else {
        timer3Status = 0;
      }
    }
  }
  // 4 - канал таймера УФ ЛАМПЫ
  if (timer4Status < 2 && prog4 == 0) {
    if (on4 < off4) {
      if (timer >= on4 && timer < off4 && timer3Status == 1) {
        timer4Status = 1;
      } else {
        timer4Status = 0;
      }
    }
    if (on4 > off4) {
      if (timer < on4 && timer >= off4) {
        timer4Status = 0;
      } else {
        timer4Status = 1;
      }
    }
  }
  // 5 - канал таймера ДОЛИВА
  if (timer5Status < 2 && prog5 == 0) {
    if (on5 < off5) {
      if (timer >= on5 && timer < off5) {
        timer5Status = 1;
      } else {
        timer5Status = 0;
      }
    }
    if (on5 > off5) {
      if (timer < on5 && timer >= off5) {
        timer5Status = 0;
      } else {
        timer5Status = 1;
      }
    }
  }
#endif

# ifdef seawater
  // 1 - канал таймера
  if (timer1Status < 2 && prog1 == 0) {
    if (on1 < off1) {
      if (timer >= on1 && timer < off1 && setAutoStop != 1) {
        timer1Status = 1;
      } else {
        timer1Status = 0;
      }
    }
    if (on1 > off1) {
      if (timer < on1 && timer >= off1) {
        timer1Status = 0;
      } else if (setAutoStop != 1) {
        timer1Status = 1;
      } else {
        timer1Status = 0;
      }
    }
  }
  // 2 - канал таймера
  if (timer2Status < 2 && prog2 == 0) {
    if (on2 < off2) {
      if (timer >= on2 && timer < off2) {
        timer2Status = 1;
      } else {
        timer2Status = 0;
      }
    }
    if (on2 > off2) {
      if (timer < on2 && timer >= off2) {
        timer2Status = 0;
      } else {
        timer2Status = 1;
      }
    }
  }
  // 3 - канал таймера
  if (timer3Status < 2 && prog3 == 0) {
    if (on3 < off3) {
      if (timer >= on3 && timer < off3 && setAutoStop != 1) {
        timer3Status = 1;
      } else {
        timer3Status = 0;
      }
    }
    if (on3 > off3) {
      if (timer < on3 && timer >= off3) {
        timer3Status = 0;
      } else if (setAutoStop != 1) {
        timer3Status = 1;
      } else {
        timer3Status = 0;
      }
    }
  }
  // 4 - канал таймера
  if (timer4Status < 2 && prog4 == 0) {
    if (on4 < off4) {
      if (timer >= on4 && timer < off4) {
        timer4Status = 1;
      } else {
        timer4Status = 0;
      }
    }
    if (on4 > off4) {
      if (timer < on4 && timer >= off4) {
        timer4Status = 0;
      } else {
        timer4Status = 1;
      }
    }
  }
  // 5 - канал таймера
  if (timer5Status < 2 && prog5 == 0) {
    if (on5 < off5) {
      if (timer >= on5 && timer < off5) {
        timer5Status = 1;
      } else {
        timer5Status = 0;
      }
    }
    if (on5 > off5) {
      if (timer < on5 && timer >= off5) {
        timer5Status = 0;
      } else {
        timer5Status = 1;
      }
    }
  }
#endif

// Modification des valeurs LOW et HIGH car cela ne marche pas !!

  if (OUT_INVERSE == true) {

    // управление портами  0-auto off, 1-auto on, 2-on, 3-off.
    // Timer 1
    if (timer1Status == 0) {
      digitalWrite(timer1, HIGH); // Auto OFF
    }
    if (timer1Status == 1) {
      digitalWrite(timer1, LOW); // Auto ON
    }
    if (timer1Status == 2) {
      digitalWrite(timer1, LOW); // ON
    }
    if (timer1Status == 3) {
      digitalWrite(timer1, HIGH); // OFF
    }
    // Timer 2
    if (timer2Status == 0) {
      digitalWrite(timer2, HIGH); // Auto OFF
    }
    if (timer2Status == 1) {
      digitalWrite(timer2, LOW); // Auto ON
    }
    if (timer2Status == 2) {
      digitalWrite(timer2, LOW); // ON
    }
    if (timer2Status == 3) {
      digitalWrite(timer2, HIGH); // OFF
    }
    // Timer 3
    if (timer3Status == 0) {
      digitalWrite(timer3, HIGH); // Auto OFF
    }
    if (timer3Status == 1) {
      digitalWrite(timer3, LOW); // Auto ON
    }
    if (timer3Status == 2) {
      digitalWrite(timer3, LOW); // ON
    }
    if (timer3Status == 3) {
      digitalWrite(timer3, HIGH); // OFF
    }
    // Timer 4
    if (timer4Status == 0) {
      digitalWrite(timer4, HIGH); // Auto OFF
    }
    if (timer4Status == 1) {
      digitalWrite(timer4, LOW); // Auto ON
    }
    if (timer4Status == 2) {
      digitalWrite(timer4, LOW); // ON
    }
    if (timer4Status == 3) {
      digitalWrite(timer4, HIGH); // OFF
    }
    // Timer 5
    if (timer5Status == 0) {
      digitalWrite(timer5, HIGH); // Auto OFF
    }
    if (timer5Status == 1) {
      digitalWrite(timer5, LOW); // Auto ON
    }
    if (timer5Status == 2) {
      digitalWrite(timer5, LOW); // ON
    }
    if (timer5Status == 3) {
      digitalWrite(timer5, HIGH); // OFF
    }
  } else {

    // управление портами  0-auto off, 1-auto on, 2-on, 3-off.
    // Timer 1
    if (timer1Status == 0) {
      digitalWrite(timer1, LOW); // Auto OFF
    }
    if (timer1Status == 1) {
      digitalWrite(timer1, HIGH); // Auto ON
    }
    if (timer1Status == 2) {
      digitalWrite(timer1, HIGH); // ON
    }
    if (timer1Status == 3) {
      digitalWrite(timer1, LOW); // OFF
    }
    // Timer 2
    if (timer2Status == 0) {
      digitalWrite(timer2, LOW); // Auto OFF
    }
    if (timer2Status == 1) {
      digitalWrite(timer2, HIGH); // Auto ON
    }
    if (timer2Status == 2) {
      digitalWrite(timer2, HIGH); // ON
    }
    if (timer2Status == 3) {
      digitalWrite(timer2, LOW); // OFF
    }
    // Timer 3
    if (timer3Status == 0) {
      digitalWrite(timer3, LOW); // Auto OFF
    }
    if (timer3Status == 1) {
      digitalWrite(timer3, HIGH); // Auto ON
    }
    if (timer3Status == 2) {
      digitalWrite(timer3, HIGH); // ON
    }
    if (timer3Status == 3) {
      digitalWrite(timer3, LOW); // OFF
    }
    // Timer 4
    if (timer4Status == 0) {
      digitalWrite(timer4, LOW); // Auto OFF
    }
    if (timer4Status == 1) {
      digitalWrite(timer4, HIGH); // Auto ON
    }
    if (timer4Status == 2) {
      digitalWrite(timer4, HIGH); // ON
    }
    if (timer4Status == 3) {
      digitalWrite(timer4, LOW); // OFF
    }
    // Timer 5
    if (timer5Status == 0) {
      digitalWrite(timer5, LOW); // Auto OFF
    }
    if (timer5Status == 1) {
      digitalWrite(timer5, HIGH); // Auto ON
    }
    if (timer5Status == 2) {
      digitalWrite(timer5, HIGH); // ON
    }
    if (timer5Status == 3) {
      digitalWrite(timer5, LOW); // OFF
    }
  }
}

void graphonoff() { /*********** Экран ручного управления таймерами ****************************** dispScreen = 25 */

  printButtonRUS(print_text[0], back[0], back[1], back[2], back[3], SMALL); // << MENU
  printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3]);        // CANCEL

  PrintStringIndex = 23; printHeader (); // РУЧНОЕ УПРАВЛЕНИЕ ТАЙМЕРАМИ

  myGLCD.setColor(64, 64, 64); // цвет серый
  //--------------------------------------------------------------------------------------------------------------
  myGLCD.drawRect(11, 23, 89, 43); // 1 кайма вокруг таймеров
  myGLCD.drawRect(11, 59, 89, 79); // 2
  myGLCD.drawRect(11, 95, 89, 115); // 3
  myGLCD.drawRect(11, 131, 89, 151); // 4
  myGLCD.drawRect(11, 167, 89, 187); // 5

  myGLCD.drawLine(1, 51, 100, 51); // 1  линии между кнопками
  myGLCD.drawLine(1, 87, 100, 87); // 2
  myGLCD.drawLine(1, 123, 100, 123); // 3
  myGLCD.drawLine(1, 159, 100, 159); // 4
  //--------------------------------------------------------------------------------------------------------------------
  myGLCD.drawRect(100, 15, 173, 51); // auto
  myGLCD.drawRect(100, 51, 173, 87);
  myGLCD.drawRect(100, 87, 173, 123);
  myGLCD.drawRect(100, 123, 173, 159);
  myGLCD.drawRect(100, 159, 173, 195);

  myGLCD.drawRect(173, 15, 246, 51); // on
  myGLCD.drawRect(173, 51, 246, 87);
  myGLCD.drawRect(173, 87, 246, 123);
  myGLCD.drawRect(173, 123, 246, 159);
  myGLCD.drawRect(173, 159, 246, 195);

  myGLCD.drawRect(246, 15, 319, 51); // off
  myGLCD.drawRect(246, 51, 319, 87);
  myGLCD.drawRect(246, 87, 319, 123);
  myGLCD.drawRect(246, 123, 319, 159);
  myGLCD.drawRect(246, 159, 319, 195);
  //--------------------------------------------------------------------------------------------------------------------------
  Menu_Text(151, 20, 29, 3, 4 );     // Таймер 1
  Menu_Text(152, 20, 65, 3, 4 );     // Таймер 2
  Menu_Text(153, 20, 102, 3, 4 );    // Таймер 3
  Menu_Text(154, 20, 137, 3, 4 );    // Таймер 4
  Menu_Text(155, 20, 173, 3, 4 );    // Таймер 5

  onoff1(); onoff2(); onoff3(); onoff4(); onoff5();
}

void onoff1() { // графика для Таймера 1
  myGLCD.setFont(BigRusFont);      // font size LARGE
  myGLCD.setBackColor(0, 0, 0);
  if (timer1Status == 0) {
    myGLCD.setColor(147, 115, 255); // AUTO OFF
    OnOffTimer1();
  }

  if (timer1Status == 1) {
    myGLCD.setColor(0, 184, 9); // AUTO ON
    OnOffTimer1();
  }

  if (timer1Status == 2) {
    myGLCD.setColor(0, 255, 0); // ON
    myGLCD.print(print_text[212], 188, 25);        // ON
    myGLCD.drawRect(175, 17, 244, 49); myGLCD.drawRect(177, 19, 242, 47);
    myGLCD.setColor(80, 80, 80);
    myGLCD.print(print_text[213], 259, 25);      // OFF
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156])));
    myGLCD.print(buffer, 105, 25);        // AUTO
    myGLCD.setColor(190, 190, 190);    // серый
    myGLCD.drawRect(248, 17, 317, 49); // OFF
    myGLCD.drawRect(250, 19, 315, 47); // off
    myGLCD.drawRect(102, 17, 171, 49);
  }   // AUTO

  if (timer1Status == 3) {
    myGLCD.setColor(255, 0, 0); // OFF
    myGLCD.print(print_text[213], 259, 25);        // OFF
    myGLCD.drawRect(248, 17, 317, 49); myGLCD.drawRect(250, 19, 315, 47);
    myGLCD.setColor(80, 80, 80);
    myGLCD.print(print_text[212], 188, 25);        // ON
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156])));
    myGLCD.print(buffer, 105, 25);        // AUTO
    myGLCD.setColor(190, 190, 190);     // серый
    myGLCD.drawRect(175, 17, 244, 49);  // ON
    myGLCD.drawRect(177, 19, 242, 47);  // on
    myGLCD.drawRect(102, 17, 171, 49);
  }
}   // AUTO

void onoff2() { // графика для Таймера 2
  myGLCD.setFont(BigRusFont);      // font size LARGE
  myGLCD.setBackColor(0, 0, 0);
  if (timer2Status == 0) {
    myGLCD.setColor(147, 115, 255); // AUTO OFF
    OnOffTimer2();
  }

  if (timer2Status == 1) {
    myGLCD.setColor(0, 184, 9); // AUTO ON
    OnOffTimer2();
  }

  if (timer2Status == 2) {
    myGLCD.setColor(0, 255, 0); // ON
    myGLCD.print(print_text[212], 188, 61);        // ON
    myGLCD.drawRect(175, 53, 244, 85); myGLCD.drawRect(177, 55, 242, 83);
    myGLCD.setColor(80, 80, 80);
    myGLCD.print(print_text[213], 259, 61);       // OFF
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156])));
    myGLCD.print(buffer, 105, 61);       // AUTO
    myGLCD.setColor(190, 190, 190);    // серый
    myGLCD.drawRect(248, 53, 317, 85); // OFF
    myGLCD.drawRect(250, 55, 315, 83); // off
    myGLCD.drawRect(102, 53, 171, 85);
  }   // AUTO

  if (timer2Status == 3) {
    myGLCD.setColor(255, 0, 0);  // OFF
    myGLCD.print(print_text[213], 259, 61);        // OFF
    myGLCD.drawRect(248, 53, 317, 85); myGLCD.drawRect(250, 55, 315, 83);
    myGLCD.setColor(80, 80, 80);
    myGLCD.print(print_text[212], 188, 61);        // ON
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156])));
    myGLCD.print(buffer, 105, 61);        // AUTO
    myGLCD.setColor(190, 190, 190);     // серый
    myGLCD.drawRect(175, 53, 244, 85);  // ON
    myGLCD.drawRect(177, 55, 242, 83);  // on
    myGLCD.drawRect(102, 53, 171, 85);
  }
}   // AUTO

void onoff3() { // графика для Таймера 3
  myGLCD.setFont(BigRusFont);      // font size LARGE
  myGLCD.setBackColor(0, 0, 0);
  if (timer3Status == 0) {
    myGLCD.setColor(147, 115, 255); // AUTO OFF
    OnOffTimer3();
  }

  if (timer3Status == 1) {
    myGLCD.setColor(0, 184, 9); // AUTO ON
    OnOffTimer3();
  }

  if (timer3Status == 2) {
    myGLCD.setColor(0, 255, 0); // ON
    myGLCD.print(print_text[212], 188, 97);        // ON
    myGLCD.drawRect(175, 89, 244, 121); myGLCD.drawRect(177, 91, 242, 119);
    myGLCD.setColor(80, 80, 80);
    myGLCD.print(print_text[213], 259, 97);        // OFF
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156])));
    myGLCD.print(buffer, 105, 97);        // AUTO
    myGLCD.setColor(190, 190, 190);     // серый
    myGLCD.drawRect(248, 89, 317, 121); // OFF
    myGLCD.drawRect(250, 91, 315, 119); // off
    myGLCD.drawRect(102, 89, 171, 121);
  }   // AUTO

  if (timer3Status == 3) {
    myGLCD.setColor(255, 0, 0); // OFF
    myGLCD.print(print_text[213], 259, 97);        // OFF
    myGLCD.drawRect(248, 89, 317, 121); myGLCD.drawRect(250, 91, 315, 119);
    myGLCD.setColor(80, 80, 80);
    myGLCD.print(print_text[212], 188, 97);        // ON
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156])));
    myGLCD.print(buffer, 105, 97);        // AUTO
    myGLCD.setColor(190, 190, 190);     // серый
    myGLCD.drawRect(175, 89, 244, 121); // ON
    myGLCD.drawRect(177, 91, 242, 119); // on
    myGLCD.drawRect(102, 89, 171, 121);
  }
}  // AUTO

void onoff4() { // графика для Таймера 4
  myGLCD.setFont(BigRusFont);      // font size LARGE
  myGLCD.setBackColor(0, 0, 0);
  if (timer4Status == 0) {
    myGLCD.setColor(147, 115, 255); // AUTO OFF
    OnOffTimer4();
  }

  if (timer4Status == 1) {
    myGLCD.setColor(0, 184, 9); // AUTO ON
    OnOffTimer4();
  }

  if (timer4Status == 2) {
    myGLCD.setColor(0, 255, 0); // ON
    myGLCD.print(print_text[212], 188, 134);        // ON
    myGLCD.drawRect(175, 125, 244, 157); myGLCD.drawRect(177, 127, 242, 155);
    myGLCD.setColor(80, 80, 80);
    myGLCD.print(print_text[213], 259, 133);       // OFF
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156])));
    myGLCD.print(buffer, 105, 134);        // AUTO
    myGLCD.setColor(190, 190, 190);      // серый
    myGLCD.drawRect(248, 125, 317, 157); // OFF
    myGLCD.drawRect(250, 127, 315, 155); // off
    myGLCD.drawRect(102, 125, 171, 157);
  }   // AUTO

  if (timer4Status == 3) {
    myGLCD.setColor(255, 0, 0); // OFF
    myGLCD.print(print_text[213], 259, 133);       // OFF
    myGLCD.drawRect(248, 125, 317, 157); myGLCD.drawRect(250, 127, 315, 155);
    myGLCD.setColor(80, 80, 80);
    myGLCD.print(print_text[212], 188, 134);        // ON
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156])));
    myGLCD.print(buffer, 105, 134);        // AUTO
    myGLCD.setColor(190, 190, 190);      // серый
    myGLCD.drawRect(175, 125, 244, 157); // ON
    myGLCD.drawRect(177, 127, 242, 155); // on
    myGLCD.drawRect(102, 125, 171, 157);
  }
}  // AUTO

void onoff5() { // графика для Таймера 5
  myGLCD.setFont(BigRusFont);      // font size LARGE
  myGLCD.setBackColor(0, 0, 0);
  if (timer5Status == 0) {
    myGLCD.setColor(147, 115, 255); // AUTO OFF
    OnOffTimer5();
  }

  if (timer5Status == 1) {
    myGLCD.setColor(0, 184, 9); // AUTO ON
    OnOffTimer5();
  }

  if (timer5Status == 2) {
    myGLCD.setColor(0, 255, 0); // ON
    myGLCD.print(print_text[212], 188, 168);       // ON
    myGLCD.drawRect(175, 161, 244, 193); myGLCD.drawRect(177, 163, 242, 191);
    myGLCD.setColor(80, 80, 80);
    myGLCD.print(print_text[213], 259, 168);       // OFF
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156])));
    myGLCD.print(buffer, 105, 168);        // AUTO
    myGLCD.setColor(190, 190, 190);      // серый
    myGLCD.drawRect(248, 161, 317, 193); // OFF
    myGLCD.drawRect(250, 163, 315, 191); // off
    myGLCD.drawRect(102, 161, 171, 193);
  }   // AUTO

  if (timer5Status == 3) {
    myGLCD.setColor(255, 0, 0); // OFF
    myGLCD.print(print_text[213], 259, 168);       // OFF
    myGLCD.drawRect(248, 161, 317, 193); myGLCD.drawRect(250, 163, 315, 191);
    myGLCD.setColor(80, 80, 80);
    myGLCD.print(print_text[212], 188, 168);       // ON
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[156])));
    myGLCD.print(buffer, 105, 168);        // AUTO
    myGLCD.setColor(190, 190, 190);      // серый
    myGLCD.drawRect(177, 163, 242, 191); // on
    myGLCD.drawRect(175, 161, 244, 193); // ON
    myGLCD.drawRect(102, 161, 171, 193);
  }
}  // AUTO

void lightdraw() {  // отображение соостояний таймеров в главном экране
  if ((dispScreen == 0) && (screenSaverCounter < setScreenSaverTimer)) {

    // Таймер 1 статус в главном экране
    if (timer1Status == 0) {
      Menu_Text(171, 84, 168, 1, 4 ); // Авто Вык
    }
    if (timer1Status == 1) {
      Menu_Text(172, 84, 168, 1, 2 ); // Авто Вкл
    }
    if (timer1Status == 2) {
      Menu_Text(172, 84, 168, 1, 8 ); // Ручной Вкл
    }
    if (timer1Status == 3) {
      Menu_Text(171, 84, 168, 1, 3 ); // Ручной Вык
    }

    // Таймер 2 статус
    if (timer2Status == 0) {
      Menu_Text(171, 84, 180, 1, 4 ); // Авто Вык
    }
    if (timer2Status == 1) {
      Menu_Text(172, 84, 180, 1, 2 ); // Авто Вкл
    }
    if (timer2Status == 2) {
      Menu_Text(172, 84, 180, 1, 8 ); // Ручной Вкл
    }
    if (timer2Status == 3) {
      Menu_Text(171, 84, 180, 1, 3 ); // Ручной Вык
    }

    // Таймер 3 статус
    if (timer3Status == 0) {
      Menu_Text(171, 84, 191, 1, 4 ); // Авто Вык
    }
    if (timer3Status == 1) {
      Menu_Text(172, 84, 191, 1, 2 ); // Авто Вкл
    }
    if (timer3Status == 2) {
      Menu_Text(172, 84, 191, 1, 8 ); // Ручной Вкл
    }
    if (timer3Status == 3) {
      Menu_Text(171, 84, 191, 1, 3 ); // Ручной Вык
    }

    // Таймер 4 статус
    if (timer4Status == 0) {
      Menu_Text(171, 84, 202, 1, 4 ); // Авто Вык
    }
    if (timer4Status == 1) {
      Menu_Text(172, 84, 202, 1, 2 ); // Авто Вкл
    }
    if (timer4Status == 2) {
      Menu_Text(172, 84, 202, 1, 8 ); // Ручной Вкл
    }
    if (timer4Status == 3) {
      Menu_Text(171, 84, 202, 1, 3 ); // Ручной Вык
    }

    // Таймер 5 статус
    if (timer5Status == 0) {
      Menu_Text(171, 84, 213, 1, 4 ); // Авто Вык
    }
    if (timer5Status == 1) {
      Menu_Text(172, 84, 213, 1, 2 ); // Авто Вкл
    }
    if (timer5Status == 2) {
      Menu_Text(172, 84, 213, 1, 8 ); // Ручной Вкл
    }
    if (timer5Status == 3) {
      Menu_Text(171, 84, 213, 1, 3 ); // Ручной Вык
    }
  }
}

void tempgScreen() { // ***************** график температуры ****************************************** dispScreen = 26

  printButtonRUS(print_text[2], backGS[0], backGS[1], backGS[2], backGS[3], SMALL);
  printButtonRUS(print_text[1], canCgs[0], canCgs[1], canCgs[2], canCgs[3], SMALL);

  int x, y, z;

  // myGLCD.setFont(SmallFont);
  myGLCD.setFont(RusFont1);
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(0, 0, 0);
  //setFont(RUS1, 255, 255, 255, 0, 0, 0);
  myGLCD.print(print_text[69], 6, 171); // 22
  myGLCD.print(print_text[70], 6, 156); // 23
  myGLCD.print(print_text[71], 6, 140); // 24
  myGLCD.print(print_text[72], 6, 124); // 25
  myGLCD.print(print_text[73], 6, 108); // 26
  myGLCD.print(print_text[74], 6, 92);  // 27
  myGLCD.print(print_text[75], 6, 76);  // 28
  myGLCD.print(print_text[76], 6, 60);  // 29
  myGLCD.print(print_text[39], 6, 44);  // 30
  myGLCD.print(print_text[78], 6, 28);  // 31

  // myGLCD.setFont(RusFont1);
  myGLCD.setColor(200, 200, 200);
  myGLCD.print(print_text[187], 28, 183);   // 0
  myGLCD.print(print_text[48], 50, 183);    // 2
  myGLCD.print(print_text[50], 72, 183);    // 4
  myGLCD.print(print_text[79], 94, 183);    // 6
  myGLCD.print(print_text[80], 116, 183);   // 8
  myGLCD.print(print_text[37], 138 - 5, 183); // 10
  myGLCD.print(print_text[81], 160 - 4, 183); // 12
  myGLCD.print(print_text[82], 182 - 5, 183); // 14
  myGLCD.print(print_text[83], 204 - 5, 183); // 16
  myGLCD.print(print_text[84], 226 - 5, 183); // 18
  myGLCD.print(print_text[38], 248 - 5, 183); // 20
  myGLCD.print(print_text[69], 270 - 5, 183); // 22
  myGLCD.print(print_text[71], 292 - 5, 183); // 24

  myGLCD.setFont(SmallFont);
  myGLCD.setColor(255, 104, 255);
  myGLCD.drawCircle(8, 13, 1);    // значек градуса
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[16])));
  myGLCD.print(buffer, 13, 10);  //  C (цельсий)

  myGLCD.setColor(64, 64, 64);    // серый цвет
  myGLCD.drawRoundRect(0, 0, 319, 225);  // нарисовать рамку
  myGLCD.setFont(RusFont1);
  myGLCD.setColor(88, 255, 238);  // цвет бирюзовый
  myGLCD.setBackColor(0, 0, 0);
  //setFont(RUS1, 88, 255, 238, 0, 0, 0);
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[157])));
  myGLCD.print(buffer, 70, 6);   // ТЕМПЕРАТУРА ВОДЫ ЗА СУТКИ

  //myGLCD.setColor(200, 200, 200);
  for (int k = 31; k < 191; k += 16) {
    myGLCD.drawLine(30, k, 24, k); // шкала напротив температуры
  }
  for (int k = 39; k < 181; k += 16) {
    myGLCD.drawLine(30, k, 27, k); // доп. шкала напротив температуры
  }

  for (int L = 30; L < 306; L += 22) {
    myGLCD.drawLine(L, 175, L, 181); // шкала (горизонт.) x-основная
  }
  for (int L = 41; L < 306; L += 22) {
    myGLCD.drawLine(L, 175, L, 178); // шкала (горизонт.) x-доп.
  }

  myGLCD.setColor(200, 200, 200);
  myGLCD.drawLine(29, 27, 29, 176);    // y вертикальная линия шкалы
  //                     ^ - по вертикали
  myGLCD.drawLine(30, 175, 310, 175);  // x горизонтальная линия шкалы

  myGLCD.setColor(64, 64, 64);        // цвет линии серый
  for (int k = 31; k < 175; k += 16) {
    myGLCD.drawLine(30, k, 310, k);
  }

  // myGLCD.drawRect(64, 3, 265, 16);    // рамка вокруг надписи лог температуры

  // Y вертикальные линии (сетка) в меню график температуры
  for (int L = 30; L < 306; L += 11) {
    myGLCD.drawLine(L, 31, L, 175); // вертикальные линии (сетка)
  }

  linhaR = setTempC;          // Линии сравнения (номинальная темппература R)
  linhaG = (setTempC + offTempC);
  linhaB = (setTempC - offTempC);

  if ((linhaR > 22) && (linhaR <= 31))x = (175 - ((linhaR - 22) * 16)); // номинальная температура
  else if (linhaR > 31) x = 15;
  else if (linhaR < 22) x = 175;

  if ((linhaG > 22) && (linhaG <= 31))y = (175 - ((linhaG - 22) * 16));
  else if (linhaG > 31) y = 15;
  else if (linhaG < 22) y = 175;

  if ((linhaB > 22) && (linhaB <= 31))z = (175 - ((linhaB - 22) * 16));
  else if (linhaB > 31) z = 15;
  else if (linhaB < 22) z = 175;

  myGLCD.setColor(255, 0, 0);
  myGLCD.drawLine(31, x, 310, x);  // Желаемая Температура (красная линия)
  myGLCD.setColor(10, 10, 255);    // Цвет синий
  myGLCD.drawLine(31, y, 310, y);  // верхняя линия
  myGLCD.drawLine(31, z, 310, z);  // нижняя линия

  // лог температуры из памяти
  myGLCD.setColor(255, 255, 0);
  for (int i = 0; i < 46; i++) {
    int stl = 5.5;
    int Ste = ((i * 5.5) + 30);
    float tLinS, tLinS1;
    float tLinE, tLinE1;
    tLinS = media[i]; if (tLinS > 310) {
      tLinS = 310;
    } if (tLinS < 220) {
      tLinS = 220;
    }
    tLinE = media[i + 1]; if (tLinE > 310) {
      tLinE = 310;
    } if (tLinE < 220) {
      tLinE = 220;
    }
    tLinS = map (tLinS, 220, 310, 175, 30);
    tLinE = map (tLinE, 220, 310, 175, 30);
    myGLCD.drawLine(Ste, tLinS, Ste + stl, tLinE );

    tLinS1 = media[46]; if (tLinS1 > 310) {
      tLinS1 = 310;
    } if (tLinS1 < 220) {
      tLinS1 = 220;
    }
    tLinE1 = media[0]; if (tLinE1 > 310) {
      tLinE1 = 310;
    } if (tLinE1 < 220) {
      tLinE1 = 220;
    }
    tLinS1 = map (tLinS1, 220, 310, 175, 30);
    tLinE1 = map (tLinE1, 220, 310, 175, 30);
    myGLCD.drawLine(283, tLinS1, 294, tLinE1 );
  }
}

// Вертикальная шкала времени
void timedrawScreen() {
  myGLCD.setFont(SmallFont);
  float x1 = (RTC.hour * 11 + 30.0); //+RTC.minute
  myGLCD.setColor(200, 200, 200);   // вертикальная линия времени
  myGLCD.setBackColor(0, 0, 0);    // цвет фона черный
  myGLCD.drawLine(x1, 177, x1, 22); // размер стрелки
  myGLCD.setColor(0, 240, 0);
  // if (RTC.hour<10){myGLCD.print("0",x1-40,16); myGLCD.printNumI(RTC.hour,x1-32,16);} // часы
  if (RTC.hour < 10) {
    myGLCD.print(print_text[187], x1 - 40, 16);  // часы
    myGLCD.printNumI(RTC.hour, x1 - 32, 16);
  }
  else {
    myGLCD.printNumI(RTC.hour, x1 - 40, 16);
  }

  myGLCD.print(print_text[56], x1 - 24, 16); // : двоеточие
  if (RTC.minute < 10) {
    myGLCD.print(print_text[187], x1 - 16, 16);  // минуты
    myGLCD.printNumI(RTC.minute, x1 - 8, 16);
  }
  else {
    myGLCD.printNumI(RTC.minute, x1 - 16, 16);
  } //}
  myGLCD.setFont(RusFont6);
  myGLCD.setColor(0, 240, 0);   // Text rotation
}
void tFANScreen() { // график температуры радиаторов датчики 1,2 *************************** dispScreen = 27

  printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
  printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);

  graph_colorFAN();
}

void graph_colorFAN() {
  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRect(1, 1, 318, 193);     // clear

  printButton100("", Fg1[0], Fg1[1], Fg1[2], Fg1[3], SMALL);   // ДАТЧ1
  printButton100("", Fg2[0], Fg2[1], Fg2[2], Fg2[3], SMALL);   // ДАТЧ2

  myGLCD.setFont(RusFont6);
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(100, 100, 100);
  myGLCD.print(print_text[87], 117, 203);  // ДАТЧ1
  myGLCD.setBackColor(100, 100, 100);
  myGLCD.print(print_text[88], 168, 203);  // ДАТЧ2

  myGLCD.setColor(90, 90, 90);        // цвет линии серый (сетка)
  for (int k = 31; k < 175; k += 8) {
    myGLCD.drawLine(30, k, 305, k); // горизонтальные линии (сетка)
  }
  for (int L = 30; L < 306; L += 11) {
    myGLCD.drawLine(L, 31, L, 175); // вертикальные линии (сетка)
  }

  myGLCD.setColor(255, 255, 255);
  myGLCD.drawLine(29, 27, 29, 176);   // y вертикальная линия шкалы
  myGLCD.drawLine(30, 175, 310, 175); // x горизонтальная линия шкалы

  for (byte i = 0; i < 3; i++) {
    //   int FAN = i;
    if ((i == 0) && (F1 == true)) {
      for (byte i = 0; i < 47; i++) tFA[i] = tF1[i]; sbR = 255; sbG = 0; sbB = 0;
      printButton104("", Fg1[0], Fg1[1], Fg1[2], Fg1[3], SMALL);
      myGLCD.setFont(RusFont6);
      myGLCD.setColor(255, 255, 255);
      myGLCD.setBackColor(255, 0, 0);   // серый
      myGLCD.print(print_text[87], 117, 203);
    } // ДАТЧ1

    if ((i == 2) && (F2 == true)) {
      for (byte i = 0; i < 47; i++) tFA[i] = tF2[i]; sbR = 180; sbG = 180; sbB = 0;
      printButton("", Fg2[0], Fg2[1], Fg2[2], Fg2[3], SMALL, GREEN_BAC);
      myGLCD.setFont(RusFont6);
      myGLCD.setColor(255, 255, 255);
      myGLCD.setBackColor(0, 180, 86);   // серый
      myGLCD.print(print_text[88], 168, 203);
    } // ДАТЧ2

    if ((F1 == false) && (F2 == false)) {
      for (byte i = 0; i < 47; i++) tFA[i] = 0;
    }

    myGLCD.setColor(sbR, sbG, sbB);
    for (int i = 0; i < 46; i++) {           // построение графиков
      float stl = 5.5;
      float Ste = ((i * 5.5) + 30);
      float tLinS, tLinS1;
      float tLinE, tLinE1;
      tLinS = tFA[i]; if (tLinS > 420) {
        tLinS = 420;
      } if (tLinS < 240) {
        tLinS = 240;
      }
      tLinE = tFA[i + 1]; if (tLinE > 420) {
        tLinE = 420;
      } if (tLinE < 240) {
        tLinE = 240;
      }
      tLinS = map (tLinS, 240, 420, 175, 30);
      tLinE = map (tLinE, 240, 420, 175, 30);
      myGLCD.drawLine(Ste, tLinS, Ste + stl, tLinE );

      tLinS1 = tFA[46]; if (tLinS1 > 420) {
        tLinS1 = 420;
      } if (tLinS1 < 240) {
        tLinS1 = 240;
      } tLinS1 = map(tLinS1, 240, 420, 175, 30);
      tLinE1 = tFA[0]; if (tLinE1 > 420) {
        tLinE1 = 420;
      } if (tLinE1 < 240) {
        tLinE1 = 240;
      } tLinE1 = map(tLinE1, 240, 420, 175, 30);
      myGLCD.drawLine(283, tLinS1, 294, tLinE1 );
    }

    //   шкала температуры
    myGLCD.setFont(RusFont1);
    myGLCD.setColor(255, 255, 255);
    myGLCD.setBackColor(0, 0, 0);
    //setFont(RUS1, 255, 255, 255, 0, 0, 0);
    myGLCD.print(F("42"), 7, 28);          // 42
    myGLCD.print(print_text[40], 7, 44);   // 40
    myGLCD.print(F("38"), 7, 60);          // 38
    myGLCD.print(F("36"), 7, 76);          // 36
    myGLCD.print(F("34"), 7, 92);          // 34
    myGLCD.print(F("32"), 7, 108);         // 32
    myGLCD.print(print_text[39], 7, 124);  // 30
    myGLCD.print(print_text[75], 7, 140);  // 28
    myGLCD.print(print_text[73] , 7, 156); // 26
    myGLCD.print(print_text[71] , 7, 171); // 24

    //   шкала времени
    myGLCD.setFont(RusFont1);
    myGLCD.setColor(255, 255, 255);
    myGLCD.print(print_text[187], 28, 183);   // 0
    myGLCD.print(print_text[48], 50, 183);    // 2
    myGLCD.print(print_text[50], 72, 183);    // 4
    myGLCD.print(print_text[79], 94, 183);    // 6
    myGLCD.print(print_text[80], 116, 183);   // 8
    myGLCD.print(print_text[37], 138 - 5, 183); // 10
    myGLCD.print(print_text[81], 160 - 4, 183); // 12
    myGLCD.print(print_text[82], 182 - 5, 183); // 14
    myGLCD.print(print_text[83], 204 - 5, 183); // 16
    myGLCD.print(print_text[84], 226 - 5, 183); // 18
    myGLCD.print(print_text[38], 248 - 5, 183); // 20
    myGLCD.print(print_text[69], 270 - 5, 183); // 22
    myGLCD.print(print_text[71] , 292 - 5, 183); // 24

    myGLCD.setFont(SmallFont);
    myGLCD.setColor(255, 104, 255);
    myGLCD.drawCircle(8, 13, 1);   // значек градуса
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[16])));
    myGLCD.print(buffer, 13, 10); // C значек цельсия

    myGLCD.setColor(64, 64, 64);   // серый цвет
    myGLCD.drawRoundRect(0, 1, 319, 225);  // нарисовать рамку

    myGLCD.setFont(RusFont1);
    myGLCD.setColor(88, 255, 238);   // цвет бирюзовый
    myGLCD.setBackColor(0, 0, 0);
    //setFont(RUS1, 88, 255, 238, 0, 0, 0);
    myGLCD.print(print_text[151], CENTER, 6);   // температура радиаторов за сутки

    myGLCD.setColor(255, 255, 255);
    for (int k = 31; k < 191; k += 16) {
      myGLCD.drawLine(30, k, 24, k); // шкала (трип) напротив температуры
    }
    for (int k = 39; k < 181; k += 16) {
      myGLCD.drawLine(30, k, 27, k); // доп. шкала напротив температуры
    }

    for (int L = 30; L < 306; L += 22) {
      myGLCD.drawLine(L, 175, L, 181); // шкала (трип)x-основная
    }
    for (int L = 41; L < 306; L += 22) {
      myGLCD.drawLine(L, 175, L, 178); // шкала (трип)x-доп.
    }
  }
}

/******************* Графики освещенности ********************************************************** dispScreen = 27 */
void AllColourGraph() {  // used in "rapid test" and "All color graph "

  myGLCD.setColor(64, 64, 64);                          // Draw Dividers in Grey
  myGLCD.drawRect(0, 196, 319, 194);                    // Bottom Horizontal Divider
  printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);  // << BACK
  printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);  // CANCEL

  calculateStartTime();  // calculate SUNRISE time
  calculateStopTime();   // calculate SUNSET time

  if (dispScreen == 29) {
    printButton(print_text[96], ledChV[0], ledChV[1], ledChV[2], ledChV[3], SMALL); // button start/stop test
  }
  graph_color();
}

void graph_color() {     //  repeat 9 time

  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRect(1, 0, 318, 193);  // Очистить экран
  printButton100(print_text[20], Wg[0], Wg[1], Wg[2], Wg[3], SMALL);       // white
  printButton100(print_text[21], Bg[0], Bg[1], Bg[2], Bg[3], SMALL);       // blue
  printButton100(print_text[22], RBg[0], RBg[1], RBg[2], RBg[3], SMALL);   // rblue
  printButton100(print_text[23], Rg[0], Rg[1], Rg[2], Rg[3], SMALL);       // R
  printButton100(print_text[24], UVg[0], UVg[1], UVg[2], UVg[3], SMALL);   // UV
  printButton100(print_text[25], ORg[0], ORg[1], ORg[2], ORg[3], SMALL);   // Or
  printButton100(print_text[26], GRg[0], GRg[1], GRg[2], GRg[3], SMALL);   // Gr

  if (LedShannelStatusByte == 0) {
    StartTime = 0;  // draw graph if all led is OFF
    StopTime = 95;
  }
  drawwtledStaticChartP1();

  for (byte i = 1; i <= 8; i++) {
    COLOR = i;

    if ((i == 1) && (W == true)) {
      for (byte i = 0; i < 96; i++)tled[i] = wwtled[i]; sbR = rgbCh0[0]; sbG = rgbCh0[1]; sbB = rgbCh0[2];
      printButton(print_text[20], Wg[0], Wg[1], Wg[2], Wg[3], SMALL);
    }

    if ((i == 2) && (B == true)) {
      for (byte i = 0; i < 96; i++)tled[i] = swtled[i]; sbR = rgbCh1[0]; sbG = rgbCh1[1]; sbB = rgbCh1[2];
      printButton(print_text[21], Bg[0], Bg[1], Bg[2], Bg[3], SMALL);
    }

    if ((i == 3) && (RB == true)) {
      for (byte i = 0; i < 96; i++)tled[i] = rswtled[i]; sbR = rgbCh2[0]; sbG = rgbCh2[1]; sbB = rgbCh2[2];
      printButton(print_text[22], RBg[0], RBg[1], RBg[2], RBg[3], SMALL);
    }        // rblue

    if ((i == 4) && (R == true)) {
      for (byte i = 0; i < 96; i++)tled[i] = rled[i]; sbR = rgbCh3[0]; sbG = rgbCh3[1]; sbB = rgbCh3[2];
      printButton(print_text[23], Rg[0], Rg[1], Rg[2], Rg[3], SMALL);
    }          // red

    if ((i == 5) && (UV == true)) {
      for (byte i = 0; i < 96; i++)tled[i] = uvled[i]; sbR = rgbCh4[0]; sbG = rgbCh4[1]; sbB = rgbCh4[2];
      printButton(print_text[24], UVg[0], UVg[1], UVg[2], UVg[3], SMALL);
    }

    if ((i == 6) && (OR == true)) {
      for (byte i = 0; i < 96; i++)tled[i] = oLed[i]; sbR = rgbCh5[0]; sbG = rgbCh5[1]; sbB = rgbCh5[2];
      printButton(print_text[25], ORg[0], ORg[1], ORg[2], ORg[3], SMALL);
    }

    if ((i == 7) && (SU == true)) {
      for (byte i = 0; i < 96; i++)tled[i] = gled[i]; sbR = rgbCh6[0]; sbG = rgbCh6[1]; sbB = rgbCh6[2];
      printButton(print_text[26], GRg[0], GRg[1], GRg[2], GRg[3], SMALL);
    }  // зеленая кнопка

    if (i != 8) {
      ReadOnOffLedStatus();
      if (LedShannelFlag_on_off == true) {
        draw_one_led_graph();
      }
      else {
        EndScale = 290; // for draw tick scale if all led is OFF
      }
    }
  } drawwtledStaticChartP2();
}

void draw_one_led_graph() {
  byte stepScale = (294 - 26) / (StopTime - StartTime);	 // 26 and 294 - left / right
  myGLCD.setColor(sbR, sbG, sbB);
  for (byte i = StartTime; i < StopTime; i++) {		 // draw led value graph (curve)
    int tempLineS;				          // start segment of line
    int tempLineE;					  // stop segment of line
    int XStep = map((i - StartTime), 0, (StopTime - StartTime), 26, 294);
    tempLineS = tled[i];

    if (tempLineS > 100) {
      tempLineS = 100; // 255
    }
    tempLineE = tled[i + 1];
    if (tempLineE > 100) {
      tempLineE = 100; // 255
    }
    tempLineS = map (tempLineS, 0, 100, BotSldY, TopSldY);  // mapping 0-100 value to chart size 0, 255
    tempLineE = map (tempLineE, 0, 100, BotSldY, TopSldY);

    myGLCD.drawLine(XStep, tempLineS, XStep + stepScale, tempLineE );
    myGLCD.drawLine(XStep, tempLineS + 1, XStep + stepScale, tempLineE + 1 );
    EndScale = XStep + stepScale;
  }
}

void drawwtledStaticChartP1() {  // draw TOP ON/OFF time and X/Y border

  TopSldY = 20; BotSldY = 170;				// graph vertical size - график - размер по вертикали
  LightDay = (StopTime - StartTime) / 4 * 10;		// light day in HOUR * 10 - свет дня в часах*10
  int Temp = 10 * (StopTime - StartTime) / 4;		// rounding to the nearest whole number if "light day" similar 3.4 or 3.75 etc...округление до целого числа
  if ((Temp - LightDay) >= 5) {
    LightDay = (StopTime - StartTime) / 4 + 1;
  }
  else {
    LightDay = (StopTime - StartTime) / 4;
  }

  myGLCD.setColor(255, 255, 255);		// цвет шкалы графиков
  setFont(SMALL, 255, 255, 255, 0, 0, 0);       // шрифт надписей на шкалах графиков
  myGLCD.drawRect(26, BotSldY, 27, TopSldY);	// print y-line
  for (byte i = 1; i < 11; i++) {
    myGLCD.drawLine(28, (i * (BotSldY - TopSldY) / 10 + 5), 30, (i * (BotSldY - TopSldY) / 10 + 5));
  } // Y tick-marks

  myGLCD.drawRect(26, BotSldY, 307, BotSldY + 1);	 // print x-line (горизонтальная линия шкалы)

  for (byte i = 0; i <= 10; i++) { // вертикальная шкала процентов 0-100 (10 делений)
    // myGLCD.setFont(RusFont1);
    myGLCD.setColor(192, 236, 255); // голубой
    if (i == 0) {
      myGLCD.printNumI(0, 18, ((BotSldY - TopSldY) / 10 * (11 - i)));
    }
    if (i == 10) {
      myGLCD.printNumI(100, 2, (BotSldY - TopSldY) / 10 * (11 - i));
    }
    else {
      myGLCD.printNumI(i * 10, 10, (BotSldY - TopSldY) / 10 * (11 - i));
    }
  }
}

void drawwtledStaticChartP2() {        // draw BOT ON/OFF time and X/Y scale and tick mark

  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRect(26, BotSldY, 307, BotSldY + 1);	 // print x-line

  if (LightDay <= 7) {					 // print X-tick and dot background
    for (byte i = 0; i < (StopTime - StartTime) + 2; i++) {	 // i-horisontal count, X tick-marks with 15min resolution
      int XStep = map(i, 0, (StopTime - StartTime), 26, EndScale);
      myGLCD.setColor(255, 255, 255);
      myGLCD.drawLine(XStep, BotSldY + 3, XStep, BotSldY - 3); // k - vertical count

      for (byte k = 1; k <= 10; k++) {                            // k - vertical count
        myGLCD.setColor(180, 180, 180);
        myGLCD.drawPixel(XStep, k * (BotSldY - TopSldY) / 10 + 5);
      }
    }
  }

  if (LightDay > 7 && LightDay <= 12) {
    for (byte i = 0; i < (StopTime - StartTime) / 2 + 2; i++) {	 // X tick-marks with 30min resolution
      int XStep = map(i, 0, (StopTime - StartTime) / 2, 26, EndScale);
      myGLCD.setColor(255, 255, 255);
      myGLCD.drawLine(XStep, BotSldY + 3, XStep, BotSldY - 3); // X mark on horisontal scale

      for (byte k = 1; k <= 10; k++) {                         // k - vertical count
        myGLCD.setColor(180, 180, 180);
        myGLCD.drawPixel(XStep, k * (BotSldY - TopSldY) / 10 + 5);
      }
    }
  }

  if (LightDay > 12 && LightDay <= 24) {
    for (byte i = 0; i < (StopTime - StartTime) / 4 + 4; i++) {  // X tick-marks with 1hour resolution
      int XStep = map(i, 0, (StopTime - StartTime) / 4, 26, EndScale);
      myGLCD.setColor(255, 255, 255);
      myGLCD.drawLine(XStep, BotSldY + 3, XStep, BotSldY - 3);
      myGLCD.setColor(255, 255, 255);

      for (byte k = 1; k <= 10; k++) {                        // k - vertical count
        myGLCD.setColor(180, 180, 180);
        myGLCD.drawPixel(XStep, k * (BotSldY - TopSldY) / 10 + 5);
      }
    }
  } // пунктир

  myGLCD.setFont(RusFont1);
  myGLCD.setColor(0, 255, 0);
  myGLCD.setBackColor(0, 0, 0);
  myGLCD.printNumI(StartTime / 4, 8, BotSldY + 8, 2, '00'); // print start time on scale
  myGLCD.print(print_text[56], 24, BotSldY + 8); // :
  myGLCD.printNumI((StartTime * 15) % 60, 32, BotSldY + 8, 2, '00');

  myGLCD.setFont(RusFont1);
  myGLCD.setColor(255, 0, 0);
  myGLCD.setBackColor(0, 0, 0);
  myGLCD.printNumI(StopTime / 4, EndScale - 16, BotSldY + 8); // print stop time on scale
  myGLCD.print(print_text[56], EndScale - 1, BotSldY + 7); // :
  myGLCD.printNumI((StopTime * 15) % 60, EndScale + 8, BotSldY + 8, 2, '00');

  myGLCD.setFont(RusFont1);
  myGLCD.setColor(190, 190, 190); // серый шрифт
  myGLCD.setBackColor(0, 0, 0);
  myGLCD.print(print_text[143], 2, 186);    // рассвет
  myGLCD.print(print_text[144], 277, 186);  // закат

  myGLCD.setColor(255, 255, 255); // белый шрифт
  myGLCD.print(print_text[81], 86, 183);    // 12
  myGLCD.print(print_text[82], 131, 183);   // 14
  myGLCD.print(print_text[83], 176, 183);   // 16
  myGLCD.print(print_text[84], 220, 183);
} // 18

// *************** Авто-поиск датчиков температуры ************************************ dispScreen
void DetectDallalsSensors(boolean refreshSensor = true) {

  PrintStringIndex = 24; printHeader (); // АВТО-ОПРЕДЕЛЕНИЕ ДАТЧИКОВ

  if (refreshSensor == true) {
    myGLCD.setColor(64, 64, 64);              // Draw Dividers in Grey
    myGLCD.drawLine(0, 93, 319, 93);
    myGLCD.drawRect(0, 196, 319, 194);        // Bottom Horizontal Divider
    myGLCD.setColor(0, 0, 255);
    myGLCD.fillRoundRect(165, 19, 295, 41);  // кнопка ПОИСК ДАТЧИКОВ  (185, 20, 275, 40);
    myGLCD.setColor(255, 255, 255);
    myGLCD.drawRoundRect(165, 19, 295, 41);

    myGLCD.setFont(RusFont2);
    myGLCD.setColor(255, 255, 255);    // шрифт белый
    myGLCD.setBackColor(0, 0, 255);
    //setFont(RUS2, 255, 255, 255, 0, 0, 255);
    myGLCD.print(print_text[204], 175, 25); // ПОИСК ДАТЧ.

    PrintBSC3b();

    myGLCD.setFont(RusFont1);
    myGLCD.setColor(0, 255, 0);      // шрифт зеленый
    myGLCD.setBackColor(0, 0, 0);
    //setFont(RUS1, 0, 255, 0, 0, 0, 0);
    myGLCD.print(print_text[208], 5, 26);     // Найдено:
    myGLCD.print(print_text[207], 90, 26);
  }  // Датчиков

  if (refreshSensor == false) {
    myGLCD.setColor(0, 0, 0);	       // clear button and text area after pressing refresh
    myGLCD.fillRoundRect(4, 95, 318, 190);
    myGLCD.fillRoundRect(4, 45, 318, 92);
  }

  sensors.begin();			             // start sensors
  numberOfDevices = sensors.getDeviceCount();          // find devices
  setFont(SMALL, 255, 255, 255, 0, 0, 0);              // шрифт белый

  //myGLCD.setFont(RusFont1);
  //myGLCD.setColor(255, 255, 255);   // шрифт белый
  //myGLCD.setBackColor(0, 0, 0);     // синий фон
  myGLCD.printNumI(numberOfDevices, 18 + 56, 24);     // колличество найденных датчиков (найдено)

  if (numberOfDevices == 0) {
    counterB1 = 0;  // set all counter N.C,
    counterB2 = 0;
    counterB3 = 0;
  }
  tempAlarmflag = false;    // clear all error flag
  tempHeatflag = false;
  tempCoolflag = false;
  digitalWrite(tempAlarmPin, LOW);

  for (byte i = 0; i < 8; i++) {                        // clear previos sensor ID value
    Heatsink1Thermometer[i] = 0;
    waterThermometer[i] = 0 ;
    Heatsink2Thermometer[i] = 0;
  }

  for (byte k = 0; k < numberOfDevices; k++) {
    myGLCD.setColor(0, 0, 255);      // синий
    myGLCD.fillRoundRect(165, 83 + 20 + 30 * k, 231, 83 + 40 + 30 * k); // кнопка
    myGLCD.setColor(255, 255, 255);  // белый
    myGLCD.drawRoundRect(165, 83 + 20 + 30 * k, 231, 83 + 40 + 30 * k); // рамка

    myGLCD.setFont(RusFont3);         // русский фонт
    myGLCD.setColor(255, 255, 255);   // шрифт белый
    myGLCD.setBackColor(0, 0, 255);   // синий фон
    //setFont(RUS3, 255, 255, 255, 0, 0, 255); // шрифт белый, синий фон
    if (k == 0) {
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[176 + counterB1])));
      myGLCD.print(buffer, 173, 85 + 24 + 30 * k);
    }   // counterBX = 0/1/2/3 -> Не исп. / Д. Воды / Д.Рад 1 / Д.Рад 2
    if (k == 1) {
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[176 + counterB2])));
      myGLCD.print(buffer, 173, 85 + 24 + 30 * k);
    }  // counterBX = 0/1/2/3 -> Не исп. / Д. Воды / Д.Рад 1 / Д.Рад 2
    if (k == 2) {
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[176 + counterB3])));
      myGLCD.print(buffer, 173, 85 + 24 + 30 * k);
    }
  } // counterBX = 0/1/2/3 -> Не исп. / Д. Воды / Д.Рад 1 / Д.Рад 2

  setFont(SMALL, 0, 255, 0, 0, 0, 0); // показать колличество найденных датчиков
  for (byte k = 0; k < numberOfDevices; k++) {

    if (sensors.getAddress(tempDeviceAddress, k)) {
      float tempC = sensors.getTempC(tempDeviceAddress);

      // connect sensor 1...3 to output Water / Heatsink1 / Heatsink2
      if (k == 0) {
        for (byte i = 0; i < 8; i++) {
          if (counterB1 == 1) {
            waterThermometer[i] = tempDeviceAddress[i];
          }
          if (counterB1 == 2) {
            Heatsink1Thermometer[i] = tempDeviceAddress[i];
          }
          if (counterB1 == 3) {
            Heatsink2Thermometer[i] = tempDeviceAddress[i];
          }
        }
      }

      if (k == 1) {
        for (byte i = 0; i < 8; i++) {
          if (counterB2 == 1) {
            waterThermometer[i] = tempDeviceAddress[i];
          }
          if (counterB2 == 2) {
            Heatsink1Thermometer[i] = tempDeviceAddress[i];
          }
          if (counterB2 == 3) {
            Heatsink2Thermometer[i] = tempDeviceAddress[i];
          }
        }
      }

      if (k == 2) {
        for (byte i = 0; i < 8; i++) {
          if (counterB3 == 1) {
            waterThermometer[i] = tempDeviceAddress[i];
          }
          if (counterB3 == 2) {
            Heatsink1Thermometer[i] = tempDeviceAddress[i];
          }
          if (counterB3 == 3) {
            Heatsink2Thermometer[i] = tempDeviceAddress[i];
          }
        }
      }

      myGLCD.printNumF( tempC, 1, 230 + 40, k * 30 + 83 + 24); // показать текущую температуру найденного датчика
      myGLCD.print(F("R.-"), 241, k * 30 + 83 + 24);
      myGLCD.setColor(255, 255, 255);    // шрифт белый
      myGLCD.print(print_text[57], 230 + 40 + 32 + 6, k * 30 + 83 + 24);
      myGLCD.drawCircle(230 + 40 + 32 + 2, k * 30 + 83 + 26, 1);

      myGLCD.setColor(255, 255, 255);    // шрифт белый
      myGLCD.printNumI(k + 1, 5 + 62, k * 15 + 48); // номер датчика
      myGLCD.print(print_text[56], 5 + 62 + 8, k * 15 + 48); // :

      myGLCD.setFont(RusFont1);      // русский фонт
      myGLCD.setColor(80, 158, 255); // cиний
      //    myGLCD.print(buffer, 5, k*30+85+24);     // Датчик (имя) (перед "подключен") (имя из буфера)
      myGLCD.print(print_text[206], 5, k * 30 + 85 + 24); // Датчик (имя) (перед "подключен")
      myGLCD.setColor(255, 255, 255);
      myGLCD.printNumI(k + 1, 5 + 55, k * 30 + 85 + 24); // номер датчика (цифра) (перед "подключен") (64)

      myGLCD.setColor(0, 255, 0);    // шрифт зеленый
      myGLCD.setBackColor(0, 0, 0);
      myGLCD.print(print_text[206], 11, k * 15 + 51);     // Датчик
      myGLCD.print(print_text[205], 9 + 55 + 8, k * 30 + 83 + 26); // Подключен К

      setFont(SMALL, 88, 255, 238, 0, 0, 0);
      myGLCD.setFont(SmallFont);
      for (byte i = 0; i < 8; i++) {             // серийный номер датчика  unic ID

        byte result;
        byte temp = tempDeviceAddress[i];
        result = temp / 16;		   // first char  convert HEX to ASCII for print
        if (result == 15) {
          myGLCD.print(F("F"), i * 24 + 88, k * 15 + 48); // F
        }
        if (result == 14) {
          myGLCD.print(print_text[85], i * 24 + 88, k * 15 + 48); // E
        }
        if (result == 13) {
          myGLCD.print(print_text[86], i * 24 + 88, k * 15 + 48); // D
        }
        if (result == 12) {
          myGLCD.print(print_text[57], i * 24 + 88, k * 15 + 48); // C
        }
        if (result == 11) {
          myGLCD.print(print_text[21], i * 24 + 88, k * 15 + 48); // B
        }
        if (result == 10) {
          myGLCD.print(F("A"), i * 24 + 88, k * 15 + 48); // A
        }
        if (result < 10 ) {
          myGLCD.printNumI(result, i * 24 + 88, k * 15 + 48);
        }

        result = temp - (temp / 16) * 16;       // second char  convert HEX to ASCII for print
        if (result == 15) {
          myGLCD.print(F("F"), i * 24 + 8 + 88, k * 15 + 48); // F
        }
        if (result == 14) {
          myGLCD.print(print_text[85], i * 24 + 8 + 88, k * 15 + 48); // E
        }
        if (result == 13) {
          myGLCD.print(print_text[86], i * 24 + 8 + 88, k * 15 + 48); // D
        }
        if (result == 12) {
          myGLCD.print(print_text[57], i * 24 + 8 + 88, k * 15 + 48); // C
        }
        if (result == 11) {
          myGLCD.print(print_text[21], i * 24 + 8 + 88, k * 15 + 48); // B
        }
        if (result == 10) {
          myGLCD.print(F("A"), i * 24 + 8 + 88, k * 15 + 48); // A
        }
        if (result < 10 ) {
          myGLCD.printNumI(result, i * 24 + 8 + 88, k * 15 + 48);
        }
        if (i < 7) {
          myGLCD.print(print_text[56], i * 24 + 16 + 88, k * 15 + 48); // двоеточие
        }
      }
    }
  }
}

void Backup() { // запись настроек на SD карту
  PrintStringIndex = 25; printHeader (); // СОХРАНИТЬ / ЗАГРУЗИТЬ НАСТРОЙКИ
  printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
  printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);
  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect(15, 39 + 34, 115, 75 + 34); // нарисовать кнопку ЗАГРУЗИТЬ
  myGLCD.fillRoundRect(205, 39 + 34, 305, 75 + 34); // нарисовать кнопку СОХРАНИТЬ
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(15, 39 + 34, 115, 75 + 34);
  myGLCD.drawRoundRect(205, 39 + 34, 305, 75 + 34); // рамка вокруг СОХРАНИТЬ

  myGLCD.setFont(RusFont1);        // русский фонт
  myGLCD.setColor(255, 255, 255);  // шрифт зеленый
  myGLCD.setBackColor(0, 0, 255);
  //setFont(RUS1, 255, 255, 255, 0, 0, 255);
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[130])));
  myGLCD.print(buffer, 215 + 5, 45 + 36);        // СОХРАНИТЬ
  myGLCD.print(print_text[150], 220, 94);        // НАСТРОЙКИ
  myGLCD.print(print_text[192], 11 + 18, 45 + 36); // ЗАГРУЗИТЬ (RESTORE)
  myGLCD.print(print_text[150], 30, 94);         // НАСТРОЙКИ

  myGLCD.setBackColor(0, 0, 0);
  myGLCD.print(print_text[149], CENTER, 24);    // ВСЕ НАСТРОЙКИ НАХОДЯТСЯ НА ФЛЕШ КАРТЕ
  myGLCD.print(print_text[148], 47, 24 + 12 + 5); // В ТЕКСТОВОМ ФАЙЛЕ

  setFont(SMALL, 0, 255, 0, 0, 0, 0);
  myGLCD.print(print_text[147], 198, 24 + 10 + 5); //  BACKUP.TXT

  myGLCD.setColor(64, 64, 64);            // Draw Dividers in Grey
  myGLCD.drawLine(0, 60, 319, 60);
  myGLCD.setColor(150, 150, 150);
  myGLCD.drawRoundRect(15, 125, 304, 182); // рамка в центре экрана

  myGLCD.setFont(RusFont1);
  myGLCD.setColor(0, 255, 0);  // шрифт зеленый
  myGLCD.setBackColor(0, 0, 0);
  //setFont(RUS1, 0, 255, 0, 0, 0, 0);
  myGLCD.print(print_text[203], CENTER, 138);   // Инициализация Флеш карты...

  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  if (!sd.begin(SDchipSelect, SPI_HALF_SPEED)) {
    sd.initErrorPrint();
    myGLCD.setFont(RusFont1);      // русский фонт
    myGLCD.setColor(255, 0, 0);    // шрифт зеленый
    myGLCD.setBackColor(0, 0, 0);
    //setFont(RUS1, 255, 0, 0, 0, 0, 0);
    myGLCD.print(print_text[202], CENTER, 140 + 13 + 5);  // Ошибка ! инициализации флеш карты
  } else {
    myGLCD.print(print_text[201], CENTER, 140 + 13 + 5);
  }
}  // Инициализация успешно завершена

void drawTestLedArrayScale() { // авто - тест
  int i = 0;       // start scale is constant
  min_cnt = StartTime * 15;

  while (LEDtestTick == true &&  i <= ((StopTime - StartTime) * 15) / 2) { // LEDtestTick== true - start test
    unsigned long currentMillis = millis();

    if (myTouch.dataAvailable()) {
      processMyTouch();
    }
    if (currentMillis - previousMillisLED > 1000)                   // change time every 1s
    { previousMillisLED = currentMillis;

      int XStep = map(i, 0, ((StopTime - StartTime) * 15) / 2, 26, EndScale);
      myGLCD.setColor(110, 110, 110);     // серый
      myGLCD.drawRect(31, 23, 111, 42);         // белая рамка вокруг отсчета времени
      setFont(LARGE, 88, 255, 238, 0, 0, 0);    // отсчет времени шрифт бирюзовый, фон черный
      if (((min_cnt / 15) / 4) <= 9) {
        myGLCD.printNumI((min_cnt / 15) / 4, 48, 25); // 2
      }
      else {
        myGLCD.printNumI((min_cnt / 15) / 4, 32, 25); // 2
      }
      myGLCD.print(print_text[56], 64, 25);      // :
      myGLCD.printNumI(min_cnt % 60, 78, 25, 2, '00'); // 20, 20

      myGLCD.setColor(88, 255, 238);   // цвет полосы времени бирюзовый
      myGLCD.drawLine(XStep, BotSldY + 8, XStep, BotSldY + 3 );
      LED_levelo_output();
      min_cnt += 2 ;           // two minutes increment
      i += 1;
    }
  }

  printButton(print_text[96], ledChV[0], ledChV[1], ledChV[2], ledChV[3], SMALL); // buton start/stop test
  LEDtestTick = false;
}                        // end test, enable button BACK/Cancel

void calculateStartTime() {
  byte i;
  for (i = 0; i <= 95; i++) {  // высчитать время "StartTime"
    if (wwtled[i] != 0 && bitRead(LedShannelStatusByte, 0) == true) {
      goto ext_Stlbl;
    }
    if (swtled[i] != 0 && bitRead(LedShannelStatusByte, 1) == true) {
      goto ext_Stlbl;
    }
    if (rswtled[i] != 0 && bitRead(LedShannelStatusByte, 2) == true) {
      goto ext_Stlbl;
    }
    if (rled[i] != 0 && bitRead(LedShannelStatusByte, 3) == true) {
      goto ext_Stlbl;
    }
    if (uvled[i] != 0 && bitRead(LedShannelStatusByte, 4) == true) {
      goto ext_Stlbl;
    }
    if (oLed[i] != 0 && bitRead(LedShannelStatusByte, 5) == true) {
      goto ext_Stlbl;
    }
    if (gled[i] != 0 && bitRead(LedShannelStatusByte, 6) == true) {
      goto ext_Stlbl;
    }
  }

ext_Stlbl:
  StartTime = i - 1;
  if (StartTime == 255) {
    StartTime = 0;  // 255
  }
}

void calculateStopTime() {
  byte i;
  for (i = 95; i > 0; i--) {   // высчитать время "StopTime"
    if (wwtled[i] != 0 && bitRead(LedShannelStatusByte, 0) == true) {
      goto ext_SStlbl;
    }
    if (swtled[i] != 0 && bitRead(LedShannelStatusByte, 1) == true) {
      goto ext_SStlbl;
    }
    if (rswtled[i] != 0 && bitRead(LedShannelStatusByte, 2) == true) {
      goto ext_SStlbl;
    }
    if (rled[i] != 0 && bitRead(LedShannelStatusByte, 3) == true) {
      goto ext_SStlbl;
    }
    if (uvled[i] != 0 && bitRead(LedShannelStatusByte, 4) == true) {
      goto ext_SStlbl;
    }
    if (oLed[i] != 0 && bitRead(LedShannelStatusByte, 5) == true) {
      goto ext_SStlbl;
    }
    if (gled[i] != 0 && bitRead(LedShannelStatusByte, 6) == true) {
      goto ext_SStlbl;
    }
  }

ext_SStlbl:
  StopTime = i + 1;
}

void testIndLedScreen2(boolean refreshTest = false) { // Sector ************************************ dispScreen = 4

  TopSldY = 53; BotSldY = TopSldY + 100;
  if (refreshTest == true) {
    myGLCD.setColor(0, 0, 0);
    myGLCD.fillRect(0, 0, 319, 15);                         // clear only header
    setFont(SMALL, 255, 255, 255, 0, 0, 0);
    myGLCD.setColor(rgbCh0[0], rgbCh0[1], rgbCh0[2]);     // белый
    myGLCD.print(print_text[184], 26, 15);    // name WHT
    myGLCD.setColor(rgbCh1[0], rgbCh1[1], rgbCh1[2]);     // голубой
    myGLCD.print(print_text[183], 61, 15);    // name BLU
    myGLCD.setColor(rgbCh2[0], rgbCh2[1], rgbCh2[2]);     // рояль
    myGLCD.print(print_text[182], 96, 15);    // name RBL
    myGLCD.setColor(rgbCh3[0], rgbCh3[1], rgbCh3[2]);     // красный
    myGLCD.print(print_text[181], 131, 15);   // name RED
    myGLCD.setColor(rgbCh4[0], rgbCh4[1], rgbCh4[2]);     // фиолетовый
    myGLCD.print(print_text[180], 166, 15);   // name UVL
    myGLCD.setColor(rgbCh5[0], rgbCh5[1], rgbCh5[2]);     // оранжевый
    myGLCD.print(print_text[179], 201, 15);  // name ORG
    myGLCD.setColor(rgbCh6[0], rgbCh6[1], rgbCh6[2]);     // зеленый
    myGLCD.print(print_text[178], 235, 15);  // name GRN
    myGLCD.setColor(176, 176, 176);  // серый
    myGLCD.print(print_text[177], 266, 15);  // name Moon

    // sector
    //                                                  v - начало по горизонтали
    for (int b = 0; b < 8; b++) {
      drawUpButtonSlide((b * 35) + 21, TopSldY - 26); // UP Buttons (button)
    }
    for (int b = 0; b < 8; b++) {
      drawDownButtonSlide((b * 35) + 21, TopSldY + 115); // DOWN Buttons
    }
    temp_sector = min_cnt / 15;     // read current time sector only first time

    myGLCD.setColor(64, 64, 64);
    myGLCD.drawRect(0, 196, 319, 194);          // рамка вокруг нижних кнопок
    printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);
    printButtonRUS(print_text[29], Psect[0], Psect[1], Psect[2], Psect[3], SMALL);  // < sector
    printButtonRUS(print_text[30], Nsect[0], Nsect[1], Nsect[2], Nsect[3], SMALL);  // sector >
    printButton("", StopDay[0], StopDay[1], StopDay[2], StopDay[3], SMALL);      // top buttons
    printButton("", StartDay[0], StartDay[1], StartDay[2], StartDay[3], SMALL);  // top buttons

    myGLCD.setFont(RusFont1);
    myGLCD.setColor(255, 255, 255);
    myGLCD.setBackColor(0, 0, 255);
    //setFont(RUS1, 255, 255, 255, 0, 0, 255);
    myGLCD.print(print_text[143], 13, 3);    // рассвет
    //myGLCD.setColor(176, 176, 176);
    myGLCD.print(print_text[144], 261, 3);
  } // закат

  if (refreshTest == true) {
    myGLCD.setColor(130, 130, 130);       // Cерая рамка вокруг сектора времени
    myGLCD.drawRect(182, 1, 228, 14);
    //  myGLCD.setColor(0, 255, 0);         // fill sector background in green
    //  myGLCD.fillRect(89, 1, 226, 14);

    // display current TIME sector = min_cnt/15, convert to hour min_cnt/15)/4, to min. (min_cnt/15)*15)%60
    setFont(SMALL, 0, 255, 0, 0, 0, 0);        // print SECTOR TIME on TOP  шрифт зел
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[14])));
    myGLCD.print(buffer, 88, 2);
  }            // TIME SECTOR
  setFont(SMALL, 88, 255, 238, 0, 0, 0);

  if ((temp_sector / 4) <= 9) {
    myGLCD.print(" ", 91 + 96, 2);
    myGLCD.printNumI(temp_sector / 4, 91 + 104, 2);
  }               // Start sector is min_cnt/15
  else {
    myGLCD.printNumI(temp_sector / 4, 91 + 96, 2);
  }
  myGLCD.printNumI((temp_sector * 15) % 60, 91 + 120, 2, 2, '00'); // min 00/15/30/45 in hour

  wwt_out = wwtled[temp_sector];      // read setting from buffer
  cwt_out = swtled[temp_sector];
  rb_out = rswtled[temp_sector];
  r_out = rled[temp_sector];
  uv_out = uvled[temp_sector];
  o_out = oLed[temp_sector];
  gr_out = gled[temp_sector];

  // check if channel on or off
  if (bitRead(LedShannelStatusByte, 0) == false) {
    wwt_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 1) == false) {
    cwt_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 2) == false) {
    rb_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 3) == false) {
    r_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 4) == false) {
    uv_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 5) == false) {
    o_out = 0;
  }
  if (bitRead(LedShannelStatusByte, 6) == false) {
    gr_out = 0;
  }

  for (byte i = 0; i < 8; i++) {
    sbX1 = (i * 35) + 21; sbX2 = (i * 35) + 51; // draw white bar outline rectangle (byte i=0; i<8; i++){sbX1=(i*35)+4; sbX2=(i*35)+34;
    setFont(SMALL, 255, 255, 255, 0, 0, 0);
    myGLCD.drawRect(sbX1, TopSldY - 1, sbX2, BotSldY);
  }

  for (byte i = 0; i < 8; i++) {
    sbX1 = (i * 35) + 21; sbX2 = (i * 35) + 51; // print Slider Bar current values
    SliderSwitch  = true;

    if (i == 0) {
      sbR = rgbCh0[0]; sbG = rgbCh0[1]; sbB = rgbCh0[2]; tSlide = wwt_out;	 // CW colour(белый)
      setFont(SMALL, sbR, sbG, sbB, 0, 0, 0);
      wwtcol_out = SliderBarsForChange(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }

    if (i == 1) {
      sbR = rgbCh1[0]; sbG = rgbCh1[1]; sbB = rgbCh1[2]; tSlide = cwt_out;	 // BL colour (синий)
      setFont(SMALL, sbR, sbG, sbB, 0, 0, 0);
      cwtcol_out = SliderBarsForChange(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }

    if (i == 2) {
      sbR = rgbCh2[0]; sbG = rgbCh2[1]; sbB = rgbCh2[2]; tSlide = rb_out;	 // RBL colour (рояль)
      setFont(SMALL, sbR, sbG, sbB, 0, 0, 0);
      rbcol_out = SliderBarsForChange(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }

    if (i == 3) {
      sbR = rgbCh3[0]; sbG = rgbCh3[1]; sbB = rgbCh3[2]; tSlide = r_out;	 // DR colour (красный)
      setFont(SMALL, sbR, sbG, sbB, 0, 0, 0);
      rcol_out = SliderBarsForChange(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }

    if (i == 4) {
      sbR = rgbCh4[0]; sbG = rgbCh4[1]; sbB = rgbCh4[2]; tSlide = uv_out; // UV colour (фиолет)
      setFont(SMALL, sbR, sbG, sbB, 0, 0, 0);
      uvcol_out = SliderBarsForChange(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }

    if (i == 5) {
      sbR = rgbCh5[0]; sbG = rgbCh5[1]; sbB = rgbCh5[2]; tSlide = o_out;	 // OR colour (оранж)
      setFont(SMALL, sbR, sbG, sbB, 0, 0, 0);
      ocol_out = SliderBarsForChange(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }

    if (i == 6) {
      sbR = rgbCh6[0]; sbG = rgbCh6[1]; sbB = rgbCh6[2]; tSlide = gr_out;	 // GR colour (зел)
      setFont(SMALL, sbR, sbG, sbB, 0, 0, 0);
      grcol_out = SliderBarsForChange(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }

    if (i == 7) {
      sbR = rgbCh8[0]; sbG = rgbCh8[1]; sbB = rgbCh8[2]; tSlide = map (moon_out, 0, 255, 0, 100); // MOON colour 255
      setFont(SMALL, sbR, sbG, sbB, 0, 0, 0);
      mooncol_out = SliderBarsForChange(TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }
  }
}

//========================= Preset Screen ==============================
void PresetLedScreen(boolean refreshPresetscreen = false) {   //  пресеты
  if (refreshPresetscreen == true) {
    myGLCD.setColor(0, 0, 0);
    myGLCD.fillRect(0, 0, 319, 15);			    // clear only header
    setFont(SMALL, 255, 255, 255, 0, 0, 0);
    myGLCD.setColor(rgbCh0[0], rgbCh0[1], rgbCh0[2]);     // белый
    myGLCD.print(print_text[184], 27, 15);   //  WHT
    myGLCD.setColor(rgbCh1[0], rgbCh1[1], rgbCh1[2]);     // голубой
    myGLCD.print(print_text[183], 67, 15);   // BLU
    myGLCD.setColor(rgbCh2[0], rgbCh2[1], rgbCh2[2]);     // рояль
    myGLCD.print(print_text[182], 107, 15);  // RBL
    myGLCD.setColor(rgbCh3[0], rgbCh3[1], rgbCh3[2]);     // красный
    myGLCD.print(print_text[181], 147, 15);  // RED
    myGLCD.setColor(rgbCh4[0], rgbCh4[1], rgbCh4[2]);     // фиолетовый
    myGLCD.print(print_text[180], 187, 15);  // UVL
    myGLCD.setColor(rgbCh5[0], rgbCh5[1], rgbCh5[2]);     // оранжевый
    myGLCD.print(print_text[179], 227, 15);  // ORG
    myGLCD.setColor(rgbCh6[0], rgbCh6[1], rgbCh6[2]);     // зеленый
    myGLCD.print(print_text[178], 267, 15);  // GRN

    TopSldY = 53; BotSldY = TopSldY + 100;

    for (byte b = 0; b < 7; b++) {   // UP Buttons
      drawUpButtonSlide((b * 40) + 4 + 18, TopSldY - 26);
    }
    for (byte b = 0; b < 7; b++) {   // DOWN Buttons
      drawDownButtonSlide((b * 40) + 4 + 18, TopSldY + 115);
    }
    temp_sector = min_cnt / 15; // read current time sector only first time

    for (byte i = 0; i < 7; i++) {
      sbX1 = (i * 40) + 4 + 18; sbX2 = (i * 40) + 34 + 18; // draw white bar outline rectangle
      setFont(SMALL, 255, 255, 255, 0, 0, 0);
      myGLCD.drawRect(sbX1, TopSldY - 1, sbX2, BotSldY);
    }
  }
  SliderSwitch  = true;

  if ((GlobalStatus2Byte & 0xF) == 0 ) {  // no preset
    wwtcol_out = 0;	                // CW colour	(белый)
    cwtcol_out = 0;	          // BL colour (синий)
    rbcol_out = 0;	            // RBL colour (рояль)
    rcol_out = 0;                          // DR colour	(красный)
    uvcol_out = 0;                         // UV colour (фиолет)
    ocol_out = 0;                          // OR colour (оранж)
    grcol_out = 0;	                        // GR colour (зелен)
    colorLEDtest = false;
  }

  for (byte i = 0; i < 7; i++) {
    sbX1 = (i * 40) + 4 + 18; sbX2 = (i * 40) + 34 + 18; // print Slider Bar current values

    if (i == 0 && bitRead(LedShannelStatusByte, 0) == true) {
      sbR = rgbCh0[0]; sbG = rgbCh0[1]; sbB = rgbCh0[2]; tSlide = wwtcol_out;	 // CW colour	(белый)
      wwtcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }

    if (i == 1 && bitRead(LedShannelStatusByte, 1) == true) {
      sbR = rgbCh1[0]; sbG = rgbCh1[1]; sbB = rgbCh1[2]; tSlide = cwtcol_out;	 // BL colour (голубой)
      cwtcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }

    if (i == 2 && bitRead(LedShannelStatusByte, 2) == true) {
      sbR = rgbCh2[0]; sbG = rgbCh2[1]; sbB = rgbCh2[2]; tSlide = rbcol_out;	 // RBL colour (рояль)
      rbcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }

    if (i == 3 && bitRead(LedShannelStatusByte, 3) == true) {
      sbR = rgbCh3[0]; sbG = rgbCh3[1]; sbB = rgbCh3[2];  tSlide = rcol_out;		 // DR colour	 (красный)
      rcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }

    if (i == 4 && bitRead(LedShannelStatusByte, 4) == true) {
      sbR = rgbCh4[0]; sbG = rgbCh4[1]; sbB = rgbCh4[2]; tSlide = uvcol_out; // UV colour (uv)
      uvcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }

    if (i == 5 && bitRead(LedShannelStatusByte, 5) == true) {
      sbR = rgbCh5[0]; sbG = rgbCh5[1]; sbB = rgbCh5[2]; tSlide = ocol_out;   // OR colour (oLed)
      ocol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }

    if (i == 6 && bitRead(LedShannelStatusByte, 6) == true) {
      sbR = rgbCh6[0]; sbG = rgbCh6[1]; sbB = rgbCh6[2]; tSlide = grcol_out;	    // GR colour (green)
      grcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
    }
  }

  // if (ledON == 0){  // включение управления каналами при настройке пресетов
  LED_levelo_output();   //}        // send calculated value to PWM

  if (refreshPresetscreen == true) {
    myGLCD.setColor(64, 64, 64);                       // Draw Dividers in Grey
    myGLCD.drawRect(0, 196, 319, 194);                 // Bottom Horizontal Divider
    PrintBSC3b();

    if (bitRead(GlobalStatus2Byte, 0) == 1) {
      printButton(print_text[31], LedPres1[0], LedPres1[1], LedPres1[2], LedPres1[3], SMALL, GREEN_BAC);
    } // ON preset
    else {
      printButton(print_text[31], LedPres1[0], LedPres1[1], LedPres1[2], LedPres1[3], SMALL); // OFF preset
    }
    if (bitRead(GlobalStatus2Byte, 1) == 1) {
      printButton(print_text[32], LedPres2[0], LedPres2[1], LedPres2[2], LedPres2[3], SMALL, GREEN_BAC);
    } // ON preset
    else {
      printButton(print_text[32], LedPres2[0], LedPres2[1], LedPres2[2], LedPres2[3], SMALL); // OFF preset
    }
    if (bitRead(GlobalStatus2Byte, 2) == 1) {
      printButton(print_text[33], LedPres3[0], LedPres3[1], LedPres3[2], LedPres3[3], SMALL, GREEN_BAC);
    } // ON preset
    else {
      printButton(print_text[33], LedPres3[0], LedPres3[1], LedPres3[2], LedPres3[3], SMALL); // OFF preset
    }
    if (bitRead(GlobalStatus2Byte, 3) == 1) {
      printButton(print_text[34], LedPres4[0], LedPres4[1], LedPres4[2], LedPres4[3], SMALL, GREEN_BAC);
    } // ON preset
    else {
      printButton(print_text[34], LedPres4[0], LedPres4[1], LedPres4[2], LedPres4[3], SMALL); // OFF preset
    }
  }
}

void SaveAndExit () {    // сохранить настройки пресетов

  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(5, 53, 315, 153);
  myGLCD.setColor(100, 100, 100);
  myGLCD.fillRoundRect(6, 54, 314, 152);

  myGLCD.setFont(RusFont2);
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(100, 100, 100);
  //setFont(RUS2, 255, 255, 255, 100, 100, 100);
  myGLCD.print(print_text[185], 40, 73);   // ВЫ ХОТИТЕ СОХРАНИТЬ НАСТРОЙКИ,
  myGLCD.print(print_text[186], 69, 95);   //     ПЕРЕД ВЫХОДОМ ИЗ МЕНЮ ?

  printButton(print_text[35], Yes[0], Yes[1], Yes[2], Yes[3], SMALL);
  printButton(print_text[36], No[0], No[1], No[2], No[3], SMALL);

  //	    DisplayTimeSector(temp_sector);
  wwt_out = wwtled[temp_sector];  // read setting from buffer
  cwt_out = swtled[temp_sector];
  rb_out = rswtled[temp_sector];
  r_out = rled[temp_sector];
  uv_out = uvled[temp_sector];
  o_out = oLed[temp_sector];
  gr_out = gled[temp_sector];

  // check if channel on or off
  if (bitRead(LedShannelStatusByte, 0) == false) {
    wwt_out = 0; // white
  }
  if (bitRead(LedShannelStatusByte, 1) == false) {
    cwt_out = 0; // blue
  }
  if (bitRead(LedShannelStatusByte, 2) == false) {
    rb_out = 0; // rblue
  }
  if (bitRead(LedShannelStatusByte, 3) == false) {
    r_out = 0; // red
  }
  if (bitRead(LedShannelStatusByte, 4) == false) {
    uv_out = 0; // uv
  }
  if (bitRead(LedShannelStatusByte, 5) == false) {
    o_out = 0; // oLed
  }
  if (bitRead(LedShannelStatusByte, 6) == false) {
    gr_out = 0; // green
  }
}

/*************************** Preset Function ******************************************/
void PresetSwitch() { // переключение пресетов в главном меню
  calculateStopTime();
  switch ((GlobalStatus2Byte & 0x0F)) {
    case 0:
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRoundRect (272, 89, 312, 102);
      printPres(print_text[52]);  // П-1
      GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0 | 0x1);
      AddressShift = 0; ReadLEDPresetFromEEPROM(); colorLEDtest = true;
      break;
    case 1:
      printPres(print_text[53]);  // П-2
      GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0 | 0x2);
      AddressShift = 9; ReadLEDPresetFromEEPROM(); colorLEDtest = true;
      break;
    case 2:
      printPres(print_text[54]);  // П-3
      GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0 | 0x4);
      AddressShift = 18; ReadLEDPresetFromEEPROM(); colorLEDtest = true;
      break;
    case 4:
      printPres(print_text[55]);  // П-4
      GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0 | 0x8);
      AddressShift = 27; ReadLEDPresetFromEEPROM(); colorLEDtest = true;
      break;
    case 8:
      printPresoff(print_text[95]); // ВЫКЛ
      GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0);
      colorLEDtest = false;
      break;
  }
}

// ======================  Скринсейв Аналоговые часы
void ATimeSaver() {

  // Нарисовать Циферблат
  myGLCD.setColor(0, 0, 255);   // циферблат синий
  myGLCD.setBackColor(0, 0, 0); // фон черный
  for (int i = 0; i < 5; i++) { // толщина внешнего синего круга 5 пикс
    myGLCD.drawCircle(clockCenterX, clockCenterY, 119 - i);
  } // диаметр циферблата 119

  myGLCD.setColor(64, 64, 64); // серый
  for (int i = 0; i < 1; i++) { // толщина внешнего синего круга 1 пикс
    myGLCD.drawCircle(clockCenterX, clockCenterY, 113 - i);
  } // диаметр циферблата 112

  myGLCD.setColor(0, 0, 255);
  for (int i = 0; i < 5; i++) { // внутренний синий круг
    myGLCD.drawCircle(clockCenterX, clockCenterY, i);
  }

  // myGLCD.setFont(BigRusFont);
  // myGLCD.setColor(192, 192, 255);   // цвет цифр серый
  setFont(LARGE, 192, 192, 255, 0, 0, 0);
  myGLCD.print(print_text[49], clockCenterX + 92, clockCenterY - 8); // 3  шкала
  myGLCD.print(print_text[79], clockCenterX - 8, clockCenterY + 95); // 6
  myGLCD.print(F("9"), clockCenterX - 109, clockCenterY - 8);      // 9
  myGLCD.print(print_text[81], clockCenterX - 16, clockCenterY - 109); // 12

  for (int i = 0; i < 12; i++) {
    if ((i % 3) != 0) drawMark(i);
  }

  RTC.getTime();
  drawMin(RTC.minute);
  drawHour(RTC.hour, RTC.minute);
  drawSec(RTC.second);
  oldsec = RTC.second;
}

void drawMark(int h) { // часовая шкала
  float x1, y1, x2, y2; h = h * 30; h = h + 270;
  x1 = 110 * cos(h * 0.0175);
  y1 = 110 * sin(h * 0.0175);
  x2 = 100 * cos(h * 0.0175);
  y2 = 100 * sin(h * 0.0175);
  myGLCD.drawLine(x1 + clockCenterX, y1 + clockCenterY, x2 + clockCenterX, y2 + clockCenterY );
}

void drawSec(int s) {      // отображение секунд
  float x1, y1, x2, y2;
  int ps = s - 1;

  myGLCD.setColor(0, 0, 0); // чёрный цвет
  if (ps == -1)ps = 59; ps = ps * 6; ps = ps + 270; // гасим след за стрелкой
  x1 = 95 * cos(ps * 0.0175);
  y1 = 95 * sin(ps * 0.0175);
  x2 = 6 * cos(ps * 0.0175);
  y2 = 6 * sin(ps * 0.0175);

  myGLCD.drawLine(x1 + clockCenterX, y1 + clockCenterY, x2 + clockCenterX, y2 + clockCenterY);
  myGLCD.setColor(255, 0, 0); s = s * 6; s = s + 270;
  x1 = 95 * cos(s * 0.0175);
  y1 = 95 * sin(s * 0.0175);
  x2 = 6 * cos(s * 0.0175);
  y2 = 6 * sin(s * 0.0175);
  myGLCD.drawLine(x1 + clockCenterX, y1 + clockCenterY, x2 + clockCenterX, y2 + clockCenterY);
}

void drawMin(int m) { // Отображение минут
  float x1, y1, x2, y2, x3, y3, x4, y4;
  int pm = m - 1;

  myGLCD.setColor(0, 0, 0);
  if (pm == -1)pm = 59; pm = pm * 6; pm = pm + 270;

  x1 = 80 * cos(pm * 0.0175);
  y1 = 80 * sin(pm * 0.0175);
  x2 = 6 * cos(pm * 0.0175);
  y2 = 6 * sin(pm * 0.0175);
  x3 = 30 * cos((pm + 4) * 0.0175);
  y3 = 30 * sin((pm + 4) * 0.0175);
  x4 = 30 * cos((pm - 4) * 0.0175);
  y4 = 30 * sin((pm - 4) * 0.0175);

  myGLCD.drawLine(x1 + clockCenterX, y1 + clockCenterY, x3 + clockCenterX, y3 + clockCenterY);
  myGLCD.drawLine(x3 + clockCenterX, y3 + clockCenterY, x2 + clockCenterX, y2 + clockCenterY);
  myGLCD.drawLine(x2 + clockCenterX, y2 + clockCenterY, x4 + clockCenterX, y4 + clockCenterY);
  myGLCD.drawLine(x4 + clockCenterX, y4 + clockCenterY, x1 + clockCenterX, y1 + clockCenterY);

  myGLCD.setColor(0, 255, 0); m = m * 6; m = m + 270;

  x1 = 80 * cos(m * 0.0175);
  y1 = 80 * sin(m * 0.0175);
  x2 = 6 * cos(m * 0.0175);
  y2 = 6 * sin(m * 0.0175);
  x3 = 30 * cos((m + 4) * 0.0175);
  y3 = 30 * sin((m + 4) * 0.0175);
  x4 = 30 * cos((m - 4) * 0.0175);
  y4 = 30 * sin((m - 4) * 0.0175);

  myGLCD.drawLine(x1 + clockCenterX, y1 + clockCenterY, x3 + clockCenterX, y3 + clockCenterY);
  myGLCD.drawLine(x3 + clockCenterX, y3 + clockCenterY, x2 + clockCenterX, y2 + clockCenterY);
  myGLCD.drawLine(x2 + clockCenterX, y2 + clockCenterY, x4 + clockCenterX, y4 + clockCenterY);
  myGLCD.drawLine(x4 + clockCenterX, y4 + clockCenterY, x1 + clockCenterX, y1 + clockCenterY);
}

void drawHour(int h, int m) { // Отображение часов
  float x1, y1, x2, y2, x3, y3, x4, y4;
  int ph = h;

  myGLCD.setColor(0, 0, 0);
  if (m == 0) {
    ph = ((ph - 1) * 30) + ((m + 59) / 2);
  } else {
    ph = (ph * 30) + ((m - 1) / 2);
  }

  ph = ph + 270;
  x1 = 60 * cos(ph * 0.0175);
  y1 = 60 * sin(ph * 0.0175);
  x2 = 6 * cos(ph * 0.0175);
  y2 = 6 * sin(ph * 0.0175);
  x3 = 20 * cos((ph + 5) * 0.0175);
  y3 = 20 * sin((ph + 5) * 0.0175);
  x4 = 20 * cos((ph - 5) * 0.0175);
  y4 = 20 * sin((ph - 5) * 0.0175);

  myGLCD.drawLine(x1 + clockCenterX, y1 + clockCenterY, x3 + clockCenterX, y3 + clockCenterY);
  myGLCD.drawLine(x3 + clockCenterX, y3 + clockCenterY, x2 + clockCenterX, y2 + clockCenterY);
  myGLCD.drawLine(x2 + clockCenterX, y2 + clockCenterY, x4 + clockCenterX, y4 + clockCenterY);
  myGLCD.drawLine(x4 + clockCenterX, y4 + clockCenterY, x1 + clockCenterX, y1 + clockCenterY);

  myGLCD.setColor(255, 255, 0); h = (h * 30) + (m / 2); h = h + 270; // часовая стрелка (желтый цвет)
  x1 = 60 * cos(h * 0.0175);
  y1 = 60 * sin(h * 0.0175);
  x2 = 6 * cos(h * 0.0175);
  y2 = 6 * sin(h * 0.0175);
  x3 = 20 * cos((h + 5) * 0.0175);
  y3 = 20 * sin((h + 5) * 0.0175);
  x4 = 20 * cos((h - 5) * 0.0175);
  y4 = 20 * sin((h - 5) * 0.0175);

  myGLCD.drawLine(x1 + clockCenterX, y1 + clockCenterY, x3 + clockCenterX, y3 + clockCenterY);
  myGLCD.drawLine(x3 + clockCenterX, y3 + clockCenterY, x2 + clockCenterX, y2 + clockCenterY);
  myGLCD.drawLine(x2 + clockCenterX, y2 + clockCenterY, x4 + clockCenterX, y4 + clockCenterY);
  myGLCD.drawLine(x4 + clockCenterX, y4 + clockCenterY, x1 + clockCenterX, y1 + clockCenterY);
}

void analogClock() {  // (главный цикл для аналоговых часов)
  int x, y;
  RTC.getTime();

  if (oldsec != RTC.second) {
    if (RTC.second == 0) {
      drawMin(RTC.minute);
      drawHour(RTC.hour, RTC.minute);
    }
    drawSec(RTC.second); oldsec = RTC.second;
  }
}
/***********************УСТАНРВКА УРОВНЯ PH********************************************/
void SetPH() {

  PrintStringIndex = 9;
  printHeader ();

  Menu_Text(78, 23, 28, 3, 8 );      //  УСТАНОВКА УРОВНЯ РН
  Menu_Text(122, 80, 95, 3, 8 );     //  КАЛИБРОВКА
  Menu_Text(79, 170, 95, 3, 8 );     //  ДАТЧИКА РН
  Menu_Text(80, 230, 23, 3, 8 );     //  ТЕКУЩЕЕ
  Menu_Text(81, 227, 35, 3, 8 );     //  ЗНАЧЕНИЕ

  myGLCD.drawRoundRect(dosval3[0], dosval3[1], dosval3[2], dosval3[3]);
  myGLCD.drawRoundRect(dosval4[0], dosval4[1], dosval4[2], dosval4[3]);
  myGLCD.setColor(255, 0, 0);
  myGLCD.fillRoundRect(dos3b[0], dos3b[1], dos3b[2] + 50, dos3b[3]);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(dos3b[0], dos3b[1], dos3b[2] + 50, dos3b[3]);
  myGLCD.setColor(255, 0, 0);
  myGLCD.fillRoundRect(dos4b[0], dos4b[1], dos4b[2] + 50, dos4b[3]);
  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(dos4b[0], dos4b[1], dos4b[2] + 50, dos4b[3]);
  setFont(SMALL, 255, 255, 0, 255, 0, 0);
  myGLCD.setFont(RusFont3);
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[122])));
  myGLCD.print(buffer, dos3b[0] + 6, dos3b[1] + 12);
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[82])));
  myGLCD.print(buffer, dos3b[0] + 96, dos3b[1] + 12);
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[84])));
  myGLCD.print(buffer, dos3b[0] + 116, dos3b[1] + 12);

  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[122])));
  myGLCD.print(buffer, dos4b[0] + 6, dos4b[1] + 12);
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[82])));
  myGLCD.print(buffer, dos4b[0] + 96, dos4b[1] + 12);
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[85])));
  myGLCD.print(buffer, dos4b[0] + 116, dos4b[1] + 12);

  SetvalPH = PlusMinusCountF (false, false, dos1b[0], dos1b[1] + 20, 150, 5, 9, 0.1, SetvalPH);
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }

  myGLCD.setColor(64, 64, 64);
  myGLCD.drawRect(0, 196, 319, 194);

  printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);        // НАЗАД
  printButtonRUS(print_text[3], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);// СОХРАНИТЬ
  printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);        // ОТМЕНА
}

/******** AUTOMATIC DOSER SCREEN / ЭКРАН ДОЗАТОРОВ ************* dispScreen = 36 **********************/
void autoDoserScreen()
{
  PrintStringIndex = 8;
  printHeader ();

  myGLCD.setColor(64, 64, 64);
  myGLCD.drawRect(0, 196, 319, 194);
  printButtonRUS(print_text[2], back[0], back[1], back[2], back[3], SMALL);        // НАЗАД
  //  printButtonRUS(print_text[215], prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3], SMALL);   // УСТАНОВ.
  printButtonRUS(print_text[1], canC[0], canC[1], canC[2], canC[3], SMALL);        // ОТМЕНА

  myGLCD.drawRoundRect(dosval1[0], dosval1[1], dosval1[2], dosval1[3]);
  myGLCD.drawRoundRect(dosval2[0], dosval2[1], dosval2[2], dosval2[3]);
  myGLCD.drawRoundRect(dosval3[0], dosval3[1], dosval3[2], dosval3[3]);
  myGLCD.drawRoundRect(dosval4[0], dosval4[1], dosval4[2], dosval4[3]);

  setFont(SMALL, 255, 255, 255, 224, 0, 224);
  printFont();
  myGLCD.printNumI(numDoz1,  dosval1[0] - 110, dosval1[1] + 5);
  if ((dozVal1 / numDoz1) > 99) {
    myGLCD.printNumI(dozVal1 / numDoz1,  dosval1[0] - 75, dosval1[1] + 5);
  } else if ((dozVal1 / numDoz1) > 9) {
    myGLCD.printNumI(dozVal1 / numDoz1,  dosval1[0] - 65, dosval1[1] + 5);
  }
  else {
    myGLCD.printNumI(dozVal1 / numDoz1,  dosval1[0] - 55, dosval1[1] + 5);
  }
  myGLCD.printNumI(numDoz2,  dosval2[0] - 110, dosval2[1] + 5);
  if ((dozVal2 / numDoz2) > 99) {
    myGLCD.printNumI(dozVal2 / numDoz2,  dosval2[0] - 75, dosval2[1] + 5);
  } else if ((dozVal2 / numDoz2) > 9) {
    myGLCD.printNumI(dozVal2 / numDoz2,  dosval2[0] - 65, dosval2[1] + 5);
  }
  else {
    myGLCD.printNumI(dozVal2 / numDoz2,  dosval2[0] - 55, dosval2[1] + 5);
  }
  myGLCD.printNumI(numDoz3,  dosval3[0] - 110, dosval3[1] + 5);
  if ((dozVal3 / numDoz3) > 99) {
    myGLCD.printNumI(dozVal3 / numDoz3,  dosval3[0] - 75, dosval3[1] + 5);
  } else if ((dozVal3 / numDoz3) > 9) {
    myGLCD.printNumI(dozVal3 / numDoz3,  dosval3[0] - 65, dosval3[1] + 5);
  }
  else {
    myGLCD.printNumI(dozVal3 / numDoz3,  dosval3[0] - 55, dosval3[1] + 5);
  }
  myGLCD.printNumI(numDoz4,  dosval4[0] - 110, dosval4[1] + 5);
  if ((dozVal4 / numDoz4) > 99) {
    myGLCD.printNumI(dozVal4 / numDoz4,  dosval4[0] - 75, dosval4[1] + 5);
  } else if ((dozVal4 / numDoz4) > 9) {
    myGLCD.printNumI(dozVal4 / numDoz4,  dosval4[0] - 65, dosval4[1] + 5);
  }
  else {
    myGLCD.printNumI(dozVal4 / numDoz4,  dosval4[0] - 55, dosval4[1] + 5);
  }

  if (dozVal1 > 99) {
    myGLCD.printNumI(dozVal1,  dosval1[0] + 8, dosval1[1] + 5);
  }
  else if (dozVal1 > 9 && dozVal1 < 100) {
    myGLCD.printNumI(dozVal1,  dosval1[0] + 24, dosval1[1] + 5);
  }
  else {
    myGLCD.printNumI(dozVal1,  dosval1[0] + 40, dosval1[1] + 5);
  }

  if (dozVal2 > 99) {
    myGLCD.printNumI(dozVal2,  dosval2[0] + 8, dosval2[1] + 5);
  }
  else if (dozVal2 > 9 && dozVal2 < 100) {
    myGLCD.printNumI(dozVal2,  dosval2[0] + 24, dosval2[1] + 5);
  }
  else {
    myGLCD.printNumI(dozVal2,  dosval2[0] + 40, dosval2[1] + 5);
  }

  if (dozVal3 > 99) {
    myGLCD.printNumI(dozVal3,  dosval3[0] + 8, dosval3[1] + 5);
  }
  else if (dozVal3 > 9 && dozVal3 < 100) {
    myGLCD.printNumI(dozVal3,  dosval3[0] + 24, dosval3[1] + 5);
  }
  else {
    myGLCD.printNumI(dozVal3,  dosval3[0] + 40, dosval3[1] + 5);
  }

  if (dozVal4 > 99) {
    myGLCD.printNumI(dozVal4,  dosval4[0] + 8, dosval4[1] + 5);
  }
  else if (dozVal4 > 9 && dozVal4 < 100) {
    myGLCD.printNumI(dozVal4,  dosval4[0] + 24, dosval4[1] + 5);
  }
  else {
    myGLCD.printNumI(dozVal4,  dosval4[0] + 40, dosval4[1] + 5);
  }

  myGLCD.setFont(RusFont2);
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[121])));      // ТЕКСТ НА КНОПКАХ (Мл.)
  myGLCD.print(buffer,  dosval1[0] + 60, dosval1[1] + 16);
  myGLCD.print(buffer,  dosval1[0] - 25, dosval1[1] + 16);
  myGLCD.print(buffer,  dosval2[0] + 60, dosval2[1] + 16);
  myGLCD.print(buffer,  dosval2[0] - 25, dosval2[1] + 16);
  myGLCD.print(buffer,  dosval3[0] + 60, dosval3[1] + 16);
  myGLCD.print(buffer,  dosval3[0] - 25, dosval3[1] + 16);
  myGLCD.print(buffer,  dosval4[0] + 60, dosval4[1] + 16);
  myGLCD.print(buffer,  dosval4[0] - 25, dosval4[1] + 16);
  myGLCD.setFont(BigRusFont);
  myGLCD.print("U", dosval1[0] - 90, dosval1[1] + 8);
  myGLCD.print("U", dosval2[0] - 90, dosval2[1] + 8);
  myGLCD.print("U", dosval3[0] - 90, dosval3[1] + 8);
  myGLCD.print("U", dosval4[0] - 90, dosval4[1] + 8);

  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(dos1b[0], dos1b[1], dos1b[2], dos1b[3]);
  myGLCD.drawRoundRect(dos2b[0], dos2b[1], dos2b[2], dos2b[3]);
  myGLCD.drawRoundRect(dos3b[0], dos3b[1], dos3b[2], dos3b[3]);
  myGLCD.drawRoundRect(dos4b[0], dos4b[1], dos4b[2], dos4b[3]);
  setFont(SMALL, 255, 255, 0, 0, 0, 0);
  myGLCD.setFont(RusFont3);
  myGLCD.print("-1", dos1b[0] + 68, dos1b[1] - 10);  //-1
  myGLCD.print("-2", dos2b[0] + 68, dos2b[1] - 10);
  myGLCD.print("-3", dos3b[0] + 68, dos3b[1] - 10);
  myGLCD.print("-4", dos4b[0] + 68, dos4b[1] - 10);
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123])));
  myGLCD.print(buffer,  dos1b[0] + 13, dos1b[1] - 10); // дозатор
  myGLCD.print(buffer,  dos2b[0] + 13, dos2b[1] - 10);
  myGLCD.print(buffer,  dos3b[0] + 13, dos3b[1] - 10);
  myGLCD.print(buffer,  dos4b[0] + 13, dos4b[1] - 10);


  if (DOZTime1 == 0)                               //Время первой дозы дозатора 1
  { setFont(SMALL, 255, 0, 0, 0, 0, 0);
    myGLCD.setFont(RusFont2);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184])));
    myGLCD.print(buffer, dos1b[0] + 33, dos1b[1] + 9); // выкл
  }
  else
  { timeDispH = dozPump1H; timeDispM = dozPump1M;
    if (setTimeFormat == 0) {
      xTimeH = dosT1[0] + 3;
    }
    if (setTimeFormat == 1) {
      xTimeH = 16;
    }
    if ((timeDispH >= 0) && (timeDispH <= 11)) {
      AM_PM = 1;
    }
    else {
      AM_PM = 2;
    }
    yTime = dos1b[1] + 5; xColon = xTimeH + 32;
    xTimeM10 = xTimeH + 48; xTimeM1 = xTimeH + 64; xTimeAMPM = xTimeH + 96;
    timeCorrectFormat();
  }


  if (DOZTime2 == 0)                               //Время первой дозы дозатора 2
  { setFont(SMALL, 255, 0, 0, 0, 0, 0);
    myGLCD.setFont(RusFont2);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184])));
    myGLCD.print(buffer, dos2b[0] + 33, dos2b[1] + 9);
  }

  else
  {
    timeDispH = dozPump2H; timeDispM = dozPump2M;
    if (setTimeFormat == 0) {
      xTimeH = dosT2[0] + 3;
    }
    if (setTimeFormat == 1) {
      xTimeH = 176;
    }
    if ((timeDispH >= 0) && (timeDispH <= 11)) {
      AM_PM = 1;
    }
    else {
      AM_PM = 2;
    }
    yTime = dos2b[1] + 5; xColon = xTimeH + 32;
    xTimeM10 = xTimeH + 48; xTimeM1 = xTimeH + 64; xTimeAMPM = xTimeH + 96;
    timeCorrectFormat();
  }

  if (DOZTime3 == 0)                               //Время первой дозы дозатора 3
  { setFont(SMALL, 255, 0, 0, 0, 0, 0);
    myGLCD.setFont(RusFont2);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184])));
    myGLCD.print(buffer, dos3b[0] + 33, dos3b[1] + 9);
  }

  else
  { timeDispH = dozPump3H; timeDispM = dozPump3M;
    if (setTimeFormat == 0) {
      xTimeH = dosT3[0] + 3;
    }
    if (setTimeFormat == 1) {
      xTimeH = 16;
    }
    if ((timeDispH >= 0) && (timeDispH <= 11)) {
      AM_PM = 1;
    }
    else {
      AM_PM = 2;
    }
    yTime = dos3b[1] + 5; xColon = xTimeH + 32;
    xTimeM10 = xTimeH + 48; xTimeM1 = xTimeH + 64; xTimeAMPM = xTimeH + 96;
    timeCorrectFormat();
  }


  if (DOZTime4 == 0)                               //Время первой дозы дозатора 4
  { setFont(SMALL, 255, 0, 0, 0, 0, 0);
    myGLCD.setFont(RusFont2);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184])));
    myGLCD.print(buffer, dos4b[0] + 33, dos4b[1] + 9);
  }

  else
  { timeDispH = dozPump4H; timeDispM = dozPump4M;
    if (setTimeFormat == 0) {
      xTimeH = dosT4[0] + 3;
    }
    if (setTimeFormat == 1) {
      xTimeH = 176;
    }
    if ((timeDispH >= 0) && (timeDispH <= 11)) {
      AM_PM = 1;
    }
    else {
      AM_PM = 2;
    }
    yTime = dos4b[1] + 5; xColon = xTimeH + 32;
    xTimeM10 = xTimeH + 48; xTimeM1 = xTimeH + 64; xTimeAMPM = xTimeH + 96;
    timeCorrectFormat();
  }


}

/**************************** ЭКРАН ВКЛ-ВЫКЛ ДОЗАТОРОВ******************************/

void dosingTimeOnOff()
{
  if ((dozTime == 1) && (DOZTime1 == 1))
  { myGLCD.setColor(0, 255, 0);
    myGLCD.fillRoundRect(70, 150, 250, 170);
    setFont(SMALL, 0, 0, 0, 0, 255, 0);
    myGLCD.setFont(RusFont3);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123])));
    myGLCD.print(buffer, 94, 157);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[128])));
    myGLCD.print(buffer, 94 + 120, 157);
  }

  if ((dozTime == 1) && (DOZTime1 == 0))
  { myGLCD.setColor(255, 0, 0);
    myGLCD.fillRoundRect(70, 150, 250, 170);
    setFont(SMALL, 255, 255, 255, 255, 0, 0);
    myGLCD.setFont(RusFont3);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123])));
    myGLCD.print(buffer, 90, 157);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184])));
    myGLCD.print(buffer, 90 + 120, 157);
  }

  if ((dozTime == 2) && (DOZTime2 == 1))
  { myGLCD.setColor(0, 255, 0);
    myGLCD.fillRoundRect(70, 150, 250, 170);
    setFont(SMALL, 0, 0, 0, 0, 255, 0);
    myGLCD.setFont(RusFont3);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123])));
    myGLCD.print(buffer, 94, 157);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[128])));
    myGLCD.print(buffer, 94 + 120, 157);
  }

  if ((dozTime == 2) && (DOZTime2 == 0))
  { myGLCD.setColor(255, 0, 0);
    myGLCD.fillRoundRect(70, 150, 250, 170);
    setFont(SMALL, 255, 255, 255, 255, 0, 0);
    myGLCD.setFont(RusFont3);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123])));
    myGLCD.print(buffer, 90, 157);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184])));
    myGLCD.print(buffer, 90 + 120, 157);
  }

  if ((dozTime == 3) && (DOZTime3 == 1))
  { myGLCD.setColor(0, 255, 0);
    myGLCD.fillRoundRect(70, 150, 250, 170);
    setFont(SMALL, 0, 0, 0, 0, 255, 0);
    myGLCD.setFont(RusFont3);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123])));
    myGLCD.print(buffer, 94, 157);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[128])));
    myGLCD.print(buffer, 94 + 120, 157);
  }

  if ((dozTime == 3) && (DOZTime3 == 0))
  { myGLCD.setColor(255, 0, 0);
    myGLCD.fillRoundRect(70, 150, 250, 170);
    setFont(SMALL, 255, 255, 255, 255, 0, 0);
    myGLCD.setFont(RusFont3);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123])));
    myGLCD.print(buffer, 90, 157);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184])));
    myGLCD.print(buffer, 90 + 120, 157);
  }

  if ((dozTime == 4) && (DOZTime4 == 1))
  { myGLCD.setColor(0, 255, 0);
    myGLCD.fillRoundRect(70, 150, 250, 170);
    setFont(SMALL, 0, 0, 0, 0, 255, 0);
    myGLCD.setFont(RusFont3);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123])));
    myGLCD.print(buffer, 94, 157);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[128])));
    myGLCD.print(buffer, 94 + 120, 157);
  }

  if ((dozTime == 4) && (DOZTime4 == 0))
  { myGLCD.setColor(255, 0, 0);
    myGLCD.fillRoundRect(70, 150, 250, 170);
    setFont(SMALL, 255, 255, 255, 255, 0, 0);
    myGLCD.setFont(RusFont3);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[123])));
    myGLCD.print(buffer, 90, 157);
    strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[184])));
    myGLCD.print(buffer, 90 + 120, 157);
  }

  myGLCD.setColor(255, 255, 255);
  myGLCD.drawRoundRect(70, 150, 250, 170);
}




/***********************************КАЛИБРОВКА ДОЗАТОРОВ **************************************************/
void doscalibrateScreen() {
  PrintStringIndex = 8;
  printHeader ();
  myGLCD.setColor(64, 64, 64);
  myGLCD.drawRect(1, 196, 319, 194);
  myGLCD.drawRect(1, 15, 319, 193);

  myGLCD.setColor(255, 255, 0);
  myGLCD.drawRoundRect(5, 40, 315, 150);
  myGLCD.drawRoundRect(dos4b[0] + 5, dos4b[1] - 6, dos4b[2] + 52, dos4b[3] - 4); //КНОПКА КАЛИБРОВКИ
  setFont(SMALL, 0, 255, 0, 0, 0, 0);
  myGLCD.setFont(RusFont3);
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[120])));
  myGLCD.print(buffer,  dos1b[0] + 10, dos1b[1] - 3);  //время работы дозатора
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[118])));
  myGLCD.print(buffer,  dos1b[0] + 248, dos1b[1] - 3);  //Сек
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[119])));
  myGLCD.print(buffer,  valUP[0] - 155, valUP[1] + 12);  //объем дозы (мл.)
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[115])));
  myGLCD.print(buffer,  numUP[0] - 155, numUP[1] + 12);  //Количество доз
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[116])));
  myGLCD.print(buffer,  intUP[0] - 155, intUP[1] + 12);  //Интервал (Часы)
  setFont(SMALL, 255, 0, 0, 0, 0, 0);






  myGLCD.setFont(RusFont3);
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[122])));
  myGLCD.print(buffer,  calUP[0] - 130, calUP[1] + 13);  //КАЛИБРОВКА





  setFont(SMALL, 0, 255, 0, 0, 0, 0);
  myGLCD.setFont(BigRusFont);
  //  myGLCD.setFont(RusFont3);

  if (CalMode == 1) {
    myGLCD.print("U", dosval1[0] - 105, dosval1[1] - 5);
    DosVal1(); dosSec1();
  }
  if (CalMode == 2) {
    myGLCD.print("U", dosval1[0] - 105, dosval1[1] - 5);
    DosVal2(); dosSec2();
  }
  if (CalMode == 3) {
    myGLCD.print("U", dosval1[0] - 105, dosval1[1] - 5);
    DosVal3(); dosSec3();
  }
  if (CalMode == 4) {
    myGLCD.print("U", dosval1[0] - 105, dosval1[1] - 5);
    DosVal4(); dosSec4();
  }


  PrintBSC3b();
}

/********************************* Экраны калибровки дозаторов*******************************************/

void dosSec1() {
  DosSec1 = (dozVal1 * 10 / dozCal1);
  setFont(SMALL, 0, 255, 0, 0, 0, 0);
  myGLCD.setFont(BigRusFont);
  myGLCD.printNumI(numDoz1,  dosval1[0] - 120, dosval1[1] - 5);
  if (DosSec1 / numDoz1 > 99) {
    myGLCD.printNumI(DosSec1 / numDoz1,  dosval1[0] - 89, dosval1[1] - 5);
  }
  else {
    if (DosSec1 / numDoz1 > 9 && DosSec1 / numDoz1 < 100) {
      myGLCD.printNumI(DosSec1 / numDoz1,  dosval1[0] - 73, dosval1[1] - 5);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(dosval1[0] - 90, dosval1[1] - 5, dosval1[0] - 74, dosval1[1] + 12);
    }
    else {
      myGLCD.printNumI(DosSec1 / numDoz1,  dosval1[0] - 57, dosval1[1] - 5);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(dosval1[0] - 73, dosval1[1] - 5, dosval1[0] - 60, dosval1[1] + 12);
    }
  }

  setFont(SMALL, 255, 255, 0, 0, 0, 0);
  myGLCD.setFont(BigRusFont);
  if (DosSec1 > 99) {
    myGLCD.printNumI(DosSec1,  dosval1[0] - 29, dosval1[1] - 5);
  }
  else {
    if (DosSec1 > 9 && DosSec1 < 100) {
      myGLCD.printNumI(DosSec1,  dosval1[0] - 13, dosval1[1] - 5);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(dosval1[0] - 30, dosval1[1] - 5, dosval1[0] - 14, dosval1[1] + 12);
    }
    else {
      myGLCD.printNumI(DosSec1,  dosval1[0] + 3, dosval1[1] - 5);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(dosval1[0] - 13, dosval1[1] - 5, dosval1[0], dosval1[1] + 12);
    }
  }
  setFont(SMALL, 255, 255, 255, 224, 0, 224);
}

void dosSec2() {
  DosSec2 = (dozVal2 * 10 / dozCal2);
  setFont(SMALL, 0, 255, 0, 0, 0, 0);
  myGLCD.setFont(BigRusFont);
  myGLCD.printNumI(numDoz2,  dosval1[0] - 120, dosval1[1] - 5);
  if (DosSec2 / numDoz2 > 99) {
    myGLCD.printNumI(DosSec2 / numDoz2,  dosval1[0] - 89, dosval1[1] - 5);
  }
  else {
    if (DosSec2 / numDoz2 > 9 && DosSec2 / numDoz2 < 100) {
      myGLCD.printNumI(DosSec2 / numDoz2,  dosval1[0] - 73, dosval1[1] - 5);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(dosval1[0] - 90, dosval1[1] - 5, dosval1[0] - 74, dosval1[1] + 12);
    }
    else {
      myGLCD.printNumI(DosSec2 / numDoz2,  dosval1[0] - 57, dosval1[1] - 5);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(dosval1[0] - 73, dosval1[1] - 5, dosval1[0] - 60, dosval1[1] + 12);
    }
  }

  setFont(SMALL, 255, 255, 0, 0, 0, 0);
  myGLCD.setFont(BigRusFont);
  if (DosSec2 > 99) {
    myGLCD.printNumI(DosSec2,  dosval1[0] - 29, dosval1[1] - 5);
  }
  else {
    if (DosSec2 > 9 && DosSec2 < 100) {
      myGLCD.printNumI(DosSec2,  dosval1[0] - 13, dosval1[1] - 5);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(dosval1[0] - 30, dosval1[1] - 5, dosval1[0] - 14, dosval1[1] + 12);
    }
    else {
      myGLCD.printNumI(DosSec2,  dosval1[0] + 3, dosval1[1] - 5);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(dosval1[0] - 13, dosval1[1] - 5, dosval1[0], dosval1[1] + 12);
    }
  }
  setFont(SMALL, 255, 255, 255, 224, 0, 224);
}

void dosSec3() {
  DosSec3 = (dozVal3 * 10 / dozCal3);
  setFont(SMALL, 0, 255, 0, 0, 0, 0);
  myGLCD.setFont(BigRusFont);
  myGLCD.printNumI(numDoz3,  dosval1[0] - 120, dosval1[1] - 5);
  if (DosSec3 / numDoz3 > 99) {
    myGLCD.printNumI(DosSec3 / numDoz3,  dosval1[0] - 89, dosval1[1] - 5);
  }
  else {
    if (DosSec3 / numDoz3 > 9 && DosSec3 / numDoz3 < 100) {
      myGLCD.printNumI(DosSec3 / numDoz3,  dosval1[0] - 73, dosval1[1] - 5);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(dosval1[0] - 90, dosval1[1] - 5, dosval1[0] - 74, dosval1[1] + 12);
    }
    else {
      myGLCD.printNumI(DosSec3 / numDoz3,  dosval1[0] - 57, dosval1[1] - 5);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(dosval1[0] - 73, dosval1[1] - 5, dosval1[0] - 60, dosval1[1] + 12);
    }
  }

  setFont(SMALL, 255, 255, 0, 0, 0, 0);
  myGLCD.setFont(BigRusFont);
  if (DosSec3 > 99) {
    myGLCD.printNumI(DosSec3,  dosval1[0] - 29, dosval1[1] - 5);
  }
  else {
    if (DosSec3 > 9 && DosSec3 < 100) {
      myGLCD.printNumI(DosSec3,  dosval1[0] - 13, dosval1[1] - 5);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(dosval1[0] - 30, dosval1[1] - 5, dosval1[0] - 14, dosval1[1] + 12);
    }
    else {
      myGLCD.printNumI(DosSec3,  dosval1[0] + 3, dosval1[1] - 5);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(dosval1[0] - 13, dosval1[1] - 5, dosval1[0], dosval1[1] + 12);
    }
  }
  setFont(SMALL, 255, 255, 255, 224, 0, 224);
}

void dosSec4() {
  DosSec4 = (dozVal4 * 10 / dozCal4);
  setFont(SMALL, 0, 255, 0, 0, 0, 0);
  myGLCD.setFont(BigRusFont);
  myGLCD.printNumI(numDoz4,  dosval1[0] - 120, dosval1[1] - 5);
  if (DosSec4 / numDoz4 > 99) {
    myGLCD.printNumI(DosSec4 / numDoz4,  dosval1[0] - 89, dosval1[1] - 5);
  }
  else {
    if (DosSec4 / numDoz4 > 9 && DosSec4 / numDoz4 < 100) {
      myGLCD.printNumI(DosSec4 / numDoz4,  dosval1[0] - 73, dosval1[1] - 5);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(dosval1[0] - 90, dosval1[1] - 5, dosval1[0] - 74, dosval1[1] + 12);
    }
    else {
      myGLCD.printNumI(DosSec4 / numDoz4,  dosval1[0] - 57, dosval1[1] - 5);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(dosval1[0] - 73, dosval1[1] - 5, dosval1[0] - 60, dosval1[1] + 12);
    }
  }

  setFont(SMALL, 255, 255, 0, 0, 0, 0);
  myGLCD.setFont(BigRusFont);
  if (DosSec4 > 99) {
    myGLCD.printNumI(DosSec4,  dosval1[0] - 29, dosval1[1] - 5);
  }
  else {
    if (DosSec4 > 9 && DosSec4 < 100) {
      myGLCD.printNumI(DosSec4,  dosval1[0] - 13, dosval1[1] - 5);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(dosval1[0] - 30, dosval1[1] - 5, dosval1[0] - 14, dosval1[1] + 12);
    }
    else {
      myGLCD.printNumI(DosSec4,  dosval1[0] + 3, dosval1[1] - 5);
      myGLCD.setColor(0, 0, 0);
      myGLCD.fillRect(dosval1[0] - 13, dosval1[1] - 5, dosval1[0], dosval1[1] + 12);
    }
  }
  setFont(SMALL, 255, 255, 255, 224, 0, 224);
}


void DosVal1() {
  dozVal1 = PlusMinusCountI (false, false, valUP[0], valUP[1], 100, numDoz1, 999, numDoz1, dozVal1);     // объем дозы 1 дозатора
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }

  numDoz1 = PlusMinusCountI (false, false, numUP[0], numUP[1], 100, 1, 4, 1, numDoz1);     // Количество доз 1 дозатора
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }

  intDoz1 = PlusMinusCountI (false, false, intUP[0], intUP[1], 100, 1, 6, 1, intDoz1);     // Интервал между дозами (часы) 1 дозатора
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }

  dozCal1 = PlusMinusCountI (false, false, calUP[0], calUP[1], 100, 1, 999, 1, dozCal1);     // количество за 10 секунд
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }
}


void DosVal2() {
  dozVal2 = PlusMinusCountI (false, false, valUP[0], valUP[1], 100, numDoz2, 999, numDoz2, dozVal2);     // объем дозы 2 дозатора
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }

  numDoz2 = PlusMinusCountI (false, false, numUP[0], numUP[1], 100, 1, 4, 1, numDoz2);     // Количество доз 2 дозатора
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }

  intDoz2 = PlusMinusCountI (false, false, intUP[0], intUP[1], 100, 1, 6, 1, intDoz2);     // Интервал между дозами (часы) 2 дозатора
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }

  dozCal2 = PlusMinusCountI (false, false, calUP[0], calUP[1], 100, 1, 999, 1, dozCal2);     // количество за 10 секунд
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }
}


void DosVal3() {
  dozVal3 = PlusMinusCountI (false, false, valUP[0], valUP[1], 100, numDoz3, 999, numDoz3, dozVal3);     // объем дозы 3 дозатора
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }

  numDoz3 = PlusMinusCountI (false, false, numUP[0], numUP[1], 100, 1, 4, 1, numDoz3);     // Количество доз 3 дозатора
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }

  intDoz3 = PlusMinusCountI (false, false, intUP[0], intUP[1], 100, 1, 6, 1, intDoz3);     // Интервал между дозами (часы) 3 дозатора
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }

  dozCal3 = PlusMinusCountI (false, false, calUP[0], calUP[1], 100, 1, 999, 1, dozCal3);     // количество за 10 секунд
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }
}


void DosVal4() {
  dozVal4 = PlusMinusCountI (false, false, valUP[0], valUP[1], 100, numDoz4, 999, numDoz4, dozVal4);     // объем дозы 4 дозатора
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }

  numDoz4 = PlusMinusCountI (false, false, numUP[0], numUP[1], 100, 1, 4, 1, numDoz4);     // Количество доз 4 дозатора
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }

  intDoz4 = PlusMinusCountI (false, false, intUP[0], intUP[1], 100, 1, 6, 1, intDoz4);     // Интервал между дозами (часы) 4 дозатора
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }

  dozCal4 = PlusMinusCountI (false, false, calUP[0], calUP[1], 100, 1, 999, 1, dozCal4);     // количество за 10 секунд
  if (PlsMnsPress == true) {
    PlsMnsPress = false;
  }
}

/***********************ВЫХОД КАЛИБРОВКИ ПРОИЗВОДИТЕЛЬНОСТИ ДОЗАТОРОВ************************/
void doscalibrate() {

  if (CalMode == 1) {
    RTC.getTime(); sec1 = ((RTC.minute * 60) + RTC.second + 10); Doscalibrate1 = 1;
    digitalWrite(pump1, HIGH);
  }

  if (CalMode == 2) {
    RTC.getTime(); sec2 = ((RTC.minute * 60) + RTC.second + 10); Doscalibrate2 = 1;
    digitalWrite(pump2, HIGH);
  }

  if (CalMode == 3) {
    RTC.getTime(); sec3 = ((RTC.minute * 60) + RTC.second + 10); Doscalibrate3 = 1;
    digitalWrite(pump3, HIGH);
  }

  if (CalMode == 4) {
    RTC.getTime(); sec4 = ((RTC.minute * 60) + RTC.second + 10); Doscalibrate4 = 1;
    digitalWrite(pump4, HIGH);
  }

}
/*******************************************РАСЧЕТ ВРЕМЕНИ ДОЗ*************************************************/
void caldosetime() {
  if (numDoz1 == 4) {
    shiftH12 = (dozPump1H + intDoz1); if (shiftH12 >= 24) {
      shiftH12 = shiftH12 - 24;
    }
    shiftH13 = (dozPump1H + (intDoz1 * 2)); if (shiftH13 >= 24) {
      shiftH13 = shiftH13 - 24;
    }
    shiftH14 = (dozPump1H + (intDoz1 * 3)); if (shiftH14 >= 24) {
      shiftH14 = shiftH14 - 24;
    }
  }
  if (numDoz1 == 3) {
    shiftH12 = (dozPump1H + intDoz1); if (shiftH12 >= 24) {
      shiftH12 = shiftH12 - 24;
    }
    shiftH13 = (dozPump1H + (intDoz1 * 2)); if (shiftH13 >= 24) {
      shiftH13 = shiftH13 - 24;
    }
    shiftH14 = 25;
  }
  if (numDoz1 == 2) {
    shiftH12 = (dozPump1H + intDoz1); if (shiftH12 >= 24) {
      shiftH12 = shiftH12 - 24;
    }
    shiftH13 = 25; shiftH14 = 25;
  }
  if (numDoz1 == 1) {
    shiftH12 = 25;
    shiftH13 = 25;
    shiftH14 = 25;
  }

  if (numDoz2 == 4) {
    shiftH22 = (dozPump2H + intDoz2); if (shiftH22 >= 24) {
      shiftH22 = shiftH22 - 24;
    }
    shiftH23 = (dozPump2H + (intDoz2 * 2)); if (shiftH23 >= 24) {
      shiftH23 = shiftH23 - 24;
    }
    shiftH24 = (dozPump2H + (intDoz2 * 3)); if (shiftH24 >= 24) {
      shiftH24 = shiftH24 - 24;
    }
  }
  if (numDoz2 == 3) {
    shiftH22 = (dozPump2H + intDoz2); if (shiftH22 >= 24) {
      shiftH22 = shiftH22 - 24;
    }
    shiftH23 = (dozPump2H + (intDoz2 * 2)); if (shiftH23 >= 24) {
      shiftH23 = shiftH23 - 24;
    }
    shiftH24 = 25;
  }
  if (numDoz2 == 2) {
    shiftH22 = (dozPump2H + intDoz2); if (shiftH22 >= 24) {
      shiftH22 = shiftH22 - 24;
    }
    shiftH23 = 25; shiftH24 = 25;
  }
  if (numDoz2 == 1) {
    shiftH22 = 25;
    shiftH23 = 25;
    shiftH24 = 25;
  }

  if (numDoz3 == 4) {
    shiftH32 = (dozPump3H + intDoz3);  if (shiftH32 >= 24) {
      shiftH32 = shiftH32 - 24;
    }
    shiftH33 = (dozPump3H + (intDoz3 * 2)); if (shiftH33 >= 24) {
      shiftH33 = shiftH33 - 24;
    }
    shiftH34 = (dozPump3H + (intDoz3 * 3)); if (shiftH34 >= 24) {
      shiftH34 = shiftH34 - 24;
    }
  }
  if (numDoz3 == 3) {
    shiftH32 = (dozPump3H + intDoz3); if (shiftH32 >= 24) {
      shiftH32 = shiftH32 - 24;
    }
    shiftH33 = (dozPump3H + (intDoz3 * 2)); if (shiftH33 >= 24) {
      shiftH33 = shiftH33 - 24;
    }
    shiftH34 = 25;
  }
  if (numDoz3 == 2) {
    shiftH32 = (dozPump3H + intDoz3); if (shiftH32 >= 24) {
      shiftH32 = shiftH32 - 24;
    }
    shiftH33 = 25; shiftH34 = 25;
  }
  if (numDoz3 == 1) {
    shiftH32 = 25;
    shiftH33 = 25;
    shiftH34 = 25;
  }

  if (numDoz4 == 4) {
    shiftH42 = (dozPump4H + intDoz4); if (shiftH42 >= 24) {
      shiftH42 = shiftH42 - 24;
    }
    shiftH43 = (dozPump4H + (intDoz4 * 2)); if (shiftH43 >= 24) {
      shiftH43 = shiftH43 - 24;
    }
    shiftH44 = (dozPump4H + (intDoz4 * 3)); if (shiftH44 >= 24) {
      shiftH44 = shiftH44 - 24;
    }
  }
  if (numDoz4 == 3) {
    shiftH42 = (dozPump4H + intDoz4); if (shiftH42 >= 24) {
      shiftH42 = shiftH42 - 24;
    }
    shiftH43 = (dozPump4H + (intDoz4 * 2)); if (shiftH43 >= 24) {
      shiftH43 = shiftH43 - 24;
    }
    shiftH44 = 25;
  }
  if (numDoz4 == 2) {
    shiftH42 = (dozPump4H + intDoz4); if (shiftH42 >= 24) {
      shiftH42 = shiftH42 - 24;
    }
    shiftH43 = 25; shiftH44 = 25;
  }
  if (numDoz4 == 1) {
    shiftH42 = 25;
    shiftH43 = 25;
    shiftH44 = 25;
  }

}

/******************************************* ВЫХОД ДОЗАТОРА ***************************************************/
void dosingTimeOutput() {
  RTC.getTime();
  sec = (RTC.minute * 60) + RTC.second;


  if ((sec >= sec1) && (Doscalibrate1 == 1)) {
    Doscalibrate1 = 0; digitalWrite(pump1, LOW);

    if (dispScreen == 10) {
      setFont(SMALL, 255, 255, 0, 0, 0, 0);
      myGLCD.setFont(RusFont3);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[117])));
      myGLCD.print(buffer,  calUP[0] - 150, calUP[1] + 13);
    }
  }

  if ((sec >= sec2) && (Doscalibrate2 == 1)) {
    Doscalibrate2 = 0; digitalWrite(pump2, LOW);

    if (dispScreen == 10) {
      setFont(SMALL, 255, 255, 0, 0, 0, 0);
      myGLCD.setFont(RusFont3);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[117])));
      myGLCD.print(buffer,  calUP[0] - 150, calUP[1] + 13);
    }
  }

  if ((sec >= sec3) && (Doscalibrate3 == 1)) {
    Doscalibrate3 = 0; digitalWrite(pump3, LOW);

    if (dispScreen == 10) {
      setFont(SMALL, 255, 255, 0, 0, 0, 0);
      myGLCD.setFont(RusFont3);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[117])));
      myGLCD.print(buffer,  calUP[0] - 150, calUP[1] + 13);
    }
  }

  if ((sec >= sec4) && (Doscalibrate4 == 1)) {
    Doscalibrate4 = 0; digitalWrite(pump4, LOW);

    if (dispScreen == 10) {
      setFont(SMALL, 255, 255, 0, 0, 0, 0);
      myGLCD.setFont(RusFont3);
      strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[117])));
      myGLCD.print(buffer,  calUP[0] - 150, calUP[1] + 13);
    }
  }

  if ((DOZTime1 == 1) && ((dozPump1H == RTC.hour) || (shiftH12 == RTC.hour) || ( shiftH13 == RTC.hour) || (shiftH14 == RTC.hour))
      && (dozPump1M == RTC.minute) && (RTC.second >= 0 && RTC.second < 1))
  { RTC.getTime(); sec1 = ((RTC.minute * 60) + RTC.second + (DosSec1 / numDoz1)); Doscalibrate1 = 1; // Длительность дозы
    digitalWrite(pump1, HIGH);
  }

  if ((DOZTime2 == 1) && ((dozPump2H == RTC.hour) || (shiftH22 == RTC.hour) || ( shiftH23 == RTC.hour) || (shiftH24 == RTC.hour))
      && (dozPump2M == RTC.minute) && (RTC.second >= 0 && RTC.second < 1))
  { RTC.getTime(); sec2 = ((RTC.minute * 60) + RTC.second + (DosSec2 / numDoz2)); Doscalibrate2 = 1;
    digitalWrite(pump2, HIGH);
  }

  if ((DOZTime3 == 1) && ((dozPump3H == RTC.hour) || (shiftH32 == RTC.hour) || ( shiftH33 == RTC.hour) || (shiftH34 == RTC.hour))
      && (dozPump3M == RTC.minute) && (RTC.second >= 0 && RTC.second < 1))
  { RTC.getTime(); sec3 = ((RTC.minute * 60) + RTC.second + (DosSec3 / numDoz3)); Doscalibrate3 = 1;
    digitalWrite(pump3, HIGH);
  }

  if ((DOZTime4 == 1) && ((dozPump4H == RTC.hour) || (shiftH42 == RTC.hour) || ( shiftH43 == RTC.hour) || (shiftH44 == RTC.hour))
      && (dozPump4M == RTC.minute) && (RTC.second >= 0 && RTC.second < 1))
  { RTC.getTime(); sec4 = ((RTC.minute * 60) + RTC.second + (DosSec4 / numDoz4)); Doscalibrate4 = 1;
    digitalWrite(pump4, HIGH);
  }
}

/*********************** END OF AUTOMATIC DOSER SETTINGS SCREEN **********************/


/***** УСТАНОВКА ВРЕМЕНИ ДОЗАТОРОВ****** SET AUTOMATIC DOSER TIMES SCREEN *****dispScreen = 37 */

void setDoserTimesScreen(boolean refreshAll = true) {

  PrintStringIndex = 26;  printHeader ();

  if (refreshAll)
  {
    if (dozTime == 1)
    {
      rtcSetMin = dozPump1M;
      rtcSetHr = dozPump1H;
    }
    if (dozTime == 2)
    {
      rtcSetMin = dozPump2M;
      rtcSetHr = dozPump2H;
    }
    if (dozTime == 3)
    {
      rtcSetMin = dozPump3M;
      rtcSetHr = dozPump3H;
    }
    if (dozTime == 4)
    {
      rtcSetMin = dozPump4M;
      rtcSetHr = dozPump4H;
    }

    myGLCD.setColor(64, 64, 64);
    myGLCD.drawRect(0, 196, 319, 194);

    PrintBSC3b();

    dosingTimeOnOff();

    drawUpButtonSlide(houP[0], houP[1]);                //hour up
    drawUpButtonSlide(minP[0], minP[1]);                //min up
    drawDownButtonSlide(houM[0], houM[1]);              //hour down
    drawDownButtonSlide(minM[0], minM[1]);              //min down
    if (setTimeFormat == 1)
    { drawUpButtonSlide(ampmP[0], ampmP[1]);          //AM/PM up
      drawDownButtonSlide(ampmM[0], ampmM[1]);
    }       //AM/PM down
  }

  timeDispH = rtcSetHr; timeDispM = rtcSetMin;
  xTimeH = 107; yTime = 68;  xColon = xTimeH + 42;
  xTimeM10 = xTimeH + 70; xTimeM1 = xTimeH + 86; xTimeAMPM = xTimeH + 155;
  timeChange();


}

/********************** END OF SET AUTOMATIC DOSER TIMES SCREEN **********************/





/***** ОГРАНИЧЕНИЕ МОЩНОСТИ (РУЧНОЕ ДИММИРОВАНИЕ) **************************** dispScreen = 31 */
void SetDimm() {
  myGLCD.setColor(255, 240, 255);
  myGLCD.fillRect(35, 50, 284, 218);
  myGLCD.setColor(0, 0, 255);
  myGLCD.drawRoundRect(35 + 2, 50 + 2, 284 - 2, 218 - 2);

  myGLCD.setFont(RusFont1);
  myGLCD.setColor(0, 0, 0);
  myGLCD.setBackColor(255, 240, 255);
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[143])));
  myGLCD.print(buffer, CENTER, 74);         //  величина сигнала всех каналов от 100 %
  myGLCD.print(print_text[142], 225, 130);  // %

  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRoundRect(115, 102, 205, 167);
  myGLCD.setColor(0, 0, 255);
  myGLCD.drawRoundRect(115, 102, 205, 167);
  drawUpButton(175, 107);
  drawDownButton(175, 137);

  setFont(LARGE, 255, 255, 255, 0, 0, 0);
  TempsetLEDsDimPercentL = setLEDsDimPercentL;
  if (TempsetLEDsDimPercentL >= 10) {
    myGLCD.printNumI(TempsetLEDsDimPercentL, 129, 126);
  }
  else {
    myGLCD.printNumI(TempsetLEDsDimPercentL, 137, 126);
  }

  myGLCD.setColor(0, 255, 0);
  myGLCD.fillRoundRect(95, 185, 135, 205);
  setFont(SMALL, 0, 0, 0, 0, 255, 0);
  strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[17])));
  myGLCD.print(buffer, 107, 190);       // OK
  myGLCD.setColor(0, 0, 255);
  myGLCD.fillRoundRect(185, 185, 250, 205);
  setFont(SMALL, 255, 255, 255, 0, 0, 255);
  myGLCD.setFont(RusFont6);
  myGLCD.print(print_text[1], 185 + 9, 185 + 5); // CANCEL

  myGLCD.setColor(0, 0, 0);
  myGLCD.drawRoundRect(95, 185, 135, 205);
  myGLCD.drawRoundRect(185, 185, 250, 205);
}

/***** ЯРКОСТЬ ПОДСВЕТКИ ЭКРАНА **************************** dispScreen = 33 */
void LCDbrigh (boolean refreshAll = false) { // Яркость подсветки экрана
  byte drgb[] = {0, 0, 125};

  if (refreshAll) {

    PrintStringIndex = 13; printHeader (); // НАСТРОЙКА ЯРКОСТИ ЭКРАНА

    Menu_Text(51, 65, 30, 3, 8 );     // ЯРКОСТЬ ПОДСВЕТКИ ЭКРАНА
    Menu_Text(52, 65, 94, 3, 8 );     // УМЕНЬШИТЬ ЯРКОСТЬ НА 50%
    Menu_Text(53, 45, 140, 3, 8 );    // В ПЕРИОД
    Menu_Text(54, 49, 160, 3, 8 );    // ВРЕМЕНИ
    Menu_Text(55, 150, 130, 3, 8 );    // С
    Menu_Text(56, 146, 170, 3, 8 );    // ДО
    Menu_Text(57, 300, 130, 3, 8 );    // Ч.
    Menu_Text(57, 300, 170, 3, 8 );    // Ч.

    LCDbright = PlusMinusCountI (false, false, temM[0], temM[1], 150, 1, 100, 1, LCDbright);          //
    if (PlsMnsPress == true) {
      PlsMnsPress = false;
    }

    LowbrH = PlusMinusCountI (false, false, intUP[0], intUP[1], 80, 0, 23, 1, LowbrH);                //
    if (PlsMnsPress == true) {
      PlsMnsPress = false;
    }

    HightbrH = PlusMinusCountI (false, false, calUP[0], calUP[1], 80, 0, 23, 1, HightbrH);            //
    if (PlsMnsPress == true) {
      PlsMnsPress = false;
    }

    PrintBSC3b();
  }
}

//============================= Effacer la mémoire ==================================

void resetScreen() { // EEPROM Clear Sets all of the bytes of the EEPROM to 0.
  int i = 0; EEPROM.write(i, 0); resetFunc();
  //  EraseAllEEPROM_SaveDefault();
  //  clearScreen(); dispScreen=0; mainScreen(true);
}
//===============================================================================

/************************************ TOUCH SCREEN ОБРАБОТКА КАСАНИЙ ЭКРАНА ************************************/
void processMyTouch() {
  myTouch.read();
#ifdef Mirror_X
  x = 319 - myTouch.getX();
#else
  x = myTouch.getX();
#endif

#ifdef Mirror_Y
  y = 239 - myTouch.getY();
#else
  y = myTouch.getY();
#endif

  returnTimer = 0; screenSaverCounter = 0; //timegraph=0; // линия графиков

  if ((x >= canC[0]) && (x <= canC[2]) && (y >= canC[1]) && (y <= canC[3]) // press cancel
      && (dispScreen != 0) && (dispScreen != 4) && (dispScreen != 5) && (dispScreen != 6)
      && (dispScreen != 8) && (dispScreen != 9) && (dispScreen != 11) && (dispScreen != 13)
      && (dispScreen != 14) && (dispScreen != 15) && (dispScreen != 28) && (dispScreen != 29)
      && (dispScreen != 32) && (dispScreen != 35)) {
    waitForIt(canC[0], canC[1], canC[2], canC[3]);

    LEDtestTick = false;
    ReadFromEEPROM(); dispScreen = 0; clearScreen(); mainScreen(true);
    myGLCD.setColor(64, 64, 64);
    myGLCD.drawRect(0, 226, 319, 239);   // рамка
    myGLCD.setColor(30, 30, 30);
    myGLCD.fillRect(0, 226, 319, 239);  // Bottom
    titledate();
  } else

    if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3])  // press back
        && (dispScreen != 0) && (dispScreen != 1) && (dispScreen != 4) && (dispScreen != 5)
        && (dispScreen != 6) && (dispScreen != 8) && (dispScreen != 9) && (dispScreen != 10)
        && (dispScreen != 11) && (dispScreen != 13) && (dispScreen != 14) && (dispScreen != 15)
        && (dispScreen != 16) && (dispScreen != 17) && (dispScreen != 18) && (dispScreen != 20)
        && (dispScreen != 21) && (dispScreen != 22) && (dispScreen != 23) && (dispScreen != 24)
        && (dispScreen != 26) && (dispScreen != 27) && (dispScreen != 28) && (dispScreen != 29)
        && (dispScreen != 30) && (dispScreen != 31) && (dispScreen != 32) && (dispScreen != 33)
        && (dispScreen != 34) && (dispScreen != 35) && (dispScreen != 37) && (dispScreen != 39) && (dispScreen != 40)) {
      waitForIt(back[0], back[1], back[2], back[3]);
      LEDtestTick = false;
      dispScreen = 1; clearScreen(); menuScreen();
      myGLCD.setColor(30, 30, 30);
      myGLCD.fillRect(0, 226, 319, 239);   // нижняя область
      titledate();       // обновить дату
    } else {
      switch (dispScreen) {

        /************************************************ГЛАВНЫЙ ЭКРАН********************************************************/
        case 0: //------------- MAIN SCREEN (Press Any Key) ---------------


          if (x > 1 && x < 100 && y > 166 && y < 222) {
            dispScreen = 25;  // ручное управление таймерами
            clearScreen();
            graphonoff();
          }
          if (x > 1 && x < 164 && y > 92 && y < 160) {
            dispScreen = 1;  // главное меню (имена каналов)
            clearScreen();
            menuScreen();
          }
          if (x > 170 && x < 318 && y > 125 && y < 225) {
            dispScreen = 1;  // главное меню (температура)
            clearScreen();
            menuScreen();
          }
          if (x > 274 && x < 317 && y > 90 && y < 105) {
            PresetSwitch();  // пресеты
            _delay_ms(200);
          }
          if (x > 273 && x < 318 && y > 46 && y < 72) {
            FeedWaveCtrl_0 = true; fiveTillBackOn0 = 0; myGLCD.setColor(0, 0, 0);
            myGLCD.fillRect(273, 56, 310, 70); _delay_ms(200);
          } //Пауза для крмления
          if (x > 1 && x < 150 && y > 5 && y < 90) {
            dispScreen = 6; clearScreen(); myGLCD.clrScr(); // cлайдерное управление

            wwtled_out = map(wwtled_out, 0, 1985, 0, 100);   // new value 0-100%
            cwtled_out = map(cwtled_out, 0, 1985, 0, 100);
            rbled_out = map(rbled_out, 0, 1985, 0, 100);
            rled_out = map(rled_out, 0, 1985, 0, 100);
            uvled_out = map(uvled_out, 0, 1985, 0, 100);
            oLed_out = map(oLed_out, 0, 1985, 0, 100);
            gled_out = map(gled_out, 0, 1985, 0, 100);

            yWHT = wwtled_out; yBLU = cwtled_out; yRBL = rbled_out; yRED = rled_out ;
            yUVL = uvled_out; ySMP = oLed_out; yGRN = gled_out;

            testIndLedScreen(); colorLEDtest = true;
            myGLCD.setColor(30, 30, 30);
            myGLCD.fillRect(0, 226, 319, 239);  titledate();   // Нижний Bar (часы и дата)

          }

          break;
        case 1: //--------------------- Главное Меню -------------------------

          if ((x >= tanD[0]) && (x <= tanD[2])) {        // Первый столбец
            if ((y >= tanD[1]) && (y <= tanD[3])) {      // Нажатие утановки времени
              waitForIt(tanD[0], tanD[1], tanD[2], tanD[3]);
              if ((timeDispH >= 0) && (timeDispH <= 11))    // Время и дата
                _delay_ms(80);  clearScreen();
              dispScreen = 2;  clockScreen();
            }
            if ((y >= temC[1]) && (y <= temC[3])) {       // press Temp Control
              waitForIt(temC[0], temC[1], temC[2], temC[3]);
              dispScreen = 3; clearScreen(); tempScreen(true);
            }
            if ((y >= feed[1]) && (y <= feed[3])) {       // press Feeder
              waitForIt(feed[0], feed[1], feed[2], feed[3]);
              dispScreen = 38; clearFscreen(); autoFeederScreen();
            }
            if ((y >= gSet[1]) && (y <= gSet[3])) {        // press General Settings
              waitForIt(gSet[0], gSet[1], gSet[2], gSet[3]);
              dispScreen = 14; clearScreen(); generalSettingsScreen_1();
            }
          }


          if ((x >= tesT[0]) && (x <= tesT[2])) {        // Второй столбец
            if ((y >= tesT[1]) && (y <= tesT[3])) {      // Автотест
              waitForIt(tesT[0], tesT[1], tesT[2], tesT[3]);
              dispScreen = 5; clearScreen(); testArrayScreen(true);
            }
            if ((y >= ledChM[1]) && (y <= ledChM[3])) {   // Помпы течения
              waitForIt(ledChM[0], ledChM[1], ledChM[2], ledChM[3]);
              dispScreen = 12; clearFscreen(); WavePWMScreen();
            }
            if ((y >= Sector[1]) && (y <= Sector[3])) {       // Настройка яркости каналов по секторам времени
              waitForIt(Sector[0], Sector[1], Sector[2], Sector[3]);
              dispScreen = 4; clearScreen();
              testIndLedScreen2(true);          // test and control individual led
              colorLEDtest = true;
              bitClear(GlobalStatus1Byte, 3);     // clear bit for " Save changes before exit "
              bitClear(GlobalStatus1Byte, 2);
            }
            if ((y >= timday[1]) && (y <= timday[3])) {        // Суточные таймеры
              waitForIt(timday[0], timday[1], timday[2], timday[3]);
              dispScreen = 19; clearScreen(); TimerScreen();
            }
          }

          if ((x >= tesT2[0]) && (x <= tesT2[2])) {        // Третий столбец
            if ((y >= tesT2[1]) && (y <= tesT2[3])) {      // Графики каналов
              waitForIt(tesT2[0], tesT2[1], tesT2[2], tesT2[3]);
              dispScreen = 29; clearFscreen(); LEDtestTick = false; AllColourGraph();
            }
            if ((y >= PHset[1]) && (y <= PHset[3])) {       // Настройка PH
              waitForIt(PHset[0], PHset[1], PHset[2], PHset[3]);
              dispScreen = 11; clearScreen(); SetPH();
            }
            if ((y >= Preset[1]) && (y <= Preset[3])) {       // Настройка яркости каналов по цветам
              waitForIt(Preset[0], Preset[1], Preset[2], Preset[3]);
              dispScreen = 13; clearScreen();
              PresetLedScreen(true);             // preset led windows
              colorLEDtest = true;
            }
            if ((y >= Pumpset[1]) && (y <= Pumpset[3])) {        // Дозатор УДО
              waitForIt(Pumpset[0], Pumpset[1], Pumpset[2], Pumpset[3]);
              dispScreen = 36; clearScreen(); autoDoserScreen();
            }
          }

          if (x >= logW[0] && x <= logW[2] && y > logW[1] && y < logW[3]) {
            waitForIt(logW[0], logW[1], logW[2], logW[3]);     // график температуры воды за сутки
            dispScreen = 28; clearFscreen(); tempgScreen(); timedrawScreen();
          }

          if (x >= logH[0] && x <= logH[2] && y > logH[1] && y < logH[3]) {
            waitForIt(logH[0], logH[1], logH[2], logH[3]);   // график температуры радиатора за сутки
            dispScreen = 27; clearFscreen(); tFANScreen(); timedrawScreen();
          }

          break;
        case 2:  //--------------- CLOCK & DATE SETUP SCREEN -----------------
          if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) { // press SAVE
            waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
            RTC.stopClock();// останавливаем часы
            RTC.fillByHMS(rtcSetHr, rtcSetMin, 0); // "подкручиваем стрелки
            RTC.fillByYMD(rtcSetYr, rtcSetMon, rtcSetDy);
            RTC.setTime();// отправляем "подкрученное время" самому модулю
            RTC.startClock(); // и опять запускаем часы

            //  rtc.setTime(rtcSetHr, rtcSetMin, 0);
            //  rtc.setDate(rtcSetDy, rtcSetMon, rtcSetYr);
            myGLCD.setColor(30, 30, 30);
            myGLCD.fillRect(0, 226, 319, 239);  // Bottom
            dispScreen = 0; clearScreen(); mainScreen(true);
          } else {

            if ((y >= houU[1]) && (y <= houU[3])) {                   // FIRST ROW (TIME UP)
              if ((x >= houU[0]) && (x <= houU[2])) {                  // press hour up
                waitForIt(houU[0], houU[1], houU[2], houU[3]); rtcSetHr++;
                if (rtcSetHr >= 24) {
                  rtcSetHr = 0;
                }
                setFont(SMALL, 255, 0, 0, 0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170])));
                myGLCD.print(buffer, 23, 21);
              }    // 28 Changed!

              if ((x >= minU[0]) && (x <= minU[2])) {                  // press min up
                waitForIt(minU[0], minU[1], minU[2], minU[3]); rtcSetMin++;
                if (rtcSetMin >= 60) {
                  rtcSetMin = 0;
                }
                setFont(SMALL, 255, 0, 0, 0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170])));
                myGLCD.print(buffer, 23, 21);
              }
            } // Changed!

            if ((y >= houD[1]) && (y <= houD[3])) {                    // SECOND ROW (TIME DOWN)
              if ((x >= houD[0]) && (x <= houD[2])) {                 // press hour down
                waitForIt(houD[0], houD[1], houD[2], houD[3]); rtcSetHr--;
                if (rtcSetHr < 0) {
                  rtcSetHr = 23;
                }
                setFont(SMALL, 255, 0, 0, 0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170])));
                myGLCD.print(buffer, 23, 21);
              }    // Changed!

              if ((x >= minD[0]) && (x <= minD[2])) {                    // press min down
                waitForIt(minD[0], minD[1], minD[2], minD[3]); rtcSetMin--;
                if (rtcSetMin < 0) {
                  rtcSetMin = 59;
                }
                setFont(SMALL, 255, 0, 0, 0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170])));
                myGLCD.print(buffer, 23, 21);
              }
            }   // Changed!

            if ((y >= dayU[1]) && (y <= dayU[3])) {      // THIRD ROW (DATE UP)  DD/MM/YYYY Format
              if ((x >= dayU[0]) && (x <= dayU[2])) {     // press day up
                waitForIt(dayU[0], dayU[1], dayU[2], dayU[3]); rtcSetDy++;
                rtcSetDy = validateDate(rtcSetDy, rtcSetMon, rtcSetYr); //calendar();
                setFont(SMALL, 255, 0, 0, 0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170])));
                myGLCD.print(buffer, 23, 114);
              }    // 118 Changed!

              if ((x >= monU[0]) && (x <= monU[2])) {           // press месяц up
                waitForIt(monU[0], monU[1], monU[2], monU[3]); rtcSetMon++; // RTC.month++;
                if (rtcSetMon > 12) {
                  rtcSetMon = 1; // месяцы от 1 до 12
                }
                //if (RTC.month>12) {RTC.month=1;}
                rtcSetDy = validateDateForMonth(rtcSetDy, rtcSetMon, rtcSetYr); //calendar();
                setFont(SMALL, 255, 0, 0, 0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170])));
                myGLCD.print(buffer, 23, 114);
              }     // Changed!

              if ((x >= yeaU[0]) && (x <= yeaU[2])) {           // press year up
                waitForIt(yeaU[0], yeaU[1], yeaU[2], yeaU[3]); rtcSetYr++;
                rtcSetDy = validateDateForMonth(rtcSetDy, rtcSetMon, rtcSetYr); //calendar();
                setFont(SMALL, 255, 0, 0, 0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170])));
                myGLCD.print(buffer, 23, 114);
              }      // Changed!

            }
            if ((y >= dayD[1]) && (y <= dayD[3])) {      // FOURTH ROW (DATE DOWN)   DD/MM/YYYY Format
              if ((x >= dayD[0]) && (x <= dayD[2])) {    // press day down
                waitForIt(dayD[0], dayD[1], dayD[2], dayD[3]); rtcSetDy--;
                rtcSetDy = validateDate(rtcSetDy, rtcSetMon, rtcSetYr); //calendar();
                setFont(SMALL, 255, 0, 0, 0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170])));
                myGLCD.print(buffer, 23, 114);
              }   // Changed!

              if ((x >= monD[0]) && (x <= monD[2])) {            // press месяц down
                waitForIt(monD[0], monD[1], monD[2], monD[3]); rtcSetMon--; // RTC.month--;
                if (rtcSetMon < 1) {
                  rtcSetMon = 12; // месяцы от 1 до 12
                }
                //if (RTC.month<1) {RTC.month=12;}

                rtcSetDy = validateDateForMonth(rtcSetDy, rtcSetMon, rtcSetYr); //calendar();
                setFont(SMALL, 255, 0, 0, 0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170])));
                myGLCD.print(buffer, 23, 114);
              }    // Changed!

              if ((x >= yeaD[0]) && (x <= yeaD[2])) {                 // press year down
                waitForIt(yeaD[0], yeaD[1], yeaD[2], yeaD[3]); rtcSetYr--;
                rtcSetDy = validateDateForMonth(rtcSetDy, rtcSetMon, rtcSetYr); //calendar();
                setFont(SMALL, 255, 0, 0, 0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[170])));
                myGLCD.print(buffer, 23, 114);
              }      // Changed!

            }
            clockScreen(false);
          }

          break;
        case 3:  //------------------ TEMPERATURE CONTROL (Вода) ----------------------

          // Переход в экран лог файла
          if (x >= 13 && x <= 37 && y > 83 && y < 127) {
            waitForIt(13, 83, 37, 127);  // график температуры воды за сутки
            dispScreen = 28; clearFscreen(); tempgScreen(); timedrawScreen();
          } //timedraw();}

          if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) { // press SAVE
            waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);

            setTempC = temp2beS;
            offTempC = temp2beO;
            alarmTempC = temp2beA;
            dispScreen = 0; SaveTempToEEPROM(); clearScreen(); mainScreen(true);
          } else {

            temp2beS = PlusMinusCountF (true, true, temM[0], temM[1], 150, 10, 40, 0.1, temp2beS);     // desired temperature
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            }
            temp2beO = PlusMinusCountF (true, true, temM[0], offM[1], 150, 0.2, 5, 0.1, temp2beO);     // temperature accurancy
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            }
            temp2beA = PlusMinusCountF (true, true, temM[0], SoundAlarmTm[1], 150, 1, 10, 0.1, temp2beA);    // alarm temperature
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            }
          }
          break;

        case 4:  //----------------- TEST INDIVIDUAL LED COLOR SCREEN (sector) ------------------------
          TopRows == true;

          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3]) && (bitRead(GlobalStatus1Byte, 2) == false)) { // press back, exit to "Test option screen"
            waitForIt(back[0], back[1], back[2], back[3]);

            if (bitRead(GlobalStatus1Byte, 3) == false) {    // bit3 =0, no changes settings
              dispScreen = 1; clearFscreen(); menuScreen(); colorLEDtest = false;
              LED_levelo_output();
            }                    // return to current values led before exit to main
            else {
              colorLEDtest = true;
              bitSet(GlobalStatus1Byte, 2);              // for detect button Yes/No in to "save and exit" windows
              SaveAndExit();
            }
          }

          if ((x >= No[0]) && (x <= No[2]) && (y >= No[1]) && (y <= No[3]) && (bitRead(GlobalStatus1Byte, 2) == true)) { //press "NO", exit to menu without save,
            waitForIt(No[0], No[1], No[2], No[3]); LEDtestTick = false; dispScreen = 1; clearFscreen(); menuScreen();
            COLOR = 0; ReadLedFromEEPROM();
            bitClear(GlobalStatus1Byte, 3);             // bit 3 - changes setting (0 - no change, 1 -change )
            bitClear(GlobalStatus1Byte, 2);             // clear bit for " Save changes before exit "
            colorLEDtest = false; LED_levelo_output();
          } // return to current values led before exit to main

          if ((x >= Yes[0]) && (x <= Yes[2]) && (y >= Yes[1]) && (y <= Yes[3]) && (bitRead(GlobalStatus1Byte, 2) == true)) { //press "Yes" for save all changes and exit
            waitForIt(Yes[0], Yes[1], Yes[2], Yes[3]); LEDtestTick = false; clearFscreen();

            setFont(LARGE, 255, 0, 0, 0, 0, 0);
            myGLCD.print(print_text[175], 5, 80);   // Сохранение настроек
            COLOR = 0 ; SaveLEDToEEPROM();    // save all colour
            bitClear(GlobalStatus1Byte, 3);   // clear bit for " Save changes before exit "
            bitClear(GlobalStatus1Byte, 2);   // clear bit for " Save changes before exit "
            dispScreen = 4; colorLEDtest = false;
            ReadLedFromEEPROM(); clearFscreen(); LED_levelo_output();       // return to current values led before exit to main
            dispScreen = 1; menuScreen();
          }

          if ((x >= StopDay[0]) && (x <= StopDay[2]) && (y >= StopDay[1]) && (y <= StopDay[3] && colorLEDtest == true)
              && (bitRead(GlobalStatus1Byte, 2) == false)) {           // press SUNSET, jump to end "light day"
            waitForIt(StopDay[0], StopDay[1], StopDay[2], StopDay[3]);
            calculateStopTime();           // calculate SUNSET Time
            temp_sector = StopTime;
            testIndLedScreen2(false);
          }

          if ((x >= StartDay[0]) && (x <= StartDay[2]) && (y >= StartDay[1]) && (y <= StartDay[3] && colorLEDtest == true)
              && (bitRead(GlobalStatus1Byte, 2) == false)) {             // press  SUNRISE, jump to start "light day"
            waitForIt(StartDay[0], StartDay[1], StartDay[2], StartDay[3]);
            calculateStartTime();           // calculate SUNRISE Time
            temp_sector = StartTime;
            testIndLedScreen2(false);
          }

          if ((x >= Nsect[0]) && (x <= Nsect[2]) && (y >= Nsect[1]) && (y <= Nsect[3] && colorLEDtest == true)
              && (bitRead(GlobalStatus1Byte, 2) == false)) {        // press NEXT SECTOR, without exit from screen
            waitForIt(Nsect[0], Nsect[1], Nsect[2], Nsect[3]);
            temp_sector += 1;                          // insrease time sector

            if (temp_sector > 95) {
              temp_sector = 95;
            } testIndLedScreen2(false);
          }
          if ((x >= Psect[0]) && (x <= Psect[2]) && (y >= Psect[1]) && (y <= Psect[3] && colorLEDtest == true)
              && (bitRead(GlobalStatus1Byte, 2) == false)) {        // press  PREV SECTOR, without exit from screen
            waitForIt(Psect[0], Psect[1], Psect[2], Psect[3]);
            temp_sector -= 1;                        // desrease time sector

            if ((temp_sector < 1) || (temp_sector > 96)) {
              temp_sector = 0;
            } testIndLedScreen2(false);
          }

          if ((y >= TopSldY) && (y <= BotSldY) && (colorLEDtest == true) && (bitRead(GlobalStatus1Byte, 2) == false)) { // change value with Slider Bars touch

            for (byte i = 0; i < 8; i++) {
              if ((x >= (i * 35) + 21 + 5) && (x <= (i * 35) + 51 - 5)) { // desrease slider touchable area (5pix)
                // WHITE=0, BLUE=1, ROYAL=2, RED=3, ULTRA=4, ORANGE=5, GREEN=6, MOON=7;
                //                                            V - ширина слайдера
                sbX1 = (i * 35) + 21; sbX2 = (i * 35) + 51; // slider width 30 pix, clearens between sliders 5pix
                TopSldY = 53, BotSldY = TopSldY + 100;
                SliderSwitch  = false;

                if (i == 0 && bitRead(LedShannelStatusByte, 0) == true) {
                  sbR = rgbCh0[0]; sbG = rgbCh0[1]; sbB = rgbCh0[2]; // CW colour	(белый)
                  wwtcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                }

                if (i == 1 && bitRead(LedShannelStatusByte, 1) == true) {
                  sbR = rgbCh1[0]; sbG = rgbCh1[1]; sbB = rgbCh1[2]; 	 // BL colour (синий)
                  cwtcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                }

                if (i == 2 && bitRead(LedShannelStatusByte, 2) == true) {
                  sbR = rgbCh2[0]; sbG = rgbCh2[1]; sbB = rgbCh2[2]; 	 // RBL colour (рояль)
                  rbcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                }

                if (i == 3 && bitRead(LedShannelStatusByte, 3) == true) {
                  sbR = rgbCh3[0]; sbG = rgbCh3[1]; sbB = rgbCh3[2]; // DR colour (красный)
                  rcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                }

                if (i == 4 && bitRead(LedShannelStatusByte, 4) == true) {
                  sbR = rgbCh4[0]; sbG = rgbCh4[1]; sbB = rgbCh4[2];	 // UV colour
                  uvcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                }

                if (i == 5 && bitRead(LedShannelStatusByte, 5) == true) {
                  sbR = rgbCh5[0]; sbG = rgbCh5[1]; sbB = rgbCh5[2];	 // OR colour (оранж)
                  ocol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                }

                if (i == 6 && bitRead(LedShannelStatusByte, 6) == true) {
                  sbR = rgbCh6[0]; sbG = rgbCh6[1]; sbB = rgbCh6[2];	 // GR colour (зелен)
                  grcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                }

                if (i == 7) {
                  sbR = rgbCh8[0]; sbG = rgbCh8[1]; sbB = rgbCh8[2];		 // MOON colour
                  mooncol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                }
                wwtled[temp_sector] = wwtcol_out;
                swtled[temp_sector] = cwtcol_out;    // save to buffer before store to eeprom
                rswtled[temp_sector] = rbcol_out;
                rled[temp_sector] = rcol_out;
                uvled[temp_sector] = uvcol_out;
                oLed[temp_sector] = ocol_out;
                gled[temp_sector] = grcol_out;
                bitSet(GlobalStatus1Byte, 3);     // set bit3=1 if setting changed, for " Save changes before exit "
                LED_levelo_output();
              }
            }
          } else   // send calculated value to PWM

            if ((y >= 29 + 2) && (y <= 44 - 2) && (colorLEDtest == true) && (bitRead(GlobalStatus1Byte, 2) == false)) { // UP Buttons were touched, desrease button touchable area (2pix)

              for (byte i = 0; i < 8; i++) {
                if ((x >= (i * 35) + 21) && (x <= (i * 35) + 51)) { // slider width 30 pix, clearens 5pix
                  // WHITE=0, BLUE=1, ROYAL=2, RED=3, ULTRA=4, ORANGE=5, GREEN=6, MOON=7; ,

                  sbX1 = (i * 35) + 21; sbX2 = (i * 35) + 51;
                  TopSldY = 53, BotSldY = TopSldY + 100;
                  SliderSwitch  = true;

                  if (i == 0 && bitRead(LedShannelStatusByte, 0) == true) {
                    sbR = rgbCh0[0]; sbG = rgbCh0[1]; sbB = rgbCh0[2]; tSlide = wwtcol_out; tSlide += 1; 	 // CW colour (белый)
                    wwtcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                    tSlide = wwtcol_out;
                  }

                  if (i == 1 && bitRead(LedShannelStatusByte, 1) == true) {
                    sbR = rgbCh1[0]; sbG = rgbCh1[1]; sbB = rgbCh1[2]; tSlide = cwtcol_out; tSlide += 1; // BL colour (голубой)
                    cwtcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                    tSlide = cwtcol_out;
                  }

                  if (i == 2 && bitRead(LedShannelStatusByte, 2) == true) {
                    sbR = rgbCh2[0]; sbG = rgbCh2[1]; sbB = rgbCh2[2]; tSlide = rbcol_out; tSlide += 1; 	 // RBL colour (рояль)
                    rbcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                    tSlide = rbcol_out;
                  }

                  if (i == 3 && bitRead(LedShannelStatusByte, 3) == true) {
                    sbR = rgbCh3[0]; sbG = rgbCh3[1]; sbB = rgbCh3[2]; tSlide = rcol_out; tSlide += 1; // DR colour (красный)
                    rcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                    tSlide = rcol_out;
                  }

                  if (i == 4 && bitRead(LedShannelStatusByte, 4) == true) {
                    sbR = rgbCh4[0]; sbG = rgbCh4[1]; sbB = rgbCh4[2]; tSlide = uvcol_out; tSlide += 1; // UV colour (фиолет)
                    uvcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                    tSlide = uvcol_out;
                  }

                  if (i == 5 && bitRead(LedShannelStatusByte, 5) == true) {
                    sbR = rgbCh5[0]; sbG = rgbCh5[1]; sbB = rgbCh5[2]; tSlide = ocol_out; tSlide += 1;  	 // OR colour (оранж)
                    ocol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                    tSlide = ocol_out;
                  }

                  if (i == 6 && bitRead(LedShannelStatusByte, 6) == true) {
                    sbR = rgbCh6[0]; sbG = rgbCh6[1]; sbB = rgbCh6[2]; tSlide = grcol_out; tSlide += 1; // GR colour (зелен)
                    grcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                    tSlide = grcol_out;
                  }

                  if (i == 7) {
                    sbR = rgbCh8[0]; sbG = rgbCh8[1]; sbB = rgbCh8[2]; tSlide = mooncol_out; tSlide += 1; // MOON colour
                    mooncol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                    tSlide = mooncol_out;
                  }

                  wwtled[temp_sector] = wwtcol_out;   // белый)
                  swtled[temp_sector] = cwtcol_out;   // save to buffer before store to eeprom
                  rswtled[temp_sector] = rbcol_out; // рояль
                  rled[temp_sector] = rcol_out;   // красный
                  uvled[temp_sector] = uvcol_out;
                  oLed[temp_sector] = ocol_out;
                  gled[temp_sector] = grcol_out;  // (зелен)
                  LED_levelo_output();             // send calculated value to PWM
                  bitSet(GlobalStatus1Byte, 3);   // set bit3=1  if setting changed, for " Save changes before exit "
                  _delay_ms(50);
                }
              }               // delay 50msec after touch UP/DOWN button
              SliderSwitch  = false;
            } else

              if ((y >= 174 + 2) && (y <= 187 - 2) && (colorLEDtest == true) && (bitRead(GlobalStatus1Byte, 2) == false)) { // DOWN Buttons were touched,desrease button touchable area (2pix)

                for (byte i = 0; i < 8; i++) {
                  if ((x >= (i * 35) + 21) && (x <= (i * 35) + 51)) { // slider width 30 pix, clearens 5pix
                    // WHITE=0, BLUE=1, ROYAL=2, RED=3, ULTRA=4, ORANGE=5, GREEN=6, MOON=7;

                    sbX1 = (i * 35) + 21; sbX2 = (i * 35) + 51;
                    TopSldY = 53, BotSldY = TopSldY + 100;
                    SliderSwitch  = true;

                    if (i == 0 && bitRead(LedShannelStatusByte, 0) == true) {
                      sbR = rgbCh0[0]; sbG = rgbCh0[1]; sbB = rgbCh0[2]; tSlide = wwtcol_out; tSlide -= 1; // CW colour (белый)
                      wwtcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                      tSlide = wwtcol_out;
                    }

                    if (i == 1 && bitRead(LedShannelStatusByte, 1) == true) {
                      sbR = rgbCh1[0]; sbG = rgbCh1[1]; sbB = rgbCh1[2]; tSlide = cwtcol_out; tSlide -= 1;	// BL colour (синий)
                      cwtcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                      tSlide = cwtcol_out;
                    }

                    if (i == 2 && bitRead(LedShannelStatusByte, 2) == true) {
                      sbR = rgbCh2[0]; sbG = rgbCh2[1]; sbB = rgbCh2[2]; tSlide = rbcol_out; tSlide -= 1; // RBL colour (рояль)
                      rbcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                      tSlide = rbcol_out;
                    }

                    if (i == 3 && bitRead(LedShannelStatusByte, 3) == true) {
                      sbR = rgbCh3[0]; sbG = rgbCh3[1]; sbB = rgbCh3[2]; tSlide = rcol_out; tSlide -= 1; // DR colour (красный)
                      rcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                      tSlide = rcol_out ;
                    }

                    if (i == 4 && bitRead(LedShannelStatusByte, 4) == true) {
                      sbR = rgbCh4[0]; sbG = rgbCh4[1]; sbB = rgbCh4[2]; tSlide = uvcol_out; tSlide -= 1; // UV colour (фиолет)
                      uvcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                      tSlide = uvcol_out;
                    }

                    if (i == 5 && bitRead(LedShannelStatusByte, 5) == true) {
                      sbR = rgbCh5[0]; sbG = rgbCh5[1]; sbB = rgbCh5[2]; tSlide = ocol_out; tSlide -= 1;	 // OR colour (оранж)
                      ocol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                      tSlide = ocol_out;
                    }

                    if (i == 6 && bitRead(LedShannelStatusByte, 6) == true) {
                      sbR = rgbCh6[0]; sbG = rgbCh6[1]; sbB = rgbCh6[2]; tSlide = grcol_out; tSlide -= 1;	 // GR colour (зелен)
                      grcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                      tSlide = grcol_out;
                    }

                    if (i == 7) {
                      sbR = rgbCh8[0]; sbG = rgbCh8[1]; sbB = rgbCh8[2]; tSlide = mooncol_out; tSlide -= 1; // MOON colour
                      mooncol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                      tSlide = mooncol_out;
                    }

                    wwtled[temp_sector] = wwtcol_out;
                    swtled[temp_sector] = cwtcol_out;    // save to buffer before store to eeprom
                    rswtled[temp_sector] = rbcol_out;
                    rled[temp_sector] = rcol_out;
                    uvled[temp_sector] = uvcol_out;
                    oLed[temp_sector] = ocol_out;
                    gled[temp_sector] = grcol_out;
                    LED_levelo_output();          // send calculated value to PWM
                    bitSet(GlobalStatus1Byte, 3);   // set bit3=1 if setting changed for, " Save changes before exit "
                    _delay_ms(50);                 // delay 50msec after touch UP/DOWN button
                  }
                }
              } SliderSwitch  = false;
          break;

        case 5:  //---------------- TEST LED ARRAY SCREEN ------------------
          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3]) // Press back
              && (LEDtestTick == false)) {
            waitForIt(back[0], back[1], back[2], back[3]);
            LEDtestTick = false; ReadFromEEPROM(); dispScreen = 1; clearScreen(); menuScreen();
          }

          if ((x >= canC[0]) && (x <= canC[2]) && (y >= canC[1]) && (y <= canC[3]) // Press CANCEL
              && (LEDtestTick == false)) {
            waitForIt(canC[0], canC[1], canC[2], canC[3]);
            LEDtestTick = false; ReadFromEEPROM(); dispScreen = 0; clearScreen(); mainScreen(true);
          }

          if ((x >= stsT[0]) && (x <= stsT[2]) && (y >= stsT[1]) && (y <= stsT[3])) { // Press start / stop test
            waitForIt(stsT[0], stsT[1], stsT[2], stsT[3]);

            if (LEDtestTick) {
              LEDtestTick = false; testArrayScreen(true);
            } else {
              LEDtestTick = true;
              testArrayScreen();
            }
          } else {

            if ((x >= tenM[0]) && (x <= tenM[2]) && (y >= tenM[1]) && (y <= tenM[3])) {
              min_cnt -= 10; // Press -10s
              if (min_cnt < 0) {
                min_cnt = 0;
              } _delay_ms(10);
            }
            if ((x >= tenP[0]) && (x <= tenP[2]) && (y >= tenP[1]) && (y <= tenP[2])) {
              min_cnt += 10; // Press +10s
              if (min_cnt > 1440) {
                min_cnt = 1440;
              } _delay_ms(10);
            }
          }

          break;

        case 6: // TEST INDIVIDUAL LED SCREEN -------------------------- Слайдерное ручное управление
          int CL_check, CL_check2;

          if ((x >= 8) && (x <= 33) && (y >= 190) && (y <= 214)) { // press back
            waitForIt(8, 190, 33, 214);                    // возврат в главный экран (крестик)
            wwtled_out = map(wwtcol_out, 0, 1985, 0, 100);      // new value 0-100%
            cwtled_out = map(cwtcol_out, 0, 1985, 0, 100);
            rbled_out = map(rbcol_out, 0, 1985, 0, 100);
            rled_out = map(rcol_out, 0, 1985, 0, 100);
            uvled_out = map(uvcol_out, 0, 1985, 0, 100);
            oLed_out = map(oLed_out, 0, 1985, 0, 100);
            gled_out = map(gled_out, 0, 1985, 0, 100);
            clearScreen();

            LEDtestTick = false; dispScreen = 0; mainScreen(true); colorLEDtest = false;
            myGLCD.setColor(30, 30, 30);
            myGLCD.fillRect(0, 226, 319, 239);
          }    // Нижний Bar (часы и дата)

          if ((y >= 44) && (y <= 172)) {
            for (int i = 0; i < 7; i++) {      // Slider Bars
              if ((x >= (i * 38) + 49) && (x <= (i * 38) + 79)) {
                if (i == 0) {
                  sbR = rgbCh0[0];  // WHITE LED Slider
                  sbG = rgbCh0[1];
                  sbB = rgbCh0[2];
                  sbX1 = 49;
                  sbX2 = 79;
                  SliderBars();
                }
                if (i == 1) {
                  sbR = rgbCh1[0];  // BLUE LED Slider
                  sbG = rgbCh1[1];
                  sbB = rgbCh1[2];
                  sbX1 = 87;
                  sbX2 = 117;
                  SliderBars();
                }
                if (i == 2) {
                  sbR = rgbCh2[0];  // R. BLUE LED Slider
                  sbG = rgbCh2[1];
                  sbB = rgbCh2[2];
                  sbX1 = 125;
                  sbX2 = 155;
                  SliderBars();
                }
                if (i == 3) {
                  sbR = rgbCh3[0];  // RED LED Slider
                  sbG = rgbCh3[1];
                  sbB = rgbCh3[2];
                  sbX1 = 163;
                  sbX2 = 193;
                  SliderBars();
                }
                if (i == 4) {
                  sbR = rgbCh4[0];  // UV LED Slider
                  sbG = rgbCh4[1];
                  sbB = rgbCh4[2];
                  sbX1 = 201;
                  sbX2 = 231;
                  SliderBars();
                }
                if (i == 5) {
                  sbR = rgbCh5[0];  // ORANGE LED Slider
                  sbG = rgbCh5[1];
                  sbB = rgbCh5[2];
                  sbX1 = 239;
                  sbX2 = 269;
                  SliderBars();
                }
                if (i == 6) {
                  sbR = rgbCh6[0];  // Green LED Slider
                  sbG = rgbCh6[1];
                  sbB = rgbCh6[2];
                  sbX1 = 277;
                  sbX2 = 307;
                  SliderBars();
                }
                _delay_ms(50);
              }
            }
          }

          if ((y >= 17) && (y <= 39)) {
            for (int i = 0; i < 7; i++) {    //Up buttons were touched
              if ((x >= (i * 38) + 49) && (x <= (i * 38) + 79)) {
                if (i == 0) {
                  sbR = rgbCh0[0];  // WHITE +
                  sbG = rgbCh0[1];
                  sbB = rgbCh0[2];
                  sbX1 = 49;
                  sbX2 = 79;
                  yWHT += 1;
                  UpDnButtonSlide();
                }
                if (i == 1) {
                  sbR = rgbCh1[0];  // BLUE +
                  sbG = rgbCh1[1];
                  sbB = rgbCh1[2];
                  sbX1 = 87;
                  sbX2 = 117;
                  yBLU += 1;
                  UpDnButtonSlide();
                }
                if (i == 2) {
                  sbR = rgbCh2[0];  // ROYAL BLUE +
                  sbG = rgbCh2[1];
                  sbB = rgbCh2[2];
                  sbX1 = 125;
                  sbX2 = 155;
                  yRBL += 1;
                  UpDnButtonSlide();
                }
                if (i == 3) {
                  sbR = rgbCh3[0];  // RED +
                  sbG = rgbCh3[1];
                  sbB = rgbCh3[2];
                  sbX1 = 163;
                  sbX2 = 193;
                  yRED += 1;
                  UpDnButtonSlide();
                }
                if (i == 4) {
                  sbR = rgbCh4[0];  // UV +
                  sbG = rgbCh4[1];
                  sbB = rgbCh4[2];
                  sbX1 = 201;
                  sbX2 = 231;
                  yUVL += 1;
                  UpDnButtonSlide();
                }
                if (i == 5) {
                  sbR = rgbCh5[0];  // ORANGE +
                  sbG = rgbCh5[1];
                  sbB = rgbCh5[2];
                  sbX1 = 239;
                  sbX2 = 269;
                  ySMP += 1;
                  UpDnButtonSlide();
                }
                if (i == 6) {
                  sbR = rgbCh6[0];  // Green +
                  sbG = rgbCh6[1];
                  sbB = rgbCh6[2];
                  sbX1 = 277;
                  sbX2 = 307;
                  yGRN += 1;
                  UpDnButtonSlide();
                }
                _delay_ms(50);
              }
            }
          }

          if ((y >= 200) && (y <= 222)) {
            for (int i = 0; i < 7; i++) {  //Down buttons were touched
              if ((x >= (i * 38) + 49) && (x <= (i * 38) + 79)) {
                if (i == 0) {
                  sbR = rgbCh0[0];  // WHITE -
                  sbG = rgbCh0[1];
                  sbB = rgbCh0[2];
                  sbX1 = 49;
                  sbX2 = 79;
                  yWHT -= 1;
                  UpDnButtonSlide();
                }
                if (i == 1) {
                  sbR = rgbCh1[0];  // BLUE -
                  sbG = rgbCh1[1];
                  sbB = rgbCh1[2];
                  sbX1 = 87;
                  sbX2 = 117;
                  yBLU -= 1;
                  UpDnButtonSlide();
                }
                if (i == 2) {
                  sbR = rgbCh2[0];  // ROYAL BLUE -
                  sbG = rgbCh2[1];
                  sbB = rgbCh2[2];
                  sbX1 = 125;
                  sbX2 = 155;
                  yRBL -= 1;
                  UpDnButtonSlide();
                }
                if (i == 3) {
                  sbR = rgbCh3[0];  // RED -
                  sbG = rgbCh3[1];
                  sbB = rgbCh3[2];
                  sbX1 = 163;
                  sbX2 = 193;
                  yRED -= 1;
                  UpDnButtonSlide();
                }
                if (i == 4) {
                  sbR = rgbCh4[0];  // UV -
                  sbG = rgbCh4[1];
                  sbB = rgbCh4[2];
                  sbX1 = 201;
                  sbX2 = 231;
                  yUVL -= 1;
                  UpDnButtonSlide();
                }
                if (i == 5) {
                  sbR = rgbCh5[0];  // ORANGE -
                  sbG = rgbCh5[1];
                  sbB = rgbCh5[2];
                  sbX1 = 239;
                  sbX2 = 269;
                  ySMP -= 1;
                  UpDnButtonSlide();
                }
                if (i == 6) {
                  sbR = rgbCh6[0];  // GREEN -
                  sbG = rgbCh6[1];
                  sbB = rgbCh6[2];
                  sbX1 = 277;
                  sbX2 = 307;
                  yGRN -= 1;
                  UpDnButtonSlide();
                }
                _delay_ms(50);
              }
            }
          }

          break;

        case 7:  // ---------------------
          break;

        case 8:  /************************** ШИМ ОХЛАЖДЕНИЯ **************************************/

          CoolFanMIN = PlusMinusCountI (true, true, temM[0] - 50, temM[1] + 15, 95, 0, 100, 1, CoolFanMIN);               //
          if (PlsMnsPress == true) {
            PlsMnsPress = false;
          }
          CoolFanPWM = PlusMinusCountI (true, true, temM[0] - 50, SoundAlarmTm[1] - 20, 95, 0, 100, 1, CoolFanPWM);       //
          if (PlsMnsPress == true) {
            PlsMnsPress = false;
          }
          HeatsinkLoPWM = PlusMinusCountI (true, true, temM[0] + 105, temM[1] + 15, 95, 0, 100, 1, HeatsinkLoPWM);        //
          if (PlsMnsPress == true) {
            PlsMnsPress = false;
          }
          HeatsinkHiPWM = PlusMinusCountI (true, true, temM[0] + 105, SoundAlarmTm[1] - 20, 95, 0, 100, 1, HeatsinkHiPWM); //
          if (PlsMnsPress == true) {
            PlsMnsPress = false;
          }

          PressButton((0), NO , back[0], back[1], back[2], back[3]);                            // Press back
          if (ButtonPress == true) {
            dispScreen = 9; clearScreen(); generalSettingsScreen_3();
            ButtonPress = false;
          }

          PressButton((0), NO , prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);                    // press SAVE
          if (ButtonPress == true) {
            SaveGenSetsToEEPROM();
            ButtonPress = false;
          }

          PressButton((0), NO , canC[0], canC[1], canC[2], canC[3]);                            // Press CANCEL
          if (ButtonPress == true) {
            dispScreen = 0; clearScreen(); mainScreen(true);
            ButtonPress = false;
          }

          break;

        case 9:  /****************************GENERAL SETTINGS (PAGE 3) ****************************/

          PressButton((0), NO , 185, 35, 305, 55);       // НАСТРОИТЬ
          if (ButtonPress == true) {
            dispScreen = 40; clearScreen(); RGBTune(); RGBcolorSet = true;
            ButtonPress = false;
          }

          PressButton((0), NO , 185, 115, 305, 135);     // НАСТРОИТЬ
          if (ButtonPress == true) {
            dispScreen = 8; clearScreen(); coolfanPWM();
            ButtonPress = false;
          }

          PressButton((0), NO , back[0], back[1], back[2], back[3]);    // Press back
          if (ButtonPress == true) {
            dispScreen = 15; clearScreen(); generalSettingsScreen_2();
            ButtonPress = false;
          }

          PressButton((0), NO , prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);    // press SAVE
          if (ButtonPress == true) {
            SaveGenSetsToEEPROM(); dispScreen = 1; clearScreen(); menuScreen();
            ButtonPress = false;
          }

          PressButton((0), NO , canC[0], canC[1], canC[2], canC[3]);   // Press CANCEL
          if (ButtonPress == true) {
            dispScreen = 0; clearScreen(); mainScreen(true);
            ButtonPress = false;
          }

          break;

        case 10://-----------------КАЛИБРОВКА УДО-----------------------

          if (CalMode == 1) {
            dozVal1 = PlusMinusCountI (true, true, valUP[0], valUP[1], 100, numDoz1, 999, numDoz1, dozVal1);     // доза
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            }

            numDoz1 = PlusMinusCountI (true, true, numUP[0], numUP[1], 100, 1, 4, 1, numDoz1);     // Количество доз 1 дозатора
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            }

            intDoz1 = PlusMinusCountI (true, true, intUP[0], intUP[1], 100, 1, 6, 1, intDoz1);     // Интервал между дозами (часы) 1 дозатора
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            }

            dozCal1 = PlusMinusCountI (true, true, calUP[0], calUP[1], 100, 1, 999, 1, dozCal1);     // калибровка 10 сек
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            } dosSec1();
          }

          if (CalMode == 2) {
            dozVal2 = PlusMinusCountI (true, true, valUP[0], valUP[1], 100, numDoz2, 999, numDoz2, dozVal2);     // доза
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            }

            numDoz2 = PlusMinusCountI (true, true, numUP[0], numUP[1], 100, 1, 4, 1, numDoz2);     // Количество доз 2 дозатора
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            }

            intDoz2 = PlusMinusCountI (true, true, intUP[0], intUP[1], 100, 1, 6, 1, intDoz2);     // Интервал между дозами (часы) 1 дозатора
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            }

            dozCal2 = PlusMinusCountI (true, true, calUP[0], calUP[1], 100, 1, 999, 1, dozCal2);     // калибровка 10 сек
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            } dosSec2();
          }

          if (CalMode == 3) {
            dozVal3 = PlusMinusCountI (true, true, valUP[0], valUP[1], 100, numDoz3, 999, numDoz3, dozVal3);     // доза
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            }

            numDoz3 = PlusMinusCountI (true, true, numUP[0], numUP[1], 100, 1, 4, 1, numDoz3);     // Количество доз 3 дозатора
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            }

            intDoz3 = PlusMinusCountI (true, true, intUP[0], intUP[1], 100, 1, 6, 1, intDoz3);     // Интервал между дозами (часы) 1 дозатора
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            }

            dozCal3 = PlusMinusCountI (true, true, calUP[0], calUP[1], 100, 1, 999, 1, dozCal3);     // калибровка 10 сек
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            } dosSec3();
          }

          if (CalMode == 4) {
            dozVal4 = PlusMinusCountI (true, true, valUP[0], valUP[1], 100, numDoz4, 999, numDoz4, dozVal4);     // доза
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            }

            numDoz4 = PlusMinusCountI (true, true, numUP[0], numUP[1], 100, 1, 4, 1, numDoz4);     // Количество доз 4 дозатора
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            }

            intDoz4 = PlusMinusCountI (true, true, intUP[0], intUP[1], 100, 1, 6, 1, intDoz4);     // Интервал между дозами (часы) 1 дозатора
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            }

            dozCal4 = PlusMinusCountI (true, true, calUP[0], calUP[1], 100, 1, 999, 1, dozCal4);     // калибровка 10 сек
            if (PlsMnsPress == true) {
              PlsMnsPress = false;
            } dosSec4();
          }

          if ((x >= dos4b[0] + 5) && (x <= dos4b[2] + 52) && (y >= dos4b[1] - 6) && (y <= dos4b[3] - 4)) {
            waitForIt(dos4b[0] + 5, dos4b[1] - 6, dos4b[2] + 52, dos4b[3] - 4);
            doscalibrate();
          }


          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3])) { // Press back
            waitForIt(back[0], back[1], back[2], back[3]);
            dispScreen = 36; clearScreen(); autoDoserScreen();
          }

          if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) { // press SAVE
            waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
            SaveDoseTimesToEEPROM();
            dispScreen = 36; clearScreen(); autoDoserScreen();
          }

          if ((x >= canC[0]) && (x <= canC[2]) && (y >= canC[1]) && (y <= canC[3])) { // Press CANCEL
            waitForIt(canC[0], canC[1], canC[2], canC[3]);
            dispScreen = 0; clearScreen(); mainScreen(true);
          }

          break;

        case 11: //-----------------ДОЗАТОР СО2 / УСТАНОВКА И КАЛИБРОВКА РН-----------------------

          SetvalPH = PlusMinusCountF (true, true, dos1b[0], dos1b[1] + 20, 150, 5, 9, 0.1, SetvalPH);   //
          if (PlsMnsPress == true) {
            PlsMnsPress = false;
          }

          if ((x >= dos3b[0]) && (x <= dos3b[2] + 50) && (y >= dos3b[1]) && (y <= dos3b[3])) { // калибровка РН 7
            waitForIt(dos3b[0], dos3b[1], dos3b[2] + 50, dos3b[3]);
            volt7 = avgPHVolts;                      // Напряжения калибровки датчика PH 7
            dispScreen = 11; clearScreen(); SetPH();
          }

          if ((x >= dos4b[0]) && (x <= dos4b[2] + 50) && (y >= dos4b[1]) && (y <= dos4b[3])) { // калибровка РН 10
            waitForIt(dos4b[0], dos4b[1], dos4b[2] + 50, dos4b[3]);
            volt10 = avgPHVolts;                      // Напряжения калибровки датчика PH 10
            dispScreen = 11; clearScreen(); SetPH();
          }

          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3])) { // Press back
            waitForIt(back[0], back[1], back[2], back[3]);
            dispScreen = 1; clearScreen(); menuScreen();
          }

          if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) { // press SAVE
            waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
            SavePHsetToEEPROM();
            dispScreen = 0; clearScreen(); mainScreen(true);
          }

          if ((x >= canC[0]) && (x <= canC[2]) && (y >= canC[1]) && (y <= canC[3])) { // Press CANCEL
            waitForIt(canC[0], canC[1], canC[2], canC[3]);
            dispScreen = 0; clearScreen(); mainScreen(true);
          }

          break;

        case 12:
          //------------------ Помпы в режиме ШИМ ------------------

          if ((x >= SpeedPplus[0]) && (x <= SpeedPplus[2]) && (y >= SpeedPplus[1]) && (y <= SpeedPplus[3])) {
            periode -= 500;  // увеличение cкорости
            waitForIt(SpeedPplus[0], SpeedPplus[1], SpeedPplus[2], SpeedPplus[3]); PumpSpeed();
          }

          if ((x >= SpeedPminus[0]) && (x <= SpeedPminus[2]) && (y >= SpeedPminus[1]) && (y <= SpeedPminus[3])) {
            periode += 500; // уменьшение cкорости
            waitForIt(SpeedPminus[0], SpeedPminus[1], SpeedPminus[2], SpeedPminus[3]); PumpSpeed();
          }

          if ((x >= MaxPlus1[0]) && (x <= MaxPlus1[2]) && (y >= MaxPlus1[1]) && (y <= MaxPlus1[3])) {
            maxP1 += 10;  // Максимальная мощность помпы 1 + (min)
            waitForIt(MaxPlus1[0], MaxPlus1[1], MaxPlus1[2], MaxPlus1[3]);
            if (maxP1 >= 60) {
              maxP1 = 0;
            }
            PumpSpeed();
          }

          if ((x >= MaxMinus1[0]) && (x <= MaxMinus1[2]) && (y >= MaxMinus1[1]) && (y <= MaxMinus1[3])) {
            maxP1 -= 10; // Максимальная мощность помпы 1 -
            waitForIt(MaxMinus1[0], MaxMinus1[1], MaxMinus1[2], MaxMinus1[3]);
            if (maxP1 < 0) {
              maxP1 = 60;
            }
            PumpSpeed();
          }

          if ((x >= MinPlus1[0]) && (x <= MinPlus1[2]) && (y >= MinPlus1[1]) && (y <= MinPlus1[3])) {
            minP1 += 10;  // Минимальная мощность помпы 1 + (max)
            waitForIt(MinPlus1[0], MinPlus1[1], MinPlus1[2], MinPlus1[3]);
            if (minP1 >= 99) {
              minP1 = 0;
            }
            PumpSpeed();
          }

          if ((x >= MinMinus1[0]) && (x <= MinMinus1[2]) && (y >= MinMinus1[1]) && (y <= MinMinus1[3])) {
            minP1 -= 10; // Минимальная мощность помпы 1 -
            waitForIt(MinMinus1[0], MinMinus1[1], MinMinus1[2], MinMinus1[3]);
            if (minP1 < 0) {
              minP1 = 99;
            }
            PumpSpeed();
          }

          if ((x >= MaxPlus2[0]) && (x <= MaxPlus2[2]) && (y >= MaxPlus2[1]) && (y <= MaxPlus2[3])) {
            maxP2 += 10;  // Максимальная мощность помпы 1 +
            waitForIt(MaxPlus2[0], MaxPlus2[1], MaxPlus2[2], MaxPlus2[3]);
            if (maxP2 >= 60) {
              maxP2 = 0;
            }
            PumpSpeed();
          }

          if ((x >= MaxMinus2[0]) && (x <= MaxMinus2[2]) && (y >= MaxMinus2[1]) && (y <= MaxMinus2[3])) {
            maxP2 -= 10; // Максимальная мощность помпы 1 -
            waitForIt(MaxMinus2[0], MaxMinus2[1], MaxMinus2[2], MaxMinus2[3]);
            if (maxP2 < 0) {
              maxP2 = 60;
            }
            PumpSpeed();
          }

          if ((x >= MinPlus2[0]) && (x <= MinPlus2[2]) && (y >= MinPlus2[1]) && (y <= MinPlus2[3])) {
            minP2 += 10;  // Минимальная мощность помпы 1 +
            waitForIt(MinPlus2[0], MinPlus2[1], MinPlus2[2], MinPlus2[3]);
            if (minP2 >= 99) {
              minP2 = 0;
            }
            PumpSpeed();
          }

          if ((x >= MinMinus2[0]) && (x <= MinMinus2[2]) && (y >= MinMinus2[1]) && (y <= MinMinus2[3])) {
            minP2 -= 10; // Минимальная мощность помпы 1 -
            waitForIt(MinMinus2[0], MinMinus2[1], MinMinus2[2], MinMinus2[3]);
            if (minP2 < 0) {
              minP2 = 99;
            }
            PumpSpeed();
          }

          // Кнопки Максимальная / минимальная скорость
          if ((x >= SpeedMax[0]) && (x <= SpeedMax[2]) && (y >= SpeedMax[1]) && (y <= SpeedMax[3])) {
            periode = 1108; // максимальная скорость
            waitForIt(SpeedMax[0], SpeedMax[1], SpeedMax[2], SpeedMax[3]); PumpSpeed();
          }

          if ((x >= SpeedMin[0]) && (x <= SpeedMin[2]) && (y >= SpeedMin[1]) && (y <= SpeedMin[3])) {
            periode = 32608; // минимальная скорость
            waitForIt(SpeedMin[0], SpeedMin[1], SpeedMin[2], SpeedMin[3]); PumpSpeed();
          }

          if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) { // press SAVE
            waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
            ModeSel; periode;
            minP1; maxP1; minP2; maxP2;
            SavePwmToEEPROM();
          }  //clearScreen(); mainScreen(true);}

          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3])) { // press back
            waitForIt(back[0], back[1], back[2], back[3]);
            dispScreen = 1; clearScreen(); menuScreen();
          } else

            if ((x >= canC[0]) && (x <= canC[2]) && (y >= canC[1]) && (y <= canC[3])) { // Press CANCEL
              waitForIt(canC[0], canC[1], canC[2], canC[3]);
              ReadFromEEPROM(); dispScreen = 0; clearScreen(); mainScreen(true);
            } else

              if ((x >= ModePump1[0]) && (x <= ModePump1[2]) && (y >= ModePump1[1]) && (y <= ModePump1[3])) { // Mode 1
                waitForIt(ModePump1[0], ModePump1[1], ModePump1[2], ModePump1[3]);
                ModeSel = 1; WavePWMScreen();
              } else

                if ((x >= ModePump2[0]) && (x <= ModePump2[2]) && (y >= ModePump2[1]) && (y <= ModePump2[3])) { // Mode 2
                  waitForIt(ModePump2[0], ModePump2[1], ModePump2[2], ModePump2[3]);
                  ModeSel = 2; WavePWMScreen();
                } else

                  if ((x >= ModePump3[0]) && (x <= ModePump3[2]) && (y >= ModePump3[1]) && (y <= ModePump3[3])) { // Mode 3
                    waitForIt(ModePump3[0], ModePump3[1], ModePump3[2], ModePump3[3]);
                    ModeSel = 3; WavePWMScreen();
                  } else

                    if ((x >= ModePump4[0]) && (x <= ModePump4[2]) && (y >= ModePump4[1]) && (y <= ModePump4[3])) { // Mode 4
                      waitForIt(ModePump4[0], ModePump4[1], ModePump4[2], ModePump4[3]);
                      ModeSel = 4; WavePWMScreen();
                    } else

                      if ((x >= ModePump5[0]) && (x <= ModePump5[2]) && (y >= ModePump5[1]) && (y <= ModePump5[3])) { // Mode 5  (перейти к настройкам помп)
                        waitForIt(ModePump5[0], ModePump5[1], ModePump5[2], ModePump5[3]);
                        // dispScreen=36; clearScreen(); PumpSeting();
                        ModeSel = 5; WavePWMScreen();
                      } else

                        if ((x >= ModePump6[0]) && (x <= ModePump6[2]) && (y >= ModePump6[1]) && (y <= ModePump6[3])) { // Mode 6 (выкл помпы)
                          waitForIt(ModePump6[0], ModePump6[1], ModePump6[2], ModePump6[3]);
                          ModeSel = 6; WavePWMScreen();
                        }

          if ((x >= ModePump7[0]) && (x <= ModePump7[2]) && (y >= ModePump7[1]) && (y <= ModePump7[3])) { // Mode 7 (максимальная скорость)
            waitForIt(ModePump7[0], ModePump7[1], ModePump7[2], ModePump7[3]);
            ModeSel = 7; WavePWMScreen();
          }
          break;

        case 13:  // LED Preset (запись пресетов) ==================================================================
          //  if ((x>=298) && (x<=317) && (y>=51) && (y<=110)){ waitForIt(298, 51, 317, 110);
          //          ledON = 1; } else { ledON = 0; }

          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3]) // press back
              && (LEDtestTick == false)) {
            waitForIt(back[0], back[1], back[2], back[3]);
            LEDtestTick = false; dispScreen = 1; clearFscreen(); menuScreen();
            GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0); colorLEDtest = false;
          } // отключить присеты

          if ((x >= canC[0]) && (x <= canC[2]) && (y >= canC[1]) && (y <= canC[3]) // press CANCEL
              && (LEDtestTick == false)) {
            waitForIt(canC[0], canC[1], canC[2], canC[3]);
            LEDtestTick = false; dispScreen = 0; clearFscreen(); mainScreen(true);
            GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0); colorLEDtest = false;
          } // отключить присеты

          if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) { // press SAVE
            waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
            if (bitRead(GlobalStatus2Byte, 0) == 1) {
              AddressShift = 0;
            }
            if (bitRead(GlobalStatus2Byte, 1) == 1) {
              AddressShift = 9;
            }
            if (bitRead(GlobalStatus2Byte, 2) == 1) {
              AddressShift = 18;
            }
            if (bitRead(GlobalStatus2Byte, 3) == 1) {
              AddressShift = 27;
            }
            if ((GlobalStatus2Byte & 0xF) != 0 ) {
              SaveLEDPresetToEEPROM();
            }
          }

          if ((x >= LedPres1[0]) && (x <= LedPres1[2]) && (y >= LedPres1[1]) && (y <= LedPres1[3]) // press Preset N1
              && (LEDtestTick == false)) {
            waitForIt(LedPres1[0], LedPres1[1], LedPres1[2], LedPres1[3]);
            if (bitRead(GlobalStatus2Byte, 0) == 0) {
              printButton(print_text[31], LedPres1[0], LedPres1[1], LedPres1[2], LedPres1[3], SMALL, GREEN_BAC); // ON preset 1
              printButton(print_text[32], LedPres2[0], LedPres2[1], LedPres2[2], LedPres2[3], SMALL);
              printButton(print_text[33], LedPres3[0], LedPres3[1], LedPres3[2], LedPres3[3], SMALL);
              printButton(print_text[34], LedPres4[0], LedPres4[1], LedPres4[2], LedPres4[3], SMALL);
              GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0 | 0x1); // set flag Preset1, clear 2..4
              colorLEDtest = true;
            }

            else {
              printButton(print_text[31], LedPres1[0], LedPres1[1], LedPres1[2], LedPres1[3], SMALL); // OFF preset
              colorLEDtest = false;
              bitClear(GlobalStatus2Byte, 0);
            } // clear flag Preset1
            AddressShift = 0;
            ReadLEDPresetFromEEPROM();
            PresetLedScreen(false);
          }       // preset led windows

          if ((x >= LedPres2[0]) && (x <= LedPres2[2]) && (y >= LedPres2[1]) && (y <= LedPres2[3]) // press Preset N2
              && (LEDtestTick == false)) {
            waitForIt(LedPres2[0], LedPres2[1], LedPres2[2], LedPres2[3]);
            if (bitRead(GlobalStatus2Byte, 1) == 0) {
              printButton(print_text[31], LedPres1[0], LedPres1[1], LedPres1[2], LedPres1[3], SMALL);
              printButton(print_text[32], LedPres2[0], LedPres2[1], LedPres2[2], LedPres2[3], SMALL, GREEN_BAC); // ON preset 2
              printButton(print_text[33], LedPres3[0], LedPres3[1], LedPres3[2], LedPres3[3], SMALL);
              printButton(print_text[34], LedPres4[0], LedPres4[1], LedPres4[2], LedPres4[3], SMALL);
              GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0 | 0x2); // set flag Preset2, clear 1,3,4
              colorLEDtest = true;
            }

            else {
              printButton(print_text[32], LedPres2[0], LedPres2[1], LedPres2[2], LedPres2[3], SMALL); // OFF preset
              colorLEDtest = false;
              bitClear(GlobalStatus2Byte, 1);
            } // clear flag Preset2
            AddressShift = 9;
            ReadLEDPresetFromEEPROM();
            PresetLedScreen(false);
          }        // preset led windows

          if ((x >= LedPres3[0]) && (x <= LedPres3[2]) && (y >= LedPres3[1]) && (y <= LedPres3[3]) // press Preset N3
              && (LEDtestTick == false)) {
            waitForIt(LedPres3[0], LedPres3[1], LedPres3[2], LedPres3[3]);
            if (bitRead(GlobalStatus2Byte, 2) == 0) {
              printButton(print_text[31], LedPres1[0], LedPres1[1], LedPres1[2], LedPres1[3], SMALL);
              printButton(print_text[32], LedPres2[0], LedPres2[1], LedPres2[2], LedPres2[3], SMALL);
              printButton(print_text[33], LedPres3[0], LedPres3[1], LedPres3[2], LedPres3[3], SMALL, GREEN_BAC); // ON preset 3
              printButton(print_text[34], LedPres4[0], LedPres4[1], LedPres4[2], LedPres4[3], SMALL);
              GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0 | 0x4); // set flag Preset3, clear 1,2,4
              colorLEDtest = true;
            }

            else {
              printButton(print_text[33], LedPres3[0], LedPres3[1], LedPres3[2], LedPres3[3], SMALL); // OFF preset
              colorLEDtest = false;
              bitClear(GlobalStatus2Byte, 2);
            } // clear flag Preset3
            AddressShift = 18;
            ReadLEDPresetFromEEPROM();
            PresetLedScreen(false);
          }       // preset led windows

          if ((x >= LedPres4[0]) && (x <= LedPres4[2]) && (y >= LedPres4[1]) && (y <= LedPres4[3]) // press Preset N4
              && (LEDtestTick == false)) {
            waitForIt(LedPres4[0], LedPres4[1], LedPres4[2], LedPres4[3]);
            if (bitRead(GlobalStatus2Byte, 3) == 0) {
              printButton(print_text[31], LedPres1[0], LedPres1[1], LedPres1[2], LedPres1[3], SMALL);
              printButton(print_text[32], LedPres2[0], LedPres2[1], LedPres2[2], LedPres2[3], SMALL);
              printButton(print_text[33], LedPres3[0], LedPres3[1], LedPres3[2], LedPres3[3], SMALL);
              printButton(print_text[34], LedPres4[0], LedPres4[1], LedPres4[2], LedPres4[3], SMALL, GREEN_BAC); // ON preset 4
              GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0 | 0x8);   // set flag Preset4, clear 1..3
              colorLEDtest = true;
            }

            else {
              printButton(print_text[34], LedPres4[0], LedPres4[1], LedPres4[2], LedPres4[3], SMALL); // OFF preset
              colorLEDtest = false;
              bitClear(GlobalStatus2Byte, 3);
            }  // clear flag Preset4
            AddressShift = 27;
            ReadLEDPresetFromEEPROM();
            PresetLedScreen(false);
          }        // preset led windows

          if ((y >= TopSldY) && (y <= BotSldY) && (colorLEDtest == true)) { // change value with Slider Bars touch
            TopSldY = 53, BotSldY = TopSldY + 100;
            SliderSwitch  = false;

            for (byte i = 0; i < 7; i++) {
              sbX1 = (i * 40) + 4 + 18; sbX2 = (i * 40) + 34 + 18;
              if (x >= sbX1 + 5 && x <= sbX2 - 5) { // desrease slider touchable area (-5pix)
                // WHITE=0, BLUE=1, ROYAL=2, RED=3, ULTRA=4, ORANGE=5, GREEN=6
                // slider width 30 pix, clearens between sliders 5pix

                if (i == 0 && bitRead(LedShannelStatusByte, 0) == true) {
                  sbR = rgbCh0[0]; sbG = rgbCh0[1]; sbB = rgbCh0[2];  // CW colour (белый)
                  wwtcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                }

                if (i == 1 && bitRead(LedShannelStatusByte, 1) == true) {
                  sbR = rgbCh1[0]; sbG = rgbCh1[1]; sbB = rgbCh1[2]; 	 // BL colour (голубой)
                  cwtcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                }

                if (i == 2 && bitRead(LedShannelStatusByte, 2) == true) {
                  sbR = rgbCh2[0]; sbG = rgbCh2[1]; sbB = rgbCh2[2];     // RBL colour (рояль)
                  rbcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                }

                if (i == 3 && bitRead(LedShannelStatusByte, 3) == true) {
                  sbR = rgbCh3[0]; sbG = rgbCh3[1]; sbB = rgbCh3[2];      	 // DR colour (красный)
                  rcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                }

                if (i == 4 && bitRead(LedShannelStatusByte, 4) == true) {
                  sbR = rgbCh4[0]; sbG = rgbCh4[1]; sbB = rgbCh4[2];   // UV colour (фиолет)
                  uvcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                }

                if (i == 5 && bitRead(LedShannelStatusByte, 5) == true) {
                  sbR = rgbCh5[0]; sbG = rgbCh5[1]; sbB = rgbCh5[2];	 // OR colour (оранж)
                  ocol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                }

                if (i == 6 && bitRead(LedShannelStatusByte, 6) == true) {
                  sbR = rgbCh6[0]; sbG = rgbCh6[1]; sbB = rgbCh6[2];		 // GR colour (зелен)
                  grcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                }

                //  if (ledON==0){ // включение управления каналами при настройке пресетов
                LED_levelo_output();  //}
              }
            }
          } else

            if ((y >= 29 + 2) && (y <= 44 - 2) && (colorLEDtest == true)) { // UP Buttons were touched, desrease button touchable area (2pix)
              TopSldY = 53, BotSldY = TopSldY + 100;
              SliderSwitch  = true;

              for (byte i = 0; i < 7; i++) {
                sbX1 = (i * 40) + 4 + 18; sbX2 = (i * 40) + 34 + 18;
                if (x >= sbX1 && x <= sbX2) { // slider width 30 pix, clearens 5pix
                  // WHITE=0, BLUE=1, ROYAL=2, RED=3, ULTRA=4, ORANGE=5, GREEN=6

                  if (i == 0 && bitRead(LedShannelStatusByte, 0) == true) {
                    sbR = rgbCh0[0]; sbG = rgbCh0[1]; sbB = rgbCh0[2]; tSlide = wwtcol_out; tSlide += 1; 		 // CW colour (белый)
                    wwtcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide = wwtcol_out;
                  }

                  if (i == 1 && bitRead(LedShannelStatusByte, 1) == true) {
                    sbR = rgbCh1[0]; sbG = rgbCh1[1]; sbB = rgbCh1[2]; tSlide = cwtcol_out; tSlide += 1; 			 // BL colour (голубой)
                    cwtcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide = cwtcol_out;
                  }

                  if (i == 2 && bitRead(LedShannelStatusByte, 2) == true) {
                    sbR = rgbCh2[0]; sbG = rgbCh2[1]; sbB = rgbCh2[2]; tSlide = rbcol_out; tSlide += 1; 		   // RBL colour (рояль)
                    rbcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide = rbcol_out;
                  }

                  if (i == 3 && bitRead(LedShannelStatusByte, 3) == true) {
                    sbR = rgbCh3[0]; sbG = rgbCh3[1]; sbB = rgbCh3[2]; tSlide = rcol_out; tSlide += 1;                // DR colour (красный)
                    rcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide = rcol_out;
                  }

                  if (i == 4 && bitRead(LedShannelStatusByte, 4) == true) {
                    sbR = rgbCh4[0]; sbG = rgbCh4[1]; sbB = rgbCh4[2]; tSlide = uvcol_out; tSlide += 1; 	    // UV colour (фиолет)
                    uvcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide = uvcol_out;
                  }

                  if (i == 5 && bitRead(LedShannelStatusByte, 5) == true) {
                    sbR = rgbCh5[0]; sbG = rgbCh5[1]; sbB = rgbCh5[2]; tSlide = ocol_out; tSlide += 1;  		 // OR colour (оранж)
                    ocol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide = ocol_out;
                  }

                  if (i == 6 && bitRead(LedShannelStatusByte, 6) == true) {
                    sbR = rgbCh6[0]; sbG = rgbCh6[1]; sbB = rgbCh6[2]; tSlide = grcol_out; tSlide += 1; 		    // GR colour (зелен)
                    grcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide = grcol_out;
                  }

                  //if (ledON==0){ // включение управления каналами при настройке пресетов
                  _delay_ms(100);                // delay after touch UP/DOWN button
                  LED_levelo_output();
                }
              }

              SliderSwitch  = false;
            } else

              if ((y >= 174 + 2) && (y <= 187 - 2) && (colorLEDtest == true)) { // DOWN Buttons were touched,desrease button touchable area (2pix)
                TopSldY = 53, BotSldY = TopSldY + 100;
                SliderSwitch  = true;
                for (byte i = 0; i < 7; i++) {
                  sbX1 = (i * 40) + 4 + 18; sbX2 = (i * 40) + 34 + 18;
                  if (x >= sbX1 && x <= sbX2) {                 // slider width 30 pix, clearens 5pix

                    // WHITE=0, BLUE=1, ROYAL=2, RED=3, ULTRA=4, ORANGE=5, GREEN=6
                    if (i == 0 && bitRead(LedShannelStatusByte, 0) == true) {
                      sbR = rgbCh0[0]; sbG = rgbCh0[1]; sbB = rgbCh0[2]; tSlide = wwtcol_out; tSlide -= 1;		 // CW colour (белый)
                      wwtcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide = wwtcol_out;
                    }

                    if (i == 1 && bitRead(LedShannelStatusByte, 1) == true) {
                      sbR = rgbCh1[0]; sbG = rgbCh1[1]; sbB = rgbCh1[2]; tSlide = cwtcol_out; tSlide -= 1;		 // BL colour (голубой)
                      cwtcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide = cwtcol_out;
                    }

                    if (i == 2 && bitRead(LedShannelStatusByte, 2) == true) {
                      sbR = rgbCh2[0]; sbG = rgbCh2[1]; sbB = rgbCh2[2]; tSlide = rbcol_out; tSlide -= 1; 	   // RBL colour (рояль)
                      rbcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide = rbcol_out;
                    }

                    if (i == 3 && bitRead(LedShannelStatusByte, 3) == true) {
                      sbR = rgbCh3[0]; sbG = rgbCh3[1]; sbB = rgbCh3[2]; tSlide = rcol_out;	tSlide -= 1;          // DR colour (красный)
                      rcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide = rcol_out ;
                    }

                    if (i == 4 && bitRead(LedShannelStatusByte, 4) == true) {
                      sbR = rgbCh4[0]; sbG = rgbCh4[1]; sbB = rgbCh4[2]; tSlide = uvcol_out; tSlide -= 1;		 // UV colour (фиолет)
                      uvcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide = uvcol_out;
                    }

                    if (i == 5 && bitRead(LedShannelStatusByte, 5) == true) {
                      sbR = rgbCh5[0]; sbG = rgbCh5[1]; sbB = rgbCh5[2]; tSlide = ocol_out; tSlide -= 1;	    // OR colour (оранж)
                      ocol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide = ocol_out;
                    }

                    if (i == 6 && bitRead(LedShannelStatusByte, 6) == true) {
                      sbR = rgbCh6[0]; sbG = rgbCh6[1]; sbB = rgbCh6[2]; tSlide = grcol_out; tSlide -= 1;		    // GR colour (зелен)
                      grcol_out = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2); tSlide = grcol_out;
                    }

                    //if (ledON==0){ // включение управления каналами при настройке пресетов
                    _delay_ms(100);                    // delay 50msec after touch UP/DOWN button
                    LED_levelo_output();
                  }
                }
              }
          SliderSwitch  = false;
          break;

        case 14:  //------------- GENERAL SETTINGS (PAGE 1) -----------------
          if ((x >= backGS[0]) && (x <= backGS[2]) && (y >= backGS[1]) && (y <= backGS[3])) { // press back
            waitForIt(backGS[0], backGS[1], backGS[2], backGS[3]);
            LEDtestTick = false;
            ReadFromEEPROM(); dispScreen = 1; clearScreen(); menuScreen();
          } else

            if ((x >= nextGS[0]) && (x <= nextGS[2]) && (y >= nextGS[1]) && (y <= nextGS[3])) { // press next
              waitForIt(nextGS[0], nextGS[1], nextGS[2], nextGS[3]);
              dispScreen = 15; clearScreen(); generalSettingsScreen_2();
            } else

              if ((x >= prSAVEgs[0]) && (x <= prSAVEgs[2]) && (y >= prSAVEgs[1]) && (y <= prSAVEgs[3])) { // press SAVE
                waitForIt(prSAVEgs[0], prSAVEgs[1], prSAVEgs[2], prSAVEgs[3]);
                SaveGenSetsToEEPROM();
                SaveLEDsFailsafeToEEPROM(); dispScreen = 1; clearScreen(); menuScreen();
              }

          if ((x >= canCgs[0]) && (x <= canCgs[2]) && (y >= canCgs[1]) && (y <= canCgs[3])) { // press cancel
            waitForIt(canCgs[0], canCgs[1], canCgs[2], canCgs[3]);
            LEDtestTick = false;
            ReadFromEEPROM(); dispScreen = 0; clearScreen(); mainScreen(true);
          } else

            if (x >= 195 && x <= 295 && y >= 20 && y <= 40) { // press DETECT button
              waitForIt(195, 20, 295, 40);
              clearScreen(); dispScreen = 30; DetectDallalsSensors(true);
            }

          if (x >= 205 && x <= 285 && y >= 54 && y <= 74) { // press BACKUP button
            waitForIt(205, 54, 285, 74);
            clearScreen(); dispScreen = 31; Backup();
          }

          if ((x >= 205) && (x <= 285) && (y >= 123) && (y <= 143)) { // brignt
            waitForIt(205, 123, 285, 143);
            dispScreen = 33; clearScreen(); LCDbrigh(true);
          }

          if (x >= 205 && x <= 285 && y >= 88 && y <= 108 && bitRead(GlobalStatus1Byte, 0) == false) { // press RESET button
            waitForIt(205, 88, 285, 108);
            myGLCD.setColor(0, 0, 0);
            myGLCD.fillRoundRect(205, 88, 305, 108);    // clear button position
            bitSet(GlobalStatus1Byte, 0);               // set bit for check Y/N button

            myGLCD.setColor(255, 0, 0);
            myGLCD.fillRoundRect(195, 88, 235, 108);       // YES
            myGLCD.setColor(0, 0, 255);
            myGLCD.fillRoundRect(255, 88, 295, 108);       // NO
            myGLCD.setColor(255, 255, 255);
            myGLCD.setBackColor(255, 0, 0);        // фон красный
            myGLCD.drawRoundRect(195, 88, 235, 108);       // YES
            myGLCD.setBackColor(0, 0, 255);        // фон синий
            myGLCD.drawRoundRect(255, 88, 295, 108);       // NO

            setFont(SMALL, 255, 255, 255, 255, 0, 0);
            strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[8])));
            myGLCD.print(buffer, 195 + 8, 92);  // YES
            setFont(SMALL, 255, 255, 255, 0, 0, 255);
            strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[9])));
            myGLCD.print(buffer, 255 + 16, 92); // NO
            myGLCD.setFont(RusFont2);
            myGLCD.setColor(255, 0, 0);
            myGLCD.setBackColor(0, 0, 0);
            myGLCD.print(print_text[200], 20, 92);     // СБРОСИТЬ НАСТРОЙКИ ?
          } else

            if (x >= 195 && x <= 235 && y >= 88 && y <= 108 && bitRead(GlobalStatus1Byte, 0) == true) { // press Yes button
              waitForIt(195, 88, 235, 108);  clearFscreen();
              myGLCD.setFont(RusFont1);
              myGLCD.setColor(255, 0, 0);
              myGLCD.setBackColor(0, 0, 0);
              myGLCD.print(print_text[191], CENTER, 110);     // WAIT, est le nettoyage de la mémoire
              resetScreen();
            } else

              if (x >= 255 && x <= 295 && y >= 88 && y <= 108 && bitRead(GlobalStatus1Byte, 0) == true) { // press NO button
                waitForIt(255, 88, 295, 108);
                dispScreen = 14; clearScreen(); generalSettingsScreen_1();
                bitClear(GlobalStatus1Byte, 0);
              }                   // clear bit for check Y/N button

          if ((x >= 195) && (x <= 295) && (y >= 159) && (y <= 179)) { // press CHANGE TEMPS
            waitForIt(195, 159, 295, 179);
            ReadFromEEPROM(); dispScreen = 16; clearScreen(); ChangeFanTempsScreen(true);
          }
          break;

        case 15:  //------------- GENERAL SETTINGS (PAGE 2) -----------------
          if ((x >= backGS[0]) && (x <= backGS[2]) && (y >= backGS[1]) && (y <= backGS[3])) { // press back
            waitForIt(backGS[0], backGS[1], backGS[2], backGS[3]);
            dispScreen = 14; clearScreen(); generalSettingsScreen_1();
          }

          if ((x >= nextGS[0]) && (x <= nextGS[2]) && (y >= nextGS[1]) && (y <= nextGS[3])) { // press next
            waitForIt(nextGS[0], nextGS[1], nextGS[2], nextGS[3]);
            dispScreen = 9; clearScreen(); generalSettingsScreen_3();
          }

          if ((x >= prSAVEgs[0]) && (x <= prSAVEgs[2]) && (y >= prSAVEgs[1]) && (y <= prSAVEgs[3])) { // press SAVE
            waitForIt(prSAVEgs[0], prSAVEgs[1], prSAVEgs[2], prSAVEgs[3]);
            SaveGenSetsToEEPROM(); SaveDimmLEDToEEPROM();
            dispScreen = 1; clearScreen(); menuScreen();
          }

          if ((x >= canCgs[0]) && (x <= canCgs[2]) && (y >= canCgs[1]) && (y <= canCgs[3])) { // press cancel
            waitForIt(canCgs[0], canCgs[1], canCgs[2], canCgs[3]);
            dispScreen = 0; clearScreen(); mainScreen(true);
          }


          if ((x >= 185) && (x <= 305) && (y >= 55) && (y <= 75)) { // press CHANGE TEMP (Dim LEDs)
            waitForIt(185, 55, 305, 75);
            // ReadLedFromEEPROM();
            dispScreen = 17; clearScreen(); DimLEDsAtTempScreen();
          }

          if ((x >= 185) && (x <= 305) && (y >= 129) && (y <= 149)) { // press SETTINGS (Screensaver)
            waitForIt(185, 129, 305, 149);
            dispScreen = 18; clearScreen(); ScreensaverSettingsScreen();
          }

          if ((x >= 195) && (x <= 235)) {                           // first column
            if ((y >= 27) && (y <= 47)) {
              waitForIt(195, 27, 235, 47);   // press ON (Dim LEDs)
              setDimLEDsOnOff = 1; genSetSelect_2();
            }

            if ((y >= 101) && (y <= 121)) {
              waitForIt(195, 101, 235, 121);  // press ON (Screensaver)
              setScreensaverOnOff = 1; genSetSelect_2();
            }

            if ((y >= 169) && (y <= 189)) {
              waitForIt(195, 169, 235, 189);     // press ON (Dimm)
              DimmL = 1; genSetSelect_2(); dispScreen = 32; SetDimm();
            }
          }

          if ((x >= 255) && (x <= 295)) {                             // second column
            if ((y >= 27) && (y <= 47)) {
              waitForIt(255, 27, 295, 47);       // press OFF (Dim LEDs)
              setDimLEDsOnOff = 0; genSetSelect_2();
            }

            if ((y >= 101) && (y <= 121)) {
              waitForIt(255, 101, 295, 121);    // press OFF (Screensaver)
              setScreensaverOnOff = 0; genSetSelect_2();
            }

            if ((y >= 169) && (y <= 189)) {
              waitForIt(255, 169, 295, 189);    // press OFF (Dimm)
              DimmL = 0; genSetSelect_2();
            }
          }

          break;

        case 16: //------------------ CHANGE Heatsink FAN TEMP ---------------------
          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3])) { // press back
            waitForIt(back[0], back[1], back[2], back[3]);
            dispScreen = 14; clearScreen(); generalSettingsScreen_1();
          }

          if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) { // press SAVE
            waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
            setTempToBeginHeatsink1FanC = temp2beHFan;
            setTempToBeginHeatsink2FanC = temp2beSFan;
            dispScreen = 14; SaveGenSetsToEEPROM(); SaveTempToEEPROM(); clearScreen();
            generalSettingsScreen_1();
          }

          if ((x >= canC[0]) && (x <= canC[2]) && (y >= canC[1]) && (y <= canC[3])) { // press cancel
            waitForIt(canC[0], canC[1], canC[2], canC[3]);
            LEDtestTick = false;
            dispScreen = 0; clearScreen(); mainScreen(true);
          }

          if ((x >= SalaRm[0]) && (x <= SalaRm[2]) && (y >= SalaRm[1]) && (y <= SalaRm[3])) { // press Sound alarm
            waitForIt(SalaRm[0], SalaRm[1], SalaRm[2], SalaRm[3]);
            dispScreen = 35; SoundAlarm();
          } //else

          setFont(LARGE, 255, 255, 255, 0, 0, 0);
          if ((x >= HoodFanTm[0]) && (x <= HoodFanTm[2])) {      // first column
            if ((y >= HoodFanTm[1]) && (y <= HoodFanTm[3])) {      // press Heatsink1 Fan Temp -0.1
              temp2beHFan -= 0.1;

              if (temp2beHFan <= 25.0) {
                temp2beHFan = 25.0;  // минимум температуры
              }
              ChangeFanTempsScreen(); _delay_ms(140);
            }

            if ((y >= SumpFanTm[1]) && (y <= SumpFanTm[3])) {
              temp2beSFan -= 0.1;  // Радиатор датчик2 Fan Temp -0.1
              if (temp2beSFan <= 25.0) {
                temp2beSFan = 25.0;
              }
              ChangeFanTempsScreen(); _delay_ms(140);
            }
          }

          if ((x >= HoodFanTp[0]) && (x <= HoodFanTp[2])) {         // second column
            if ((y >= HoodFanTp[1]) && (y <= HoodFanTp[3])) {
              temp2beHFan += 0.1; // Радиатор датчик1 Fan Temp +0.1
              if (temp2beHFan >= 50.0) {
                temp2beHFan = 50.0;
              }
              ChangeFanTempsScreen(); _delay_ms(140);
            }

            if ((y >= SumpFanTp[1]) && (y <= SumpFanTp[3])) {
              temp2beSFan += 0.1;  // press Heatsink2 Fan Temp +0.1
              if (temp2beSFan >= 50.0) {
                temp2beSFan = 50.0;
              }
              ChangeFanTempsScreen(); _delay_ms(140);
            }
          }
          break;

        case 17:  // Температура авто-уменьшения яркости светильника
          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3])) { // press back
            waitForIt(back[0], back[1], back[2], back[3]);
            dispScreen = 15; clearScreen(); generalSettingsScreen_2();
          }

          if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) { // press SAVE
            waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
            setLEDsDimTempC = TempLEDsDimTemp; //tempLED=255;
            TempLEDsDimTemp = setDimLEDsOnOff;
            setLEDsDimPercent = TempLEDsDimPercent;
            TempLEDsDimPercent = setLEDsDimPercent;
            SaveLEDsFailsafeToEEPROM();
            dispScreen = 15; clearScreen(); generalSettingsScreen_2();
          }

          if ((x >= canC[0]) && (x <= canC[2]) && (y >= canC[1]) && (y <= canC[3])) { // press cancel
            waitForIt(canC[0], canC[1], canC[2], canC[3]);
            LEDtestTick = false;
            ReadFromEEPROM();
            dispScreen = 0; clearScreen(); mainScreen(true);
          }

          if ((x >= 235) && (x <= 260) && (y >= 36) && (y <= 61)) { // press TEMP UP button
            waitForIt(235, 36, 260, 61); TempLEDsDimTemp++;

            setFont(LARGE, 255, 108, 72, 0, 0, 0); // красный шрифт
            if (TempLEDsDimTemp >= 255) {
              TempLEDsDimTemp = 255; // 255
            }
            if (TempLEDsDimTemp >= 100) {
              myGLCD.printNumI(TempLEDsDimTemp, 181, 55);
            }
            if ((TempLEDsDimTemp <= 99) && (TempLEDsDimTemp >= 10)) {
              myGLCD.printNumI(TempLEDsDimTemp, 189, 55);
            }

            if (TempLEDsDimTemp <= 9) {
              myGLCD.printNumI(TempLEDsDimTemp, 198, 55);
            }
          } else

            if ((x >= 235) && (x <= 260) && (y >= 66) && (y <= 91)) { // press TEMP DOWN button
              waitForIt(235, 66, 260, 91); TempLEDsDimTemp--;
              setFont(LARGE, 255, 108, 72, 0, 0, 0);
              if (TempLEDsDimTemp <= 0) {
                TempLEDsDimTemp = 0;
              }
              if (TempLEDsDimTemp >= 100) {
                myGLCD.printNumI(TempLEDsDimTemp, 181, 55);
              }
              if ((TempLEDsDimTemp <= 99) && (TempLEDsDimTemp >= 10)) {
                myGLCD.setColor(0, 0, 0);
                myGLCD.fillRect(181, 55, 188, 71);
                myGLCD.fillRect(221, 55, 229, 71);
                setFont(LARGE, 0, 255, 0, 0, 0, 0);
                myGLCD.printNumI(TempLEDsDimTemp, 189, 55);
              }

              if (TempLEDsDimTemp <= 9) {
                myGLCD.setColor(0, 0, 0);
                myGLCD.fillRect(181, 55, 197, 71);
                myGLCD.fillRect(214, 55, 229, 71);
                setFont(LARGE, 0, 255, 0, 0, 0, 0);
                myGLCD.printNumI(TempLEDsDimTemp, 198, 55);
              }
            } else

              if ((x >= 235) && (x <= 260) && (y >= 117) && (y <= 142)) { // press % UP button
                waitForIt(235, 117, 260, 142); TempLEDsDimPercent++;
                setFont(LARGE, 255, 255, 255, 0, 0, 0);
                if (TempLEDsDimPercent >= 100) {
                  TempLEDsDimPercent = 100;
                }
                if (TempLEDsDimPercent >= 100) {
                  myGLCD.printNumI(TempLEDsDimPercent, 181, 136);
                }
                if ((TempLEDsDimPercent <= 99) && (TempLEDsDimPercent >= 10)) {
                  myGLCD.printNumI(TempLEDsDimPercent, 189, 136);
                }
                if (TempLEDsDimPercent <= 9) {
                  myGLCD.printNumI(TempLEDsDimPercent, 198, 136);
                }
              } else

                if ((x >= 235) && (x <= 260) && (y >= 147) && (y <= 172)) { // press % DOWN button
                  waitForIt(235, 147, 260, 172); TempLEDsDimPercent--;
                  setFont(LARGE, 255, 255, 255, 0, 0, 0);
                  if (TempLEDsDimPercent <= 0) {
                    TempLEDsDimPercent = 0;
                  }
                  if ((TempLEDsDimPercent <= 99) && (TempLEDsDimPercent >= 10)) {
                    myGLCD.setColor(0, 0, 0);
                    myGLCD.fillRect(181, 136, 188, 152);
                    myGLCD.fillRect(221, 136, 229, 152);
                    setFont(LARGE, 0, 255, 0, 0, 0, 0);
                    myGLCD.printNumI(TempLEDsDimPercent, 189, 136);
                  }
                  if (TempLEDsDimPercent <= 9) {
                    myGLCD.setColor(0, 0, 0);
                    myGLCD.fillRect(181, 136, 197, 152);
                    myGLCD.fillRect(214, 136, 229, 152);
                    setFont(LARGE, 0, 255, 0, 0, 0, 0);
                    myGLCD.printNumI(TempLEDsDimPercent, 198, 136);
                  }
                }
          break;

        case 18:  //-------------- SET SCREENSAVER WAIT TIME ----------------
          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3])) { // press back
            waitForIt(back[0], back[1], back[2], back[3]);
            dispScreen = 15; clearScreen(); generalSettingsScreen_2();
          } else

            if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) { // press SAVE
              waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
              setSSmintues = TempSSminutes; SaveGenSetsToEEPROM();
              dispScreen = 0; clearScreen(); mainScreen(true);
            } else

              if ((x >= canC[0]) && (x <= canC[2]) && (y >= canC[1]) && (y <= canC[3])) { // press cancel
                waitForIt(canC[0], canC[1], canC[2], canC[3]);
                LEDtestTick = false;
                ReadFromEEPROM(); dispScreen = 0; clearScreen(); mainScreen(true);
              } else

                if ((x >= 185) && (x <= 235)) {            // first column
                  if ((y >= 20) && (y <= 40)) {          // press BLANK screensaver
                    waitForIt(185, 20, 235, 40); setClockOrBlank = 1; ScreensaverSelect();
                  }

                  if ((y >= 51) && (y <= 71)) {              // press YES, show Date on Screensaver
                    waitForIt(185, 51, 235, 71); setScreensaverDOWonOff = 1; ScreensaverSelect();
                  }
                }

          if ((x >= 255) && (x <= 305)) {              // second column
            if ((y >= 20) && (y <= 40)) {              // press CLOCK screensaver
              waitForIt(255, 20, 305, 40); setClockOrBlank = 0; ScreensaverSelect();
            }

            if ((y >= 51) && (y <= 71)) {              // press NO, show Date on Screensaver
              waitForIt(255, 51, 305, 71); setScreensaverDOWonOff = 0; ScreensaverSelect();
            }
          }

          if ((x >= 245) && (x <= 301) && (y >= 100) && (y <= 119)) { // Выбрать A.Clock скринсейв
            waitForIt(245, 100, 301, 119);
            //  digital = 1; analog = 0;
            setScreensaverTupe = 1; ScreensaverSelect();
          }

          if ((x >= 245) && (x <= 301) && (y >= 132) && (y <= 152)) { // Выбрать D.Clock скринсейв
            waitForIt(245, 132, 301, 152);
            //  analog = 1; digital = 0;
            setScreensaverTupe = 0; ScreensaverSelect();
          }

          if ((x >= 135) && (x <= 160) && (y >= 92) && (y <= 117)) { // Кнопка минуты плюс
            waitForIt(135, 92, 160, 117); TempSSminutes++; _delay_ms(10);
            myGLCD.setFont(DotMatrix_M_Num);  // Выбор шрифта
            myGLCD.setColor(80, 255, 246);    // цвет голубой
            myGLCD.setBackColor(0, 0, 0);     // цвет фона черный

            if (TempSSminutes >= 99) {
              TempSSminutes = 99;
            }
            if (TempSSminutes >= 10) {
              myGLCD.printNumI(TempSSminutes, 87, 112);
            } else {   // Время включения скринсейва
              myGLCD.printNumI(TempSSminutes, 102, 112);
            }
          } else

            if ((x >= 135) && (x <= 160) && (y >= 132) && (y <= 157)) { // Кнопка минуты минус
              waitForIt(135, 132, 160, 157); TempSSminutes--; _delay_ms(10);
              myGLCD.setFont(DotMatrix_M_Num);  // Выбор шрифта
              myGLCD.setColor(80, 255, 246);    // цвет голубой
              myGLCD.setBackColor(0, 0, 0);     // цвет фона черный

              if (TempSSminutes <= 1) {
                TempSSminutes = 1;
              }
              if (TempSSminutes >= 10) {
                myGLCD.printNumI(TempSSminutes, 87, 112);
              }
              else {
                myGLCD.printNumI(TempSSminutes, 102, 112);
                myGLCD.setColor(0, 0, 0);
                myGLCD.fillRect(87, 112, 98, 135);
              }
            }  // очистка при смене цифр

          break;

        case 19:  // Timer
          // Таймер 1
          if ((x >= 5) && (x <= 58) && (y >= 20) && (y <= 186)) {
            waitForIt(5, 20, 58, 186);     // координаты кнопки
            dispScreen = 20; clearScreen(); light1set();
          }
          // Таймер 2
          if ((x >= 69) && (x <= 122) && (y >= 20) && (y <= 186)) {
            waitForIt(69, 20, 122, 186); // координаты кнопки
            dispScreen = 21; clearScreen(); light2set();
          }
          // Таймер 3
          if ((x >= 133) && (x <= 186) && (y >= 20) && (y <= 186)) {
            waitForIt(133, 20, 186, 186);// координаты кнопки
            dispScreen = 22; clearScreen(); light3set();
          }
          // Таймер 4
          if ((x >= 197) && (x <= 250) && (y >= 20) && (y <= 186)) {
            waitForIt(197, 20, 250, 186);// координаты кнопки
            dispScreen = 23; clearScreen(); light4set();
          }
          // Таймер 5
          if ((x >= 261) && (x <= 314) && (y >= 20) && (y <= 186)) {
            waitForIt(261, 20, 314, 186);// координаты кнопки
            dispScreen = 24; clearScreen(); light5set();
          }

          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3])) { // press back
            waitForIt(back[0], back[1], back[2], back[3]);
            dispScreen = 1; clearScreen(); menuScreen();
          }
          break;

        case 20:  // Timer 1
          // Часы включения (плюс)
          if (x > btonhup[0] && x < btonhup[2] && y > btonhup[1] && y < btonhup[3]) {
            on1 = on1 + 60; // Часы включения +
          }
          if (x > btonhdn[0] && x < btonhdn[2] && y > btonhdn[1] && y < btonhdn[3]) {
            on1 = on1 - 60; // Часы включения -
          }
          // Минуты включения (плюс)
          if (x > btonmup[0] && x < btonmup[2] && y > btonmup[1] && y < btonmup[3]) {
            on1++; // Минуты включения +
          }
          if (x > btonmdn[0] && x < btonmdn[2] && y > btonmdn[1] && y < btonmdn[3]) {
            on1--; // Минуты включения -
          }
          // Часы выключения
          if (x > btofhup[0] && x < btofhup[2] && y > btofhup[1] && y < btofhup[3]) {
            off1 = off1 + 60; // Часы выключения +
          }
          if (x > btofhdn[0] && x < btofhdn[2] && y > btofhdn[1] && y < btofhdn[3]) {
            off1 = off1 - 60; // Часы выключения -
          }
          // Минуты выключения
          if (x > btofmup[0] && x < btofmup[2] && y > btofmup[1] && y < btofmup[3]) {
            off1++; // Минуты выключения +
          }
          if (x > btofmdn[0] && x < btofmdn[2] && y > btofmdn[1] && y < btofmdn[3]) {
            off1--; // Минуты выключения -
          }

          if (on1 < 0) {
            on1 = 1439; // кнопка (плюс) ON
          }
          if (on1 > 1439) {
            on1 = 0; // кнопка (минус) ON
          }
          if (off1 < 0) {
            off1 = 1439;
          }
          if (off1 > 1439) {
            off1 = 0; // установки таймера освещения канала 1
          }
          timer1Change(); _delay_ms(100);

          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3])) { // press back
            waitForIt(back[0], back[1], back[2], back[3]);
            dispScreen = 19; clearScreen(); TimerScreen();
          } else  // Суточные Таймеры

            // возврат в меню cуточных таймеров
            if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) { // press SAVE
              waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
              SaveTimerEEPROM(); _delay_ms(10); clearScreen(); dispScreen = 19; TimerScreen();
            }
          break;
        case 21:   // Timer 2
          // Часы включения (плюс)
          if (x > btonhup[0] && x < btonhup[2] && y > btonhup[1] && y < btonhup[3]) {
            on2 = on2 + 60; // Часы включения +
          }
          if (x > btonhdn[0] && x < btonhdn[2] && y > btonhdn[1] && y < btonhdn[3]) {
            on2 = on2 - 60; // Часы включения -
          }
          // Минуты включения (плюс)
          if (x > btonmup[0] && x < btonmup[2] && y > btonmup[1] && y < btonmup[3]) {
            on2++; // Минуты включения +
          }
          if (x > btonmdn[0] && x < btonmdn[2] && y > btonmdn[1] && y < btonmdn[3]) {
            on2--; // Минуты включения -
          }
          // Часы выключения
          if (x > btofhup[0] && x < btofhup[2] && y > btofhup[1] && y < btofhup[3]) {
            off2 = off2 + 60; // Часы выключения +
          }
          if (x > btofhdn[0] && x < btofhdn[2] && y > btofhdn[1] && y < btofhdn[3]) {
            off2 = off2 - 60; // Часы выключения -
          }
          // Минуты выключения
          if (x > btofmup[0] && x < btofmup[2] && y > btofmup[1] && y < btofmup[3]) {
            off2++; // Минуты выключения +
          }
          if (x > btofmdn[0] && x < btofmdn[2] && y > btofmdn[1] && y < btofmdn[3]) {
            off2--; // Минуты выключения -
          }

          if (on2 < 0) {
            on2 = 1439; // кнопка (плюс) ON
          }
          if (on2 > 1439) {
            on2 = 0; // кнопка (минус) ON
          }
          if (off2 < 0) {
            off2 = 1439;
          }
          if (off2 > 1439) {
            off2 = 0; // установки таймера освещения канала 1
          }
          timer2Change(); _delay_ms(100);

          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3])) { // press back
            waitForIt(back[0], back[1], back[2], back[3]);
            dispScreen = 19; clearScreen(); TimerScreen();
          } else  // Суточные Таймеры

            // запись настроек
            if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) { // press SAVE
              waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
              SaveTimerEEPROM(); _delay_ms(10); clearScreen(); dispScreen = 19; TimerScreen();
            }
          break;
        case 22:  // Timer 3
          // Часы включения (плюс)
          if (x > btonhup[0] && x < btonhup[2] && y > btonhup[1] && y < btonhup[3]) {
            on3 = on3 + 60; // Часы включения +
          }
          if (x > btonhdn[0] && x < btonhdn[2] && y > btonhdn[1] && y < btonhdn[3]) {
            on3 = on3 - 60; // Часы включения -
          }
          // Минуты включения (плюс)
          if (x > btonmup[0] && x < btonmup[2] && y > btonmup[1] && y < btonmup[3]) {
            on3++; // Минуты включения +
          }
          if (x > btonmdn[0] && x < btonmdn[2] && y > btonmdn[1] && y < btonmdn[3]) {
            on3--; // Минуты включения -
          }
          // Часы выключения
          if (x > btofhup[0] && x < btofhup[2] && y > btofhup[1] && y < btofhup[3]) {
            off3 = off3 + 60; // Часы выключения +
          }
          if (x > btofhdn[0] && x < btofhdn[2] && y > btofhdn[1] && y < btofhdn[3]) {
            off3 = off3 - 60; // Часы выключения -
          }
          // Минуты выключения
          if (x > btofmup[0] && x < btofmup[2] && y > btofmup[1] && y < btofmup[3]) {
            off3++; // Минуты выключения +
          }
          if (x > btofmdn[0] && x < btofmdn[2] && y > btofmdn[1] && y < btofmdn[3]) {
            off3--; // Минуты выключения -
          }

          if (on3 < 0) {
            on3 = 1439; // кнопка (плюс) ON
          }
          if (on3 > 1439) {
            on3 = 0;
          }
          if (off3 < 0) {
            off3 = 1439; // кнопка (минус)
          }
          if (off3 > 1439) {
            off3 = 0; // установки таймера освещения канала 3
          }
          timer3Change(); _delay_ms(100);

          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3])) { // press back
            waitForIt(back[0], back[1], back[2], back[3]);
            dispScreen = 19; clearScreen(); TimerScreen();
          } else  // Суточные Таймеры

            // запись настроек
            if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) { // press SAVE
              waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
              SaveTimerEEPROM(); _delay_ms(10); clearScreen(); dispScreen = 19; TimerScreen();
            }
          break;
        case 23:   // Timer 4
          // Часы включения (плюс)
          if (x > btonhup[0] && x < btonhup[2] && y > btonhup[1] && y < btonhup[3]) {
            on4 = on4 + 60; // Часы включения +
          }
          if (x > btonhdn[0] && x < btonhdn[2] && y > btonhdn[1] && y < btonhdn[3]) {
            on4 = on4 - 60; // Часы включения -
          }
          // Минуты включения (плюс)
          if (x > btonmup[0] && x < btonmup[2] && y > btonmup[1] && y < btonmup[3]) {
            on4++; // Минуты включения +
          }
          if (x > btonmdn[0] && x < btonmdn[2] && y > btonmdn[1] && y < btonmdn[3]) {
            on4--; // Минуты включения -
          }
          // Часы выключения
          if (x > btofhup[0] && x < btofhup[2] && y > btofhup[1] && y < btofhup[3]) {
            off4 = off4 + 60; // Часы выключения +
          }
          if (x > btofhdn[0] && x < btofhdn[2] && y > btofhdn[1] && y < btofhdn[3]) {
            off4 = off4 - 60; // Часы выключения -
          }
          // Минуты выключения
          if (x > btofmup[0] && x < btofmup[2] && y > btofmup[1] && y < btofmup[3]) {
            off4++; // Минуты выключения +
          }
          if (x > btofmdn[0] && x < btofmdn[2] && y > btofmdn[1] && y < btofmdn[3]) {
            off4--; // Минуты выключения -
          }

          if (on4 < 0) {
            on4 = 1439; // кнопка (плюс) ON
          }
          if (on4 > 1439) {
            on4 = 0;
          }
          if (off4 < 0) {
            off4 = 1439; // кнопка (минус)
          }
          if (off4 > 1439) {
            off4 = 0; // установки таймера освещения канала 4
          }
          timer4Change(); _delay_ms(100);

          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3])) { // press back
            waitForIt(back[0], back[1], back[2], back[3]);
            dispScreen = 19; clearScreen(); TimerScreen();
          } else  // Суточные Таймеры

            // запись настроек
            if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) { // press SAVE
              waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
              SaveTimerEEPROM(); _delay_ms(10); clearScreen(); dispScreen = 19; TimerScreen();
            }
          break;
        case 24:   // Timer 5
          // Часы включения (плюс)
          if (x > btonhup[0] && x < btonhup[2] && y > btonhup[1] && y < btonhup[3]) {
            on5 = on5 + 60; // Часы включения +
          }
          if (x > btonhdn[0] && x < btonhdn[2] && y > btonhdn[1] && y < btonhdn[3]) {
            on5 = on5 - 60; // Часы включения -
          }
          // Минуты включения (плюс)
          if (x > btonmup[0] && x < btonmup[2] && y > btonmup[1] && y < btonmup[3]) {
            on5++; // Минуты включения +
          }
          if (x > btonmdn[0] && x < btonmdn[2] && y > btonmdn[1] && y < btonmdn[3]) {
            on5--; // Минуты включения -
          }
          // Часы выключения
          if (x > btofhup[0] && x < btofhup[2] && y > btofhup[1] && y < btofhup[3]) {
            off5 = off5 + 60; // Часы выключения +
          }
          if (x > btofhdn[0] && x < btofhdn[2] && y > btofhdn[1] && y < btofhdn[3]) {
            off5 = off5 - 60; // Часы выключения -
          }
          // Минуты выключения
          if (x > btofmup[0] && x < btofmup[2] && y > btofmup[1] && y < btofmup[3]) {
            off5++; // Минуты выключения +
          }
          if (x > btofmdn[0] && x < btofmdn[2] && y > btofmdn[1] && y < btofmdn[3]) {
            off5--; // Минуты выключения -
          }

          if (on5 < 0) {
            on5 = 1439; // кнопка (плюс) ON
          }
          if (on5 > 1439) {
            on5 = 0;
          }
          if (off5 < 0) {
            off5 = 1439; // кнопка (минус)
          }
          if (off5 > 1439) {
            off5 = 0; // установки таймера освещения канала 5
          }
          timer5Change(); _delay_ms(100);

          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3])) { // press back
            waitForIt(back[0], back[1], back[2], back[3]);
            dispScreen = 19; clearScreen(); TimerScreen();
          } else  // Суточные Таймеры

            // запись настроек
            if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) { // press SAVE
              waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
              SaveTimerEEPROM(); _delay_ms(10); clearScreen(); dispScreen = 19; TimerScreen();
            }
          break;

        case 25: // ручное управление таймерами
          // Таймер 1
          if (x > 107 && x < 166 && y > 18 && y < 48) {
            _delay_ms(200);  // auto
            timer1Status = 0;
            onoff1();
          }
          if (x > 175 && x < 240 && y > 18 && y < 48) {
            _delay_ms(200);  // on
            timer1Status = 2;
            onoff1();
          }
          if (x > 245 && x < 313 && y > 18 && y < 48) {
            _delay_ms(200);  // off
            timer1Status = 3;
            onoff1();
          }
          // Таймер 2
          if (x > 107 && x < 166 && y > 54 && y < 84) {
            _delay_ms(200);
            timer2Status = 0;
            onoff2();
          }
          if (x > 175 && x < 240 && y > 54 && y < 84) {
            _delay_ms(200);
            timer2Status = 2;
            onoff2();
          }
          if (x > 245 && x < 313 && y > 54 && y < 84) {
            _delay_ms(200);
            timer2Status = 3;
            onoff2();
          }
          // Таймер 3
          if (x > 107 && x < 166 && y > 90 && y < 120) {
            _delay_ms(200);
            timer3Status = 0;
            onoff3();
          }
          if (x > 175 && x < 240 && y > 90 && y < 120) {
            _delay_ms(200);
            timer3Status = 2;
            onoff3();
          }
          if (x > 245 && x < 313 && y > 90 && y < 120) {
            _delay_ms(200);
            timer3Status = 3;
            onoff3();
          }
          // Таймер 4
          if (x > 107 && x < 166 && y > 126 && y < 156) {
            _delay_ms(200);
            timer4Status = 0;
            onoff4();
          }
          if (x > 175 && x < 240 && y > 126 && y < 156) {
            _delay_ms(200);
            timer4Status = 2;
            onoff4();
          }
          if (x > 245 && x < 313 && y > 126 && y < 156) {
            _delay_ms(200);
            timer4Status = 3;
            onoff4();
          }
          // Таймер 5
          if (x > 107 && x < 166 && y > 162 && y < 192) {
            _delay_ms(200);
            timer5Status = 0;
            onoff5();
          }
          if (x > 175 && x < 240 && y > 162 && y < 192) {
            _delay_ms(200);
            timer5Status = 2;
            onoff5();
          }
          if (x > 245 && x < 313 && y > 162 && y < 192) {
            _delay_ms(200);
            timer5Status = 3;
            onoff5();
          }
          break;

        case 26: // экран настройки яркости луны
          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3])) {
            waitForIt(back[0], back[1], back[2], back[3]);
            dispScreen = 40; clearScreen(); RGBTune();
          } else

            if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) { // press SAVE
              waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
              SaveMoonLEDToEEPROM(); dispScreen = 40; clearScreen(); RGBTune();
            } else

              if ((x >= canC[0]) && (x <= canC[2]) && (y >= canC[1]) && (y <= canC[3])) { // press CANCEL
                waitForIt(canC[0], canC[1], canC[2], canC[3]);
                COLOR = 8; ReadFromEEPROM(); LEDtestTick = false;
                dispScreen = 0; clearScreen(); mainScreen(true);
              } else

                // настройки ЛУНЫ
                if ((x >= MINiM[0]) && (x <= MINiM[2]) && (y >= MINiM[1]) && (y <= MINiM[3])) { // press MinI minus
                  waitForIt(MINiM[0], MINiM[1], MINiM[2], MINiM[3]); tMinI -= 1;
                  if (tMinI <= 0) {
                    tMinI = 0;
                  } MinI = tMinI;
                  setFont(LARGE, 255, 255, 255, 0, 0, 0); myGLCD.print(print_text[111], 55, 152);
                  if (tMinI <= 9) {
                    myGLCD.printNumI(tMinI, 71, 152);
                  }
                  if ((tMinI >= 10) && (tMinI <= 99)) {
                    myGLCD.printNumI(tMinI, 63, 152);
                  }
                  if (tMinI >= 100) {
                    myGLCD.printNumI(tMinI, 55, 152);
                  }
                } else

                  if ((x >= MINiP[0]) && (x <= MINiP[2]) && (y >= MINiP[1]) && (y <= MINiP[3])) { // press MinI plus
                    waitForIt(MINiP[0], MINiP[1], MINiP[2], MINiP[3]); tMinI += 5;
                    if (tMinI > 100) {
                      tMinI = 100;
                    } MinI = tMinI;    // 255
                    setFont(LARGE, 255, 255, 255, 0, 0, 0); myGLCD.print(print_text[111], 55, 152);
                    if (tMinI <= 9) {
                      myGLCD.printNumI(tMinI, 71, 152);
                    }
                    if ((tMinI >= 10) && (tMinI <= 99)) {
                      myGLCD.printNumI(tMinI, 63, 152);
                    }
                    if (tMinI >= 100) {
                      myGLCD.printNumI(tMinI, 55, 152);
                    }
                  } else

                    if ((x >= MAXiM[0]) && (x <= MAXiM[2]) && (y >= MAXiM[1]) && (y <= MAXiM[3])) { // press MaxI minus
                      waitForIt(MAXiM[0], MAXiM[1], MAXiM[2], MAXiM[3]); tMaxI -= 1;
                      if (tMaxI <= 0) {
                        tMaxI = 0;
                      } MaxI = tMaxI;
                      setFont(LARGE, 255, 255, 255, 0, 0, 0); myGLCD.print(print_text[111], 217, 152);
                      if (tMaxI <= 9) {
                        myGLCD.printNumI(tMaxI, 233, 152);
                      }
                      if ((tMaxI >= 10) && (tMaxI <= 99)) {
                        myGLCD.printNumI(tMaxI, 225, 152);
                      }
                      if (tMaxI >= 100) {
                        myGLCD.printNumI(tMaxI, 217, 152);
                      }
                    } else

                      if ((x >= MAXiP[0]) && (x <= MAXiP[2]) && (y >= MAXiP[1]) && (y <= MAXiP[3])) { // press MaxI plus
                        waitForIt(MAXiP[0], MAXiP[1], MAXiP[2], MAXiP[3]); tMaxI += 5;
                        if (tMaxI > 100) {
                          tMaxI = 100;
                        } MaxI = tMaxI;   // 255
                        setFont(LARGE, 255, 255, 255, 0, 0, 0); myGLCD.print(print_text[111], 217, 152);
                        if (tMaxI <= 9) {
                          myGLCD.printNumI(tMaxI, 233, 152);
                        }
                        if ((tMaxI >= 10) && (tMaxI <= 99)) {
                          myGLCD.printNumI(tMaxI, 225, 152);
                        }
                        if (tMaxI >= 100) {
                          myGLCD.printNumI(tMaxI, 217, 152);
                        }
                      }
          break;

        case 27:  // График ЛОГ температуры радиаторов F1, F2

          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3])) { // press back
            waitForIt(back[0], back[1], back[2], back[3]);
            dispScreen = 1; clearFscreen(); menuScreen();
          }

          if ((x >= canC[0]) && (x <= canC[2]) && (y >= canC[1]) && (y <= canC[3])) { // press CANCEL
            waitForIt(canC[0], canC[1], canC[2], canC[3]);
            LEDtestTick = false; dispScreen = 0; clearScreen(); mainScreen(true);
          }

          if ((x >= Fg1[0]) && (x <= Fg1[2]) && (y >= Fg1[1]) && (y <= Fg1[3])) {
            waitForIt(Fg1[0], Fg1[1], Fg1[2], Fg1[3]);
            if (F1 == true) {
              F1 = false;
              graph_colorFAN();
              timedrawScreen();
            }
            else {
              F1 = true;
              graph_colorFAN();
              timedrawScreen();
            }
          }

          if ((x >= Fg2[0]) && (x <= Fg2[2]) && (y >= Fg2[1]) && (y <= Fg2[3])) {
            waitForIt(Fg2[0], Fg2[1], Fg2[2], Fg2[3]);
            if (F2 == true) {
              F2 = false;
              graph_colorFAN();
              timedrawScreen();
            }
            else {
              F2 = true;
              graph_colorFAN();
              timedrawScreen();
            }
          }
          break;

        case 28:  // Лог температуры Воды в аквариуме

          if ((x >= backGS[0]) && (x <= backGS[2]) && (y >= backGS[1]) && (y <= backGS[3])) { // press << back
            waitForIt(backGS[0], backGS[1], backGS[2], backGS[3]);
            LEDtestTick = false;
            dispScreen = 1; clearFscreen(); menuScreen();
          } else     // выход в главное меню

            if ((x >= canCgs[0]) && (x <= canCgs[2]) && (y >= canCgs[1]) && (y <= canCgs[3])) { // press cancel
              waitForIt(canCgs[0], canCgs[1], canCgs[2], canCgs[3]);
              ReadFromEEPROM(); dispScreen = 0; clearFscreen(); mainScreen(true);
            }

          break;

        case 29: // авто-тест каналов с графиками

          if ((x >= Wg[0]) && (x <= Wg[2]) && (y >= Wg[1]) && (y <= Wg[3])) {    // W
            waitForIt(Wg[0], Wg[1], Wg[2], Wg[3]);
            if (W == true) {
              W = false;
              graph_color();
            } else {
              W = true;
              graph_color();
            }
          }
          if ((x >= RBg[0]) && (x <= RBg[2]) && (y >= RBg[1]) && (y <= RBg[3])) { // RB
            waitForIt(RBg[0], RBg[1], RBg[2], RBg[3]);
            if (RB == true) {
              RB = false;
              graph_color();
            } else {
              RB = true;
              graph_color();
            }
          }
          if ((x >= Bg[0]) && (x <= Bg[2]) && (y >= Bg[1]) && (y <= Bg[3])) {    // B
            waitForIt(Bg[0], Bg[1], Bg[2], Bg[3]);
            if (B == true) {
              B = false;
              graph_color();
            } else {
              B = true;
              graph_color();
            }
          }
          if ((x >= Rg[0]) && (x <= Rg[2]) && (y >= Rg[1]) && (y <= Rg[3])) {    // R
            waitForIt(Rg[0], Rg[1], Rg[2], Rg[3]);
            if (R == true) {
              R = false;
              graph_color();
            } else {
              R = true;
              graph_color();
            }
          }
          if ((x >= UVg[0]) && (x <= UVg[2]) && (y >= UVg[1]) && (y <= UVg[3])) { // UV
            waitForIt(UVg[0], UVg[1], UVg[2], UVg[3]);
            if (UV == true) {
              UV = false;
              graph_color();
            } else {
              UV = true;
              graph_color();
            }
          }
          if ((x >= GRg[0]) && (x <= GRg[2]) && (y >= GRg[1]) && (y <= GRg[3])) { // SU
            waitForIt(GRg[0], GRg[1], GRg[2], GRg[3]);
            if (SU == true) {
              SU = false;
              graph_color();
            } else {
              SU = true;
              graph_color();
            }
          }
          if ((x >= ORg[0]) && (x <= ORg[2]) && (y >= ORg[1]) && (y <= ORg[3])) { // OR
            waitForIt(ORg[0], ORg[1], ORg[2], ORg[3]);
            if (OR == true) {
              OR = false;
              graph_color();
            } else {
              OR = true;
              graph_color();
            }
          }

          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3]) // press back
              && (LEDtestTick == false)) {
            waitForIt(back[0], back[1], back[2], back[3]);
            LEDtestTick = false; ReadFromEEPROM(); ReadLedFromEEPROM();
            dispScreen = 1; clearFscreen(); menuScreen();
          }   // display buttons "Rapid test" and "Control Individual Leds"

          if ((x >= canC[0]) && (x <= canC[2]) && (y >= canC[1]) && (y <= canC[3]) // press CANCEL
              && (LEDtestTick == false)) {
            waitForIt(canC[0], canC[1], canC[2], canC[3]);
            LEDtestTick = false; ReadLedFromEEPROM(); dispScreen = 0; clearFscreen(); mainScreen(true);
          }

          // Кнопка Старт / Стоп
          if ((x >= ledChV[0]) && (x <= ledChV[2]) && (y >= ledChV[1]) && (y <= ledChV[3]) && dispScreen == 29) { // press start/stop test
            waitForIt(ledChV[0], ledChV[1], ledChV[2], ledChV[3]);

            //printButton(print_text[4], ledChV[0], ledChV[1], ledChV[2], ledChV[3], SMALL, GREEN_BAC); // STOP buton start/stop test
            myGLCD.setColor(255, 255, 255);
            printButton104(print_text[4], ledChV[0], ledChV[1], ledChV[2], ledChV[3], SMALL);          // красная кнопка

            if (LEDtestTick == true) {
              LEDtestTick = false;    // stop test
            } else {
              LEDtestTick = true;     // start test
              myGLCD.setColor(0, 0, 0);
              myGLCD.fillRect(26, BotSldY + 8, 318, BotSldY + 3); // clear test bar
              drawTestLedArrayScale();
            }
          }
          break;

        case 30:  // ============== меню авто-определения датчиков температуры ==================
          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3]) && dispScreen == 30 ) { // press back
            waitForIt(back[0], back[1], back[2], back[3]);
            dispScreen = 14; clearScreen(); generalSettingsScreen_1();
          } else

            if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) {    	// press SAVE
              waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
              DetectDallalsSensors(false);
              SaveDallasAddress(); dispScreen = 0; clearScreen(); mainScreen(true);
            } else

              if (x >= 165 && x <= 295 && y >= 19 && y <= 41) { // поиск датчиков
                waitForIt(165, 19, 295, 41);        // re-read all sensor data
                DetectDallalsSensors(false);
              }

          if ((x >= 165) && (x <= 231) && (y >= 83 + 20) && (y <= 83 + 40) && numberOfDevices >= 1) { // sensor N1 button
            waitForIt(165, 83 + 20, 231, 83 + 40); printCoun(); counterB1 += 1;

            if (counterB1  > 3) {
              counterB1 = 0;
            }
            strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[176 + counterB1])));
            myGLCD.print(buffer, 173, 85 + 24);
          } else  // counterBX = 0/1/2/3 -> Отключ / Д.Воды / Д.Рад:1 / Д.Рад:2

            if ((x >= 165) && (x <= 231) && (y >= 83 + 50) && (y <= 83 + 70) && numberOfDevices >= 2) { // sensor N2 button
              waitForIt(165, 83 + 50, 231, 83 + 70); printCoun(); counterB2 += 1;

              if (counterB2  > 3) {
                counterB2 = 0;
              }
              strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[176 + counterB2])));
              myGLCD.print(buffer, 173, 85 + 54);
            } else // counterBX = 0/1/2/3 -> Отключ./ Д.Воды / Д.Рад:1 / Д.Рад:2

              if ((x >= 165) && (x <= 231) && (y >= 83 + 80) && (y <= 83 + 100) && numberOfDevices == 3) { // sensor N3 button
                waitForIt(165, 83 + 80, 231, 83 + 100); printCoun(); counterB3 += 1;

                if (counterB3  > 3) {
                  counterB3 = 0;
                }
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[176 + counterB3])));
                myGLCD.print(buffer, 173, 85 + 84);
              }    // counterBX = 0/1/2/3 -> Отключ / Д.Воды / Д.Рад:1 / Д.Рад:2
          break;

        case 31:  // ----------- BACKUP ALL EEPROM SETTING -----------------
          if ((x >= back[0]) && (x <= back[2]) && (y >= back[1]) && (y <= back[3]) && dispScreen == 31 ) { // press back
            waitForIt(back[0], back[1], back[2], back[3]);
            dispScreen = 14; clearScreen(); generalSettingsScreen_1();
          } else

            if ((x >= 205 && x <= 305 && y >= 73 && y <= 109) && dispScreen == 31) { // BACKUP  сохранить настройки на флеш карту
              waitForIt(205, 73, 305, 109);
              myGLCD.setColor(0, 0, 0);			        // clear text area
              myGLCD.fillRoundRect(16, 126, 303, 181);
              sd.remove("Backup.txt");                                // remove old file from card
              myFile.open("Backup.txt", O_CREAT | O_EXCL | O_WRITE) ;

              if (myFile.isOpen()) {
                myGLCD.setFont(RusFont1);
                myGLCD.setColor(38, 195, 178);  // цвет бирюзовый
                myGLCD.setBackColor(0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[174])));
                myGLCD.print(buffer, CENTER, 139);       // Запись настроек на флеш карту

                for (int i = 0; i <= 4095; i++) {
                  byte TEMP = EEPROM.read(i);  // readed data in DEC format
                  myFile.print(TEMP, DEC);                                // store to file in DEC
                  myFile.print(',');
                }         // store separator "," between bytes
                myFile.close();

                myGLCD.setFont(RusFont1);
                myGLCD.setColor(0, 255, 0);  // цвет бирюзовый
                myGLCD.setBackColor(0, 0, 0);
                strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[175])));
                myGLCD.print(buffer, CENTER, 158);       // Настройки успешно сохранены
              } else {
                myGLCD.setColor(255, 0, 0);   // красный цвет
                myGLCD.setBackColor(0, 0, 0);
                myGLCD.print(print_text[199], CENTER, 158);
              }  // Ошибка открытия файла настроек
            } else

              if ((x >= 15 && x <= 115 && y >= 73 && y <= 109) && dispScreen == 31) { // press RESTORE button ВОССТАНОВИТЬ
                waitForIt(15, 73, 115, 109);
                myGLCD.setColor(0, 0, 0);			      // clear text area
                myGLCD.fillRoundRect(16, 126, 303, 181);

                // re-open the file for reading:
                if (myFile.open("Backup.txt", O_READ)) {
                  myGLCD.setFont(RusFont1);
                  myGLCD.setColor(0, 255, 0);  // цвет бирюзовый
                  myGLCD.setBackColor(0, 0, 0);
                  myGLCD.print(print_text[198], CENTER, 139);   // Чтение файла настроек на флеш карте
                  myGLCD.print(print_text[197], CENTER, 158);   // Пожалуйста подождите....

                  int TEMP; int i = 0; byte SD_buff[3]; byte k = 0;

                  // read from the file until there's nothing else in it:
                  while ((TEMP = myFile.read()) > 0) {   // readed data in ASCII only, need convert ASCII to DEC

                    //	  Serial.write((char)TEMP);		        // output to serial port
                    if (TEMP != 44 && k <= 2) {
                      TEMP = TEMP - '0';    // convert ASCII to DEC
                      SD_buff[k] = TEMP; k++;
                    } else {

                      int Result = SD_buff[0] * 100 + SD_buff[1] * 10 + SD_buff[2]; // convert three consecutive bytes to one decimal
                      if (k == 2) {
                        Result = Result / 10; // before save to eeprom
                      }
                      if (k == 1) {
                        Result = Result / 100;
                      } EEPROM.write(i, Result); // store data to EEPROM
                      i += 1; k = 0;
                      SD_buff[0] = 0;                          // clean buffer before new read
                      SD_buff[1] = 0;
                      SD_buff[2] = 0;
                    }
                  }
                  myFile.close();
                  ReadDallasAddress ();	                  // read temp sensor address from eeprom
                  ReadLedFromEEPROM();			  // read led setting from EEPROM
                  ReadFromEEPROM();			  // read other setting from eeprom

                  myGLCD.setFont(RusFont1);
                  myGLCD.setColor(0, 255, 0);
                  myGLCD.setBackColor(0, 0, 0);
                  myGLCD.print(print_text[196], CENTER, 158);     // Восстановление настроек завершено
                } else {
                  myGLCD.setColor(255, 0, 0);
                  myGLCD.setBackColor(0, 0, 0);
                  myGLCD.print(print_text[199], CENTER, 158);
                }
              } // Ошибка открытия файла настроек
          break;

        case 32:   //------------ Ограничение Мощности  ------------

          if ((x >= 95) && (x <= 135) && (y >= 185) && (y <= 205)) { // ok
            waitForIt (95, 185, 135, 205);
            setLEDsDimPercentL = TempsetLEDsDimPercentL;
            DimmL = 1; SaveDimmLEDToEEPROM();
            dispScreen = 15; clearScreen(); generalSettingsScreen_2();
          }

          if ((x >= 185) && (x <= 250) && (y >= 185) && (y <= 205)) { // cancel
            waitForIt (185, 185, 250, 205);
            dispScreen = 15; clearScreen(); generalSettingsScreen_2();
          }

          if ((x >= 175) && (x <= 200) && (y >= 107) && (y <= 132)) { // press Minute UP button
            TempsetLEDsDimPercentL++; _delay_ms(60);
            setFont(LARGE, 255, 255, 255, 0, 0, 0);
            if (TempsetLEDsDimPercentL >= 99) {
              TempsetLEDsDimPercentL = 99;
            }
            if (TempsetLEDsDimPercentL >= 10) {
              myGLCD.printNumI(TempsetLEDsDimPercentL, 129, 126);
            }
            else {
              myGLCD.printNumI(TempsetLEDsDimPercentL, 137, 126);
            }
          } else

            if ((x >= 175) && (x <= 200) && (y >= 137) && (y <= 162)) { // press Minute DOWN button
              TempsetLEDsDimPercentL--; _delay_ms(60);
              setFont(LARGE, 255, 255, 255, 0, 0, 0);
              if (TempsetLEDsDimPercentL <= 1) {
                TempsetLEDsDimPercentL = 1;
              }
              if (TempsetLEDsDimPercentL >= 10) {
                myGLCD.printNumI(TempsetLEDsDimPercentL, 129, 126);
              }
              else {
                myGLCD.printNumI(TempsetLEDsDimPercentL, 137, 126);
                myGLCD.setColor(0, 0, 0);
                myGLCD.fillRect(129, 126, 136, 142);
                myGLCD.fillRect(153, 126, 161, 142);
              }
            }
          break;

        case 33:  //========================= подсветка экрана  ===========================
          LCDbright = PlusMinusCountI (true, true, temM[0], temM[1], 150, 1, 100, 1, LCDbright);     // регулировка яркости
          if (PlsMnsPress == true) {
            PlsMnsPress = false;
          }

          LowbrH = PlusMinusCountI (true, true, intUP[0], intUP[1], 80, 0, 23, 1, LowbrH);               // время начала уменьшения яркости
          if (PlsMnsPress == true) {
            PlsMnsPress = false;
          }

          HightbrH = PlusMinusCountI (true, true, calUP[0], calUP[1], 80, 0, 23, 1, HightbrH);           // время начала нормальной яркости
          if (PlsMnsPress == true) {
            PlsMnsPress = false;
          }

          PressButton((0), NO , back[0], back[1], back[2], back[3]);                                          // Press back
          if (ButtonPress == true) {
            dispScreen = 14;  clearScreen(); generalSettingsScreen_1();
            ButtonPress = false;
          }

          PressButton((0), NO , prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);                                  // press SAVE
          if (ButtonPress == true) {
            SaveLCDbrightToEEPROM();
            ButtonPress = false;
          }

          PressButton((0), NO , canC[0], canC[1], canC[2], canC[3]);                                          // Press CANCEL
          if (ButtonPress == true) {
            dispScreen = 0; clearScreen(); ReadLCDbright(); mainScreen(true);
            ButtonPress = false;
          }

          /*
             if ((x>=165) && (x<=311) && (y>=151) && (y<=179)){     // загрузить настройки
                   waitForIt (165, 151, 311, 179);
                 ReadLCDbright();
               LCDbrigh(); } else

             if ((x>=165) && (x<=311) && (y>=61) && (y<=89)){   // сохранить настройки
                   waitForIt (165, 61, 311, 89); LCDbright = tmpLCDbright;
                    byte bout = map(LCDbright, 0, 100, 2, 255); // 255
                    analogWrite(LCDbrightPin, bout);
            SaveLCDbrightToEEPROM();
                    dispScreen=0; clearScreen(); mainScreen(true); } else

              if ((x>=gseB[0]) && (x<=gseB[0]+29) && (y>=gseB[1]) && (y<=gseB[1]+145)){
                if ((y>=gseB[1]) && (y<=gseB[1]-15)){ drawUpButtonSlide(gseB[0], gseB[1]-15);         // plus   gseB[1]-10
                 if (tmpLCDbright <100) tmpLCDbright++; } else

              if ((y>=gseB[1]+23) && (y<=gseB[1]+122)) tmpLCDbright = gseB[1] + 127 - y; else
                if ((y>=gseB[1]) && (y<=gseB[1]+140)){ drawDownButtonSlide(gseB[0], gseB[1]+140);      // minus   gseB[1]+134
                 if (tmpLCDbright>0) tmpLCDbright--; }
                    byte bout = map(tmpLCDbright, 0, 100, 2, 255); // 255
                    analogWrite(LCDbrightPin, bout); LCDbrigh(); }
          */
          break;

        case 34:
          break;

        case 35: // настройка звуковой тревоги при перегреве радиатора (case 16:)
          if ((x >= 95) && (x <= 135) && (y >= 185) && (y <= 205)) { // ok
            waitForIt (95, 185, 135, 205);
            SaveTempToEEPROM();
            dispScreen = 16; clearScreen(); ChangeFanTempsScreen(true);
          }

          if ((x >= 175) && (x <= 240) && (y >= 185) && (y <= 205)) { // cancel
            waitForIt (175, 185, 240, 205);
            dispScreen = 16; clearScreen(); ChangeFanTempsScreen(true);
          }

          //------ звуковая тревога при перегреве радиатора
          if ((x >= SoundATm[0]) && (x <= SoundATm[2])) {
            if ((y >= SoundATm[1]) && (y <= SoundATm[3])) {             // press Sound Alarm -
              setTempToSoundAlarmC -= 1; _delay_ms(150);
              if (setTempToSoundAlarmC == 254) {
                setTempToSoundAlarmC = 99;
              }
              if (setTempToSoundAlarmC <= 39.0) {
                setTempToSoundAlarmC = 255;
              }
              setSalarm();
            }
          } else

            if ((x >= SoundATp[0]) && (x <= SoundATp[2])) {
              if ((y >= SoundATp[1]) && (y <= SoundATp[3])) {            // press Sound Alarm +
                setTempToSoundAlarmC += 1; _delay_ms(150);
                if (setTempToSoundAlarmC == 256) {
                  setTempToSoundAlarmC = 40;
                }
                if (setTempToSoundAlarmC > 99.0) {
                  setTempToSoundAlarmC = 255; // OFF alarm
                }
                setSalarm();
              }
            }
          break;

        case 36: // ДОЗАТОР УДО
          //--------------- AUTOMATIC DOSER PAGE --------------                                                                                          ДОЗАТОР
          if ((x >= dos1b[0]) && (x <= dos1b[2]) && (y >= dos1b[1]) && (y <= dos1b[3])) //press Feeding Time 1
          {
            waitForIt(dos1b[0], dos1b[1], dos1b[2], dos1b[3]);
            dozTime = 1;
            dispScreen = 37;
            clearScreen();
            setDoserTimesScreen();
          } else if ((x >= dos2b[0]) && (x <= dos2b[2]) && (y >= dos2b[1]) && (y <= dos2b[3])) //press Feeding Time 2
          {
            waitForIt(dos2b[0], dos2b[1], dos2b[2], dos2b[3]);
            dozTime = 2;
            dispScreen = 37;
            clearScreen();
            setDoserTimesScreen();
          } else if ((x >= dos3b[0]) && (x <= dos3b[2]) && (y >= dos3b[1]) && (y <= dos3b[3])) //press Feeding Time 3
          {
            waitForIt(dos3b[0], dos3b[1], dos3b[2], dos3b[3]);
            dozTime = 3;
            dispScreen = 37;
            clearScreen();
            setDoserTimesScreen();
          } else if ((x >= dos4b[0]) && (x <= dos4b[2]) && (y >= dos4b[1]) && (y <= dos4b[3])) //press Feeding Time 4
          {
            waitForIt(dos4b[0], dos4b[1], dos4b[2], dos4b[3]);
            dozTime = 4;
            dispScreen = 37;
            clearScreen();
            setDoserTimesScreen();
          } else if ((x >= dosval1[0]) && (x <= dosval1[2]) && (y >= dosval1[1]) && (y <= dosval1[3])) //КАЛИБРОВКА 1
          {
            waitForIt(dosval1[0], dosval1[1], dosval1[2], dosval1[3]);
            dispScreen = 10;
            CalMode = 1;
            clearScreen();
            doscalibrateScreen();
          }
          if ((x >= dosval2[0]) && (x <= dosval2[2]) && (y >= dosval2[1]) && (y <= dosval2[3])) //КАЛИБРОВКА 2
          {
            waitForIt(dosval2[0], dosval2[1], dosval2[2], dosval2[3]);
            dispScreen = 10;
            CalMode = 2;
            clearScreen();
            doscalibrateScreen();
          }
          if ((x >= dosval3[0]) && (x <= dosval3[2]) && (y >= dosval3[1]) && (y <= dosval3[3])) //КАЛИБРОВКА 3
          {
            waitForIt(dosval3[0], dosval3[1], dosval3[2], dosval3[3]);
            dispScreen = 10;
            CalMode = 3;
            clearScreen();
            doscalibrateScreen();
          }
          if ((x >= dosval4[0]) && (x <= dosval4[2]) && (y >= dosval4[1]) && (y <= dosval4[3])) //КАЛИБРОВКА 4
          {
            waitForIt(dosval4[0], dosval4[1], dosval4[2], dosval4[3]);
            dispScreen = 10;
            CalMode = 4;
            clearScreen();
            doscalibrateScreen();
          }
          break;

        case 37:  // УСТАНОВКА ВРЕМЕНИ ДОЗИРОВАНИЯ
          //------------ SET AUTOMATIC DOSER TIMES ------------
          if ((x >= back[0]) && (x <= back[2]) && (y > back[1]) && (y <= back[3]))  //press back
          {
            waitForIt(back[0], back[1], back[2], back[3]);
            if ((timeDispH >= 0) && (timeDispH <= 11)) {
              AM_PM = 1;
            }
            else {
              AM_PM = 2;
            }
            dispScreen = 36;
            clearScreen();
            autoDoserScreen();
          } else if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) //press SAVE
          {
            waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
            if (setTimeFormat == 1)
            {
              if ((rtcSetHr == 0) && (AM_PM == 2))
              {
                rtcSetHr += 12;
              }
              if (((rtcSetHr >= 1) && (rtcSetHr <= 11)) && (AM_PM == 2))
              {
                rtcSetHr += 12;
              }
              if (((rtcSetHr >= 12) && (rtcSetHr <= 23)) && (AM_PM == 1))
              {
                rtcSetHr -= 12;
              }
            }
            if (dozTime == 1)
            {
              dozPump1H = rtcSetHr;
              dozPump1M = rtcSetMin;
            }
            if (dozTime == 2)
            {
              dozPump2H = rtcSetHr;
              dozPump2M = rtcSetMin;
            }
            if (dozTime == 3)
            {
              dozPump3H = rtcSetHr;
              dozPump3M = rtcSetMin;
            }
            if (dozTime == 4)
            {
              dozPump4H = rtcSetHr;
              dozPump4M = rtcSetMin;
            }
            SaveDoseTimesToEEPROM();
            dispScreen = 36;
            clearScreen();
            autoDoserScreen();
          } else if ((x >= 70) && (x <= 250) && (y >= 150) && (y <= 170)) //Вкл выкл дозатора ON/OFF
          {
            waitForIt(70, 150, 250, 170);
            if (dozTime == 1)
            { if (DOZTime1 == 1) {
                DOZTime1 = 0;
              }
              else {
                DOZTime1 = 1;
              }
            }
            if (dozTime == 2)
            { if (DOZTime2 == 1) {
                DOZTime2 = 0;
              }
              else {
                DOZTime2 = 1;
              }
            }
            if (dozTime == 3)
            { if (DOZTime3 == 1) {
                DOZTime3 = 0;
              }
              else {
                DOZTime3 = 1;
              }
            }
            if (dozTime == 4)
            { if (DOZTime4 == 1) {
                DOZTime4 = 0;
              }
              else {
                DOZTime4 = 1;
              }
            }
            dosingTimeOnOff();
          }
          else
          {
            if ((y >= houP[1]) && (y <= houP[3]))           //FIRST ROW
            {
              if ((x >= houP[0]) && (x <= houP[2]))        //press hour up
              {
                waitForIt(houP[0], houP[1], houP[2], houP[3]);
                rtcSetHr++;
                if (rtcSetHr >= 24)
                {
                  rtcSetHr = 0;
                }
              }
              if ((x >= minP[0]) && (x <= minP[2]))        //press min up
              {
                waitForIt(minP[0], minP[1], minP[2], minP[3]);
                rtcSetMin = rtcSetMin + 30;
                if (rtcSetMin > 30) {
                  rtcSetMin = 0;
                }
              }
              if ((x >= ampmP[0]) && (x <= ampmP[2])       //press AMPM up
                  && (setTimeFormat == 1))
              {
                waitForIt(ampmP[0], ampmP[1], ampmP[2], ampmP[3]);
                if (AM_PM == 1) {
                  AM_PM = 2;
                }
                else {
                  AM_PM = 1;
                }
              }
            }
            if ((y >= houM[1]) && (y <= houM[3]))           //SECOND ROW
            {
              if ((x >= houM[0]) && (x <= houM[2]))        //press hour down
              {
                waitForIt(houM[0], houM[1], houM[2], houM[3]);
                rtcSetHr--;
                if (rtcSetHr < 0)
                {
                  rtcSetHr = 23;
                }
              }
              if ((x >= minM[0]) && (x <= minM[2]))        //press min down
              {
                waitForIt(minM[0], minM[1], minM[2], minM[3]);
                rtcSetMin = rtcSetMin - 30;
                if (rtcSetMin < 0) {
                  rtcSetMin = 30;
                }
                if (rtcSetMin < 30) {
                  rtcSetMin = 0;
                }
              }
              if ((x >= ampmM[0]) && (x <= ampmM[2])       //press AMPM down
                  && (setTimeFormat == 1))
              {
                waitForIt(ampmM[0], ampmM[1], ampmM[2], ampmM[3]);
                if (AM_PM == 1) {
                  AM_PM = 2;
                }
                else {
                  AM_PM = 1;
                }
              }
            }
            setDoserTimesScreen(false);
          }
          break;

        case 38:
          //--------------- AUTOMATIC FISH FEEDER PAGE -----------КОРМУШКА
          if ((x >= 5) && (x <= 155) && (y >= 20) && (y <= 40)) //press Feeding Time 1
          {
            waitForIt(5, 20, 155, 40);
            feedTime = 1;
            dispScreen = 39;
            clearScreen();
            setFeederTimesScreen();
          } else if ((x >= 165) && (x <= 315) && (y >= 20) && (y <= 40)) //press Feeding Time 2
          {
            waitForIt(165, 20, 315, 40);
            feedTime = 2;
            dispScreen = 39;
            clearScreen();
            setFeederTimesScreen();
          } else if ((x >= 5) && (x <= 155) && (y >= 168) && (y <= 188)) //press Feeding Time 3
          {
            waitForIt(5, 168, 155, 188);
            feedTime = 3;
            dispScreen = 39;
            clearScreen();
            setFeederTimesScreen();
          } else if ((x >= 165) && (x <= 315) && (y >= 168) && (y <= 188)) //press Feeding Time 4
          {
            waitForIt(165, 168, 315, 188);
            feedTime = 4;
            dispScreen = 39;
            clearScreen();
            setFeederTimesScreen();
          } else if ((x >= 85) && (x <= 235) && (y >= 94) && (y <= 114)) //press Feeding Fish Now!
          {
            waitForIt(85, 94, 235, 114);
            myGLCD.setColor(0, 255, 0);
            myGLCD.fillRoundRect(85, 94, 235, 114);
            myGLCD.setColor(255, 255, 255);
            myGLCD.drawRoundRect(85, 94, 235, 114);
            setFont(SMALL, 0, 0, 0, 0, 255, 0);
            myGLCD.setFont(RusFont3);
            strcpy_P(buffer, (char*)pgm_read_word_near(&(Text_table[181])));
            myGLCD.print(buffer, 126, 101);
            ManFeed = 1;
          }
          break;

        case 39:
          //------------ SET AUTOMATIC FISH FEEDER TIMES ------------
          if ((x >= back[0]) && (x <= back[2]) && (y > back[1]) && (y <= back[3]))  //press back
          {
            waitForIt(back[0], back[1], back[2], back[3]);
            if ((timeDispH >= 0) && (timeDispH <= 11)) {
              AM_PM = 1;
            }
            else {
              AM_PM = 2;
            }
            dispScreen = 38;
            clearScreen();
            autoFeederScreen();
          } else

            if ((x >= prSAVE[0]) && (x <= prSAVE[2]) && (y >= prSAVE[1]) && (y <= prSAVE[3])) //press SAVE
            {
              waitForIt(prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);
              if (setTimeFormat == 1)
              {
                if ((rtcSetHr == 0) && (AM_PM == 2))
                {
                  rtcSetHr += 12;
                }
                if (((rtcSetHr >= 1) && (rtcSetHr <= 11)) && (AM_PM == 2))
                {
                  rtcSetHr += 12;
                }
                if (((rtcSetHr >= 12) && (rtcSetHr <= 23)) && (AM_PM == 1))
                {
                  rtcSetHr -= 12;
                }
              }
              if (feedTime == 1)
              {
                feedFish1H = rtcSetHr;
                feedFish1M = rtcSetMin;
              }
              if (feedTime == 2)
              {
                feedFish2H = rtcSetHr;
                feedFish2M = rtcSetMin;
              }
              if (feedTime == 3)
              {
                feedFish3H = rtcSetHr;
                feedFish3M = rtcSetMin;
              }
              if (feedTime == 4)
              {
                feedFish4H = rtcSetHr;
                feedFish4M = rtcSetMin;
              }
              SaveFeedTimesToEEPROM();
              dispScreen = 38;
              clearScreen();
              autoFeederScreen();
            } else if ((x >= 70) && (x <= 250) && (y >= 150) && (y <= 170)) //Feeding ON/OFF
            {
              waitForIt(70, 150, 250, 170);
              if (feedTime == 1)
              { if (FEEDTime1 == 1) {
                  FEEDTime1 = 0;
                }
                else {
                  FEEDTime1 = 1;
                }
              }
              if (feedTime == 2)
              { if (FEEDTime2 == 1) {
                  FEEDTime2 = 0;
                }
                else {
                  FEEDTime2 = 1;
                }
              }
              if (feedTime == 3)
              { if (FEEDTime3 == 1) {
                  FEEDTime3 = 0;
                }
                else {
                  FEEDTime3 = 1;
                }
              }
              if (feedTime == 4)
              { if (FEEDTime4 == 1) {
                  FEEDTime4 = 0;
                }
                else {
                  FEEDTime4 = 1;
                }
              }
              feedingTimeOnOff();
            }
            else
            {
              if ((y >= houP[1]) && (y <= houP[3]))           //FIRST ROW
              {
                if ((x >= houP[0]) && (x <= houP[2]))        //press hour up
                {
                  waitForIt(houP[0], houP[1], houP[2], houP[3]);
                  rtcSetHr++;
                  if (rtcSetHr >= 24)
                  {
                    rtcSetHr = 0;
                  }
                }
                if ((x >= minP[0]) && (x <= minP[2]))        //press min up
                {
                  waitForIt(minP[0], minP[1], minP[2], minP[3]);
                  rtcSetMin++;
                  if (rtcSetMin >= 60) {
                    rtcSetMin = 0;
                  }
                }
                if ((x >= ampmP[0]) && (x <= ampmP[2])       //press AMPM up
                    && (setTimeFormat == 1))
                {
                  waitForIt(ampmP[0], ampmP[1], ampmP[2], ampmP[3]);
                  if (AM_PM == 1) {
                    AM_PM = 2;
                  }
                  else {
                    AM_PM = 1;
                  }
                }
              }
              if ((y >= houM[1]) && (y <= houM[3]))           //SECOND ROW
              {
                if ((x >= houM[0]) && (x <= houM[2]))        //press hour down
                {
                  waitForIt(houM[0], houM[1], houM[2], houM[3]);
                  rtcSetHr--;
                  if (rtcSetHr < 0)
                  {
                    rtcSetHr = 23;
                  }
                }
                if ((x >= minM[0]) && (x <= minM[2]))        //press min down
                {
                  waitForIt(minM[0], minM[1], minM[2], minM[3]);
                  rtcSetMin--;
                  if (rtcSetMin < 0) {
                    rtcSetMin = 59;
                  }
                }
                if ((x >= ampmM[0]) && (x <= ampmM[2])       //press AMPM down
                    && (setTimeFormat == 1))
                {
                  waitForIt(ampmM[0], ampmM[1], ampmM[2], ampmM[3]);
                  if (AM_PM == 1) {
                    AM_PM = 2;
                  }
                  else {
                    AM_PM = 1;
                  }
                }
              }
              setFeederTimesScreen(false);
            }
          break;

        case 40: /****************************** Присвоение цвета каналам ********************************/
          if ((y >= 29 + 2) && (y <= 44 - 2) && ( RGBcolorSet == true)) { // UP Buttons were touched, desrease button touchable area (2pix)

            for (byte i = 5; i < 8; i++) {
              if ((x >= (i * 35) + 21) && (x <= (i * 35) + 51)) {

                sbX1 = (i * 35) + 21; sbX2 = (i * 35) + 51;
                TopSldY = 53, BotSldY = TopSldY + 100;
                SliderSwitch  = true;

                if (i == 5 && RGBcolorSet == true) {
                  sbR = 255; sbG = 0; sbB = 0;   tSlide = R_color; tSlide += 1; // CW colour (белый)
                  R_color = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                  temp_R_color = map(R_color, 0, 100, 0, 255);
                }

                if (i == 6 && RGBcolorSet == true) {
                  sbR = 0; sbG = 255; sbB = 0;    tSlide = G_color; tSlide += 1; // BL colour (синий)
                  G_color = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                  temp_G_color = map(G_color, 0, 100, 0, 255);
                }

                if (i == 7 && RGBcolorSet == true) {
                  sbR = 0; sbG = 0; sbB = 255;    tSlide = B_color; tSlide += 1; // RBL colour (рояль)
                  B_color = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                  temp_B_color = map(B_color, 0, 100, 0, 255);
                }
              }
            }

            myGLCD.setColor(temp_R_color, temp_G_color, temp_B_color);
            myGLCD.fillRoundRect (15, 45, 180, 75);

          }

          if ((y >= 174 + 2) && (y <= 187 - 2) && ( RGBcolorSet == true)) { // DOWN Buttons were touched,desrease button touchable area (2pix)

            for (byte i = 5; i < 8; i++) {
              if ((x >= (i * 35) + 21) && (x <= (i * 35) + 51)) {

                sbX1 = (i * 35) + 21; sbX2 = (i * 35) + 51;
                TopSldY = 53, BotSldY = TopSldY + 100;
                SliderSwitch  = true;

                if (i == 5 && RGBcolorSet == true) {
                  sbR = 255; sbG = 0; sbB = 0;  tSlide = R_color; tSlide -= 1; // CW colour (белый)
                  R_color = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                  temp_R_color = map(R_color, 0, 100, 0, 255);
                }

                if (i == 6 && RGBcolorSet == true) {
                  sbR = 0; sbG = 255; sbB = 0; tSlide = G_color; tSlide -= 1; // BL colour (синий)
                  G_color = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                  temp_G_color = map(G_color, 0, 100, 0, 255);
                }

                if (i == 7 && RGBcolorSet == true) {
                  sbR = 0; sbG = 0; sbB = 255;  tSlide = B_color; tSlide -= 1; // RBL colour (рояль)
                  B_color = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                  temp_B_color = map(B_color, 0, 100, 0, 255);
                }
              }
            }

            myGLCD.setColor(temp_R_color, temp_G_color, temp_B_color);
            myGLCD.fillRoundRect (15, 45, 180, 75);
          }

          if ((y >= TopSldY) && (y <= BotSldY) && ( RGBcolorSet == true)) { // change value with Slider Bars touch

            for (byte i = 5; i < 8; i++) {
              if ((x >= ((i * 35) + 21 + 5)) && (x <= ((i * 35) + 51 - 5))) { // desrease slider touchable area (5pix)

                sbX1 = ((i * 35) + 21); sbX2 = (((i * 35) + 51)); // slider width 30 pix, clearens between sliders 5pix
                TopSldY = 53, BotSldY = (TopSldY + (100));
                SliderSwitch  = false;

                if (i == 5 && RGBcolorSet == true) {
                  sbR = 255; sbG = 0; sbB = 0; // Красный
                  R_color = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                  temp_R_color = map(R_color, 0, 100, 0, 255);
                }

                if (i == 6 && RGBcolorSet == true) {
                  sbR = 0; sbG = 255; sbB = 0; 	 // Синий
                  G_color = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                  temp_G_color = map(G_color, 0, 100, 0, 255);
                }

                if (i == 7 && RGBcolorSet == true) {
                  sbR = 0; sbG = 0; sbB = 255; 	 // Зеленый
                  B_color = SliderBarsForChange( TopSldY, BotSldY, y, sbR, sbG, sbB, sbX1, sbX2);
                  temp_B_color = map(B_color, 0, 100, 0, 255);
                }
              }
            }

            myGLCD.setColor(temp_R_color, temp_G_color, temp_B_color);
            myGLCD.fillRoundRect (15, 45, 180, 75);

          }

          PressButton((0), NO , RGBch1[0], RGBch1[1], RGBch1[2], RGBch1[3]);    // 1 канал
          if (ButtonPress == true) {
            rgbCh0[0] = temp_R_color; rgbCh0[1] = temp_G_color; rgbCh0[2] = temp_B_color;
            myGLCD.setColor(rgbCh0[0], rgbCh0[1], rgbCh0[2]);
            myGLCD.fillRoundRect (RGBch1[0], RGBch1[1], RGBch1[2], RGBch1[3]);
            ButtonPress = false;
          }

          PressButton((0), NO , RGBch2[0], RGBch2[1], RGBch2[2], RGBch2[3]);    // 2 канал
          if (ButtonPress == true) {
            rgbCh1[0] = temp_R_color; rgbCh1[1] = temp_G_color; rgbCh1[2] = temp_B_color;
            myGLCD.setColor(rgbCh1[0], rgbCh1[1], rgbCh1[2]);
            myGLCD.fillRoundRect (RGBch2[0], RGBch2[1], RGBch2[2], RGBch2[3]);
            ButtonPress = false;
          }

          PressButton((0), NO , RGBch3[0], RGBch3[1], RGBch3[2], RGBch3[3]);    // 3 канал
          if (ButtonPress == true) {
            rgbCh2[0] = temp_R_color; rgbCh2[1] = temp_G_color; rgbCh2[2] = temp_B_color;
            myGLCD.setColor(rgbCh2[0], rgbCh2[1], rgbCh2[2]);
            myGLCD.fillRoundRect (RGBch3[0], RGBch3[1], RGBch3[2], RGBch3[3]);
            ButtonPress = false;
          }

          PressButton((0), NO , RGBch4[0], RGBch4[1], RGBch4[2], RGBch4[3]);    // 4 канал
          if (ButtonPress == true) {
            rgbCh3[0] = temp_R_color; rgbCh3[1] = temp_G_color; rgbCh3[2] = temp_B_color;
            myGLCD.setColor(rgbCh3[0], rgbCh3[1], rgbCh3[2]);
            myGLCD.fillRoundRect (RGBch4[0], RGBch4[1], RGBch4[2], RGBch4[3]);
            ButtonPress = false;
          }

          PressButton((0), NO , RGBch5[0], RGBch5[1], RGBch5[2], RGBch5[3]);     // 5 канал
          if (ButtonPress == true) {
            rgbCh4[0] = temp_R_color; rgbCh4[1] = temp_G_color; rgbCh4[2] = temp_B_color;
            myGLCD.setColor(rgbCh4[0], rgbCh4[1], rgbCh4[2]);
            myGLCD.fillRoundRect (RGBch5[0], RGBch5[1], RGBch5[2], RGBch5[3]);
            ButtonPress = false;
          }

          PressButton((0), NO , RGBch6[0], RGBch6[1], RGBch6[2], RGBch6[3]);    // 6 канал
          if (ButtonPress == true) {
            rgbCh5[0] = temp_R_color; rgbCh5[1] = temp_G_color; rgbCh5[2] = temp_B_color;
            myGLCD.setColor(rgbCh5[0], rgbCh5[1], rgbCh5[2]);
            myGLCD.fillRoundRect (RGBch6[0], RGBch6[1], RGBch6[2], RGBch6[3]);
            ButtonPress = false;
          }

          PressButton((0), NO , RGBch7[0], RGBch7[1], RGBch7[2], RGBch7[3]);    // 7 канал
          if (ButtonPress == true) {
            rgbCh6[0] = temp_R_color; rgbCh6[1] = temp_G_color; rgbCh6[2] = temp_B_color;
            myGLCD.setColor(rgbCh6[0], rgbCh6[1], rgbCh6[2]);
            myGLCD.fillRoundRect (RGBch7[0], RGBch7[1], RGBch7[2], RGBch7[3]);
            ButtonPress = false;
          }

          PressButton((0), NO , MOONch[0], MOONch[1], MOONch[2], MOONch[3]);    // ЛУНА
          if (ButtonPress == true) {
            clearScreen(); ledValuesScreen(); dispScreen = 26;

            ButtonPress = false;
          }

          PressButton((0), NO , back[0], back[1], back[2], back[3]);    // Press back
          if (ButtonPress == true) {
            dispScreen = 9;  clearScreen(); generalSettingsScreen_3();
            ButtonPress = false;
          }

          PressButton((0), NO , prSAVE[0], prSAVE[1], prSAVE[2], prSAVE[3]);    // press SAVE
          if (ButtonPress == true) {
            dispScreen = 0; clearScreen(); mainScreen(true); SaveRGBbarsToEEPROM();
            ButtonPress = false;
          }

          PressButton((0), NO , canC[0], canC[1], canC[2], canC[3]);   // Press CANCEL
          if (ButtonPress == true) {
            dispScreen = 0; clearScreen(); ReadFromEEPROM(); mainScreen(true);
            ButtonPress = false;
          }
          break;
      }
    }
}

void setup(void) { // ============ SETUP
  Serial.begin(9600); // Setup usb serial connection to computer
  Wire.begin(); // I2C bus init
  Wire.beginTransmission(0); //Generall call
  Wire.write(0x06); // reset device on I2C bus
  Wire.endTransmission(); // release bus
  adc_addr = i2c_adr_find(0x48, 0x58); //ADS 1100 - find devices in ADS1115 address space
  adc_init(adc_addr);

#ifdef Timers_11bit
  //**************** 11 bit timer setting **********************************
  TCCR5B = (TCCR5B & 0xF8) | PWM_FRQ_Value_Fan; // 30hz  pin 44, 45, 46 for FANs

  // set timer mode 14 - fast PWM
  TCCR4A = B00000010;		// mode 14Fast PWM timer4
  TCCR3A = B00000010;		// mode 14Fast PWM timer3
  TCCR1A = B00000010;		// mode 14Fast PWM timer1
  TCCR4B = B00011000;
  TCCR3B = B00011000;
  TCCR1B = B00011000;

  // set prescaler value
  TCCR1B = TCCR1B | PWM_FRQ_Value;  // pin 11, 12
  TCCR3B = TCCR3B | PWM_FRQ_Value;  // pin 2, 3,
  TCCR4B = TCCR4B | PWM_FRQ_Value;  // pin 6, 7, 8

  OCR1A = 0;  // 0 vary this value between 0 and 1024 for 10-bit precision
  OCR1B = 0;
  OCR3A = 0;
  OCR3B = 0;
  OCR3C = 0;
  OCR4A = 0;
  OCR4B = 0;
  OCR4C = 0;

  ICR1 = 2000; // количество шагов
  ICR3 = 2000;
  ICR4 = 2000;

  cbi_mix( PORTB, 5 ); // Timer1, port 11
  sbi_mix( DDRB , 5 );
  cbi_mix( PORTB, 6 ); // Timer1, port 12
  sbi_mix( DDRB , 6 );
  cbi_mix( PORTE, 3 ); // Timer3, port 5
  sbi_mix( DDRE , 3 );
  cbi_mix( PORTE, 4 ); // Timer3, port 2
  sbi_mix( DDRE , 4 );
  cbi_mix( PORTE, 5 ); // Timer3, port 3
  sbi_mix( DDRE , 5 );
  cbi_mix( PORTH, 3 ); // Timer4, port 6
  sbi_mix( DDRH , 3 );
  cbi_mix( PORTH, 4 ); // Timer4, port 7
  sbi_mix( DDRH , 4 );
  cbi_mix( PORTH, 5 ); // Timer4, port 8
  sbi_mix( DDRH , 5 );
#endif

#ifdef Timers_12bit
  //**************** 11 bit timer setting **********************************
  TCCR2B = (TCCR2B & 0xF8) | PWM_FRQ_Value_Fan + 2 ; // 30Hz  pin 10, 9  8bit
  TCCR5B = (TCCR5B & 0xF8) | PWM_FRQ_Value_Fan;    // 30hz  pin 44, 45, 46 for FANs 8bit

  /* B00011001 - 8khz  TCCRXB register, no prescaling
     B00011010 - 1khz  TCCRXB register, clkI/O/8
     B00011011 - 125hz TCCRXB register, clkI/O/64
     B00011100 - 31hz  TCCRXB register, clkI/O/256

    TCCR4A = B00000010;    // mode 14 Fast PWM timer4   WGM41=1, WGM40=0
    TCCR4B = B00011010;   //  clkI/O/8                 WGM43=1, WGM42=1 + CS42..CS40 =10
    TCCR3A = B00000010;   // mode 14 Fast PWM timer3
    TCCR3B = B00011010;   //  clkI/O/8
    TCCR1A = B00000010;   // mode 14 Fast PWM timer 1
    TCCR1B = B00011010;   //  clkI/O/8
  */

  // set timer mode 14 - fast PWM
  TCCR4A = B00000010;   // mode 14Fast PWM timer4
  TCCR3A = B00000010;   // mode 14Fast PWM timer3
  TCCR1A = B00000010;   // mode 14Fast PWM timer1
  TCCR4B = B00011000;
  TCCR3B = B00011000;
  TCCR1B = B00011000;

  // set prescaler value
  TCCR1B = TCCR1B | 2;  // pin 11, 12
  TCCR3B = TCCR3B | 2;  // pin 2, 3,
  TCCR4B = TCCR4B | 2;  // pin 6, 7, 8

  OCR1A = 0;
  OCR1B = 0;
  OCR3A = 0;
  OCR3B = 0;
  OCR3C = 0;
  OCR4A = 0;
  OCR4B = 0;
  OCR4C = 0;

  ICR1 = 4000;
  ICR3 = 4000;
  ICR4 = 4000;

  cbi_mix( PORTB, 5 ); //Timer1, port 11
  sbi_mix( DDRB , 5 );
  cbi_mix( PORTB, 6 ); //Timer1, port 12
  sbi_mix( DDRB , 6 ); //
  cbi_mix( PORTE, 3 ); //Timer3, port 5
  sbi_mix( DDRE , 3 ); //
  cbi_mix( PORTE, 4 ); //Timer3, port 2
  sbi_mix( DDRE , 4 ); //
  cbi_mix( PORTE, 5 ); //Timer3, port 3
  sbi_mix( DDRE , 5 ); //
  cbi_mix( PORTH, 3 ); //Timer4, port 6
  sbi_mix( DDRH , 3 ); //
  cbi_mix( PORTH, 4 ); //Timer4, port 7
  sbi_mix( DDRH , 4 ); //
  cbi_mix( PORTH, 5 ); //Timer4, port 8
  sbi_mix( DDRH , 5 ); //
#endif

  pinMode(ledPinWarmWhite, OUTPUT);   // warm white
  pinMode(ledPinCoolWhite, OUTPUT);    // cool white
  pinMode(ledPinRoyBlue, OUTPUT); // royal
  pinMode(ledPinRed, OUTPUT);     // red
  pinMode(ledPinUV, OUTPUT);      // uv
  pinMode(ledPinOrange, OUTPUT);  // oLed
  pinMode(ledPinGr, OUTPUT);      // green

  pinMode(ledPinMoon, OUTPUT);    // Пин луны

  pinMode(LCDbrightPin, OUTPUT);  // подсветка экрана

  pinMode(tempHeatPin, OUTPUT);     // нагреватель
  pinMode(tempChillPin, OUTPUT);    // холодильник
  pinMode(tempAlarmPin, OUTPUT);    // тревога
  pinMode(autoFeeder, OUTPUT);
  pinMode(Heatsink1_FansPWM, OUTPUT); // вентилятор 1 (шим)
  pinMode(Heatsink2_FansPWM, OUTPUT); // вентилятор 2 (шим)

  // Timers
  pinMode(timer1, OUTPUT);
  pinMode(timer2, OUTPUT);
  pinMode(timer3, OUTPUT);
  pinMode(timer4, OUTPUT);
  pinMode(timer5, OUTPUT);

  // Distributeurs
  pinMode(pump1, OUTPUT);
  pinMode(pump2, OUTPUT);
  pinMode(pump3, OUTPUT);
  pinMode(pump4, OUTPUT);

  pinMode(vacpump, OUTPUT);
  pinMode(SensLevel, INPUT);

  ReadDallasAddress ();
  sensors.begin();         // start up temperature library
  sensors.setResolution(waterThermometer, resolution);     // set the resolution to 11 bit
  sensors.setResolution(Heatsink1Thermometer, resolution);
  sensors.setResolution(Heatsink2Thermometer, resolution);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  delayInMillis = 750 / (1 << (12 - resolution));
  lastTempRequest = millis();

  SetRoomTemperatureResolutionBits (12);// 12 bits room temp resolution in celcius

  myGLCD.InitLCD(LANDSCAPE);
  myGLCD.clrScr();

  EraseAllEEPROM_SaveDefault();  // check and erase eeprom for my format, save all default
  myGLCD.clrScr();

  myTouch.InitTouch(LANDSCAPE);
  myTouch.setPrecision(PREC_EXTREME); // LOW / MEDIUM / HIGH
  myTouch.read();                    // dummy read, enale IRQ

  myGLCD.setColor(30, 30, 30);
  myGLCD.fillRect(0, 226, 319, 239);  // нижний Bar

  RTC.startClock();                    // Set the clock to run-mode
  RTC.getTime();
  min_cnt = (RTC.hour * 60) + RTC.minute;
  timer = (RTC.hour * 60) + RTC.minute;

  sec_cnt = (RTC.minute * 60) + RTC.second;

  COLOR = 0;             // read all colors
  ReadLedFromEEPROM();   // read led setting from EEPROM
  ReadRGBColorFromEEPROM();   // read led setting from EEPROM
  ReadFromEEPROM();      // считать из памяти все настройки
  LED_levelo_output();
  checkTempC();
  ReadLCDbright();
  mainScreen(true);
  titledate();         // отображение даты в нижнем баре

  calculateStartTime();  // calculate SUNRISE time
  calculateStopTime();   // calculate SUNSET time

  periode = 50000;        // time in ms from 0-255 or 0-255
  starttime = millis();
} // change nothing below here

// ------------------ main loop ---------------------------------------
void loop(void) {

  Serial.print("Temperature: ");
  Serial.println(tempH1);//prints the Temperature value

  waterlevel = digitalRead(SensLevel);

  if (aclock == 1) {
    analogClock();  // cкринсейв аналоговые часы (секундная стрелка)
  }

  // Выход из хранителя экрана
  if ((myTouch.dataAvailable()) && (dispScreen == 55) || ((tempAlarmflag == true) && (dispScreen == 55))) {
    //  if ((myTouch.dataAvailable()) && (screenSaverCounter>=setScreenSaverTimer)){

    LEDtestTick = false;
    screenSaverCounter = 0; myGLCD.clrScr(); // выход из хранителя экрана
    myGLCD.setColor(30, 30, 30);
    myGLCD.fillRect(0, 226, 319, 239);    // заполнение Bar
    myGLCD.setColor(64, 64, 64);
    myGLCD.drawRect(0, 226, 319, 239);    // рамка

    mainScreen(true); dispScreen = 0;
  }    // счетчик возврата в главный экран
  else {
    if (myTouch.dataAvailable() && dispScreen != 55) {
      processMyTouch();
    }
  }

  pumpPWM(); // цикл для помп в режиме шим
  P1(minP1, maxP1);
  P2(minP2, maxP2);

  if (dispScreen == 12) {
    GraphPWMPump(); // графика помп шим
  }

  //----------check overheat Alarm Function ---------------------------
  Alarm();

  //----------check LED levels every 1s for 8/11 & 11 bit version ---------------------------
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisOne > 1000) {  // check LED levels каждую секунду
    previousMillisOne = currentMillis;
    min_cnt = (RTC.hour * 60) + RTC.minute;
    LED_levelo_output();
    dosingTimeOutput();

    RTC.getTime();
    if (dispScreen == 0 && screenSaverCounter < setScreenSaverTimer) {
      if (RTC.hour == 0 && RTC.minute == 0 && RTC.second == 0) { // в полночь очистить область даты
        setFont(SMALL, 30, 30, 30, 30, 30, 30);
        myGLCD.fillRect(0, 227, 215, 238);
        myGLCD.print(F("                          "), 2, 227);
      }
    }
  }

  if (currentMillis - previousMillisFive > 5000) { // проверка времени, температуры и уровней каналов, каждые 5 сек
    previousMillisFive = currentMillis;
    RTC.getTime();

    //------------------------
    if (screenSaverCounter < setScreenSaverTimer) {
      TimeDateBar();
      titledate();
    }

    checkTempC();							 // проверка температуры
    timer = (RTC.hour * 60) + RTC.minute;  // проверка таймеров
    light();								 // срабатывание таймеров (проверка каждые 5сек)
    lightdraw();							 // чтение состояния таймеров и отображение состояний
    Speed_Fan();							 //vitesse du ventilateur sur le radiateur (dans l'écran principal)
    feedingTimeOutput();
    waterlevels();
    caldosetime();

#ifdef PH_sensor_I2C
    CheckPH();
#endif

#ifdef ADC_1115
    CheckADC1115PH();
#endif

#ifdef Analog_PH_sensor
    CheckAPH();
#endif

    //------------ Désactiver les préréglages 45 minutes avant la fin de la journée -------------------
    if (((GlobalStatus2Byte & 0x0F) != 0) && (min_cnt / 15 == StopTime - 3 )) { // !=0 if preset is ON
      GlobalStatus2Byte = (GlobalStatus2Byte & 0xF0);                   // clear flags Preset1..Preset4
      colorLEDtest = false;
    } screenReturn(); screenSaver(); checkTempC();
  }

  if ((dispScreen == 0) && (screenSaverCounter < setScreenSaverTimer)) {
    aclock = 0;  // dans la boucle principale
    mainScreen();
  }
}

/********************************** FIN DU MAIN LOOP *********************************/
#ifdef Timers_12bit
unsigned int calcExponentialToLinear( unsigned int aLevel )                      //процент * 5 =500 переводим в шим 12 bit
{ // check overdrive
  if ( aLevel > MAX_LEVEL )     aLevel = MAX_LEVEL;
  // prepare
  if ( aLevel >= 42 )     return (aLevel - 41) * 8 + 312;
  else if ( aLevel >= 31 )     aLevel += 520;
  else if ( aLevel >= 26 )     aLevel += 208;
  else if ( aLevel >= 20 )     aLevel += 118;
  else if ( aLevel >= 11 )     aLevel += 88;
  // calc
  return  ( aLevel % 32 ) * ( ( aLevel >> 5 ) + 1 );
}
#else
unsigned int calcExponentialToLinear( unsigned int aLevel )                        //процент * 5 =500 переводим в шим
{ // overdrive
  if ( aLevel > MAX_LEVEL )     aLevel = MAX_LEVEL;
  // preparation
  if ( aLevel >= 42 )     return (aLevel - 41) * 4 + 156;
  else if ( aLevel >= 38 )     aLevel += 118;
  else if ( aLevel >= 34 )     aLevel += 74;
  else if ( aLevel >= 31 )     aLevel += 46;
  else if ( aLevel >= 24 )     aLevel += 33;
  else if ( aLevel >= 16 )     aLevel += 8;
  // calcul
  return  ( aLevel % 16 ) * ( ( aLevel >> 4 ) + 1 );
}
#endif

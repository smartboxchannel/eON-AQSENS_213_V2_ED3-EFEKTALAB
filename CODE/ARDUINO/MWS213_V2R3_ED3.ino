
// ###################   Mini air quality and wither station with electronic ink display 2.13 Inch | nRF52   ############### //
//                                                                                                                           //
//        @filename   :   MWS213_V2R3_ED3.ino                                                                                //
//        @brief en   :   Wireless, battery-operated air quality(VOC Sensor SGP40), temperature,humidity and pressure        //
//                        sensor(BME280) with electronic ink display(Good Display GDEH0213B72, GDEH0213B73, Waveshare V2).   //
//                        Works on SOC nRF52.                                                                                //
//        @brief ru   :   Беcпроводной, батарейный датчик качества воздуха (ЛОС сенсорс SGP40), температуры, влажности       //
//                        и давления(BME280) с дисплеем на электронных чернилах(Good Display GDEH0213B72, GDEH0213B73,       //
//                        Waveshare V2).                                                                                     //
//                        Работает на nRF52832, nRF52840.                                                                    //
//                                                                                                                           //
//        @author     :   Andrew Lamchenko aka Berk                                                                          //
//                                                                                                                           //
//        Copyright (C) 2021, EFEKTALAB                                                                                      //
//        Copyright (c) 2020, Sensirion AG                                                                                   //
//        Copyright (c) 2020, Bosch Sensortec GmbH. All rights reserved.                                                     //
//        Copyright (c) 2014-2015, Arduino LLC.  All right reserved.                                                         //
//        Copyright (c) 2016, Arduino Srl.  All right reserved.                                                              //
//        Copyright (c) 2017, Sensnology AB. All right reserved.                                                             //
//        Copyright (C) 2020, Waveshare                                                                                      //
//                                                                                                                           //
// ######################################################################################################################### //



#include "aConfig.h" //Edit this file for your sensor configuration


// ##############################################################################################################
// #                                                 DRIVERS & LIB                                              #
// ##############################################################################################################
#include "eink213_V2.h"
#include "imagedata.h"
#include "einkpaint.h"
#include "src/Adafruit_SGP40.h" // !!!Non-standard constants are used for the air quality algorithm!!!
#include <Adafruit_Sensor.h>
#ifdef BME280
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
#else
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bme;
#include "Adafruit_SHTC3.h"
Adafruit_SHTC3 sensor = Adafruit_SHTC3();
#endif
// ##############################################################################################################



// ##############################################################################################################
// #                                                   VARIABLES                                                #
// ##############################################################################################################
unsigned char image[4000];
Paint paint(image, 0, 0);
Epd epd;
Adafruit_SGP40 sgp;

volatile int32_t vocIndex;
volatile int32_t oldVocIndex = -500;
const int32_t vocIndexThreshold = 10;
uint16_t sraw;

float last_tmp;
bool upTemp;
bool downTemp;

float last_hm;
bool upHum;
bool downHum;

float last_pr;
bool upPress;
bool downPress;

long last_aq;
bool upAq;
bool downAq;

bool learnOK;
bool mesColorSet;
#ifdef SEND_RESET_REASON
bool mesReset;
#endif
bool mesBatSet;
bool mesAq;
bool mesVolt;
bool mesLink;
bool mesForec;
bool mesBaro;
bool mesHum;
bool mesTemp;
bool mesInfo;
bool needPresent;
bool metric;
bool colorPrint;
bool opposite_colorPrint;
bool sendAfterResTask;
bool changeB = true;
bool changeC = true;
bool updateink1;
bool updateink2;
bool updateink3;
bool updateink4;
bool updateinkclear;
bool check;
bool chek_h = true;
bool flag_update_transport_param;
bool flag_sendRoute_parent;
bool flag_no_present;
bool flag_nogateway_mode;
bool flag_find_parent_process;
bool nosleep;
volatile bool change;
bool Ack_FP;
bool tch;
bool hch;
bool bch;
bool vch;
bool pch;
bool fch;
bool ach;
bool lch;
bool configMode;
bool button_flag;
uint8_t lang;
uint8_t icon_bl;
uint8_t battery;
uint8_t old_battery;
uint8_t err_delivery_beat;
const uint16_t shortWait = 5;
int16_t temperatureInt;
int16_t humidityInt;
int16_t pressureInt;
int16_t pressure_mmInt;
uint16_t batteryVoltage;
uint16_t old_batteryVoltage;
volatile int16_t nRFRSSI;
volatile int16_t old_nRFRSSI = 127;
int16_t myid;
int16_t mypar;
int16_t old_mypar = -1;
volatile float temperatureSend;
volatile float temperatureSend2;
volatile float pressureSend;
volatile float humiditySend;
volatile float old_temperature;
volatile float old_humidity;
volatile float old_pressure;
const float tempThreshold = 0.5; // порог сравнения в предыдущими показаниями температуры
const float humThreshold = 2.5; // порог сравнения в предыдущими показаниями влажности
const float pressThreshold = 1.0; // порог сравнения в предыдущими показаниями влажности
float batteryVoltageF;

uint32_t configMillis;
uint32_t previousMillis;
uint32_t sleepTime;
uint32_t stopTimer;
uint32_t startTimer;
uint32_t precisionTimeWDT;
const uint32_t sleepTimeWDT = 3000;
const uint32_t minuteT = 60000;
uint32_t BATT_TIME;
volatile uint32_t BATT_COUNT;
uint32_t cpNom;
volatile uint32_t cpCount;
uint32_t battSend;
const uint32_t timeSend = 1;
volatile uint32_t sleepTimeCount;
uint32_t baseLineSaveClock;
volatile uint32_t baseLineSaveCount;
uint32_t periodTimer;
unsigned char oneStateBytes[4];
int32_t oneState;
unsigned char twoStateBytes[4];
int32_t twoState;
const uint8_t fastView = 3;
volatile uint8_t checkFast;
//uint16_t learningTime;
volatile uint16_t learningCount;
// ##############################################################################################################



// ##############################################################################################################
// #                                                   MYSENSORS                                                #
// ##############################################################################################################
int16_t mtwr;
#define MY_TRANSPORT_WAIT_READY_MS (mtwr)
#include <MySensors.h>
#define TEMP_CHILD_ID 0
#define HUM_CHILD_ID 1
#define BARO_CHILD_ID 2
#define FORECAST_CHILD_ID 3
#define AQ_CHILD_ID 4
#define SIGNAL_Q_ID 100
#define BATTERY_VOLTAGE_ID 101
#define SET_BATT_SEND_ID 103
#define SET_COLOR_ID 104
#ifdef SEND_RESET_REASON
#define RESET_REASON_ID 105
#endif
#define DEBMES_1 121
#define DEBMES_2 122
MyMessage debMsg1(DEBMES_1, V_VAR1);
MyMessage debMsg2(DEBMES_2, V_VAR1);
MyMessage sqMsg(SIGNAL_Q_ID, V_VAR1);
MyMessage bvMsg(BATTERY_VOLTAGE_ID, V_VAR1);
// ##############################################################################################################


// ##############################################################################################################
// #                                                 FORECAST                                                   #
// ##############################################################################################################
#define CONVERSION_FACTOR (1.0/10.0)
const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
  STABLE = 0,                     // "Stable Weather Pattern"
  SUNNY = 1,                      // "Slowly rising Good Weather", "Clear/Sunny "
  CLOUDY = 2,                     // "Slowly falling L-Pressure ", "Cloudy/Rain "
  UNSTABLE = 3,                   // "Quickly rising H-Press",     "Not Stable"
  THUNDERSTORM = 4,               // "Quickly falling L-Press",    "Thunderstorm"
  UNKNOWN = 5                     // "Unknown (More Time needed)
};
int16_t forecast;
int16_t  old_forecast = -1;
const int LAST_SAMPLES_COUNT = 5;
float lastPressureSamples[LAST_SAMPLES_COUNT];
int minuteCount = 0;
bool firstRound = true;
float pressureAvg;
float pressureAvg2;
float dP_dt;
// ##############################################################################################################


// ##############################################################################################################
// #                                                 INTERRUPTS                                                 #
// ##############################################################################################################
uint32_t PIN_BUTTON_MASK;
volatile byte buttIntStatus = 0;
#define APP_GPIOTE_MAX_USERS 1
extern "C" {
#include "app_gpiote.h"
#include "nrf_gpio.h"
}
static app_gpiote_user_id_t m_gpiote_user_id;
// ##############################################################################################################


void preHwInit() {
  pinMode(PIN_BUTTON, INPUT);
}

void before() {

  // ##############################################################################################################
  // #                                                   CONFIG MCU                                               #
  // ##############################################################################################################
  NRF_POWER->DCDCEN = 1;
  NRF_SAADC ->ENABLE = 0;
  NRF_PWM0  ->ENABLE = 0;
  NRF_NFCT->TASKS_DISABLE = 1;
  NRF_NVMC->CONFIG = 1;
  NRF_UICR->NFCPINS = 0;
  NRF_NVMC->CONFIG = 0;
  NRF_PWM1  ->ENABLE = 0;
  NRF_PWM2  ->ENABLE = 0;
  NRF_TWIM1 ->ENABLE = 0;
  NRF_TWIS1 ->ENABLE = 0;
#ifndef MY_DEBUG
  NRF_UART0->ENABLE = 0;
#endif


  // ##############################################################################################################
  // #                                                  INIT HAPPY MODE                                           #
  // ##############################################################################################################
  happy_init();


  // ##############################################################################################################
  // #                                                    CONFIG PROG                                             #
  // ##############################################################################################################
  battSend = loadState(103);  // Saving in memory the interval of sending data about battery charge and signal quality,
  // maximized 24 hours, if 0, the sending does not do, only updates the info on the screen
  if (battSend > 24) {
    battSend = 6;
    saveState(103, battSend);
  }
  //battSend = 1; // for the test, 1 hour

  if (loadState(104) > 1) {
    saveState(104, 0);
  }
  colorChange(loadState(104));
  //colorChange(true); // for the test, true || false

  timeConf();
  delay(500);

  // ##############################################################################################################
  // #                                                     EINK INIT                                              #
  // ##############################################################################################################
  displayStart();
}


void presentation()
{
  if (needPresent == true) {
    if (flag_nogateway_mode == false) {
      if (mesInfo == false) {
        check = sendSketchInfo(SN, SV);
        if (!check) {
          wait(shortWait * 10);
          _transportSM.failedUplinkTransmissions = 0;
        }
      }
      if (check) {
        mesInfo = true;
      }

      if (mesTemp == false) {
        check = present(TEMP_CHILD_ID, S_TEMP, "TEMPERATURE");
        if (!check) {
          needPresent = true;
          wait(shortWait * 10);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesTemp = true;
          needPresent = false;
        }
      }

      if (mesHum == false) {
        check = present(HUM_CHILD_ID, S_HUM, "HUMIDITY");
        if (!check) {
          needPresent = true;
          wait(shortWait * 10);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesHum = true;
          needPresent = false;
        }
      }

      if (mesAq == false) {
        check = present(AQ_CHILD_ID, S_AIR_QUALITY, "AIR QUALITY (VOC)");
        if (!check) {
          needPresent = true;
          wait(shortWait * 10);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesAq = true;
          needPresent = false;
        }
      }

      if (mesBaro == false) {
        check = present(BARO_CHILD_ID, S_BARO, "PRESSURE");
        if (!check) {
          needPresent = true;
          wait(shortWait * 10);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesBaro = true;
          needPresent = false;
        }
      }

      if (mesForec == false) {
        check = present(FORECAST_CHILD_ID, S_CUSTOM, "FORECAST");
        if (!check) {
          needPresent = true;
          wait(shortWait * 10);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesForec = true;
          needPresent = false;
        }
      }

      if (mesLink == false) {
        check = present(SIGNAL_Q_ID, S_CUSTOM, "SIGNAL %");
        if (!check) {
          needPresent = true;
          wait(shortWait * 10);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesLink = true;
          needPresent = false;
        }
      }

      if (mesVolt == false) {
        check = present(BATTERY_VOLTAGE_ID, S_CUSTOM, "BATTERY VOLTAGE");
        if (!check) {
          needPresent = true;
          wait(shortWait * 10);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesVolt = true;
          needPresent = false;
        }
      }

      if (mesBatSet == false) {
        check = present(SET_BATT_SEND_ID, S_CUSTOM, "BATT SEND INTERTVAL | H");
        if (!check) {
          needPresent = true;
          wait(shortWait * 10);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesBatSet = true;
          needPresent = false;
        }
      }

#ifdef SEND_RESET_REASON
      if (mesReset == false) {
        check = present(RESET_REASON_ID, S_CUSTOM, "RESTART REASON");
        if (!check) {
          needPresent = true;
          wait(shortWait * 10);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesReset = true;
          needPresent = false;
        }
      }
#endif

      if (mesColorSet == false) {
        check = present(SET_COLOR_ID, S_CUSTOM, "COLOR W/B");
        if (!check) {
          needPresent = true;
          wait(shortWait * 10);
          _transportSM.failedUplinkTransmissions = 0;
        } else {
          mesColorSet = true;
          needPresent = false;
        }
      }

      if (mesInfo == true && mesTemp == true && mesHum == true && mesAq == true && mesBaro == true && mesForec == true && mesLink == true && mesVolt == true && mesColorSet == true) {
        needPresent = false;
#ifdef SEND_RESET_REASON
        if (mesReset == false) {
          needPresent = true;
        }
#endif
      }
      wait(shortWait * 10);
      sendAfterResTask = true;
      configSend();
    }
  }
}


void setup()
{
  hwSleep(1000);
  config_Happy_node();
  CORE_DEBUG(PSTR("MyS: CONFIG HAPPY NODE\n"));
  if (flag_nogateway_mode == false) {
    epd.Init(PART);
    epd.Clear(colorPrint, PART);
    paint.Clear(opposite_colorPrint);
#ifdef LANG_RU
    DrawImageWH(&paint, 42, 71, IMAGE_CON, 48, 108, colorPrint);
    DrawImageWH(&paint, 85, 71, IMAGE_CONACTIV, 16, 108, colorPrint);
#else
    DrawImageWH(&paint, 42, 71, IMAGE_ENCON, 48, 108, colorPrint);
    DrawImageWH(&paint, 85, 71, IMAGE_ECONACTIV, 16, 108, colorPrint);
#endif
    epd.DisplayPart(paint.GetImage());
    epd.Sleep();
    sendResetReason();
  } else {
    epd.Init(PART);
    epd.Clear(colorPrint, PART);
    paint.Clear(opposite_colorPrint);
#ifdef LANG_RU
    DrawImageWH(&paint, 42, 71, IMAGE_CON, 48, 108, colorPrint);
    DrawImageWH(&paint, 85, 71, IMAGE_NOCONACTIV, 16, 108, colorPrint);
#else
    DrawImageWH(&paint, 42, 71, IMAGE_ENCON, 48, 108, colorPrint);
    DrawImageWH(&paint, 85, 71, IMAGE_ENOCONACTIV, 16, 108, colorPrint);
#endif
    epd.DisplayPart(paint.GetImage());
    epd.Sleep();
  }

  CORE_DEBUG(PSTR("MyS: SEND CONFIG PARAMETERS\n"));
  sendAfterResTask = true;
  sleepTimeCount = sleepTime;
  metric = getControllerConfig().isMetric;
  metric = false;
  transportDisable();
  wait(20);
  bme_initAsleep();
  wait(50);
  sgp.begin();
  loadBaseLine();
  wait(50);
  interrupt_Init();
  wait(20);
  readBatt();
  startTimer = millis();
}



void loop() {
  if (flag_update_transport_param == true) {
    update_Happy_transport();
  }
  if (flag_sendRoute_parent == true) {
    present_only_parent();
  }
  if (isTransportReady() == true) {
    if (flag_find_parent_process == true) {
      find_parent_process();
    }
    if (flag_nogateway_mode == false) {
      if (configMode == false) {
        if (buttIntStatus == PIN_BUTTON) {
          if (digitalRead(PIN_BUTTON) == 0 && button_flag == false) {
            button_flag = true;
            previousMillis = millis();
          }
          if (digitalRead(PIN_BUTTON) == 0 && button_flag == true) {
            if ((millis() - previousMillis > 0) && (millis() - previousMillis <= 3000)) {
              if (updateink1 == false) {
                einkZeropush();
                updateink1 = true;
              }
            }
            if ((millis() - previousMillis > 3000) && (millis() - previousMillis <= 4000)) {
              if (updateinkclear == false) {
                epd.Clear(colorPrint, PART);
                updateinkclear = true;
              }
            }
            if ((millis() - previousMillis > 4000) && (millis() - previousMillis <= 7000)) {
              if (updateink2 == false) {
                einkOnepush();
                updateink2 = true;
                updateinkclear = false;
              }
            }
            if ((millis() - previousMillis > 7000) && (millis() - previousMillis <= 8000)) {
              if (updateinkclear == false) {
                epd.Clear(colorPrint, PART);
                updateinkclear = true;
              }
            }
            if ((millis() - previousMillis > 8000) && (millis() - previousMillis <= 11000)) {
              if (updateink3 == false) {
                einkOnePluspush();
                updateink3 = true;
                updateinkclear = false;
              }
            }
            if ((millis() - previousMillis > 11000) && (millis() - previousMillis <= 12000)) {
              if (updateinkclear == false) {
                epd.Clear(colorPrint, PART);
                updateinkclear = true;
              }
            }
            if ((millis() - previousMillis > 12000) && (millis() - previousMillis <= 15000)) {
              if (updateink4 == false) {
                einkTwopush();
                updateink4 = true;
                updateinkclear = false;
              }
            }
            if (millis() - previousMillis > 15000) {
              if (updateinkclear == false) {
                epd.Clear(colorPrint, PART);
                updateinkclear = true;
                buttIntStatus = 0;
                change = true;
                sleepTimeCount = sleepTime;
              }
            }
          }

          if (digitalRead(PIN_BUTTON) == 1 && button_flag == true) {
            if (millis() - previousMillis <= 3000 && button_flag == true)
            {
              einkPushEnd();
              reseteinkset();
              if (colorPrint == true) {
                colorPrint = false;
                colorChange(colorPrint);
              } else {
                colorPrint = true;
                colorChange(colorPrint);
              }
              sleepTimeCount = 0;
              readSensor();
              sendData();
              //wait(20);
              eInkUpdate();
              change = false;
              button_flag = false;
              buttIntStatus = 0;
              nosleep = false;
            }
            if ((millis() - previousMillis > 4000) && (millis() - previousMillis <= 7000) && button_flag == true)
            {
              einkPushEnd();
              reseteinkset();
              transportReInitialise();
              wait(shortWait);
              resetBeforePresent();
              presentation();
              transportDisable();
              wait(shortWait);
              sleepTimeCount = 0;
              readSensor();
              sendData();
              //wait(20);
              eInkUpdate();
              change = false;
              button_flag = false;
              buttIntStatus = 0;
              nosleep = false;
            }
            if ((millis() - previousMillis > 8000) && (millis() - previousMillis <= 11000) && button_flag == true)
            {
              einkPushEnd();
              reseteinkset();
              configMode = true;
              button_flag = false;
              buttIntStatus = 0;
              transportReInitialise();
              wait(shortWait);
              NRF5_ESB_startListening();
              wait(shortWait * 5);
              configMillis = millis();
            }
            if ((millis() - previousMillis > 12000) && (millis() - previousMillis <= 15000) && button_flag == true)
            {
              einkPushEnd();
              wait(1500);
              new_device();
            }
            if (((millis() - previousMillis > 3000) && (millis() - previousMillis <= 4000) && button_flag == true) || ((millis() - previousMillis > 7000) && (millis() - previousMillis <= 8000) && button_flag == true) || ((millis() - previousMillis > 11000) && (millis() - previousMillis <= 12000) && button_flag == true) || ((millis() - previousMillis > 15000) && button_flag == true))
            {
              wdt_nrfReset();
              reseteinkset();
              sleepTimeCount = 0;
              readSensor();
              sendData();
              //wait(20);
              eInkUpdate();
              change = false;
              button_flag = false;
              buttIntStatus = 0;
              nosleep = false;
            }
          }
        } else if (buttIntStatus == 0) {
          if (sleepTimeCount == sleepTime) {
            readSensor();
            if (change == true) {
              sendData();
              //delay(200);
              eInkUpdate();
              change = false;
            }
            nosleep = false;
            sleepTimeCount = 0;
          } else if (sleepTimeCount < sleepTime) {
            readSGP();
            if (change == true) {
              sendData();
              //delay(200);
              eInkUpdate();
              change = false;
            }
            nosleep = false;
          }
          sleepTimeCount++;
        }
      } else if (configMode == true) {
        if (millis() - configMillis > 20000) {
          transportDisable(); // вроде потому что один фиг сразу в сон? ....не все таки раскоментить потому что сон не сразу а сначала обновление экрана
          configMode = false;
          sleepTimeCount = 0;
          readSensor();
          sendData();
          //wait(20);
          eInkUpdate();
          change = false;
          button_flag = false;
          buttIntStatus = 0;
          nosleep = false;
        }
        wdt_nrfReset();
      }
    } else {
      if (buttIntStatus == PIN_BUTTON) {
        if (digitalRead(PIN_BUTTON) == 0 && button_flag == 0) {
          button_flag = 1;
          previousMillis = millis();
        }
        if (digitalRead(PIN_BUTTON) == 0 && button_flag == 1) {
          if ((millis() - previousMillis > 0) && (millis() - previousMillis <= 3000)) {
            if (updateink1 == false) {
              einkZeropush();
              updateink1 = true;
            }
          }
          if ((millis() - previousMillis > 3000) && (millis() - previousMillis <= 4000)) {
            if (updateinkclear == false) {
              epd.Clear(colorPrint, PART);
              updateinkclear = true;
            }
          }
          if ((millis() - previousMillis > 4000) && (millis() - previousMillis <= 7000)) {
            if (updateink2 == false) {
              einkOnepush();
              updateink2 = true;
              updateinkclear = false;
            }
          }
          if ((millis() - previousMillis > 7000) && (millis() - previousMillis <= 8000)) {
            if (updateinkclear == false) {
              epd.Clear(colorPrint, PART);
              updateinkclear = true;
            }
          }
          if ((millis() - previousMillis > 8000) && (millis() - previousMillis <= 11000)) {
            if (updateink4 == false) {
              einkTwopush();
              updateink4 = true;
              updateinkclear = false;
            }
          }
          if (millis() - previousMillis > 11000) {
            if (updateinkclear == false) {
              epd.Clear(colorPrint, PART);
              updateinkclear = true;
              buttIntStatus = 0;
              change = true;
              sleepTimeCount = sleepTime;
            }
          }
        }

        if (digitalRead(PIN_BUTTON) == 1 && button_flag == true) {
          if (millis() - previousMillis <= 3000 && button_flag == true)
          {
            einkPushEnd();
            reseteinkset();
            if (colorPrint == true) {
              colorPrint = false;
              colorChange(colorPrint);
            } else {
              colorPrint = true;
              colorChange(colorPrint);
            }
            sleepTimeCount = 0;
            readSensor();
            sendData();
            //wait(20);
            eInkUpdate();
            change = false;
            button_flag = false;
            buttIntStatus = 0;
            nosleep = false;
          }
          if ((millis() - previousMillis > 4000) && (millis() - previousMillis <= 7000) && button_flag == true)
          {
            wdt_nrfReset();
            einkPushEnd();
            reseteinkset();
            button_flag = false;
            buttIntStatus = 0;
            transportReInitialise();
            check_parent();
            cpCount = 0;
            change = true;
            sleepTimeCount = sleepTime;
            nosleep = false;
          }
          if ((millis() - previousMillis > 8000) && (millis() - previousMillis <= 11000) && button_flag == true)
          {
            einkPushEnd();
            wait(1500);
            new_device();
          }
          if (((millis() - previousMillis > 3000) && (millis() - previousMillis <= 4000) && button_flag == true) || ((millis() - previousMillis > 7000) && (millis() - previousMillis <= 8000) && button_flag == true) || ((millis() - previousMillis > 11000) && button_flag == true))
          {
            wdt_nrfReset();
            reseteinkset();
            sleepTimeCount = 0;
            readSensor();
            sendData();
            //wait(20);
            eInkUpdate();
            change = false;
            button_flag = false;
            buttIntStatus = 0;
            nosleep = false;
          }
        }

      } else if (buttIntStatus == 0) {
        if (sleepTimeCount == sleepTime) {
          sleepTimeCount = 0;
          cpCount++;
          if (cpCount >= cpNom) {
            transportReInitialise();
            check_parent();
            cpCount = 0;
          }
          readSensor();
          if (change == true) {
            sendData();
            //wait(100);
            eInkUpdate();
            change = false;
          }
        } else if (sleepTimeCount < sleepTime) {
          readSGP();
          if (change == true) {
            sendData();
            //wait(100);
            eInkUpdate();
            change = false;
          }
        }
        if (cpCount < cpNom) {
          nosleep = false;
        }
        sleepTimeCount++;
      }
    }
  }

  if (_transportSM.failureCounter > 0)
  {
    _transportConfig.parentNodeId = loadState(201);
    _transportConfig.nodeId = myid;
    _transportConfig.distanceGW = loadState(202);
    mypar = _transportConfig.parentNodeId;
    nosleep = false;
    err_delivery_beat = 7;
    happy_node_mode();
    gateway_fail();
  }

  if (nosleep == false) {
    transportDisable();
    wdt_nrfReset();

    stopTimer = millis();
    if (stopTimer < startTimer) {
      periodTimer = (4294967295 - startTimer) + stopTimer;
    } else {
      periodTimer = stopTimer - startTimer;
    }
    if (periodTimer >= sleepTimeWDT) {
      precisionTimeWDT = sleepTimeWDT;
    } else {
      precisionTimeWDT = sleepTimeWDT - periodTimer;
    }
    //send(debMsg1.set(sleepTimeCount));
    //send(debMsg2.set(precisionTimeWDT));
    hwSleep(precisionTimeWDT);
    startTimer = millis();
    nosleep = true;
    wdt_nrfReset();
  }
}


// ##############################################################################################################
// #                                                 E-PAPER DISP                                               #
// ##############################################################################################################

void DrawImageWH(Paint * paint, int x, int y, const unsigned char* imgData, int Width, int Height, int colored)
{
  int i, j;
  const unsigned char* prt = imgData;
  for (j = 0; j < Height; j++) {
    for (i = 0; i < Width; i++) {
      if (pgm_read_byte(prt) & (0x80 >> (i % 8))) {
        paint->DrawPixel(x + i, y + j, colored);
      }
      if (i % 8 == 7) {
        prt++;
      }
    }
    if (Width % 8 != 0) {
      prt++;
    }
  }
}

void colorChange(bool flag) {
  if (flag == true) {
    colorPrint = true;
    opposite_colorPrint = false;
  } else {
    colorPrint = false;
    opposite_colorPrint = true;
  }
  saveState(104, flag);
}

void displayStart() {
  epd.Init(FULL);
  epd.Clear(opposite_colorPrint, FULL);
  paint.SetWidth(122);
  paint.SetHeight(250);
  paint.SetRotate(ROTATE_180);
  paint.Clear(opposite_colorPrint);
  DrawImageWH(&paint, 8, 78, IMAGE_LOGO4, 105, 95, colorPrint);
  DrawImageWH(&paint, 8, 78, IMAGE_LOGO5, 105, 95, colorPrint);
  DrawImageWH(&paint, 116, 67, IMAGE_LOGO6, 7, 117, colorPrint);
  epd.Display(paint.GetImage(), FULL);
  hwSleep(2400);
  epd.Init(PART);
  epd.Clear(opposite_colorPrint, PART);
  paint.Clear(opposite_colorPrint);

  // ###################################           Especially for            ################################### //
#ifdef ESPECIALLY
  DrawImageWH(&paint, 8, 0, Especially, 122, 250, colorPrint);
  epd.Clear(opposite_colorPrint, PART);
  epd.Display(paint.GetImage(), PART);
  epd.Clear(opposite_colorPrint, PART);
  epd.Display(paint.GetImage(), PART);
  hwSleep(4000);
  epd.Clear(opposite_colorPrint, PART);
  paint.Clear(opposite_colorPrint);
#endif
  // ###################################           Especially for            ################################### //

#ifdef LANG_RU
  DrawImageWH(&paint, 42, 71, IMAGE_CON, 48, 108, colorPrint);
#else
  DrawImageWH(&paint, 42, 71, IMAGE_ENCON, 48, 108, colorPrint);
#endif
  epd.Clear(opposite_colorPrint, PART);
  epd.Display(paint.GetImage(), PART);
  epd.Clear(opposite_colorPrint, PART);
  epd.Display(paint.GetImage(), PART);
  epd.Sleep();
}


void eInkUpdate() {
  wdt_nrfReset();
  epd.Init(PART);
  paint.Clear(opposite_colorPrint);
  if (!metric) {
  displayTemp(temperatureSend2, metric);
  }else{
   displayTemp(temperatureSend, metric); 
  }
  displayForecast(forecast);
  displayAQ(vocIndex);
  displayPres(pressureSend);
  displayHum(humiditySend);
  display_Icons();
  epd.Clear(opposite_colorPrint, PART);
  epd.DisplayPart(paint.GetImage());
  epd.Clear(opposite_colorPrint, PART);
  epd.DisplayPart(paint.GetImage());
  epd.Sleep();
}


void displayTemp(float temp, bool metr) {

  int temperature_temp = round(temp * 10.0);

  if (metr) {

    DrawImageWH(&paint, 97, 30, IMAGE_DATA_NNC, 12, 25, colorPrint);

    if (temperature_temp >= 100) {

      if (last_tmp != 0) {
        if (temp > last_tmp) {
          DrawImageWH(&paint, 77, 72, IMAGE_UP, 14, 12, colorPrint);
          upTemp = true;
          downTemp = false;
        } else if (temp < last_tmp) {
          DrawImageWH(&paint, 77, 72, IMAGE_DOWN, 14, 12, colorPrint);
          upTemp = false;
          downTemp = true;
        } else {
          if (upTemp == true) {
            DrawImageWH(&paint, 77, 72, IMAGE_UP, 14, 12, colorPrint);
          }
          if (downTemp == true) {
            DrawImageWH(&paint, 77, 72, IMAGE_DOWN, 14, 12, colorPrint);
          }
        }
      }
      last_tmp = temp;

      byte one_t = temperature_temp / 100;
      byte two_t = temperature_temp % 100 / 10;
      byte three_t = temperature_temp % 10;

      switch (one_t) {
        case 1:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
          break;
      }

      switch (two_t) {
        case 0:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS0, 16, 18, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
          break;
      }

      DrawImageWH(&paint, 77, 50, IMAGE_DATA_NNSSSPOINT, 16, 4, colorPrint);

      switch (three_t) {
        case 0:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS0, 16, 18, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
          break;
      }
    } else {

      if (last_tmp != 0) {
        if (temp > last_tmp) {
          DrawImageWH(&paint, 77, 63, IMAGE_UP, 14, 12, colorPrint);
          upTemp = true;
          downTemp = false;
        } else if (temp < last_tmp) {
          DrawImageWH(&paint, 77, 63, IMAGE_DOWN, 14, 12, colorPrint);
          upTemp = false;
          downTemp = true;
        } else {
          if (upTemp == true) {
            DrawImageWH(&paint, 77, 63, IMAGE_UP, 14, 12, colorPrint);
          }
          if (downTemp == true) {
            DrawImageWH(&paint, 77, 63, IMAGE_DOWN, 14, 12, colorPrint);
          }
        }
      }
      last_tmp = temp;

      byte one_t = temperature_temp / 10;
      byte two_t = temperature_temp % 10;

      switch (one_t) {
        case 1:
          DrawImageWH(&paint, 77, 23, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 77, 23, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 77, 23, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 77, 23, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 77, 23, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 77, 23, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 77, 23, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 77, 23, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 77, 23, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
          break;
      }

      DrawImageWH(&paint, 77, 41, IMAGE_DATA_NNSSSPOINT, 16, 4, colorPrint);

      switch (two_t) {
        case 0:
          DrawImageWH(&paint, 77, 45, IMAGE_DATA_NNSS0, 16, 18, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 77, 45, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 77, 45, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 77, 45, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 77, 45, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 77, 45, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 77, 45, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 77, 45, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 77, 45, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 77, 45, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
          break;
      }
    }
  } else {

    DrawImageWH(&paint, 97, 30, IMAGE_DATA_NNF, 12, 25, colorPrint);

    if (temperature_temp < 1000) {

      if (last_tmp != 0) {
        if (temp > last_tmp) {
          DrawImageWH(&paint, 77, 72, IMAGE_UP, 14, 12, colorPrint);
          upTemp = true;
          downTemp = false;
        } else if (temp < last_tmp) {
          DrawImageWH(&paint, 77, 72, IMAGE_DOWN, 14, 12, colorPrint);
          upTemp = false;
          downTemp = true;
        } else {
          if (upTemp == true) {
            DrawImageWH(&paint, 77, 72, IMAGE_UP, 14, 12, colorPrint);
          }
          if (downTemp == true) {
            DrawImageWH(&paint, 77, 72, IMAGE_DOWN, 14, 12, colorPrint);
          }
        }
      }
      last_tmp = temp;

      byte one_t = temperature_temp / 100;
      byte two_t = temperature_temp % 100 / 10;
      byte three_t = temperature_temp % 10;

      switch (one_t) {
        case 1:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
          break;
      }

      switch (two_t) {
        case 0:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS0, 16, 18, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
          break;
      }

      DrawImageWH(&paint, 77, 50, IMAGE_DATA_NNSSSPOINT, 16, 4, colorPrint);

      switch (three_t) {
        case 0:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS0, 16, 18, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
          break;
      }
    } else {

      if (last_tmp != 0) {
        if (temp > last_tmp) {
          DrawImageWH(&paint, 77, 68, IMAGE_UP, 14, 12, colorPrint);
          upTemp = true;
          downTemp = false;
        } else if (temp < last_tmp) {
          DrawImageWH(&paint, 77, 68, IMAGE_DOWN, 14, 12, colorPrint);
          upTemp = false;
          downTemp = true;
        } else {
          if (upTemp == true) {
            DrawImageWH(&paint, 77, 68, IMAGE_UP, 14, 12, colorPrint);
          }
          if (downTemp == true) {
            DrawImageWH(&paint, 77, 68, IMAGE_DOWN, 14, 12, colorPrint);
          }
        }
      }
      last_tmp = temp;

      byte one_t = temperature_temp / 1000;
      byte two_t = temperature_temp % 1000 / 100;
      byte three_t = temperature_temp % 100 / 10;

      switch (one_t) {
        case 1:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 77, 14, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
          break;
      }

      switch (two_t) {
        case 0:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS0, 16, 18, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 77, 32, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
          break;
      }

      switch (three_t) {
        case 0:
          DrawImageWH(&paint, 77, 50, IMAGE_DATA_NNSS0, 16, 18, colorPrint);
          break;
        case 1:
          DrawImageWH(&paint, 77, 50, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
          break;
        case 2:
          DrawImageWH(&paint, 77, 50, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
          break;
        case 3:
          DrawImageWH(&paint, 77, 50, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
          break;
        case 4:
          DrawImageWH(&paint, 77, 54, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
          break;
        case 5:
          DrawImageWH(&paint, 77, 50, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
          break;
        case 6:
          DrawImageWH(&paint, 77, 50, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
          break;
        case 7:
          DrawImageWH(&paint, 77, 50, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
          break;
        case 8:
          DrawImageWH(&paint, 77, 50, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
          break;
        case 9:
          DrawImageWH(&paint, 77, 50, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
          break;
      }
    }
  }
}


void displayHum(float hum) {

  if ((int)hum < 10) {
    hum = 10.0;
  }

  int hum_temp = round(hum * 10.0);
  if (hum_temp < 100) {
    hum_temp = 100;
  }

  if (last_hm != 0) {
    if (hum > last_hm) {
      DrawImageWH(&paint, 77, 235, IMAGE_UP, 14, 12, colorPrint);
      upHum = true;
      downHum = false;
    } else if (hum < last_hm) {
      DrawImageWH(&paint, 77, 235, IMAGE_DOWN, 14, 12, colorPrint);
      upHum = false;
      downHum = true;
    } else {
      if (upHum == true) {
        DrawImageWH(&paint, 77, 235, IMAGE_UP, 14, 12, colorPrint);
      }
      if (downHum == true) {
        DrawImageWH(&paint, 77, 235, IMAGE_DOWN, 14, 12, colorPrint);
      }
    }
  }
  last_hm = hum;

  byte one_h = hum_temp / 100;
  byte two_h = hum_temp % 100 / 10;
  byte three_h = hum_temp % 10;

  DrawImageWH(&paint, 97, 193, IMAGE_DATA_HHPR, 12, 25, colorPrint);

  switch (one_h) {
    case 0:
      DrawImageWH(&paint, 77, 177, IMAGE_DATA_NNSS0, 16, 18, colorPrint);
      break;
    case 1:
      DrawImageWH(&paint, 77, 177, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
      break;
    case 2:
      DrawImageWH(&paint, 77, 177, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
      break;
    case 3:
      DrawImageWH(&paint, 77, 177, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
      break;
    case 4:
      DrawImageWH(&paint, 77, 177, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
      break;
    case 5:
      DrawImageWH(&paint, 77, 177, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
      break;
    case 6:
      DrawImageWH(&paint, 77, 177, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
      break;
    case 7:
      DrawImageWH(&paint, 77, 177, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
      break;
    case 8:
      DrawImageWH(&paint, 77, 177, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
      break;
    case 9:
      DrawImageWH(&paint, 77, 177, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
      break;
  }

  switch (two_h) {
    case 0:
      DrawImageWH(&paint, 77, 195, IMAGE_DATA_NNSS0, 16, 18, colorPrint);
      break;
    case 1:
      DrawImageWH(&paint, 77, 195, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
      break;
    case 2:
      DrawImageWH(&paint, 77, 195, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
      break;
    case 3:
      DrawImageWH(&paint, 77, 195, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
      break;
    case 4:
      DrawImageWH(&paint, 77, 195, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
      break;
    case 5:
      DrawImageWH(&paint, 77, 195, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
      break;
    case 6:
      DrawImageWH(&paint, 77, 195, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
      break;
    case 7:
      DrawImageWH(&paint, 77, 195, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
      break;
    case 8:
      DrawImageWH(&paint, 77, 195, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
      break;
    case 9:
      DrawImageWH(&paint, 77, 195, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
      break;
  }

  DrawImageWH(&paint, 77, 213, IMAGE_DATA_NNSSSPOINT, 16, 4, colorPrint);

  switch (three_h) {
    case 0:
      DrawImageWH(&paint, 77, 217, IMAGE_DATA_NNSS0, 16, 18, colorPrint);
      break;
    case 1:
      DrawImageWH(&paint, 77, 217, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
      break;
    case 2:
      DrawImageWH(&paint, 77, 217, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
      break;
    case 3:
      DrawImageWH(&paint, 77, 217, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
      break;
    case 4:
      DrawImageWH(&paint, 77, 217, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
      break;
    case 5:
      DrawImageWH(&paint, 77, 217, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
      break;
    case 6:
      DrawImageWH(&paint, 77, 217, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
      break;
    case 7:
      DrawImageWH(&paint, 77, 217, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
      break;
    case 8:
      DrawImageWH(&paint, 77, 217, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
      break;
    case 9:
      DrawImageWH(&paint, 77, 217, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
      break;
  }
}


void displayPres(float pres) {

  int pressure_temp = round(pres);
  if (pressure_temp < 1000) {
    pressure_temp = round(pres * 10.0);
  }

#ifdef LANG_RU
  DrawImageWH(&paint, 101, 102, IMAGE_DATA_PPMM, 5, 47, colorPrint);
#else
  DrawImageWH(&paint, 101, 102, IMAGE_DATA_PPGPA, 5, 47, colorPrint);
#endif

  byte one_p = pressure_temp / 1000;
  byte two_p = pressure_temp % 1000 / 100;
  byte three_p = pressure_temp % 100 / 10;
  byte four_p = pressure_temp % 10;

  if (one_p == 1) {

    if (last_pr != 0) {
      if (pres > last_pr) {
        DrawImageWH(&paint, 87, 151, IMAGE_UP, 14, 12, colorPrint);
        upPress = true;
        downPress = false;
      } else if (pres < last_pr) {
        DrawImageWH(&paint, 87, 151, IMAGE_DOWN, 14, 12, colorPrint);
        upPress = false;
        downPress = true;
      } else {
        if (upPress == true) {
          DrawImageWH(&paint, 87, 151, IMAGE_UP, 14, 12, colorPrint);
        }
        if (downPress == true) {
          DrawImageWH(&paint, 87, 151, IMAGE_DOWN, 14, 12, colorPrint);
        }
      }
    }
    last_pr = pres;

    switch (one_p) {
      case 1:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS1, 12, 14, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS2, 12, 14, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS3, 12, 14, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS4, 12, 14, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS5, 12, 14, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS6, 12, 14, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS7, 12, 14, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS8, 12, 14, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS9, 12, 14, colorPrint);
        break;
    }

    switch (two_p) {
      case 0:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS0, 12, 14, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS1, 12, 14, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS2, 12, 14, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS3, 12, 14, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS4, 12, 14, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS5, 12, 14, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS6, 12, 14, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS7, 12, 14, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS8, 12, 14, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS9, 12, 14, colorPrint);
        break;
    }

    switch (three_p) {
      case 0:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS0, 12, 14, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS1, 12, 14, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS2, 12, 14, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS3, 12, 14, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS4, 12, 14, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS5, 12, 14, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS6, 12, 14, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS7, 12, 14, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS8, 12, 14, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS9, 12, 14, colorPrint);
        break;
    }

    switch (four_p) {
      case 0:
        DrawImageWH(&paint, 87, 137, IMAGE_DATA_NNSSS0, 12, 14, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 87, 137, IMAGE_DATA_NNSSS1, 12, 14, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 87, 137, IMAGE_DATA_NNSSS2, 12, 14, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 87, 137, IMAGE_DATA_NNSSS3, 12, 14, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 87, 137, IMAGE_DATA_NNSSS4, 12, 14, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 87, 137, IMAGE_DATA_NNSSS5, 12, 14, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 87, 137, IMAGE_DATA_NNSSS6, 12, 14, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 87, 137, IMAGE_DATA_NNSSS7, 12, 14, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 87, 137, IMAGE_DATA_NNSSS8, 12, 14, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 87, 137, IMAGE_DATA_NNSSS9, 12, 14, colorPrint);
        break;
    }
  } else {

    if (last_pr != 0) {
      if (pres > last_pr) {
        DrawImageWH(&paint, 87, 155, IMAGE_UP, 14, 12, colorPrint);
        upPress = true;
        downPress = false;
      } else if (pres < last_pr) {
        DrawImageWH(&paint, 87, 155, IMAGE_DOWN, 14, 12, colorPrint);
        upPress = false;
        downPress = true;
      } else {
        if (upPress == true) {
          DrawImageWH(&paint, 87, 155, IMAGE_UP, 14, 12, colorPrint);
        }
        if (downPress == true) {
          DrawImageWH(&paint, 87, 155, IMAGE_DOWN, 14, 12, colorPrint);
        }
      }
    }
    last_pr = pres;

    switch (one_p) {
      case 1:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS1, 12, 14, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS2, 12, 14, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS3, 12, 14, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS4, 12, 14, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS5, 12, 14, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS6, 12, 14, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS7, 12, 14, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS8, 12, 14, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 87, 95, IMAGE_DATA_NNSSS9, 12, 14, colorPrint);
        break;
    }

    switch (two_p) {
      case 0:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS0, 12, 14, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS1, 12, 14, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS2, 12, 14, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS3, 12, 14, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS4, 12, 14, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS5, 12, 14, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS6, 12, 14, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS7, 12, 14, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS8, 12, 14, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 87, 109, IMAGE_DATA_NNSSS9, 12, 14, colorPrint);
        break;
    }

    switch (three_p) {
      case 0:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS0, 12, 14, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS1, 12, 14, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS2, 12, 14, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS3, 12, 14, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS4, 12, 14, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS5, 12, 14, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS6, 12, 14, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS7, 12, 14, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS8, 12, 14, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 87, 123, IMAGE_DATA_NNSSS9, 12, 14, colorPrint);
        break;
    }

    DrawImageWH(&paint, 87, 137, IMAGE_DATA_NNSSPOINT, 12, 4, colorPrint);

    switch (four_p) {
      case 0:
        DrawImageWH(&paint, 87, 141, IMAGE_DATA_NNSSS0, 12, 14, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 87, 141, IMAGE_DATA_NNSSS1, 12, 14, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 87, 141, IMAGE_DATA_NNSSS2, 12, 14, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 87, 141, IMAGE_DATA_NNSSS3, 12, 14, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 87, 141, IMAGE_DATA_NNSSS4, 12, 14, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 87, 141, IMAGE_DATA_NNSSS5, 12, 14, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 87, 141, IMAGE_DATA_NNSSS6, 12, 14, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 87, 141, IMAGE_DATA_NNSSS7, 12, 14, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 87, 141, IMAGE_DATA_NNSSS8, 12, 14, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 87, 141, IMAGE_DATA_NNSSS9, 12, 14, colorPrint);
        break;
    }
  }
}


void displayForecast(uint8_t f) {
  //f=1;
#ifdef LANG_RU
  switch (f) {
    case 0:
      DrawImageWH(&paint, 108, 75, IMAGE_DATA_W0, 16, 100, colorPrint);
      break;
    case 1:
      DrawImageWH(&paint, 108, 75, IMAGE_DATA_W1, 16, 100, colorPrint);
      break;
    case 2:
      DrawImageWH(&paint, 108, 75, IMAGE_DATA_W2, 16, 100, colorPrint);
      break;
    case 3:
      DrawImageWH(&paint, 108, 75, IMAGE_DATA_W3, 16, 100, colorPrint);
      break;
    case 4:
      DrawImageWH(&paint, 108, 75, IMAGE_DATA_W4, 16, 100, colorPrint);
      break;
    case 5:
      DrawImageWH(&paint, 108, 75, IMAGE_DATA_W5, 16, 100, colorPrint);
      break;
  }
#else
  switch (f) {
    case 0:
      DrawImageWH(&paint, 108, 75, IMAGE_DATA_EW0, 16, 100, colorPrint);
      break;
    case 1:
      DrawImageWH(&paint, 108, 75, IMAGE_DATA_EW1, 16, 100, colorPrint);
      break;
    case 2:
      DrawImageWH(&paint, 108, 75, IMAGE_DATA_EW2, 16, 100, colorPrint);
      break;
    case 3:
      DrawImageWH(&paint, 108, 75, IMAGE_DATA_EW3, 16, 100, colorPrint);
      break;
    case 4:
      DrawImageWH(&paint, 108, 75, IMAGE_DATA_EW4, 16, 100, colorPrint);
      break;
    case 5:
      DrawImageWH(&paint, 108, 75, IMAGE_DATA_EW5, 16, 100, colorPrint);
      break;
  }
#endif
}


void displayAQ(int32_t aq_temp) {

#ifdef LANG_RU
  DrawImageWH(&paint, 13, 55, AQ, 7, 140, colorPrint);
#else
  DrawImageWH(&paint, 13, 55, AQEN, 7, 140, colorPrint);
#endif


  //aq_temp = 45;
  if (aq_temp >= 300) {
#ifdef LANG_EN
    DrawImageWH(&paint, 22, 60, AQST6EN, 13, 130, colorPrint);
#else
    DrawImageWH(&paint, 22, 60, AQST6, 13, 130, colorPrint);
#endif
  } else if (aq_temp >= 250 && aq_temp < 300) {
#ifdef LANG_EN
    DrawImageWH(&paint, 22, 60, AQST5EN, 13, 130, colorPrint);
#else
    DrawImageWH(&paint, 22, 60, AQST5, 13, 130, colorPrint);
#endif
  } else if (aq_temp >= 200 && aq_temp < 250) {
#ifdef LANG_EN
    DrawImageWH(&paint, 22, 60, AQST4EN, 13, 130, colorPrint);
#else
    DrawImageWH(&paint, 22, 60, AQST4, 13, 130, colorPrint);
#endif
  } else if (aq_temp >= 150 && aq_temp < 200) {
#ifdef LANG_EN
    DrawImageWH(&paint, 22, 60, AQST3EN, 13, 130, colorPrint);
#else
    DrawImageWH(&paint, 22, 60, AQST3, 13, 130, colorPrint);
#endif
  } else if (aq_temp >= 100 && aq_temp < 150) {
#ifdef LANG_EN
    DrawImageWH(&paint, 22, 60, AQST2EN, 13, 130, colorPrint);
#else
    DrawImageWH(&paint, 22, 60, AQST2, 13, 130, colorPrint);
#endif
  } else if (aq_temp > 0 && aq_temp < 100) {
#ifdef LANG_EN
    DrawImageWH(&paint, 22, 60, AQST1EN, 13, 130, colorPrint);
#else
    DrawImageWH(&paint, 22, 60, AQST1, 13, 130, colorPrint);
#endif
  } else if (aq_temp == 0) {
#ifdef LANG_EN
    DrawImageWH(&paint, 22, 60, AQST0EN, 13, 130, colorPrint);
#else
    DrawImageWH(&paint, 22, 60, AQST0, 13, 130, colorPrint);
#endif
  }

  if (icon_bl == 0) {

    if (learningCount <= 720 && learningCount > 660) {
      DrawImageWH(&paint, 38, 222, NOSAVE12, 16, 15, colorPrint);
    } else if (learningCount <= 660 && learningCount > 600) {
      DrawImageWH(&paint, 38, 222, NOSAVE11, 16, 15, colorPrint);
    } else if (learningCount <= 600 && learningCount > 540) {
      DrawImageWH(&paint, 38, 222, NOSAVE10, 16, 15, colorPrint);
    } else if (learningCount <= 540 && learningCount > 480) {
      DrawImageWH(&paint, 38, 222, NOSAVE9, 16, 15, colorPrint);
    } else if (learningCount <= 480 && learningCount > 420) {
      DrawImageWH(&paint, 38, 222, NOSAVE8, 16, 15, colorPrint);
    } else if (learningCount <= 420 && learningCount > 360) {
      DrawImageWH(&paint, 38, 222, NOSAVE7, 16, 15, colorPrint);
    } else if (learningCount <= 360 && learningCount > 300) {
      DrawImageWH(&paint, 38, 222, NOSAVE6, 16, 15, colorPrint);
    } else if (learningCount <= 300 && learningCount > 240) {
      DrawImageWH(&paint, 38, 222, NOSAVE5, 16, 15, colorPrint);
    } else if (learningCount <= 240 && learningCount > 180) {
      DrawImageWH(&paint, 38, 222, NOSAVE4, 16, 15, colorPrint);
    } else if (learningCount <= 180 && learningCount > 120) {
      DrawImageWH(&paint, 38, 222, NOSAVE3, 16, 15, colorPrint);
    } else if (learningCount <= 120 && learningCount > 60) {
      DrawImageWH(&paint, 38, 222, NOSAVE2, 16, 15, colorPrint);
    } else if (learningCount <= 60 && learningCount > 0) {
      DrawImageWH(&paint, 38, 222, NOSAVE1, 16, 15, colorPrint);
    } else {
      //DrawImageWH(&paint, 38, 222, NOSAVE, 16, 15, colorPrint);
    }
  } else if (icon_bl == 1) {
    DrawImageWH(&paint, 38, 222, LOAD, 16, 15, colorPrint);
  } else if (icon_bl == 2) {
    DrawImageWH(&paint, 38, 222, SAVE, 16, 15, colorPrint);
  }

  if (aq_temp >= 100) {

    byte one_aq = aq_temp / 100;
    byte two_aq = aq_temp % 100 / 10;
    byte three_aq = aq_temp % 10;

#ifdef LANG_RU
    DrawImageWH(&paint, 52, 175, LOC, 18, 7, colorPrint);
#else
    DrawImageWH(&paint, 52, 175, VOC, 18, 7, colorPrint);
#endif

    if (aq_temp > last_aq) {
      DrawImageWH(&paint, 38, 173, IMAGE_UP, 14, 12, colorPrint);
      upAq = true;
      downAq = false;
    } else if (aq_temp < last_aq) {
      DrawImageWH(&paint, 38, 173, IMAGE_DOWN, 14, 12, colorPrint);
      upAq = false;
      downAq = true;
    } else {
      if (upAq == true) {
        DrawImageWH(&paint, 38, 173, IMAGE_UP, 14, 12, colorPrint);
      }
      if (downAq == true) {
        DrawImageWH(&paint, 38, 173, IMAGE_DOWN, 14, 12, colorPrint);
      }
    }
    last_aq = aq_temp;

    switch (one_aq) {
      case 0:
        DrawImageWH(&paint, 41, 77, IMAGE_DATA_NN0, 30, 32, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 41, 77, IMAGE_DATA_NN1, 30, 32, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 41, 77, IMAGE_DATA_NN2, 30, 32, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 41, 77, IMAGE_DATA_NN3, 30, 32, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 41, 77, IMAGE_DATA_NN4, 30, 32, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 41, 77, IMAGE_DATA_NN5, 30, 32, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 41, 77, IMAGE_DATA_NN6, 30, 32, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 41, 77, IMAGE_DATA_NN7, 30, 32, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 41, 77, IMAGE_DATA_NN8, 30, 32, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 41, 77, IMAGE_DATA_NN9, 30, 32, colorPrint);
        break;
    }

    switch (two_aq) {
      case 0:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN0, 30, 32, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN1, 30, 32, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN2, 30, 32, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN3, 30, 32, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN4, 30, 32, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN5, 30, 32, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN6, 30, 32, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN7, 30, 32, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN8, 30, 32, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN9, 30, 32, colorPrint);
        break;
    }

    switch (three_aq) {
      case 0:
        DrawImageWH(&paint, 41, 141, IMAGE_DATA_NN0, 30, 32, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 41, 141, IMAGE_DATA_NN1, 30, 32, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 41, 141, IMAGE_DATA_NN2, 30, 32, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 41, 141, IMAGE_DATA_NN3, 30, 32, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 41, 141, IMAGE_DATA_NN4, 30, 32, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 41, 141, IMAGE_DATA_NN5, 30, 32, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 41, 141, IMAGE_DATA_NN6, 30, 32, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 41, 141, IMAGE_DATA_NN7, 30, 32, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 41, 141, IMAGE_DATA_NN8, 30, 32, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 41, 141, IMAGE_DATA_NN9, 30, 32, colorPrint);
        break;
    }

  } else if (aq_temp >= 10) {

    byte one_aq = aq_temp / 10;
    byte two_aq = aq_temp % 10;

#ifdef LANG_RU
    DrawImageWH(&paint, 52, 159, LOC, 18, 7, colorPrint);
#else
    DrawImageWH(&paint, 52, 159, VOC, 18, 7, colorPrint);
#endif

    if (aq_temp > last_aq) {
      DrawImageWH(&paint, 38, 157, IMAGE_UP, 14, 12, colorPrint);
      upAq = true;
      downAq = false;
    } else if (aq_temp < last_aq) {
      DrawImageWH(&paint, 38, 157, IMAGE_DOWN, 14, 12, colorPrint);
      upAq = false;
      downAq = true;
    } else {
      if (upAq == true) {
        DrawImageWH(&paint, 38, 157, IMAGE_UP, 14, 12, colorPrint);
      }
      if (downAq == true) {
        DrawImageWH(&paint, 38, 157, IMAGE_DOWN, 14, 12, colorPrint);
      }
    }
    last_aq = aq_temp;

    switch (one_aq) {
      case 0:
        DrawImageWH(&paint, 41, 93, IMAGE_DATA_NN0, 30, 32, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 41, 93, IMAGE_DATA_NN1, 30, 32, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 41, 93, IMAGE_DATA_NN2, 30, 32, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 41, 93, IMAGE_DATA_NN3, 30, 32, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 41, 93, IMAGE_DATA_NN4, 30, 32, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 41, 93, IMAGE_DATA_NN5, 30, 32, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 41, 93, IMAGE_DATA_NN6, 30, 32, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 41, 93, IMAGE_DATA_NN7, 30, 32, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 41, 93, IMAGE_DATA_NN8, 30, 32, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 41, 93, IMAGE_DATA_NN9, 30, 32, colorPrint);
        break;
    }

    switch (two_aq) {
      case 0:
        DrawImageWH(&paint, 41, 125, IMAGE_DATA_NN0, 30, 32, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 41, 125, IMAGE_DATA_NN1, 30, 32, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 41, 125, IMAGE_DATA_NN2, 30, 32, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 41, 125, IMAGE_DATA_NN3, 30, 32, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 41, 125, IMAGE_DATA_NN4, 30, 32, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 41, 125, IMAGE_DATA_NN5, 30, 32, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 41, 125, IMAGE_DATA_NN6, 30, 32, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 41, 125, IMAGE_DATA_NN7, 30, 32, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 41, 125, IMAGE_DATA_NN8, 30, 32, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 41, 125, IMAGE_DATA_NN9, 30, 32, colorPrint);
        break;
    }

  } else {
#ifdef LANG_RU
    DrawImageWH(&paint, 52, 143, LOC, 18, 7, colorPrint);
#else
    DrawImageWH(&paint, 52, 143, VOC, 18, 7, colorPrint);
#endif

    if (aq_temp > last_aq) {
      DrawImageWH(&paint, 38, 141, IMAGE_UP, 14, 12, colorPrint);
      upAq = true;
      downAq = false;
    } else if (aq_temp < last_aq) {
      DrawImageWH(&paint, 38, 141, IMAGE_DOWN, 14, 12, colorPrint);
      upAq = false;
      downAq = true;
    } else {
      if (upAq == true) {
        DrawImageWH(&paint, 38, 141, IMAGE_UP, 14, 12, colorPrint);
      }
      if (downAq == true) {
        DrawImageWH(&paint, 38, 141, IMAGE_DOWN, 14, 12, colorPrint);
      }
    }
    last_aq = aq_temp;

    switch (aq_temp) {
      case 0:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN0, 30, 32, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN1, 30, 32, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN2, 30, 32, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN3, 30, 32, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN4, 30, 32, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN5, 30, 32, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN6, 30, 32, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN7, 30, 32, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN8, 30, 32, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 41, 109, IMAGE_DATA_NN9, 30, 32, colorPrint);
        break;
    }
  }
}


void display_Icons()
{
  displayLink(nRFRSSI);
  displayBatt(battery);
}


void displayLink(int8_t s) {

  if (flag_nogateway_mode == true) {
    s = 0;
  }

  if (s == 0) {
    DrawImageWH(&paint, 16, 220, IMAGE_DATA_L0, 16, 19, colorPrint);
  }
  if (s > 0 && s < 20) {
    DrawImageWH(&paint, 16, 220, IMAGE_DATA_L20, 16, 19, colorPrint);
  }
  if (s >= 20 && s < 40) {
    DrawImageWH(&paint, 16, 220, IMAGE_DATA_L40, 16, 19, colorPrint);
  }
  if (s >= 40 && s < 60) {
    DrawImageWH(&paint, 16, 220, IMAGE_DATA_L60, 16, 19, colorPrint);
  }
  if (s >= 60 && s <= 80) {
    DrawImageWH(&paint, 16, 220, IMAGE_DATA_L80, 16, 19, colorPrint);
  }
  if (s >= 80 && s <= 100) {
    DrawImageWH(&paint, 16, 220, IMAGE_DATA_L100, 16, 19, colorPrint);
  }
}


void displayBatt(uint8_t b) {


  if (b < 2) {
    DrawImageWH(&paint, 16, 11, IMAGE_DATA_B0, 16, 25, colorPrint);
  }
  if (b >= 2 && b < 13) {
    DrawImageWH(&paint, 16, 11, IMAGE_DATA_B13, 16, 25, colorPrint);
  }
  if (b >= 13 && b < 25) {
    DrawImageWH(&paint, 16, 11, IMAGE_DATA_B25, 16, 25, colorPrint);
  }
  if (b >= 25 && b < 38) {
    DrawImageWH(&paint, 16, 11, IMAGE_DATA_B38, 16, 25, colorPrint);
  }
  if (b >= 38 && b <= 50) {
    DrawImageWH(&paint, 16, 11, IMAGE_DATA_B50, 16, 25, colorPrint);
  }
  if (b >= 50 && b <= 63) {
    DrawImageWH(&paint, 16, 11, IMAGE_DATA_B63, 16, 25, colorPrint);
  }
  if (b >= 63 && b <= 75) {
    DrawImageWH(&paint, 16, 11, IMAGE_DATA_B75, 16, 25, colorPrint);
  }
  if (b >= 75 && b <= 87) {
    DrawImageWH(&paint, 16, 11, IMAGE_DATA_B87, 16, 25, colorPrint);
  }
  if (b >= 87 && b <= 100) {
    DrawImageWH(&paint, 16, 11, IMAGE_DATA_B100, 16, 25, colorPrint);
  }
}


void reseteinkset() {
  updateink1 = false;
  updateink2 = false;
  updateink3 = false;
  updateink4 = false;
  updateinkclear = false;
}


void einkZeropush() {
  wdt_nrfReset();
  epd.Init(PART);
  paint.Clear(opposite_colorPrint);
#ifdef LANG_RU
  DrawImageWH(&paint, 24, 71, IMAGE_COLOR, 48, 108, colorPrint);
#else
  DrawImageWH(&paint, 24, 71, IMAGE_ECOLOR, 48, 108, colorPrint);
#endif
  epd.Clear(colorPrint, PART);
  epd.Display(paint.GetImage(), PART);
}


void einkOnepush() {
  wdt_nrfReset();
  paint.Clear(opposite_colorPrint);
  if (flag_nogateway_mode == false) {
#ifdef LANG_RU
    DrawImageWH(&paint, 24, 71, IMAGE_PRESENT, 48, 108, colorPrint);
#else
    DrawImageWH(&paint, 24, 71, IMAGE_EPRESENT, 48, 108, colorPrint);
#endif
  } else {
#ifdef LANG_RU
    DrawImageWH(&paint, 24, 71, IMAGE_SEARCH, 48, 108, colorPrint);
#else
    DrawImageWH(&paint, 24, 71, IMAGE_ESEARCH, 48, 108, colorPrint);
#endif
  }
  epd.Display(paint.GetImage(), PART);

}


void einkOnePluspush() {
  wdt_nrfReset();
  paint.Clear(opposite_colorPrint);
#ifdef LANG_RU
  DrawImageWH(&paint, 24, 71, IMAGE_CONF, 48, 108, colorPrint);
#else
  DrawImageWH(&paint, 24, 71, IMAGE_ECONF, 48, 108, colorPrint);
#endif
  epd.Display(paint.GetImage(), PART);
}


void einkTwopush() {
  wdt_nrfReset();
  paint.Clear(opposite_colorPrint);
#ifdef LANG_RU
  DrawImageWH(&paint, 24, 71, IMAGE_RESET, 48, 108, colorPrint);
#else
  DrawImageWH(&paint, 24, 71, IMAGE_ERESET, 48, 108, colorPrint);
#endif
  epd.Display(paint.GetImage(), PART);
}


void einkPushEnd() {
  wdt_nrfReset();
#ifdef LANG_RU
  DrawImageWH(&paint, 76, 71, IMAGE_ACTIV, 16, 108, colorPrint);
#else
  DrawImageWH(&paint, 76, 71, IMAGE_EACTIV, 16, 108, colorPrint);
#endif
  epd.Display(paint.GetImage(), PART);
}


void reportTimeInk() {
  wdt_nrfReset();
  epd.Init(PART);
  paint.Clear(opposite_colorPrint);

#ifdef LANG_RU
  DrawImageWH(&paint, 24, 71, IMAGE_RTIME, 48, 108, colorPrint);
#else
  DrawImageWH(&paint, 24, 71, IMAGE_ERTIME, 48, 108, colorPrint);
#endif

  if (timeSend >= 10) {
    byte one_t = timeSend / 10;
    byte two_t = timeSend % 10;
    switch (one_t) {
      case 1:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
        break;
    }

    switch (two_t) {
      case 0:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS0, 16, 18, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
        break;
    }
  } else {
    switch (timeSend) {
      case 0:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS0, 16, 18, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
        break;
    }
  }
  epd.Clear(colorPrint, PART);
  epd.Display(paint.GetImage(), PART);
  epd.Sleep();
  wait(2000);
}


void reportBattInk() {
  wdt_nrfReset();
  epd.Init(PART);
  paint.Clear(opposite_colorPrint);

#ifdef LANG_RU
  DrawImageWH(&paint, 24, 71, IMAGE_RBATT, 48, 108, colorPrint);
#else
  DrawImageWH(&paint, 24, 71, IMAGE_ERBATT, 48, 108, colorPrint);
#endif

  if (battSend >= 10) {
    byte one_t = battSend / 10;
    byte two_t = battSend % 10;
    switch (one_t) {
      case 1:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 76, 109, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
        break;
    }

    switch (two_t) {
      case 0:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS0, 16, 18, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 76, 125, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
        break;
    }
  } else {
    switch (battSend) {
      case 0:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS0, 16, 18, colorPrint);
        break;
      case 1:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS1, 16, 18, colorPrint);
        break;
      case 2:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS2, 16, 18, colorPrint);
        break;
      case 3:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS3, 16, 18, colorPrint);
        break;
      case 4:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS4, 16, 18, colorPrint);
        break;
      case 5:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS5, 16, 18, colorPrint);
        break;
      case 6:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS6, 16, 18, colorPrint);
        break;
      case 7:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS7, 16, 18, colorPrint);
        break;
      case 8:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS8, 16, 18, colorPrint);
        break;
      case 9:
        DrawImageWH(&paint, 76, 117, IMAGE_DATA_NNSS9, 16, 18, colorPrint);
        break;
    }
  }
  epd.Clear(colorPrint, PART);
  epd.Display(paint.GetImage(), PART);
  epd.Sleep();
  wait(2000);
}




// #####################################################

void bme_initAsleep() {
#ifdef BME280
  if (! bme.begin(&Wire)) {
    while (1);
  }

  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1,  // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1,  // humidity
                  Adafruit_BME280::FILTER_OFF);
  wait(500);
#else
  if (! bme.begin()) {
    while (1);
  }
  bme.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X1,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF);
  wait(300);
  sensor.begin();
#endif
}

void readSGP() {
  checkFast++;
  //wait(10);
  sraw = sgp.measureRaw(temperatureSend, humiditySend);
  //hwSleep(30);
  //wait(10);
  vocIndex = sgp.measureVocIndex(temperatureSend, humiditySend);
  sgp.heaterOff();

  if (checkFast == fastView) {
    checkFast = 0;
    if ((vocIndex != 0) && (vocIndex >= (oldVocIndex + 30))) {
      oldVocIndex = vocIndex;
      change = true;
      ach = true;
      sleepTimeCount = 0;
    }
  }
}


void saveBaseLine() {
  baseLineSaveCount++;
  if (baseLineSaveCount == baseLineSaveClock) {
    sgp.get_States();

    saveState(171, sgp.stat1bytes[0]);
    saveState(172, sgp.stat1bytes[1]);
    saveState(173, sgp.stat1bytes[2]);
    saveState(174, sgp.stat1bytes[3]);

    saveState(175, sgp.stat2bytes[0]);
    saveState(176, sgp.stat2bytes[1]);
    saveState(177, sgp.stat2bytes[2]);
    saveState(178, sgp.stat2bytes[3]);

    baseLineSaveCount = 0;
    if (loadState(170) != 100) {
      saveState(170, 100);
    }
    icon_bl = 2;
  }
  //send(debMsg1.set(sgp.stat1));
  //send(debMsg2.set(sgp.stat2));
}


void loadBaseLine() {
  if (loadState(170) == 100) {

    //oneStateBytes[4];
    oneStateBytes[0] = loadState(171);
    oneStateBytes[1] = loadState(172);
    oneStateBytes[2] = loadState(173);
    oneStateBytes[3] = loadState(174);
    oneState = (int32_t&)oneStateBytes;

    //twoStateBytes[4];
    twoStateBytes[0] = loadState(175);
    twoStateBytes[1] = loadState(176);
    twoStateBytes[2] = loadState(177);
    twoStateBytes[3] = loadState(178);
    twoState = (int32_t&)twoStateBytes;

    sgp.set_States(oneState, twoState);
    icon_bl = 1;
    learnOK = true;
    //send(debMsg1.set(oneState));
    //send(debMsg2.set(twoState));
  } else {
    learnOK = false;
  }
}



void readSensor() {
  //hwSleep(200);
  if (sendAfterResTask == true) {
    change = true;
  }
  checkFast = 0;
  wait(5);
#ifdef BME280
  bme.takeForcedMeasurement();
  temperatureSend = bme.readTemperature();
  humiditySend = bme.readHumidity();
  pressureSend = bme.readPressure();
#else
  bme.takeForcedMeasurement();
  pressureSend = bme.readPressure();
  wait(10);
  sensors_event_t humidity, temp;
  sensor.getEvent(&humidity, &temp);
  temperatureSend = temp.temperature;
  humiditySend = humidity.relative_humidity;
#endif
  wait(10);

  readSGP();

  if (learnOK == false) {
    learningCount--;
    if (learningCount == 660 || learningCount == 600 || learningCount == 540 || learningCount == 480 || learningCount == 420 || learningCount == 360 || learningCount == 300 || learningCount == 240 || learningCount == 180 || learningCount == 120 || learningCount == 60) {
      change = true;
    }
    if (learningCount == 0) {
      learnOK = true;
      change = true;
    }
  } else {
    saveBaseLine();
  }

  if (abs(vocIndex - oldVocIndex)  >= vocIndexThreshold) {
    oldVocIndex = vocIndex;
    change = true;
    ach = true;
  }

  if (abs(temperatureSend - old_temperature) >= tempThreshold) {
    old_temperature = temperatureSend;
    change = true;
    tch = true;
  }

  temperatureInt = round(temperatureSend);
  if (temperatureInt < 0) {
    temperatureSend = 0.0;
  }
  if (temperatureInt >= 100) {
    temperatureSend = 99.9;
  }

  if (!metric) {
    temperatureSend2 = temperatureSend * 9.0 / 5.0 + 32.0;
  }

  if (abs(temperatureSend - old_temperature) >= tempThreshold) {
    old_temperature = temperatureSend;
    change = true;
    tch = true;
  }

  humidityInt = round(humiditySend);
  if (humidityInt < 10) {
    humiditySend = 10.0;
  }
  if (humidityInt >= 100) {
    humiditySend = 99.9;
  }

  if (abs(humiditySend - old_humidity) >= humThreshold) {
    old_humidity = humiditySend;
    change = true;
    hch = true;
  }

  pressureSend = pressureSend / 100.0;
  forecast = sample(pressureSend);  // Run the forecast function with a new pressure update.

#ifdef LANG_RU
  pressureSend = pressureSend * 0.75006375541921;
#endif

  if (abs(pressureSend - old_pressure) >= pressThreshold) {
    old_pressure = pressureSend;
    change = true;
    pch = true;
  }

  if (forecast != old_forecast) {
    change = true;
    fch = true;
    if ((old_forecast != 5) && (forecast == 0)) {
      forecast = old_forecast;
      change = false;
      fch = false;
    }
    if ((old_forecast != 5) && (forecast != 0)) {
      old_forecast = forecast;
    }
    if (old_forecast == 5) {
      old_forecast = forecast;
    }
  }
  BATT_COUNT++;
  CORE_DEBUG(PSTR("BATT_COUNT: %d\n"), BATT_COUNT);
  if (BATT_COUNT >= BATT_TIME) {
    CORE_DEBUG(PSTR("BATT_COUNT == BATT_TIME: %d\n"), BATT_COUNT);
    readBatt();
    BATT_COUNT = 0;
  }
  wdt_nrfReset();
}


void sendData() {

  if (flag_nogateway_mode == false) {
    transportReInitialise();
    wait(30);
    configSend();

    if (tch == true) {
      static MyMessage temperatureMsg(TEMP_CHILD_ID, V_TEMP);
      if (!metric) {
      check = send(temperatureMsg.set(temperatureSend2, 1));
      }else{
       check = send(temperatureMsg.set(temperatureSend, 1)); 
      }
      if (check == true) {
        tch = false;
      } else {
        wait(shortWait * 5);
        _transportSM.failedUplinkTransmissions = 0;
      }
      checkSend();
    }

    if (hch == true) {
      static MyMessage humidityMsg(HUM_CHILD_ID, V_HUM);
      check = send(humidityMsg.set(humiditySend, 1));
      if (check == true) {
        hch = false;
      } else {
        wait(shortWait * 5);
        _transportSM.failedUplinkTransmissions = 0;
      }
      checkSend();
    }

    if (pch == true) {
      static MyMessage pressureMsg(BARO_CHILD_ID, V_PRESSURE);
      check = send(pressureMsg.set(pressureSend, 1));
      if (check == true) {
        pch = false;
      } else {
        wait(shortWait * 5);
        _transportSM.failedUplinkTransmissions = 0;
      }
      checkSend();
    }

    if (fch == true) {
      static MyMessage forecastMsg(FORECAST_CHILD_ID, V_VAR1);
      check = send(forecastMsg.set(forecast));
      if (check == true) {
        fch = false;
      } else {
        wait(shortWait * 5);
        _transportSM.failedUplinkTransmissions = 0;
      }
      checkSend();
    }

    if (ach == true) {
      static MyMessage aqMsg(AQ_CHILD_ID, V_LEVEL);
      check = send(aqMsg.set(vocIndex));
      if (check == true) {
        ach = false;
      } else {
        wait(shortWait * 5);
        _transportSM.failedUplinkTransmissions = 0;
      }
      checkSend();
    }

    if (bch == true) {
      batLevSend();
    }

    if (vch == true) {
      voltLevSend();
    }

    if (lch == true) {
      lqSend();
    }

    if (needPresent == true) {
      presentation();
    }
    transportDisable();
    wait(30);

    checkSent();
  } else {
    tch = false;
    hch = false;
    bch = false;
    vch = false;
    pch = false;
    fch = false;
    ach = false;
  }
  wdt_nrfReset();
}


void checkSend() {
  if (check == true) {
    err_delivery_beat = 0;
    if (flag_nogateway_mode == true) {
      flag_nogateway_mode = false;
      CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
      err_delivery_beat = 0;
    }
  } else {
    _transportSM.failedUplinkTransmissions = 0;
    if (err_delivery_beat < 7) {
      err_delivery_beat++;
    }
    if (err_delivery_beat == 6) {
      if (flag_nogateway_mode == false) {
        gateway_fail();
        CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
      }
    }
  }
}

void checkSent() {
  if (tch == true || hch == true || vch == true || pch == true || fch == true || ach == true || lch == true) {
    change = true;
  }
}


void configSend() {
  wdt_nrfReset();
  static MyMessage setBattSend(SET_BATT_SEND_ID, V_VAR1);
  static MyMessage setColor(SET_COLOR_ID, V_VAR1);

  if (sendAfterResTask == true) {

    if (changeB == true) {
      check = send(setBattSend.set(battSend));
      if (check == false) {
        _transportSM.failedUplinkTransmissions = 0;
        wait(50);
        check = send(setBattSend.set(battSend));
        if (check == false) {
          _transportSM.failedUplinkTransmissions = 0;
          wait(100);
        }
      }
      if (check == true) {
        changeB = false;
      }
    }

    if (changeC == true) {
      check = send(setColor.set(colorPrint));
      if (check == false) {
        _transportSM.failedUplinkTransmissions = 0;
        wait(50);
        check = send(setColor.set(colorPrint));
        if (check == false) {
          _transportSM.failedUplinkTransmissions = 0;
          wait(100);
        }
      }
      if (check == true) {
        changeC = false;
      }
    }
    if (changeB == false || changeC == false) {
      sendAfterResTask = false;
    }
  }
}

void resetBeforePresent() {
  needPresent = true;
  mesInfo = false;
  mesTemp = false;
  mesHum = false;
  mesAq = false;
  mesBaro = false;
  mesForec = false;
  mesLink = false;
  mesVolt = false;
  mesColorSet = false;
#ifdef SEND_RESET_REASON
  mesReset = false;
#endif

  sendAfterResTask = true;
  changeB = true;
  changeC = true;

  oldVocIndex = 0;
  old_temperature = 0.0;
  old_humidity = 0.0;
  old_pressure = 0.0;
  old_battery = 0;
  old_batteryVoltage = 0;
  old_nRFRSSI = 127;
}


#ifdef SEND_RESET_REASON
void sendResetReason() {

  static MyMessage sendReset(RESET_REASON_ID, V_VAR1);

  String reason;
#ifdef MY_RESET_REASON_TEXT
  if (NRF_POWER->RESETREAS == 0) {
    reason = "POWER_ON";
  } else {
    if (NRF_POWER->RESETREAS & (1UL << 0)) reason += "PIN_RESET ";
    if (NRF_POWER->RESETREAS & (1UL << 1)) reason += "WDT ";
    if (NRF_POWER->RESETREAS & (1UL << 2)) reason += "SOFT_RESET ";
    if (NRF_POWER->RESETREAS & (1UL << 3)) reason += "LOCKUP";
    if (NRF_POWER->RESETREAS & (1UL << 16)) reason += "WAKEUP_GPIO ";
    if (NRF_POWER->RESETREAS & (1UL << 17)) reason += "LPCOMP ";
    if (NRF_POWER->RESETREAS & (1UL << 17)) reason += "WAKEUP_DEBUG";
  }
#else
  reason = NRF_POWER->RESETREAS;
#endif

  check = send(sendReset.set(reason.c_str()));
  if (check == false) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(shortWait * 10);
    check = send(sendReset.set(reason.c_str()));
    wait(shortWait * 10);
    if (check == false) {
      _transportSM.failedUplinkTransmissions = 0;
    }
  }
  NRF_POWER->RESETREAS = (0xFFFFFFFF);
}
#endif


//########################################## SET ###################################################
void timeConf() {

  sleepTime = ((timeSend * minuteT) / sleepTimeWDT);

  BATT_TIME = ((battSend * 60) / timeSend);

  cpNom = (120 / timeSend);

  baseLineSaveClock = (60 * 4);

  learningCount = (60 * (uint16_t)VocAlgorithm_TAU_MEAN_VARIANCE_HOURS);

  CORE_DEBUG(PSTR("sleepTime: %d\n"), sleepTime);
}


static __INLINE void wdt_init(void)
{
  NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
  NRF_WDT->CRV = 35 * 32768;
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk;
  NRF_WDT->TASKS_START = 1;
}


static __INLINE void wdt_nrfReset() {
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}


// ##############################################################################################################
// #                                                    BATTARY                                                 #
// ##############################################################################################################
void readBatt() {
  batteryVoltage = hwCPUVoltage();
  battery = battery_level_in_percent(batteryVoltage);
  batteryVoltageF = (float)batteryVoltage / 1000.00;
  CORE_DEBUG(PSTR("battery voltage: %d\n"), batteryVoltage);
  CORE_DEBUG(PSTR("battery percentage: %d\n"), battery);

  if (battery > 100) {
    battery = 100;
  }

  if (battery != old_battery) {
    change = true;
    bch = true;
    old_battery =  battery;
  }

  if (batteryVoltage != old_batteryVoltage) {
    change = true;
    vch = true;
    old_batteryVoltage =  batteryVoltage;
  }
}


void batLevSend() {

  static MyMessage bvMsg(BATTERY_VOLTAGE_ID, V_VAR1);
  wdt_nrfReset();
  sendBatteryLevel(battery, 1);
  wait(500, C_INTERNAL, I_BATTERY_LEVEL);
  bch = false;
  lqSend();
}


static __INLINE uint8_t battery_level_in_percent(const uint16_t mvolts)
{
  uint8_t battery_level;

  if (mvolts >= 3000)
  {
    battery_level = 100;
  }
  else if (mvolts > 2900)
  {
    battery_level = 100 - ((3000 - mvolts) * 58) / 100;
  }
  else if (mvolts > 2740)
  {
    battery_level = 42 - ((2900 - mvolts) * 24) / 160;
  }
  else if (mvolts > 2440)
  {
    battery_level = 18 - ((2740 - mvolts) * 12) / 300;
  }
  else if (mvolts > 2100)
  {
    battery_level = 6 - ((2440 - mvolts) * 6) / 340;
  }
  else
  {
    battery_level = 0;
  }
  return battery_level;
}


void lqSend() {
  if (lch == true) {
    check = send(sqMsg.set(nRFRSSI));
    if (check == true) {
      lch = false;
    } else {
      wait(shortWait * 5);
      _transportSM.failedUplinkTransmissions = 0;
    }
  } else {
    nRFRSSI = transportGetReceivingRSSI();
    nRFRSSI = map(nRFRSSI, -85, -40, 0, 100);
    if (nRFRSSI < 0) {
      nRFRSSI = 0;
    }
    if (nRFRSSI > 100) {
      nRFRSSI = 100;
    }
    /*
        if ((nRFRSSI >= 90) && (NRF_RADIO->TXPOWER == 0x8UL)) {
          NRF_RADIO->TXPOWER = 0x4UL;
        } else if ((nRFRSSI <= 25) && (NRF_RADIO->TXPOWER == 0x4UL))  {
          NRF_RADIO->TXPOWER = 0x8UL;
        }
    */
    if (nRFRSSI != old_nRFRSSI) {
      lch = true;
      check = send(sqMsg.set(nRFRSSI));
      if (check == true) {
        lch = false;
      } else {
        wait(shortWait * 5);
        _transportSM.failedUplinkTransmissions = 0;
      }
      old_nRFRSSI = nRFRSSI;
    }
  }
}


void voltLevSend() {
  check = send(bvMsg.set(batteryVoltageF, 2));
  if (!check) {
    vch = true;
    wait(shortWait * 5);
    _transportSM.failedUplinkTransmissions = 0;
  } else {
    vch = false;
    CORE_DEBUG(PSTR("MyS: SEND BATTERY VOLTAGE\n"));
  }
}


// ##############################################################################################################
// #                                                      RECEIVE                                               #
// ##############################################################################################################
void receive(const MyMessage & message)
{
  if (message.sensor == SET_BATT_SEND_ID) {
    if (message.type == V_VAR1) {
      battSend = message.getByte();
      if (battSend > 24) {
        battSend = 24;
      }
      if (battSend < 1) {
        battSend = 1;
      }
      saveState(103, battSend);
      wait(5);
      transportDisable(); // вроде потому что один фиг сразу в сон? ....не все таки раскоментить потому что сон не сразу а сначала обновление экрана
      reportBattInk();
      configMode = false;
      change = true;
      sendAfterResTask = true;
      changeB = true;
      timeConf();
      sleepTimeCount = sleepTime;
    }
  }

  if (message.sensor == SET_COLOR_ID) {
    if (message.type == V_VAR1) {
      bool colorPrintTemp = message.getBool();
      colorChange(colorPrintTemp);
      transportDisable();
      configMode = false;
      change = true;
      sendAfterResTask = true;
      changeC = true;
      sleepTimeCount = sleepTime;
    }
  }
}


// ##############################################################################################################
// #                                                    INTERRUPTS                                              #
// ##############################################################################################################
void interrupt_Init() {
  //***
  //SET
  //NRF_GPIO_PIN_NOPULL
  //NRF_GPIO_PIN_PULLUP
  //NRF_GPIO_PIN_PULLDOWN
  //***
  nrf_gpio_cfg_input(PIN_BUTTON, NRF_GPIO_PIN_NOPULL);
  APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
  PIN_BUTTON_MASK = 1 << PIN_BUTTON;
  app_gpiote_user_register(&m_gpiote_user_id, PIN_BUTTON_MASK, PIN_BUTTON_MASK, gpiote_event_handler);
  app_gpiote_user_enable(m_gpiote_user_id);
  buttIntStatus = 0;
}


void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
  MY_HW_RTC->CC[0] = (MY_HW_RTC->COUNTER + 2);

  if (PIN_BUTTON_MASK & event_pins_high_to_low) {
    if (buttIntStatus == 0) {
      buttIntStatus = PIN_BUTTON;
    }
  }
}


// ##############################################################################################################
// #                                                      FRESET                                                #
// ##############################################################################################################
void new_device() {
  hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255);  // delete network id in mysensors routing table
  saveState(200, 255);  // delete network id in user area
  saveState(170, 255); // delete sgp40 base line
  hwReboot();
}



// ####################################################################################################
// #                                                                                                  #
// #                                            HAPPY MODE                                            #
// #                                                                                                  #
// ####################################################################################################

void happy_init() {
  //hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255); // ******************** checking the node config reset *************************

  if (hwReadConfig(EEPROM_NODE_ID_ADDRESS) == 0) {
    hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255);
  }
  if (loadState(200) == 0) {
    saveState(200, 255);
  }
  CORE_DEBUG(PSTR("EEPROM NODE ID: %d\n"), hwReadConfig(EEPROM_NODE_ID_ADDRESS));
  CORE_DEBUG(PSTR("USER MEMORY SECTOR NODE ID: %d\n"), loadState(200));

  if (hwReadConfig(EEPROM_NODE_ID_ADDRESS) == 255) {
    mtwr = 26000;
    needPresent = true;
  } else {
    mtwr = 8000;
    no_present();
  }
  CORE_DEBUG(PSTR("MY_TRANSPORT_WAIT_MS: %d\n"), mtwr);
}


void config_Happy_node() {
  if (mtwr == 26000) {
    myid = getNodeId();
    saveState(200, myid);
    if (isTransportReady() == true) {
      mypar = _transportConfig.parentNodeId;
      old_mypar = mypar;
      saveState(201, mypar);
      saveState(202, _transportConfig.distanceGW);
    }
    if (isTransportReady() == false)
    {
      no_present();
      err_delivery_beat = 7;
      _transportConfig.nodeId = myid;
      _transportConfig.parentNodeId = loadState(201);
      _transportConfig.distanceGW = loadState(202);
      mypar = _transportConfig.parentNodeId;
      happy_node_mode();
      gateway_fail();
    }
  }
  if (mtwr != 26000) {
    myid = getNodeId();
    if (myid != loadState(200)) {
      saveState(200, myid);
    }
    if (isTransportReady() == true) {
      mypar = _transportConfig.parentNodeId;
      if (mypar != loadState(201)) {
        saveState(201, mypar);
      }
      if (_transportConfig.distanceGW != loadState(202)) {
        saveState(202, _transportConfig.distanceGW);
      }
      present_only_parent();
    }
    if (isTransportReady() == false)
    {
      no_present();
      err_delivery_beat = 7;
      _transportConfig.nodeId = myid;
      _transportConfig.parentNodeId = loadState(201);
      _transportConfig.distanceGW = loadState(202);
      mypar = _transportConfig.parentNodeId;
      happy_node_mode();
      gateway_fail();
    }
  }
}


void check_parent() {
  _transportSM.findingParentNode = true;
  CORE_DEBUG(PSTR("MyS: SEND FIND PARENT REQUEST, WAIT RESPONSE\n"));
  _sendRoute(build(_msg, 255, NODE_SENSOR_ID, C_INTERNAL, 7).set(""));
  wait(1000, C_INTERNAL, 8);
  if (_msg.sensor == 255) {
    if (mGetCommand(_msg) == C_INTERNAL) {
      if (_msg.type == 8) {
        Ack_FP = true;
        CORE_DEBUG(PSTR("MyS: PARENT RESPONSE FOUND\n"));
      }
    }
  }
  if (Ack_FP == true) {
    CORE_DEBUG(PSTR("MyS: FIND PARENT PROCESS\n"));
    Ack_FP = false;
    transportSwitchSM(stParent);
    flag_nogateway_mode = false;
    flag_find_parent_process = true;
  } else {
    _transportSM.findingParentNode = false;
    CORE_DEBUG(PSTR("MyS: PARENT RESPONSE NOT FOUND\n"));
    _transportSM.failedUplinkTransmissions = 0;
    CORE_DEBUG(PSTR("TRANSPORT: %d\n"), isTransportReady());
    transportDisable();
    change = true;
  }
}


void find_parent_process() {
  flag_update_transport_param = true;
  flag_find_parent_process = false;
  CORE_DEBUG(PSTR("MyS: STANDART TRANSPORT MODE IS RESTORED\n"));
  err_delivery_beat = 0;
}


void gateway_fail() {
  flag_nogateway_mode = true;
  flag_update_transport_param = false;
  change = true;
}


void happy_node_mode() {
  _transportSM.findingParentNode = false;
  _transportSM.transportActive = true;
  _transportSM.uplinkOk = true;
  _transportSM.pingActive = false;
  _transportSM.failureCounter = 0u;
  _transportSM.uplinkOk = true;
  _transportSM.failureCounter = 0u;
  _transportSM.failedUplinkTransmissions = 0u;
  transportSwitchSM(stReady);
  CORE_DEBUG(PSTR("TRANSPORT: %d\n"), isTransportReady());
}


void present_only_parent() {
  if (old_mypar != mypar) {
    CORE_DEBUG(PSTR("MyS: SEND LITTLE PRESENT:) WITH PARENT ID\n"));
    if (_sendRoute(build(_msgTmp, 0, NODE_SENSOR_ID, C_INTERNAL, 6).set(mypar))) {
      flag_sendRoute_parent = false;
      old_mypar = mypar;
    } else {
      flag_sendRoute_parent = true;
    }
  }
}


void update_Happy_transport() {
  CORE_DEBUG(PSTR("MyS: UPDATE TRANSPORT CONFIGURATION\n"));
  mypar = _transportConfig.parentNodeId;
  if (mypar != loadState(201))
  {
    saveState(201, mypar);
  }
  if (_transportConfig.distanceGW != loadState(202))
  {
    saveState(202, _transportConfig.distanceGW);
  }
  present_only_parent();
  wait(shortWait * 5);
  flag_update_transport_param = false;
  sleepTimeCount = sleepTime;
  BATT_COUNT = BATT_TIME;
  change = true;
}


void no_present() {
  _coreConfig.presentationSent = true;
  _coreConfig.nodeRegistered = true;
}


// ####################################################################################################
// #                                                                                                  #
// #            These functions are only included if the forecast function is enables.                #
// #          The are used to generate a weater prediction by checking if the barometric              #
// #                          pressure is rising or falling over time.                                #
// #                                                                                                  #
// ####################################################################################################

float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++) {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }
  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

  return lastPressureSamplesAverage;
}

// Forecast algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure) {
  // Calculate the average of the last n minutes.
  int index = minuteCount % LAST_SAMPLES_COUNT;
  lastPressureSamples[index] = pressure;

  minuteCount++;
  if (minuteCount > 185) {
    minuteCount = 6;
  }

  if (minuteCount == 5) {
    pressureAvg = getLastPressureSamplesAverage();
  }
  else if (minuteCount == 35) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change * 2; // note this is for t = 0.5hour
    }
    else {
      dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
    }
  }
  else if (minuteCount == 65) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { //first time initial 3 hour
      dP_dt = change; //note this is for t = 1 hour
    }
    else {
      dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 95) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 1.5; // note this is for t = 1.5 hour
    }
    else {
      dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 125) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    pressureAvg2 = lastPressureAvg; // store for later use.
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 2; // note this is for t = 2 hour
    }
    else {
      dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 155) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 2.5; // note this is for t = 2.5 hour
    }
    else {
      dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 185) {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) { // first time initial 3 hour
      dP_dt = change / 3; // note this is for t = 3 hour
    }
    else {
      dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
    }
    pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
    firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  }

  int16_t forecast = UNKNOWN;
  if (minuteCount < 35 && firstRound) { //if time is less than 35 min on the first 3 hour interval.
    forecast = UNKNOWN;
  }
  else if (dP_dt < (-0.25)) {
    forecast = THUNDERSTORM;
  }
  else if (dP_dt > 0.25) {
    forecast = UNSTABLE;
  }
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05))) {
    forecast = CLOUDY;
  }
  else if ((dP_dt > 0.05) && (dP_dt < 0.25))
  {
    forecast = SUNNY;
  }
  else if ((dP_dt > (-0.05)) && (dP_dt < 0.05)) {
    forecast = STABLE;
  }
  else {
    forecast = UNKNOWN;
  }
  return forecast;
}

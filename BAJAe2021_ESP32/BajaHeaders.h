//本文件为ino顶部的引用和初始化部分。

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x)       \
    Serial.print(F(#x ":")); \
    Serial.print(x);         \
    Serial.print(F("\t"));
#else
#define DEBUG_PRINT(x)
#endif
#ifdef DEBUG
#define DEBUG_PRINTLN(x)     \
    Serial.print(F(#x ":")); \
    Serial.print(x);         \
    Serial.println("");
#else
#define DEBUG_PRINTLN(x)
#endif

#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>

// BNO055姿态 ====================================
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
sensors_event_t linearAccelData; //BNO event
bool BNO055isOK = true;

// GPS ====================================
#include <TinyGPS++.h> // Tiny GPS Plus Library

// I2C ====================================
#define I2C_SDA 13
#define I2C_SCL 4
//BNO055 address: 41 (0x29)

// WIFIOTA ====================================
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
const char *ssid = "TiWifi";
const char *password = "hexiaoqi";

// LED灯条 =====================================
#include <FastLED.h>
#define LED_DATA_PIN 26
#define LED_CLOCK_PIN 27
#define NUM_LEDS 10
CRGB leds[NUM_LEDS]; // 定义LED阵列
#define RPM_Display_MIN 1500
#define RPM_Display_MAX 3750
#define SPD_Display_MIN 3
#define SPD_Display_MAX 60
int nShiftlightPos = 0;
int RPM_Green_Before = 3;
int RPM_Red_After = 8;

// 定义任务 ====================================
void PrintToOLED(void *pvParameters);
// void getBNO055Data(void *pvParameters);
void getGPSData(void *pvParameters);
void handleOTAtask(void *pvParameters);
void updateRevving(void *pvParameters);
void UpdateLEDstrip(void *pvParameters);
void updateSPDdata(void *pvParameters);  // 测速任务
void writeToSDCard(void *pvParameters);  // SD卡写入任务
void updateDateTime(void *pvParameters); // SD卡写入任务

// OLED屏幕 ====================================
U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/5, /* dc=*/12, /* reset=*/14); // Enable U8G2_16BIT in u8g2.h

// GPS ====================================
#define Serial1_RXPIN 21           //to GPS TX
#define Serial1_TXPIN 22           //to GPS RX
const float Home_LAT = 33.205288;  // Your Home Latitude
const float Home_LNG = -96.951125; // Your Home Longitude
int incomingByte;
TinyGPSPlus gps; // 创建一个名为gps的TinyGPS++对象的实例。

static void printFloat(float val, bool valid, int len, int prec)
{
    if (!valid)
    {
        while (len-- > 1)
            Serial.print('*');
        Serial.print(' ');
    }
    else
    {
        Serial.print(val, prec);
        int vi = abs((int)val);
        int flen = prec + (val < 0.0 ? 2 : 1); // . and -
        flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                             : vi >= 10    ? 2
                                           : 1;
        for (int i = flen; i < len; ++i)
            Serial.print(' ');
    }
    vTaskDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
    char sz[32] = "*****************";
    if (valid)
        sprintf(sz, "%ld", val);
    sz[len] = 0;
    for (int i = strlen(sz); i < len; ++i)
        sz[i] = ' ';
    if (len > 0)
        sz[len - 1] = ' ';
    Serial.print(sz);
    vTaskDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
    if (!d.isValid())
    {
        Serial.print(F("********** "));
    }
    else
    {
        char sz[32];
        sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
        Serial.print(sz);
    }

    if (!t.isValid())
    {
        Serial.print(F("******** "));
    }
    else
    {
        char sz[32];
        sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
        Serial.print(sz);
    }

    printInt(d.age(), d.isValid(), 5);
    vTaskDelay(0);
}

static void printStr(const char *str, int len)
{
    int slen = strlen(str);
    for (int i = 0; i < len; ++i)
        Serial.print(i < slen ? str[i] : ' ');
    vTaskDelay(0);
}

// TIME ====================================

// // Date and time functions using a DS3231 RTC connected via I2C and Wire lib
// #include <ErriezDS3231.h>

// // Create RTC object
// ErriezDS3231 rtc;

// #define DATE_STRING_SHORT           3

// // Month names in flash
// const char monthNames_P[] PROGMEM = "JanFebMarAprMayJunJulAugSepOctNovDec";

// // Day of the week names in flash
// const char dayNames_P[] PROGMEM= "SunMonTueWedThuFriSat";

//BTRY ====================================
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
// Click here to get the library: http://librarymanager/All#SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library

// SFE_MAX1704X lipo; // Defaults to the MAX17043
SFE_MAX1704X lipo(MAX1704X_MAX17043); // Create a MAX17043

double voltage = 0; // Variable to keep track of LiPo voltage
double soc = 0;     // Variable to keep track of LiPo state-of-charge (SOC)
bool alert;         // Variable to keep track of whether alert has been triggered

// 函数的头文件
long map(long x, long in_min, long in_max, long out_min, long out_max);
int intMapping(int x, int in_min, int in_max, int out_min, int out_max);
float floatMapping(float x, float in_min, float in_max, float out_min, float out_max);

// LORA ====================================
#define Serial2_RXPIN 16 //to LORA TX
#define Serial2_TXPIN 17 //to LORA RX

// CLI ====================================
// Inlcude Library
#include <SimpleCLI.h>

// Create CLI Object
SimpleCLI cli;

// Commands
Command updatetime;

// Callback function for cowsay command
void update_time_to_given_time(cmd *c)
{
}

// Callback in case of an error
void errorCallback(cmd_error *e)
{
    CommandError cmdError(e); // Create wrapper object

    Serial2.print("ERROR: ");
    Serial2.println(cmdError.toString());

    if (cmdError.hasCommand())
    {
        Serial2.print("Did you mean \"");
        Serial2.print(cmdError.getCommand().toString());
        Serial2.println("\"?");
    }
}

// 频率计1
int lastmSec = 0;

int lastPulseCounter_SPD = 0;
float SPD_freq_in_mHz = 0.0;
int SPD_Calc_Factor = 10533132; //频率换算系数，计算方法为如下

int lastPulseCounter_RPM = 0;
float RPM_freq_in_mHz = 0.0;
int RPM_Calc_Factor = 60000; //频率换算系数，计算方法为如下

/*
速度：
前面的单位是毫Hz，需要再乘1000才是Hz
车轮周长 175.5522 cm
0.1755522m

每圈60个脉冲
每秒 1000 mhz*0.1755522m/60
每小时 1000*3600m*hz*0.1755522/60 m

mhz*60*175.5522*1000 km/h = 10533132

*/

#include "driver/pcnt.h" // ESP32 library for pulse count
// e.g. stored in following path C:\Users\User\Documents\Arduino\hardware\arduino-esp32-master\tools\sdk\include\driver\driver\pcnt.h
// when in the Arduino IDE properties the sketchbook storage location is set to C:\Users\User\Documents\Arduino

#define PCNT_FREQ_UNIT_SPD PCNT_UNIT_0 // select ESP32 pulse counter unit 0 (out of 0 to 7 indipendent counting units)
#define PCNT_FREQ_UNIT_RPM PCNT_UNIT_1 // select ESP32 pulse counter unit 0 (out of 0 to 7 indipendent counting units)

// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html

int SPD_INPUT_PIN = 33; //
int RPM_INPUT_PIN = 32; //

int16_t PulseCounter_SPD = 0;    // pulse counter, max. value is 65536
int OverflowCounter_SPD = 0;     // pulse counter overflow counter
int16_t PulseCounter_RPM = 0;    // pulse counter, max. value is 65536
int OverflowCounter_RPM = 0;     // pulse counter overflow counter
int PCNT_H_LIM_VAL = 10000;      // upper limit of counting  max. 32767, write +1 to overflow counter, when reached
uint16_t PCNT_FILTER_VAL = 1000; // filter (damping, inertia) value for avoiding glitches in the count, max. 1023

pcnt_isr_handle_t user_isr_handle = NULL; // interrupt handler - not used

void IRAM_ATTR CounterOverflow_SPD(void *arg)
{                                                  // Interrupt for overflow of pulse counter
    OverflowCounter_SPD = OverflowCounter_SPD + 1; // increase overflow counter
    PCNT.int_clr.val = BIT(PCNT_FREQ_UNIT_SPD);    // clean overflow flag
    pcnt_counter_clear(PCNT_FREQ_UNIT_SPD);        // zero and reset of pulse counter unit
}

void IRAM_ATTR CounterOverflow_RPM(void *arg)
{                                                  // Interrupt for overflow of pulse counter
    OverflowCounter_RPM = OverflowCounter_RPM + 1; // increase overflow counter
    PCNT.int_clr.val = BIT(PCNT_FREQ_UNIT_RPM);    // clean overflow flag
    pcnt_counter_clear(PCNT_FREQ_UNIT_RPM);        // zero and reset of pulse counter unit
}

void initPulseCounter_SPD()
{                                                      // initialise pulse counter
    pcnt_config_t pcntFreqConfig_SPD = {};             // Instance of pulse counter
    pcntFreqConfig_SPD.pulse_gpio_num = SPD_INPUT_PIN; // pin assignment for pulse counter = GPIO 15
    pcntFreqConfig_SPD.pos_mode = PCNT_COUNT_INC;      // count rising edges (=change from low to high logical level) as pulses
    pcntFreqConfig_SPD.counter_h_lim = PCNT_H_LIM_VAL; // set upper limit of counting
    pcntFreqConfig_SPD.unit = PCNT_FREQ_UNIT_SPD;      // select ESP32 pulse counter unit 0
    pcntFreqConfig_SPD.channel = PCNT_CHANNEL_0;       // select channel 0 of pulse counter unit 0
    pcnt_unit_config(&pcntFreqConfig_SPD);             // configur rigisters of the pulse counter

    pcnt_counter_pause(PCNT_FREQ_UNIT_SPD); // pause puls counter unit
    pcnt_counter_clear(PCNT_FREQ_UNIT_SPD); // zero and reset of pulse counter unit

    pcnt_event_enable(PCNT_FREQ_UNIT_SPD, PCNT_EVT_H_LIM);             // enable event for interrupt on reaching upper limit of counting
    pcnt_isr_register(CounterOverflow_SPD, NULL, 0, &user_isr_handle); // configure register overflow interrupt handler
    pcnt_intr_enable(PCNT_FREQ_UNIT_SPD);                              // enable overflow interrupt

    pcnt_set_filter_value(PCNT_FREQ_UNIT_SPD, PCNT_FILTER_VAL); // set damping, inertia
    pcnt_filter_enable(PCNT_FREQ_UNIT_SPD);                     // enable counter glitch filter (damping)

    pcnt_counter_resume(PCNT_FREQ_UNIT_SPD); // resume counting on pulse counter unit
}

void initPulseCounter_RPM()
{                                                      // initialise pulse counter
    pcnt_config_t pcntFreqConfig_RPM = {};             // Instance of pulse counter
    pcntFreqConfig_RPM.pulse_gpio_num = RPM_INPUT_PIN; // pin assignment for pulse counter = GPIO 15
    pcntFreqConfig_RPM.pos_mode = PCNT_COUNT_INC;      // count rising edges (=change from low to high logical level) as pulses
    pcntFreqConfig_RPM.counter_h_lim = PCNT_H_LIM_VAL; // set upper limit of counting
    pcntFreqConfig_RPM.unit = PCNT_FREQ_UNIT_RPM;      // select ESP32 pulse counter unit 0
    pcntFreqConfig_RPM.channel = PCNT_CHANNEL_1;       // select channel 0 of pulse counter unit 0
    pcnt_unit_config(&pcntFreqConfig_RPM);             // configur rigisters of the pulse counter

    pcnt_counter_pause(PCNT_FREQ_UNIT_RPM); // pause puls counter unit
    pcnt_counter_clear(PCNT_FREQ_UNIT_RPM); // zero and reset of pulse counter unit

    pcnt_event_enable(PCNT_FREQ_UNIT_RPM, PCNT_EVT_H_LIM);             // enable event for interrupt on reaching upper limit of counting
    pcnt_isr_register(CounterOverflow_RPM, NULL, 0, &user_isr_handle); // configure register overflow interrupt handler
    pcnt_intr_enable(PCNT_FREQ_UNIT_RPM);                              // enable overflow interrupt

    pcnt_set_filter_value(PCNT_FREQ_UNIT_RPM, PCNT_FILTER_VAL); // set damping, inertia
    pcnt_filter_enable(PCNT_FREQ_UNIT_RPM);                     // enable counter glitch filter (damping)

    pcnt_counter_resume(PCNT_FREQ_UNIT_RPM); // resume counting on pulse counter unit
}

void Read_Reset_PCNT_SPD()
{                                                                  // function for reading pulse counter (for timer)
    pcnt_get_counter_value(PCNT_FREQ_UNIT_SPD, &PulseCounter_SPD); // get pulse counter value - maximum value is 16 bits

    // resetting counter as if example, delet for application in PiedPiperS
    OverflowCounter_SPD = 0;                // set overflow counter to zero
    pcnt_counter_clear(PCNT_FREQ_UNIT_SPD); // zero and reset of pulse counter unit
}

void Read_Reset_PCNT_RPM()
{                                                                  // function for reading pulse counter (for timer)
    pcnt_get_counter_value(PCNT_FREQ_UNIT_RPM, &PulseCounter_RPM); // get pulse counter value - maximum value is 16 bits

    // resetting counter as if example, delet for application in PiedPiperS
    OverflowCounter_RPM = 0;                // set overflow counter to zero
    pcnt_counter_clear(PCNT_FREQ_UNIT_RPM); // zero and reset of pulse counter unit
}
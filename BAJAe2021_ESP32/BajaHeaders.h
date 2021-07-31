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
#define DEBUG_PRINTLN(x) \
    DEBUG_PRINT(x);      \
    Serial.print("\n");
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
void updateSPDdata(void *pvParameters); // 测速任务
void writeToSDCard(void *pvParameters); // SD卡写入任务

// OLED屏幕 ====================================
U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/5, /* dc=*/12, /* reset=*/14); // Enable U8G2_16BIT in u8g2.h

// GPS ====================================
#define Serial2_RXPIN 16           //to GPS TX
#define Serial2_TXPIN 17           //to GPS RX
const float Home_LAT = 33.205288;  // Your Home Latitude
const float Home_LNG = -96.951125; // Your Home Longitude
int incomingByte;
TinyGPSPlus gps; // 创建一个名为gps的TinyGPS++对象的实例。

// TIME ====================================
#include <DS3231.h>

DS3231 Clock;

byte Year;
byte Month;
byte Date;
byte DoW;
byte Hour;
byte Minute;
byte Second;
void GetDateStuff(byte &Year, byte &Month, byte &Day, byte &DoW,
                  byte &Hour, byte &Minute, byte &Second)
{
    // Call this if you notice something coming in on
    // the serial port. The stuff coming in should be in
    // the order YYMMDDwHHMMSS, with an 'x' at the end.
    boolean GotString = false;
    char InChar;
    byte Temp1, Temp2;
    char InString[20];

    byte j = 0;
    while (!GotString)
    {
        if (Serial.available())
        {
            InChar = Serial.read();
            InString[j] = InChar;
            j += 1;
            if (InChar == 'x')
            {
                GotString = true;
            }
        }
    }
    Serial.println(InString);
    // Read Year first
    Temp1 = (byte)InString[0] - 48;
    Temp2 = (byte)InString[1] - 48;
    Year = Temp1 * 10 + Temp2;
    // now month
    Temp1 = (byte)InString[2] - 48;
    Temp2 = (byte)InString[3] - 48;
    Month = Temp1 * 10 + Temp2;
    // now date
    Temp1 = (byte)InString[4] - 48;
    Temp2 = (byte)InString[5] - 48;
    Day = Temp1 * 10 + Temp2;
    // now Day of Week
    DoW = (byte)InString[6] - 48;
    // now Hour
    Temp1 = (byte)InString[7] - 48;
    Temp2 = (byte)InString[8] - 48;
    Hour = Temp1 * 10 + Temp2;
    // now Minute
    Temp1 = (byte)InString[9] - 48;
    Temp2 = (byte)InString[10] - 48;
    Minute = Temp1 * 10 + Temp2;
    // now Second
    Temp1 = (byte)InString[11] - 48;
    Temp2 = (byte)InString[12] - 48;
    Second = Temp1 * 10 + Temp2;
}

// 函数的头文件
long map(long x, long in_min, long in_max, long out_min, long out_max);
int intMapping(int x, int in_min, int in_max, int out_min, int out_max);
float floatMapping(float x, float in_min, float in_max, float out_min, float out_max);

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

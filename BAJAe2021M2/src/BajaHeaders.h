// TODO:
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

#define TELL_EVERYONE_LN(x) \
    Serial.println(x);      \
    Serial2.println(x);
//futureSDcardhere

#define TELL_EVERYONE(x) \
    Serial.print(x);     \
    Serial2.print(x);
//futureSDcardhere

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

// GPS ====================================
#include <TinyGPSPlus.h> // Tiny GPS Plus Library

// I2C ====================================
#define I2C_SDA 33
#define I2C_SCL 4
//BNO055 address: 41 (0x29)

// WIFIOTA ====================================
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
const char *ssid = "TiWifi";
const char *password = "hexiaoqi";
const char *ntpServer = "cn.ntp.org.cn";
const long gmtOffset_sec = 28800; //Replace with your GMT offset (seconds) 8 * 60 * 60

// LED灯条 =====================================
#include <Adafruit_MCP23X17.h> //https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library/
Adafruit_MCP23X17 mcp;
#define RPM_Display_MIN 1500
#define RPM_Display_MAX 3750
#define SPD_Display_MIN 3
#define SPD_Display_MAX 60
int nShiftlightPos = 0;

// OLED屏幕 ====================================
U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/5, /* dc=*/25, /* reset=*/26); // Enable U8G2_16BIT in u8g2.h

// OLED log ====================================
U8G2LOG u8g2log; // Create a U8x8log object
// Define the dimension of the U8x8log window
#define U8LOG_WIDTH 32
#define U8LOG_HEIGHT 8
uint8_t u8log_buffer[U8LOG_WIDTH * U8LOG_HEIGHT]; // Allocate static memory for the U8x8log window

// GPS ====================================
#define Serial1_RXPIN 21           //to GPS TX
#define Serial1_TXPIN 22           //to GPS RX
const float Home_LAT = 33.205288;  // Your Home Latitude
const float Home_LNG = -96.951125; // Your Home Longitude
int incomingByte;
TinyGPSPlus gps; // 创建一个名为gps的TinyGPS++对象的实例。

// sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
// sprintf(sz,"%02d/%02d/%02d ", d.month(), d.day(), d.year());

// TIME ====================================
#include "time.h"
#include <ErriezDS3231.h> //https://github.com/Erriez/ErriezDS3231
struct tm time_in_RAM;    //time in ESP32 RAM
ErriezDS3231 rtc;

//BTRY ====================================
float BTRYvoltage = 0;  //电池电压
int BTRYpercentage = 0; //电池剩余电量，0~100

// LORA ====================================
#define Serial2_RXPIN 16 //to LORA TX
#define Serial2_TXPIN 17 //to LORA RX

// CLI ====================================
#include <SimpleCLI.h> // Inlcude Library
SimpleCLI cli;         // Create CLI Object
void turnoffwifi()     // Callback function for cowsay command
{
    WiFi.mode(WIFI_OFF);
}
void turnonwifi() // Callback function for cowsay command
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
}

void errorCallback(cmd_error *e) // Callback in case of an error
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

//测速 RPM SPD====================================
//https://github.com/madhephaestus/ESP32Encoder

#include <ESP32Encoder.h>
ESP32Encoder encoder_speed;
ESP32Encoder encoder_rpm;

// Stats 状态变量 ====================================
bool setLEDtoSpeed = false; // 1:SPEED 0:RPM
bool enableFakeData = true; //开启假数据

unsigned int fpsOLED = 0;
long lastOLEDrefreshTime = 0;
int fpsGPS = 0;

bool wifi_connected = false;
bool isOTAing = false;
unsigned int OTAprogress = 0;
unsigned int OTAtotal = 0;

bool BNO055isOK = true;
bool DS3231isOK = true;

bool wifiNeverConnected = true;
bool GPSNeverConnected = true;
bool timeSyncedFromNTP = false;
bool timeSyncedFromRTC = false;
bool timeSyncedFromGPS = false;

bool I2C_is_Busy = false;

//行车数据变量=============================
unsigned int RPM = 0;
unsigned int SPD = 0; //千米每小时

float GFx = 0;
float GFy = 0;
int GFx_OLED = 0;
int GFy_OLED = 0;
float GFx_OLED_ZoomLevel = 1.5;

float gps_hdop = 0.0;
float gps_speed = 0.0;
int gps_sat_count = 0;

void bootUpPrint(String textHere) //开机输出记录
{
    Serial.println(textHere);
    u8g2log.print(textHere);
    u8g2log.print("\n");
}
void bootUpPrintWithLora(String textHere) //开机输出记录，带无线输出
{
    Serial.println(textHere);
    Serial2.println(textHere);
    u8g2log.print(textHere);
    u8g2log.print("\n");
}

void printRTCtime() //输出RTC芯片的时间
{
    struct tm TimeInRTC;
    if (rtc.read(&TimeInRTC))
    {
        TELL_EVERYONE("RTC time:")
        TELL_EVERYONE_LN(asctime(&TimeInRTC))
    }
    else
    {
        TELL_EVERYONE_LN("RTC error")
    }
}
void printRAMtime() //输出内存的时间
{
    TELL_EVERYONE_LN("RAM Time:")
    // TELL_EVERYONE_LN(&time_in_RAM, "%F %T")
    // 现在这里还没搞好
}

void RTCtoRAM() //读取RTC的时间到内存
{
    printRTCtime();
    printRAMtime();
    I2C_is_Busy = true;
    struct tm TimeInRTC;
    if (rtc.read(&TimeInRTC))
    {
        time_t t = mktime(&TimeInRTC);
        struct timeval now = {.tv_sec = t};
        settimeofday(&now, NULL);
        timeSyncedFromRTC = true;
        printRTCtime();
        printRAMtime();
    }
    else
    {
        TELL_EVERYONE_LN("RTC error,RTCtoRAM FAIL")
    }
    I2C_is_Busy = false;
}
void RAMtoRTC() //写入时间至RTC，未测试
{
    if (DS3231isOK)
    {
        printRAMtime();
        printRTCtime();
        I2C_is_Busy = true;
        TELL_EVERYONE_LN("RAMtoRTC");
        time_t nowEpoch;
        time(&nowEpoch);
        rtc.setEpoch(nowEpoch - 8 * 3600); //我们时区在+8区
        I2C_is_Busy = false;
        printRAMtime();
        printRTCtime();
    }
}
void NTPtoRAM() //联网获取正确时间，需要WiFi
{
    if (WiFi.status() == WL_CONNECTED)
    {
        printRAMtime();
        TELL_EVERYONE_LN("getting Time From NTP server")
        configTime(gmtOffset_sec, 0, ntpServer);
        timeSyncedFromNTP = true;
        getLocalTime(&time_in_RAM); //连接服务器更新时间
        printRAMtime();
        RAMtoRTC();
        wifiNeverConnected = false;
    }
    else
        TELL_EVERYONE_LN("wifi error")
}
void GPStoRAM() //读取GPS时间
{
    // gps.date, gps.time
    // !d.isValid()
    // char sz[32];
    // sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    // Serial.print(sz);
}


// 函数的头文件
long map(long x, long in_min, long in_max, long out_min, long out_max);
int intMapping(int x, int in_min, int in_max, int out_min, int out_max);
float floatMapping(float x, float in_min, float in_max, float out_min, float out_max);
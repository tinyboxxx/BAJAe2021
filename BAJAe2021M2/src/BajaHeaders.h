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

// GPS ====================================
#include <TinyGPSPlus.h> // Tiny GPS Plus Library

// I2C ====================================
#define I2C_SDA 4
#define I2C_SCL 33

// WIFIOTA ====================================
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
const char *ssid = "TiWifi";
const char *password = "hexiaoqi";
const char *ntpServer = "cn.ntp.org.cn";
#define gmtOffset_sec 28800 //GMT 时差 (seconds) 8 * 60 * 60

// LED灯条 =====================================
// http://www.esp32learning.com/code/esp32-and-mcp23017-flashy-led-example.php

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

#include <ESP32Time.h> //https://www.arduinolibraries.info/libraries/esp32-time
ESP32Time rtc_builtin;

//BTRY ====================================
float BTRYvoltage = 0;  //电池电压
int BTRYpercentage = 0; //电池剩余电量，0~100

// LORA ====================================
#define Serial2_RXPIN 16 //to LORA TX
#define Serial2_TXPIN 17 //to LORA RX

//测速 RPM SPD====================================
//https://github.com/madhephaestus/ESP32Encoder

// #include <ESP32Encoder.h>
// ESP32Encoder encoder_speed;
// ESP32Encoder encoder_rpm;

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

int lora_power_mode = 0; // 0是正常功耗、2是低功耗。低功耗模式需要电脑端在1模式 唤醒模式，M0-1、M1-0。

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


// CLI ====================================
#include <SimpleCLI.h>   // Inlcude Library
SimpleCLI cli;           // Create CLI Object
void turnoffwifi(cmd *c) // Callback function for cowsay command
{
    Command cmd(c); // Create wrapper object
    WiFi.mode(WIFI_OFF);
}
void turnonwifi(cmd *c) // Callback function for cowsay command
{
    Command cmd(c); // Create wrapper object
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
}
void loralowpower(cmd *c) // Callback function for cowsay command
{
    Command cmd(c); // Create wrapper object
    lora_power_mode = 2;
}
void lorahighpower(cmd *c) // Callback function for cowsay command
{
    Command cmd(c); // Create wrapper object
    lora_power_mode = 0;
}

void i2cscan(cmd *c)
{
    Command cmd(c); // Create wrapper object
    Serial.println();
    Serial.println("I2C scanner. Scanning ...");
    byte count = 0;

    for (byte i = 8; i < 120; i++)
    {
        Wire.beginTransmission(i);       // Begin I2C transmission Address (i)
        if (Wire.endTransmission() == 0) // Receive 0 = success (ACK response)
        {
            Serial.print("Found address: ");
            Serial.print(i, DEC);
            Serial.print(" (0x");
            Serial.print(i, HEX); // PCF8574 7 bit address
            Serial.println(")");
            count++;
        }
    }
    Serial.print("Found ");
    Serial.print(count, DEC); // numbers of devices
    Serial.println(" device(s).");
}

void errorCallback(cmd_error *e) // Callback in case of an error
{
    CommandError cmdError(e); // Create wrapper object
    Serial.print("ERROR: ");
    Serial2.print("ERROR: ");
    Serial.println(cmdError.toString());
    Serial2.println(cmdError.toString());
    if (cmdError.hasCommand())
    {
        Serial2.print("Did you mean \"");
        Serial2.print(cmdError.getCommand().toString());
        Serial2.println("\"?");

        Serial.print("Did you mean \"");
        Serial.print(cmdError.getCommand().toString());
        Serial.println("\"?");
    }
}




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
    if (getLocalTime(&time_in_RAM)) // 从本机内读取时间进内存
    {
        Serial.println(&time_in_RAM, "%F %T");
        Serial2.println(&time_in_RAM, "%F %T");
    }
    else
    {
        Serial.println("Failed to obtain time");
        Serial2.println("Failed to obtain time");
    }
}

void RTCtoRAM() //读取RTC的时间到内存
{
    printRTCtime();
    struct tm TimeInRTC;
    if (rtc.read(&TimeInRTC))
    {
        time_t t = mktime(&TimeInRTC);
        struct timeval now = {.tv_sec = t};
        settimeofday(&now, NULL);
        timeSyncedFromRTC = true;
        printRAMtime();
    }
    else
    {
        TELL_EVERYONE_LN("RTC error,RTCtoRAM FAIL")
    }
}
void RAMtoRTC() //写入时间至RTC，未测试
{
    TELL_EVERYONE_LN("RAMtoRTC");
    if (DS3231isOK)
    {
        printRAMtime();
        printRTCtime();

        time_t nowEpoch;
        time(&nowEpoch);
        rtc.setEpoch(nowEpoch - 8 * 3600); //我们时区在+8区
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

void printGpsTime() //读取GPS时间
{
    if (!gps.time.isValid())
    {
        TELL_EVERYONE_LN("GPS Time is not Valid")
    }
    else
    {
        if (gps.time.hour() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
    }
}

void GPStoRAM() //读取GPS时间
{
    if (!gps.time.isValid())
    {
        TELL_EVERYONE_LN("GPS Time is not Valid")
    }
    else
    {
        rtc_builtin.setTime(gps.time.second(), gps.time.minute(), gps.time.hour(), gps.date.day(), gps.date.month(), gps.date.day()); // 17th Jan 2021 15:24:30
    }
}

int SPD_count;
volatile int lastPulseCounter_SPD = 0;
int SPD_Calc_Factor = 6350; //频率换算系数，计算方法见excel表

unsigned long last_RPM_millis = 0;
unsigned long last_SPD_millis = 0;

int RPM_count; // only for debug
volatile int lastPulseCounter_RPM = 0;
int RPM_Calc_Factor = 60000; //频率换算系数

void IRAM_ATTR SPD_TRIGGERED()
{
    if (millis() > last_SPD_millis + 50)
    {
        SPD_count++;
        SPD = SPD_Calc_Factor / (millis() - last_SPD_millis);
        last_SPD_millis = millis();
    }
}
void IRAM_ATTR RPM_TRIGGERED()
{
    RPM_count++;
    RPM = RPM_Calc_Factor / (millis() - last_RPM_millis);
    last_RPM_millis = millis();
}

// Gear Ratio
int GearRatio = 0;
static int GearRatio_Calc_Facotr = 310;
// 3750, 60, gear = 5
/*
GearRatio = SPD * GearRatio_Calc_Facotr / RPM;
*/

// 函数的头文件
long map(long x, long in_min, long in_max, long out_min, long out_max);
int intMapping(int x, int in_min, int in_max, int out_min, int out_max);
float floatMapping(float x, float in_min, float in_max, float out_min, float out_max);

void set_MCP_send_cmd(int Channal_A, int Channal_B)
{
    Wire.beginTransmission(0x20);
    Wire.write(0x12);            // address bank A
    Wire.write((byte)Channal_A); // value to send - all HIGH
    Wire.endTransmission();

    Wire.beginTransmission(0x20);
    Wire.write(0x13);            // address bank B
    Wire.write((byte)Channal_B); // value to send - all LED HIGH
    // 0000 1111 -> 0F
    // ^GPB7   ^GBP0
    Wire.endTransmission();
}

void set_MCP(int num_of_ON_LED, int lora_lowpower_mode)
{
    switch (lora_lowpower_mode)
    {
    case 0: // Lora正常高功耗工作状态
        switch (num_of_ON_LED)
        {
        case 0:
            set_MCP_send_cmd(0x00, 0x00);
            break;
        case 1:
            set_MCP_send_cmd(0x01, 0x00);
            break;
        case 2:
            set_MCP_send_cmd(0x03, 0x00);
            break;
        case 3:
            set_MCP_send_cmd(0x07, 0x00);
            break;
        case 4:
            set_MCP_send_cmd(0x0F, 0x00);
            break;
        case 5:
            set_MCP_send_cmd(0x1F, 0x00);
            break;
        case 6:
            set_MCP_send_cmd(0x3F, 0x00);
            break;
        case 7:
            set_MCP_send_cmd(0x7F, 0x00);
            break;
        case 8:
            set_MCP_send_cmd(0xFF, 0x00);
            break;
        case 9:
            set_MCP_send_cmd(0xFF, 0x01);
            break;
        case 10:
            set_MCP_send_cmd(0xFF, 0x03);
            break;
        case 11:
            set_MCP_send_cmd(0xFF, 0x07);
            break;
        case 12:
            set_MCP_send_cmd(0xFF, 0x0F);
            break;
        default:
            break;
        }
        break;

    case 2: //低功耗休眠模式
        switch (num_of_ON_LED)
        {
        case 0:
            set_MCP_send_cmd(0x00, 0x80);
            break;
        case 1:
            set_MCP_send_cmd(0x01, 0x80);
            break;
        case 2:
            set_MCP_send_cmd(0x03, 0x80);
            break;
        case 3:
            set_MCP_send_cmd(0x07, 0x80);
            break;
        case 4:
            set_MCP_send_cmd(0x0F, 0x80);
            break;
        case 5:
            set_MCP_send_cmd(0x1F, 0x80);
            break;
        case 6:
            set_MCP_send_cmd(0x3F, 0x80);
            break;
        case 7:
            set_MCP_send_cmd(0x7F, 0x80);
            break;
        case 8:
            set_MCP_send_cmd(0xFF, 0x80);
            break;
        case 9:
            set_MCP_send_cmd(0xFF, 0x81);
            break;
        case 10:
            set_MCP_send_cmd(0xFF, 0x83);
            break;
        case 11:
            set_MCP_send_cmd(0xFF, 0x87);
            break;
        case 12:
            set_MCP_send_cmd(0xFF, 0x8F);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}
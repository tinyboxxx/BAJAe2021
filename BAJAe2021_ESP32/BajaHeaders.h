
#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>

// BNO055姿态 ====================================
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
sensors_event_t event; //BNO event
bool BNO055isOK = 1;

// GPS ====================================
#include <TinyGPS++.h> // Tiny GPS Plus Library

// I2C ====================================
#define I2C_SDA 2
#define I2C_SCL 15

// WIFIOTA ====================================
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
const char *ssid = "TiWifi";
const char *password = "";

// LED灯条 =====================================
#include <FastLED.h>
#define LED_DATA_PIN 26
#define LED_CLOCK_PIN 27
#define NUM_LEDS 10
CRGB leds[NUM_LEDS]; // Define the array of leds

// 定义任务 ====================================
void PrintToOLED(void *pvParameters);
void getBNO055Data(void *pvParameters);
void getGPSData(void *pvParameters);
void handleOTAtask(void *pvParameters);
void updateRevving(void *pvParameters);
void UpdateLEDstrip(void *pvParameters);

// OLED屏幕 ====================================
U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/5, /* dc=*/12, /* reset=*/14); // Enable U8G2_16BIT in u8g2.h

// GPS ====================================
// #define Serial2_RXPIN 16           //to GPS TX
// #define Serial2_TXPIN 17           //to GPS RX
// const float Home_LAT = 33.205288;  // Your Home Latitude
// const float Home_LNG = -96.951125; // Your Home Longitude
// int incomingByte;
// TinyGPSPlus gps; // Create an Instance of the TinyGPS++ object called gps

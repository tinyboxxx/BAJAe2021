//所有字体列表：https://github.com/olikraus/u8g2/wiki/fntlistall
//模拟器：https://p3dt.net/u8g2sim/

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

// 定义任务 ====================================

void PrintToOLED(void *pvParameters);
void getBNO055Data(void *pvParameters);
void getGPSData(void *pvParameters);

// OLED屏幕 ====================================

U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/5, /* dc=*/12, /* reset=*/14); // Enable U8G2_16BIT in u8g2.h

// GPS ====================================

// #define Serial2_RXPIN 16           //to GPS TX
// #define Serial2_TXPIN 17           //to GPS RX
// const float Home_LAT = 33.205288;  // Your Home Latitude
// const float Home_LNG = -96.951125; // Your Home Longitude
// int incomingByte;
// TinyGPSPlus gps; // Create an Instance of the TinyGPS++ object called gps
int i = 0;
int fpsOLED = 0;
long lastOLEDrefreshTime = 0;

int fpsGPS = 0;

int RPM = 0;
int SPD = 0; //千米每小时

void setup(void)
{
    Serial.begin(115200);
    Serial.println("Booting");

    // WIFIOTA ====================================

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        Serial.println("Connection Failed! Rebooting...");
        // delay(5000);
        // ESP.restart();
    }
    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);
    //--
    // Hostname defaults to esp3232-[MAC]
    // ArduinoOTA.setHostname("myesp32");
    //--
    // No authentication by default
    // ArduinoOTA.setPassword("admin");
    //--
    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

    ArduinoOTA
        .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
            else // U_SPIFFS
                type = "filesystem";

            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
            Serial.println("Start updating " + type);
        })
        .onEnd([]() {
            Serial.println("\nEnd");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)
                Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR)
                Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR)
                Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR)
                Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR)
                Serial.println("End Failed");
        });

    ArduinoOTA.begin();
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // I2C ====================================

    Wire.begin(I2C_SDA, I2C_SCL);

    // GPS ====================================

    // Serial2.begin(9600, SERIAL_8N1, Serial2_RXPIN, Serial2_TXPIN); // GPS com

    // BNO055姿态 ====================================

    if (!bno.begin()) // 初始化传感器
    {
        Serial.print("ERR no BNO055 detected"); // 检测BNO055时出现问题...请检查您的连接
        BNO055isOK = 0;
    }
    bno.setExtCrystalUse(true); //使用外部晶振以获得更好的精度

    // OLED屏幕 ====================================

    u8g2.begin();

    // 任务定义 ====================================

    xTaskCreate(
        getBNO055Data, "getBNO055Data" // 为了方便人读，取的名字
        ,
        8192 // 可以通过阅读Stack Highwater来检查和调整此堆栈大小
        ,
        NULL, 1 // 优先级，其中3（configMAX_PRIORITIES-1）为最高，0为最低。
        ,
        NULL);

    xTaskCreate(
        PrintToOLED, "PrintToOLED" // 为了方便人读，取的名字
        ,
        16384 // 可以通过阅读Stack Highwater来检查和调整此堆栈大小
        ,
        NULL, 1 // 优先级，其中3（configMAX_PRIORITIES-1）为最高，0为最低。
        ,
        NULL);
    xTaskCreate(
        getGPSData, "getGPSData" // 为了方便人读，取的名字
        ,
        16384 // 可以通过阅读Stack Highwater来检查和调整此堆栈大小
        ,
        NULL, 1 // 优先级，其中3（configMAX_PRIORITIES-1）为最高，0为最低。
        ,
        NULL);

    xTaskCreate(
        handleOTAtask, "handleOTAtask" // 为了方便人读，取的名字
        ,
        2048 // 可以通过阅读Stack Highwater来检查和调整此堆栈大小
        ,
        NULL, 2 // 优先级，其中3（configMAX_PRIORITIES-1）为最高，0为最低。
        ,
        NULL);
    xTaskCreate(
        updateRevvingI, "updateRevvingI" // 为了方便人读，取的名字
        ,
        512 // 可以通过阅读Stack Highwater来检查和调整此堆栈大小
        ,
        NULL, 0 // 优先级，其中3（configMAX_PRIORITIES-1）为最高，0为最低。
        ,
        NULL);
}

void loop(void)
{

    vTaskDelay(1); //延时，或许以后删掉
}

void drawSignal(U8G2 u8g2, uint8_t x, uint8_t y, uint8_t strength)
{
    for (uint8_t i = 0; i < strength; i++)
    {
        u8g2.drawCircle(x, y, i * 3, U8G2_DRAW_UPPER_RIGHT);
    }
}

int revving(int UPlimit, int Downlimit, int i)
{
    int pp = UPlimit - Downlimit;
    int currentRev;
    bool goingDown = (i / pp) / 2;
    if (goingDown)
        currentRev = UPlimit - i % pp;
    else
        currentRev = Downlimit + i % pp;
    return currentRev;
}

void drawGForce(int x, int y) //x:-100~100
{
    u8g2.drawFrame(0, 11, 52, 52); //GForce
    //长宽都是53，一半的长度是26
    //X:0->26->52
    //y:11->37->63
    //中心的点是x26,y37
    u8g2.drawLine(0, 37, 52, 37);  //横向中心线
    u8g2.drawLine(26, 11, 26, 63); //纵向中心线
    //u8g2.drawFrame(13, 24, 26, 26); //小圈,方的
    u8g2.drawCircle(26, 37, 13, U8G2_DRAW_ALL); //小圈，圆的
    //
    u8g2.drawBox(25, 36, 3, 3); //指示点
}
void PrintToOLED(void *pvParameters) // OLED 刷新任务
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 40;
    //设置为100时，大约为10fps
    //设置为16时，大约48fps

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    char PrinterStr[31];
    for (;;) // 任务永远不会返回或退出。
    {

        vTaskDelayUntil(&xLastWakeTime, xFrequency); // 等待下一个周期。

        u8g2.clearBuffer();   //清空屏幕
        u8g2.setDrawColor(1); // White
        u8g2.setFontMode(0);
        if (BNO055isOK)
        {
            sprintf(PrinterStr, "R%04.0fP%04.0fH%04.0f", (float)event.orientation.x, (float)event.orientation.y, (float)event.orientation.z);
            u8g2.setFont(u8g2_font_8x13B_tf);
            u8g2.drawStr(0, 10, PrinterStr);
            Serial.println(PrinterStr);
        }
        u8g2.setDrawColor(1);
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.drawStr(0, 6, "X-000,Y-000");

        u8g2.drawFrame(85, 1, 10, 30);
        u8g2.drawFrame(85, 33, 10, 30);
        u8g2.drawFrame(158, 1, 10, 30);
        u8g2.drawFrame(158, 33, 10, 30);

        char bufferStr2[2];
        sprintf(bufferStr2, "%02d", SPD);

        SPD = revving(60, 1, i);
        u8g2.setFont(u8g2_font_logisoso42_tn);
        u8g2.drawStr(98, 45, bufferStr2); //SPD文字显示

        RPM = revving(3800, 1800, i);
        u8g2.setFont(u8g2_font_logisoso18_tn);

        char bufferStr4[4];
        sprintf(bufferStr4, "%04d", RPM);

        u8g2.drawStr(210, 18, bufferStr4); //RPM文字显示

        u8g2.setFont(u8g2_font_6x10_mr);
        u8g2.drawStr(222, 25, "RPM");

        u8g2.setFont(u8g2_font_6x10_mr);//日期时间显示
        u8g2.drawStr(97, 55, "2021-04-24");
        u8g2.drawStr(103, 64 ,"18:05:05");

        drawSignal(u8g2, 180, 12, 4); //满格信号，三格

        drawGForce(0,0);

        u8g2.setFont(u8g2_font_siji_t_6x10);
        // u8g2.drawGlyph(x, y, 0xe242);   //empty
        // u8g2.drawGlyph(x, y, 0xe250);   //half
        u8g2.drawGlyph(194, 12, 0xe254); //full

        // u8x8.setCursor(0, 1);
        // /* Latitude */
        // u8x8.print("a" + String(gps.location.lat(), 4));
        // /* Longitude */
        // u8x8.print("o" + String(gps.location.lng(), 4));

        // u8x8.setCursor(0, 2);
        // /* Satellites */
        // u8x8.print("S" + String(gps.satellites.value()));
        // u8x8.print("op" + String(gps.hdop.hdop(), 1));
        // u8x8.print("sp" + String(gps.speed.kmph(), 0));

        // u8x8.setCursor(0, 3);
        // sprintf(PrinterStr, "%016d", i);
        // u8x8.print(PrinterStr);
        u8g2.sendBuffer(); //更新至屏幕

        fpsOLED = 1000.0 / (millis() - lastOLEDrefreshTime);
        Serial.println(fpsOLED);
        lastOLEDrefreshTime = millis();

        vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
    }
}

void getBNO055Data(void *pvParameters) // BNO055姿态任务
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100;

    // 用当前时间初始化xLastWakeTime变量。
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // 等待下一个周期。
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        /* 获取一个新的传感器事件 */
        if (BNO055isOK)
        {
            bno.getEvent(&event);
        }

        vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
    }
}

void getGPSData(void *pvParameters) // GPS刷新任务
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100;

    // 用当前时间初始化xLastWakeTime变量。
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // 等待下一个周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
        /* Get GPS data */
        // while (Serial2.available())
        //     gps.encode(Serial2.read());
    }
}
void handleOTAtask(void *pvParameters) // GPS刷新任务

{
    (void)pvParameters;
    for (;;)
    {
        ArduinoOTA.handle(); //OTA必须运行的检测语句
        vTaskDelay(1);       // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
    }
}

void updateRevvingI(void *pvParameters) // GPS刷新任务
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100;

    // 用当前时间初始化xLastWakeTime变量。
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // 等待下一个周期。
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        /* 获取一个新的传感器事件 */
        i++;

        vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
    }
}
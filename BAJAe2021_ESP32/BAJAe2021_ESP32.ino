#include <Arduino.h>
#include <U8g2lib.h>

#include <SPI.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h> // Tiny GPS Plus Library

#define I2C_SDA 2
#define I2C_SCL 15

// 定义任务
void PrintToOLED(void *pvParameters);
void getBNO055Data(void *pvParameters);
void getGPSData(void *pvParameters);

U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/5, /* dc=*/12, /* reset=*/14); // Enable U8G2_16BIT in u8g2.h

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
sensors_event_t event; //BNO event

// #define Serial2_RXPIN 16           //to GPS TX
// #define Serial2_TXPIN 17           //to GPS RX
// const float Home_LAT = 33.205288;  // Your Home Latitude
// const float Home_LNG = -96.951125; // Your Home Longitude
// int incomingByte;
// TinyGPSPlus gps; // Create an Instance of the TinyGPS++ object called gps

char str[16];
int i = 0;
void setup(void)
{
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.begin(115200);
    Serial.println("Orientation Sensor Test");

    // Serial2.begin(9600, SERIAL_8N1, Serial2_RXPIN, Serial2_TXPIN); // GPS com

    if (!bno.begin()) // 初始化传感器
    {
        Serial.print("ERR no BNO055 detected"); // 检测BNO055时出现问题...请检查您的连接
    }
    bno.setExtCrystalUse(true); //使用外部晶振以获得更好的精度

    u8g2.begin();

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
}

void loop(void)
{
    vTaskDelay(1);
}

void PrintToOLED(void *pvParameters) // This is a task.
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100;

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    for (;;) // 任务永远不会返回或退出。
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // 等待下一个周期。
        u8g2.setFont(u8g2_font_8x13B_tf);
        u8g2.setCursor(0, 13);
        sprintf(str, "R%04.0fP%04.0fH%04.0f", (float)event.orientation.x, (float)event.orientation.y, (float)event.orientation.z);
        u8g2.println(str);
        Serial.println(str);
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
        // sprintf(str, "%016d", i);
        // i++;
        // u8x8.print(str);

        vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
    }
}

void getBNO055Data(void *pvParameters) // This is a task.
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
        bno.getEvent(&event);
        vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
    }
}

void getGPSData(void *pvParameters) // This is a task.
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

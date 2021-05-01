#include "BajaHeaders.h"

//所有字体列表：https://github.com/olikraus/u8g2/wiki/fntlistall
//模拟器：https://p3dt.net/u8g2sim/

unsigned int fpsOLED = 0;
long lastOLEDrefreshTime = 0;
int fpsGPS = 0;

bool isOTAing = 0;
unsigned int OTAprogress = 0;
unsigned int OTAtotal = 0;

//行车数据变量=============================
unsigned int RPM = 0;
unsigned int SPD = 0; //千米每小时

unsigned int SUS_LF = 0; //减振器，ADC值，0->4096
unsigned int SUS_RF = 0;
unsigned int SUS_LR = 0;
unsigned int SUS_RR = 0;

int GFx = 0;
int GFy = 0;

void setup(void)
{
    Serial.begin(115200);
    Serial.println("Booting");

    // WIFIOTA ====================================

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
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
            isOTAing = 1;
            OTAprogress = progress;
            OTAtotal = total;
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

    // LED =========================================
    FastLED.addLeds<APA102, LED_DATA_PIN, LED_CLOCK_PIN, RGB>(leds, NUM_LEDS);

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
        updateRevving, "updateRevving" // 为了方便人读，取的名字
        ,
        512 // 可以通过阅读Stack Highwater来检查和调整此堆栈大小
        ,
        NULL, 0 // 优先级，其中3（configMAX_PRIORITIES-1）为最高，0为最低。
        ,
        NULL);
    xTaskCreate(
        UpdateLEDstrip, "UpdateLEDstrip" // 为了方便人读，取的名字
        ,
        512 // 可以通过阅读Stack Highwater来检查和调整此堆栈大小
        ,
        NULL, 1 // 优先级，其中3（configMAX_PRIORITIES-1）为最高，0为最低。
        ,
        NULL);
}

void loop(void)
{

    vTaskDelay(1); //延时，或许以后删掉
}

#include "BajaTasks.h"
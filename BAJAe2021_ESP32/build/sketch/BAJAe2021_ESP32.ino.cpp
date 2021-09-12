#include <Arduino.h>
#line 1 "c:\\Users\\tibx\\Documents\\GitHub\\BAJAe2021\\BAJAe2021_ESP32\\BAJAe2021_ESP32.ino"
#include "BajaHeaders.h" //引用和初始化部分。
#include "BajaTasks.h"

#line 4 "c:\\Users\\tibx\\Documents\\GitHub\\BAJAe2021\\BAJAe2021_ESP32\\BAJAe2021_ESP32.ino"
void setup(void);
#line 161 "c:\\Users\\tibx\\Documents\\GitHub\\BAJAe2021\\BAJAe2021_ESP32\\BAJAe2021_ESP32.ino"
void loop(void);
#line 4 "c:\\Users\\tibx\\Documents\\GitHub\\BAJAe2021\\BAJAe2021_ESP32\\BAJAe2021_ESP32.ino"
void setup(void)
{
    btStop(); //关闭蓝牙
    Serial.begin(115200);
    Serial.println("Booting");

    // OLED屏幕 ====================================
    u8g2.begin();
    u8g2.setDrawColor(1); // White
    u8g2.setFontMode(0);
    u8g2.setContrast(255);
    u8g2.clearBuffer(); //清空屏幕

    u8g2.setFont(u8g2_font_5x7_tr);                               // 设置合适的字体。 此字体将用于 U8x8log
    u8g2log.begin(u8g2, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer); // 启动U8x8log，连接U8x8，设置维度，分配静态内存
    u8g2log.setRedrawMode(0);                                     // 0：用换行符更新屏幕，1：每个字符更新屏幕
    bootUpPrint("OLED booted!");
    // LORA ====================================
    Serial2.begin(9600, SERIAL_8N1, Serial2_RXPIN, Serial2_TXPIN); // Lora
    mcp.pinMode(14, OUTPUT);
    mcp.pinMode(15, OUTPUT);
    bootUpPrint("LORA Serial 2 Begin!");
    // TIME ====================================
    if (rtc.begin()) // 初始化 RTC https://github.com/Erriez/ErriezDS3231
    {
        rtc.setSquareWave(SquareWaveDisable);
        RTCtoRAM();
    }
    else
    {
        bootUpPrint(F("RTC not found"));
        DS3231isOK = false;
    }
    // LED =========================================

    if (!mcp.begin_I2C())
    {
        bootUpPrint("MCP23X17 Error");
    }
    for (size_t i = 0; i < 16; i++)
    {
        mcp.pinMode(i, OUTPUT); // configure pin for output
    }
    // CLI ====================================
    cli.setOnError(errorCallback); // Set error Callback
    Command turnoffwifi = cli.addCmd("turnoffwifi");
    Command turnonwifi = cli.addCmd("turnonwifi");
    bootUpPrint("CLI booted!");

    // WIFIOTA ====================================
    /*
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  bootUpPrint("WiFi booted!");
  if (WiFi.waitForConnectResult() != WL_CONNECTED) //WIFI 连接失败
  {
    wifi_connected = 0;
    Serial.println("Connection Failed!");
    bootUpPrint("WiFi Connection Failed!");
  }
  else //WiFi连接成功
  {
    wifi_connected = 1;
    bootUpPrint("WiFi wifi_connected! IP address:");
    bootUpPrint(WiFi.localIP().toString());
  }
  ArduinoOTA
      .onStart([]()
               {
                 String type;
                 if (ArduinoOTA.getCommand() == U_FLASH)
                   type = "sketch";
                 else // U_SPIFFS
                   type = "filesystem";

                 // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                 Serial.println("Start updating " + type);
               })
      .onEnd([]()
             { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  {
                    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
                    isOTAing = 1;
                    OTAprogress = progress;
                    OTAtotal = total;
                  })
      .onError([](ota_error_t error)
               {
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
  */

    // GPS ====================================
    Serial1.begin(9600, SERIAL_8N1, Serial1_RXPIN, Serial1_TXPIN); // GPS
    bootUpPrint("GPS Serial 1 Begin!");

    // I2C ====================================
    Wire.begin(I2C_SDA, I2C_SCL);

    // BTY ====================================
    //IO35 -> BTY voltage x0.5

    // Encoder ====================================
    // https://github.com/madhephaestus/ESP32Encoder
    ESP32Encoder::useInternalWeakPullResistors = UP;
    encoder_speed.attachSingleEdge(27, 27); // SPD
    encoder_rpm.attachSingleEdge(34, 34);   // RPM

    // BNO055姿态 ====================================
    if (!bno.begin()) // 初始化传感器
    {
        bootUpPrintWithLora("BNO055 Booted!");
        bno.setExtCrystalUse(true); //使用外部晶振以获得更好的精度
    }
    else
    {
        bootUpPrintWithLora("ERR no BNO055 detected"); // 检测BNO055时出现问题...请检查您的连接
        BNO055isOK = false;
    }

    // 任务定义 ====================================
    // xTaskCreate(
    //     getBNO055Data, "getBNO055Data" // 姿态传感器任务
    //     ,
    //     8192 // 堆栈大小
    //     ,
    //     NULL, 1 // 优先级，其中3（configMAX_PRIORITIES-1）为最高，0为最低。
    //     ,
    //     NULL);
    bootUpPrintWithLora("ALL BOOT Complete!");
    xTaskCreatePinnedToCore(Task_UpdateDisplay, "Task_UpdateDisplay" // 屏幕刷新任务
                            ,
                            30000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(Task_GetGpsLora, "Task_GetGpsLora" // GPS数据更新任务
                            ,
                            16384, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(Task_UpdateData, "Task_UpdateData" // 测速任务
                            ,
                            4000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(Task_UpdateTime, "Task_UpdateTime" // 测速任务
                            ,
                            4000, NULL, 1, NULL, 1);
}

void loop(void)
{
    vTaskDelay(1); //延时，或许以后删掉
}


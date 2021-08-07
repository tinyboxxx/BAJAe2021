#include "BajaHeaders.h" //引用和初始化部分。
#include "BajaTasks.h"

void setup(void)
{

  btStop();                  //关闭蓝牙
  pinMode(32, INPUT_PULLUP); //rpm 引脚
  pinMode(33, INPUT_PULLUP); //SPD 引脚
  initPulseCounter_RPM();
  initPulseCounter_SPD();
  Serial.begin(115200);
  Serial.println("Booting");
  // OLED屏幕 ====================================
  u8g2.begin();
  u8g2.setDrawColor(1); // White
  u8g2.setFontMode(0);
  u8g2.setContrast(255);
  u8g2.clearBuffer(); //清空屏幕

  u8g2.setFont(u8g2_font_5x7_tr);                               // Set a suitable font. This font will be used for U8x8log
  u8g2log.begin(u8g2, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer); // Start U8x8log, connect to U8x8, set the dimension and assign the static memory
  u8g2log.setRedrawMode(0);                                     // 0: Update screen with newline, 1: Update screen for every char
  bootUpPrint("OLED booted!");

  // TIME ====================================
  // Initialize RTC
  if (rtc.begin())
  {
    rtc.setSquareWave(SquareWaveDisable);
    RTCtoRAM();
  }
  else
  {
    bootUpPrintWithLora(F("RTC not found"));
    DS3231isOK = false;
  }

  // CLI ====================================

  cli.setOnError(errorCallback); // Set error Callback
  Command turnoffwifi = cli.addCmd("turnoffwifi");
  Command turnonwifi = cli.addCmd("turnonwifi");
  bootUpPrint("CLI booted!");


  // WIFIOTA ====================================

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  bootUpPrint("WiFi booted!");
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    wifi_connected = 0;
    Serial.println("Connection Failed!");
    bootUpPrint("WiFi Connection Failed!");
  }
  else
  {
    wifi_connected = 1;
    bootUpPrint("WiFi wifi_connected!");
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
    bootUpPrint("OTA Booted, IP address:");
    bootUpPrint(WiFi.localIP().toString());
  }
  // GPS ====================================
  Serial1.begin(9600, SERIAL_8N1, Serial1_RXPIN, Serial1_TXPIN); // GPS com
  bootUpPrint("GPS Serial 1 Begin!");
  // LORA ====================================
  Serial2.begin(9600, SERIAL_8N1, Serial2_RXPIN, Serial2_TXPIN); // GPS com
  bootUpPrint("LORA Serial 2 Begin!");

  // I2C ====================================
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  // BTY ====================================
  if (gauge.begin() == 0)
  {
    bootUpPrintWithLora("gauge begin successful!");
  }
  else
  {
    bootUpPrintWithLora("gauge begin failed!");
  }

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

  // LED =========================================
  FastLED.addLeds<APA102, LED_DATA_PIN, LED_CLOCK_PIN, BGR>(leds, NUM_LEDS);

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

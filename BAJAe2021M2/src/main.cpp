#include <Arduino.h>
#include "BajaHeaders.h" //引用和初始化部分。
#include "BajaTasks.h"

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
  // I2C ====================================
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  bootUpPrint("I2C Begin!");
  // LED =========================================
  bootUpPrint("Test LED");

  Wire.beginTransmission(0x20);
  Wire.write(0x00); // IODIRA register
  Wire.write(0x00); // set all of port A to outputs
  Wire.endTransmission();

  Wire.beginTransmission(0x20);
  Wire.write(0x01); // IODIRB register
  Wire.write(0x00); // set all of port B to outputs
  Wire.endTransmission();

  set_MCP(12, lora_power_mode);
  vTaskDelay(200);
  set_MCP(0, lora_power_mode);
  bootUpPrint("LED OK!");

  // LORA ====================================
  Serial2.begin(9600, SERIAL_8N1, Serial2_RXPIN, Serial2_TXPIN); // Lora
  bootUpPrint("LORA Serial 2 Begin!");                           // need MCP BPIB 6&7 LOW

  // TIME ====================================
  if (rtc.begin()) // 初始化 RTC https://github.com/Erriez/ErriezDS3231
  {
    rtc.clockEnable(true);
    rtc.setSquareWave(SquareWaveDisable);
    bootUpPrint(F("RTC OK.Time now:"));
    RTCtoRAM();
  }
  else
  {
    bootUpPrint("RTC error");
  }
  // CLI ====================================
  cli.setOnError(errorCallback); // Set error Callback
  Command turnoffwifi_Cmd = cli.addCmd("turnoffwifi", turnoffwifi);
  Command turnonwifi_Cmd = cli.addCmd("turnonwifi", turnonwifi);
  Command printip_cmd = cli.addCmd("printip", printip);
  Command loralowpower_Cmd = cli.addCmd("loralowpower", loralowpower);
  Command lorahighpower_Cmd = cli.addCmd("lorahighpower", lorahighpower);
  Command Scanner_Cmd = cli.addCmd("i2cscan", i2cscan);
  Command reboot_Cmd = cli.addCmd("reboot", reboot);
  Command printGpsTime_Cmd = cli.addCmd("gpstime", cmd_gpstime);
  Command settime_cmd = cli.addSingleArgCmd("settime", settime);
  Command sendtele = cli.addSingleArgCmd("teleon", cmd_sendtele);
  Command telepoff = cli.addSingleArgCmd("teleoff", cmd_teleoff);

  bootUpPrint("CLI booted!");

  // GPS ====================================
  Serial1.begin(9600, SERIAL_8N1, Serial1_RXPIN, Serial1_TXPIN); // GPS
  bootUpPrint("GPS Serial 1 Begin!");

  // BTY ====================================
  //IO35 -> BTY voltage x0.5

  // Encoder ====================================
  pinMode(27, INPUT_PULLUP); //SPD
  attachInterrupt(27, SPD_TRIGGERED, FALLING);
  pinMode(34, INPUT_PULLUP); //RPM
  attachInterrupt(34, RPM_TRIGGERED, RISING);

  // IMU ====================================
  if (!mpu.begin(0x69))
  {
    TELL_EVERYONE_LN("MPU6050 ERROR")
  }
  else
  {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
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
  xTaskCreatePinnedToCore(Task_UpdateTime, "Task_UpdateTime" // 测速任务
                          ,
                          4000, NULL, 1, NULL, 1);

  // ArduinoOTA
  //     .onStart([]()
  //              {
  //                String type;
  //                if (ArduinoOTA.getCommand() == U_FLASH)
  //                  type = "sketch";
  //                else // U_SPIFFS
  //                  type = "filesystem";

  //                // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
  //                Serial.println("Start updating " + type);
  //              })
  //     .onEnd([]()
  //            { Serial.println("\nEnd"); })
  //     .onProgress([](unsigned int progress, unsigned int total)
  //                 { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
  //     .onError([](ota_error_t error)
  //              {
  //                Serial.printf("Error[%u]: ", error);
  //                if (error == OTA_AUTH_ERROR)
  //                  Serial.println("Auth Failed");
  //                else if (error == OTA_BEGIN_ERROR)
  //                  Serial.println("Begin Failed");
  //                else if (error == OTA_CONNECT_ERROR)
  //                  Serial.println("Connect Failed");
  //                else if (error == OTA_RECEIVE_ERROR)
  //                  Serial.println("Receive Failed");
  //                else if (error == OTA_END_ERROR)
  //                  Serial.println("End Failed");
  //              });

  // // ArduinoOTA.begin();

  // // Serial.print("IP address: ");
  // // Serial.println(WiFi.localIP());
}

void loop(void)
{
  vTaskDelay(1); //延时，或许以后删掉
}

//本文件为后部Tasks任务具体调用的函数。

void drawSignal(U8G2 u8g2, uint8_t x, uint8_t y, uint8_t strength)
{
    for (uint8_t i = 0; i < strength; i++)
    {
        u8g2.drawCircle(x, y, i * 3, U8G2_DRAW_UPPER_RIGHT);
    }
}

void Task_UpdateDisplay(void *pvParameters) // OLED 刷新任务
{

    //所有字体列表：https://github.com/olikraus/u8g2/wiki/fntlistall
    //模拟器：https://p3dt.net/u8g2sim/

    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 50;

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount(); //获取当前tick
    for (;;)                             // 任务永远不会返回或退出。
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // 等待下一个周期。

        if (isOTAing == 0) //正常运行中
        {
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);

            GFx = a.acceleration.y;
            GFy = a.acceleration.z;

            //(float)linearAccelData.acceleration.y 是车前后
            //(float)linearAccelData.acceleration.z 是车左右

            u8g2.clearBuffer(); //清空屏幕
            u8g2.setFont(u8g2_font_5x7_tr);
            char PrinterStr[20];
            sprintf(PrinterStr, "X%04.1f ", GFx);
            u8g2.drawStr(67, 9, PrinterStr);

            sprintf(PrinterStr, "Y%04.1f", GFy);
            u8g2.drawStr(67, 17, PrinterStr);

            if (RPM > 1700 && SPD > 5)
            {
                GearRatio = SPD * GearRatio_Calc_Facotr / RPM;
            }
            else
                GearRatio = 0;

            u8g2.setFont(u8g2_font_6x10_mr);
            u8g2.setCursor(68, 26);
            u8g2.print("Gear");
            u8g2.setCursor(72, 36);
            u8g2.print(GearRatio);

            // u8g2.drawFrame(0, 0, 65, 64);               //GForce 外框
            u8g2.drawFrame(0, 0, 65, 64);               //GForce 外框
            u8g2.drawLine(0, 32, 64, 32);               //横向中心线
            u8g2.drawLine(32, 0, 32, 64);               //纵向中心线
            u8g2.drawCircle(32, 32, 16, U8G2_DRAW_ALL); //小圈,圆的
            GFx_OLED = GFx * GFx_OLED_ZoomLevel;
            GFy_OLED = GFy * GFx_OLED_ZoomLevel;
            if (GFx_OLED > 26)
                GFx_OLED = 26;
            else if (GFx_OLED < -26)
                GFx_OLED = -26;
            if (GFy_OLED > 26)
                GFy_OLED = 26;
            else if (GFy_OLED < -26)
                GFy_OLED = -26;

            u8g2.drawBox(GFx_OLED + 31, GFy_OLED + 31, 3, 3); //指示点 初始位置25,36,3,3

            char bufferStr2[2];
            if (gps_hdop <= 1.1 && gps_speed > 5)
            {
                sprintf(bufferStr2, "%02.0f", gps_speed);
            }
            else
            {
                // if (SPD > 99)
                // {
                //     sprintf(bufferStr2, "99");
                // }
                // else
                // {

                //     if (SPD < 5)
                //     {
                sprintf(bufferStr2, "00");
                //     }
                //     else
                //     {
                //         sprintf(bufferStr2, "%02d", SPD);
                //     }
                // }
            }

            if (millis() - last_SPD_millis > 1300)
            {
                SPD = 0;
            }
            if (millis() - last_RPM_millis > 1500)
            {
                RPM = 0;
            }
            u8g2.setFont(u8g2_font_logisoso50_tn);
            u8g2.drawStr(95, 50, bufferStr2); //SPD文字显示

            u8g2.setFont(u8g2_font_t0_14b_mr); //日期时间显示
            u8g2.setCursor(98, 64);            //时间
            u8g2.print(&time_in_RAM, "%T");

            u8g2.setFont(u8g2_font_t0_12_mr); //日期显示
            u8g2.setCursor(194, 62);
            u8g2.print(&time_in_RAM, "%F");

            u8g2.setFont(u8g2_font_logisoso22_tn);
            char bufferStr4[4];
            sprintf(bufferStr4, "%04d", RPM);
            u8g2.drawStr(196, 24, bufferStr4); //RPM文字显示

            u8g2.setFont(u8g2_font_6x10_mr);
            u8g2.drawStr(216, 33, "RPM");

            u8g2.setCursor(196, 42);
            TRIP = SPD_count * 1.75 * 0.001;
            u8g2.print(TRIP);

            u8g2.setCursor(213, 51);
            u8g2.print("TRIP");

            // if (WiFi.waitForConnectResult() != WL_CONNECTE)
            //     drawSignal(u8g2, 180, 12, 4); //满格信号，三格

            u8g2.setFont(u8g2_font_siji_t_6x10); //battery BTY 电量
            if (BTRYpercentage > 80)
            {
                u8g2.drawGlyph(166, 9, 0xe254); //full
            }
            else if (BTRYpercentage > 30)
            {
                u8g2.drawGlyph(166, 9, 0xe250); //half
            }
            else
            {
                u8g2.drawGlyph(166, 9, 0xe242); //empty
            }

            u8g2.setFont(u8g2_font_6x10_mr);
            u8g2.setCursor(179, 9);
            if (BTRYpercentage > 0)
            {
                u8g2.print(BTRYpercentage);
            }
            else
                u8g2.print("ER");

            u8g2.setFont(u8g2_font_6x10_mr);
            u8g2.setCursor(166, 22);
            u8g2.println("SATn");
            u8g2.setCursor(166, 32);
            u8g2.println(gps_sat_count);
            u8g2.setCursor(166, 44);
            u8g2.println("Temp");
            u8g2.setCursor(166, 54);
            u8g2.println(temp.temperature,1);

            // fpsOLED = 1000.0 / (millis() - lastOLEDrefreshTime); //FPS
            // lastOLEDrefreshTime = millis();
            // u8g2.setFont(u8g2_font_6x10_mr);
            // sprintf(bufferStr4, "%02d", fpsOLED);
            // u8g2.drawStr(180, 64, bufferStr4);

            u8g2.drawFrame(64, 0, 29, 64); //右侧外框

            u8g2.drawFrame(164, 0, 29, 64); //右侧外框

            u8g2.drawFrame(192, 0, 64, 64); //右侧外框

            u8g2.sendBuffer(); //更新至屏幕

            // if (setLEDtoSpeed == 1) //开始计算LED灯
            // {
            //     nShiftlightPos = intMapping(SPD, SPD_Display_MIN, SPD_Display_MAX, 0, 12);
            // }
            // else
            // {
            nShiftlightPos = floatMapping(RPM, RPM_Display_MIN, RPM_Display_MAX, 0, 12);
            if (nShiftlightPos > 12)
            {
                nShiftlightPos = 12;
            }
            if (nShiftlightPos < 0)
            {
                nShiftlightPos = 0;
            }
            // }

            set_MCP(nShiftlightPos, lora_power_mode);
        }
        else //正在OTA中
        {
            u8g2.clearBuffer(); //清空屏幕
            char bufferStr[8];
            u8g2.setFont(u8g2_font_5x7_tr);
            u8g2.drawStr(0, 6, "UPDATING");
            u8g2.drawStr(0, 13, "Progress:");
            sprintf(bufferStr, "%04d", OTAprogress);
            u8g2.drawStr(40, 13, bufferStr);
            u8g2.drawStr(0, 20, "total:");
            sprintf(bufferStr, "%04d", OTAtotal);
            u8g2.drawStr(40, 20, bufferStr);
            u8g2.sendBuffer(); //更新至屏幕
        }
        vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
    }
}

void Task_GetGpsLora(void *pvParameters) // GPS刷新任务
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100;
    xLastWakeTime = xTaskGetTickCount(); // 用当前时间初始化xLastWakeTime变量。
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // 等待下一个周期

        while (Serial1.available())
            gps.encode(Serial1.read()); /* Get GPS data */

        gps_hdop = gps.hdop.hdop();             //GPS 精度，越低越好
        gps_speed = gps.speed.kmph();           // GPS速度 kph
        gps_sat_count = gps.satellites.value(); //GPS卫星数量

        if (Serial2.available())
        {
            String input = Serial2.readStringUntil('\n'); // Read out string from the serial monitor
            cli.parse(input);                             // Parse the user input into the CLI
        }
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n'); // Read out string from the serial monitor
            cli.parse(input);                            // Parse the user input into the CLI
        }

        if (cli.errored())
        {
            CommandError cmdError = cli.getError();
            Serial2.print("ERROR: ");
            Serial2.println(cmdError.toString());
            if (cmdError.hasCommand())
            {
                Serial2.print("Did you mean \"");
                Serial2.print(cmdError.getCommand().toString());
                Serial2.println("\"?");
            }
        }

        // ArduinoOTA.handle(); //OTA必须运行的检测语句
    }
}

void Task_UpdateData(void *pvParameters) // 测时速、转速、姿态、SD卡写入
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 8;
    xLastWakeTime = xTaskGetTickCount(); // 用当前时间初始化xLastWakeTime变量。

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // 等待下一个周期

        vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
    }
}

//时间更新任务
//时间的准确性排序：GPS、NTP、RTC、RAM
//时间的效率排序：RAM、RTC、NTP、GPS
//应当开机时从RTC读取时间到RAM，随后手动要求更新RTC。
void Task_UpdateTime(void *pvParameters) //时间更新任务，1秒钟更新1次。电量更新
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 999;
    xLastWakeTime = xTaskGetTickCount(); // 用当前时间初始化xLastWakeTime变量。
    for (;;)
    {
        // 等待下一个周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        getLocalTime(&time_in_RAM);
        // Serial.println(&time_in_RAM, "%F %T");

        // BTRYvoltage=analogRead(35)/4095*3.3*2;
        BTRYvoltage = analogRead(35) * 0.001795;
        BTRYpercentage = floatMapping(BTRYvoltage, 2.8, 3.4, 0, 100);
        if (BTRYpercentage > 99)
            BTRYpercentage = 99;

        // if (GPSNeverConnected && gps.satellites.value() >= 3)
        // {
        //     GPSNeverConnected = false;
        //     GPStoRAM();
        //     // RAMtoRTC();
        // }
        if (sendtele)
        {
            Serial2.print(RPM);
            Serial2.print(",");
            Serial2.print(SPD);
            Serial2.print(",");
            Serial2.print(gps.location.lat(), 6);
            Serial2.print(",");
            Serial2.print(gps.location.lng(), 6);
            Serial2.print("\n");
        }
    }
}

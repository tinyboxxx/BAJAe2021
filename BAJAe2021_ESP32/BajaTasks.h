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

        if (isOTAing == 0)
        {
            if (BNO055isOK && I2C_is_Busy == false)
            {
                I2C_is_Busy = true;
                bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
                GFy = linearAccelData.acceleration.z;
                GFx = linearAccelData.acceleration.y;
                vTaskDelay(3);
                I2C_is_Busy = false;

                //(float)linearAccelData.acceleration.y 是车前后
                //(float)linearAccelData.acceleration.z 是车左右
            }

            u8g2.clearBuffer(); //清空屏幕

            u8g2.setFont(u8g2_font_5x7_tr);
            char PrinterStr[20];
            sprintf(PrinterStr, "X%04.1f Y%04.1f", GFx, GFy);
            u8g2.drawStr(0, 6, PrinterStr);
            u8g2.drawFrame(0, 11, 53, 53); //GForce
            //长宽都是53，一半的长度是26
            //X:0->26->52
            //y:11->37->63
            //中心的点是x26,y37
            u8g2.drawLine(0, 37, 52, 37);  //横向中心线
            u8g2.drawLine(26, 11, 26, 63); //纵向中心线
            //u8g2.drawFrame(13, 24, 26, 26); //小圈,方的
            u8g2.drawCircle(26, 37, 13, U8G2_DRAW_ALL); //小圈，圆的
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

            u8g2.drawFrame(GFx_OLED + 25, GFy_OLED + 36, 3, 3); //指示点 初始位置25,36,3,3

            char bufferStr2[2];
            if (gps_hdop <= 1.3)
            {
                sprintf(bufferStr2, "%02d", gps_speed);
            }
            else
            {
                sprintf(bufferStr2, "%02d", SPD);
            }
            u8g2.setFont(u8g2_font_logisoso42_tn);
            u8g2.drawStr(98, 45, bufferStr2); //SPD文字显示

            u8g2.setFont(u8g2_font_logisoso18_tn);
            char bufferStr4[4];
            sprintf(bufferStr4, "%04d", RPM);
            u8g2.drawStr(210, 18, bufferStr4); //RPM文字显示

            u8g2.setFont(u8g2_font_6x10_mr);
            u8g2.drawStr(222, 25, "RPM");

            u8g2.setFont(u8g2_font_6x10_mr); //日期时间显示

            u8g2.setCursor(97, 55);
            u8g2.print(&timeinfo, "%F");
            u8g2.setCursor(103, 64);
            u8g2.print(&timeinfo, "%T");

            if (wifi_connected)
                drawSignal(u8g2, 180, 12, 4); //满格信号，三格

            u8g2.setFont(u8g2_font_siji_t_6x10);

            if (BTRYpercentage > 80)
            {
                u8g2.drawGlyph(194, 12, 0xe254); //full
            }
            else if (BTRYpercentage > 30)
            {
                u8g2.drawGlyph(194, 12, 0xe250); //half
            }
            else
            {
                u8g2.drawGlyph(194, 12, 0xe242); //empty
            }

            u8g2.setFont(u8g2_font_6x10_mr);
            u8g2.setCursor(180, 28);
            u8g2.print(gps_sat_count);
            u8g2.setCursor(180, 37);
            u8g2.print(gps_speed);
            u8g2.setCursor(180, 46);
            u8g2.print(BTRYvoltage);
            u8g2.setCursor(180, 55);
            u8g2.print(BTRYpercentage);

            fpsOLED = 1000.0 / (millis() - lastOLEDrefreshTime);
            lastOLEDrefreshTime = millis();
            u8g2.setFont(u8g2_font_6x10_mr);
            sprintf(bufferStr4, "%02d", fpsOLED);
            u8g2.drawStr(180, 64, bufferStr4);

            u8g2.sendBuffer(); //更新至屏幕

            // 关闭灯条所有LED
            for (int i = 0; i < NUM_LEDS; i++)
                leds[i] = CRGB::Black;

            //开始计算速度
            if (setLEDtoSpeed == 1)
            {
                nShiftlightPos = intMapping(SPD, SPD_Display_MIN, SPD_Display_MAX, 0, NUM_LEDS);
            }
            else
            {
                nShiftlightPos = intMapping(RPM, RPM_Display_MIN, RPM_Display_MAX, 0, NUM_LEDS);
            }

            // Turn the LED on
            for (int i = 0; i < nShiftlightPos; i++)
            {
                if (nShiftlightPos >= RPM_Red_After)
                {
                    leds[i] = CRGB::Red;
                }
                else if (nShiftlightPos > RPM_Green_Before)
                {
                    leds[i] = CRGB::Blue;
                }
                else
                {
                    leds[i] = CRGB::Green;
                }
            }
            FastLED.show();
        }
        else
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
        vTaskDelay(1);                               // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
        while (Serial1.available())
            gps.encode(Serial1.read()); /* Get GPS data */

        gps_hdop = gps.hdop.hdop();
        gps_speed = gps.speed.kmph();
        gps_sat_count = gps.satellites.value();

        if (Serial2.available())
        {
            String input = Serial2.readStringUntil('\n'); // Read out string from the serial monitor
            cli.parse(input);                             // Parse the user input into the CLI
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
    }
}

void Task_UpdateData(void *pvParameters) // 测时速、转速、姿态、SD卡写入
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 8;
    xLastWakeTime = xTaskGetTickCount(); // 用当前时间初始化xLastWakeTime变量。

    int lastmSec = 0;

    int lastPulseCounter_SPD = 0;
    float SPD_freq_in_mHz = 0.0;
    float SPD_Calc_Factor = 105.3; //频率换算系数，计算方法见excel表

    int lastPulseCounter_RPM = 0;
    float RPM_freq_in_mHz = 0.0;
    int RPM_Calc_Factor = 60000; //频率换算系数

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // 等待下一个周期
        ArduinoOTA.handle();                         //OTA必须运行的检测语句

        pcnt_get_counter_value(PCNT_FREQ_UNIT_SPD, &PulseCounter_SPD); // get pulse counter value - maximum value is 16 bits
        pcnt_get_counter_value(PCNT_FREQ_UNIT_RPM, &PulseCounter_RPM); // get pulse counter value - maximum value is 16 bits
        // DEBUG_PRINTLN((millis() - lastmSec))
        SPD = SPD_Calc_Factor * (PulseCounter_SPD - lastPulseCounter_SPD) / (millis() - lastmSec);
        RPM = RPM_Calc_Factor * (PulseCounter_RPM - lastPulseCounter_RPM) / (millis() - lastmSec);
        lastPulseCounter_RPM = PulseCounter_RPM;
        lastPulseCounter_SPD = PulseCounter_SPD;
        lastmSec = millis();

        vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
    }
}

void Task_UpdateTime(void *pvParameters) //
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 999;

    // 用当前时间初始化xLastWakeTime变量。
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // 等待下一个周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        struct tm dt;
        if (I2C_is_Busy == false && DS3231isOK)
        {
            I2C_is_Busy = true;
            if (!rtc.read(&dt))
            {
                Serial.println(F("Read date/time failed"));
            }
            else
            {
                vTaskDelay(5);
                Serial.println(asctime(&dt));
            }
            I2C_is_Busy = false;
        }

        if (wifiNeverConnected == true)
        {
            // if (GPSconnected)
            // {
            //     //get time from gps
            // }

            if (WiFi.status() == WL_CONNECTED)
            {
                //init and get the time
                configTime(gmtOffset_sec, 0, ntpServer);
                timeSyncedFromNTP = true;
                Serial.println("Got Time From NTP server, Success");
                getLocalTime(&timeinfo);
                vTaskDelay(5);
                wifiNeverConnected = false;
                if (DS3231isOK)
                    setRTCTimeFromRAM();
            }
        }
        if (getLocalTime(&timeinfo))
        {
            // Serial.println(&timeinfo, "%F %T");
        }
        else
        {
            Serial.println("Failed to obtain time");
        }
        if (I2C_is_Busy == false)
        {
            I2C_is_Busy = true;
            int temp1 = gauge.readPercentage();
            if (temp1 <= 100 && temp1 > 0)
            {
                BTRYpercentage = temp1;
            }
            int temp2 = gauge.readVoltage();
            if (temp2 < 5000 && temp2 > 0)
            {
                BTRYvoltage = temp2;
            }
            vTaskDelay(3);
            I2C_is_Busy = false;
        }
    }
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int intMapping(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float floatMapping(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

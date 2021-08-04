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

            u8g2.clearBuffer();   //清空屏幕
            u8g2.setDrawColor(1); // White
            u8g2.setFontMode(0);

            u8g2.setFont(u8g2_font_5x7_tr);
            char PrinterStr[20];
            sprintf(PrinterStr, "X%04.0f Y%04.0f", GFx, GFy);
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

            u8g2.drawBox(GFx_OLED + 25, GFy_OLED + 36, 3, 3); //指示点 初始位置25,36,3,3

            char bufferStr2[2];
            sprintf(bufferStr2, "%02d", SPD);

            u8g2.setFont(u8g2_font_logisoso42_tn);
            u8g2.drawStr(98, 45, bufferStr2); //SPD文字显示

            u8g2.setFont(u8g2_font_logisoso18_tn);

            char bufferStr4[4];
            sprintf(bufferStr4, "%04d", RPM);

            u8g2.drawStr(210, 18, bufferStr4); //RPM文字显示

            u8g2.setFont(u8g2_font_6x10_mr);
            u8g2.drawStr(222, 25, "RPM");

            u8g2.setFont(u8g2_font_6x10_mr); //日期时间显示
            u8g2.drawStr(97, 55, "2021-04-24");
            u8g2.drawStr(103, 64, "18:05:05");

            if (wifi_connected)
                drawSignal(u8g2, 180, 12, 4); //满格信号，三格

            u8g2.setFont(u8g2_font_6x10_mr);
            u8g2.setCursor(180, 35);
            u8g2.print(gps.satellites.value());
            u8g2.setCursor(180, 44);
            u8g2.print(gps.hdop.hdop());
            u8g2.setCursor(180, 53);
            u8g2.print(gps.speed.kmph());

            // voltage = lipo.getVoltage();
            // // lipo.getSOC() returns the estimated state of charge (e.g. 79%)
            // soc = lipo.getSOC();
            // // lipo.getAlert() returns a 0 or 1 (0=alert not triggered)
            // alert = lipo.getAlert();

            // // Print the variables:
            // Serial.print("Voltage: ");
            // Serial.print(voltage); // Print the battery voltage
            // Serial.println(" V");

            // Serial.print("Percentage: ");
            // Serial.print(soc); // Print the battery state of charge
            // Serial.println(" %");

            // Serial.print("Alert: ");
            // Serial.println(alert);
            // Serial.println();

            u8g2.setFont(u8g2_font_siji_t_6x10);
            // u8g2.drawGlyph(x, y, 0xe242);   //empty
            // u8g2.drawGlyph(x, y, 0xe250);   //half
            u8g2.drawGlyph(194, 12, 0xe254); //full

            fpsOLED = 1000.0 / (millis() - lastOLEDrefreshTime);
            lastOLEDrefreshTime = millis();
            u8g2.setFont(u8g2_font_6x10_mr);
            sprintf(bufferStr4, "%04d", fpsOLED);
            u8g2.drawStr(150, 64, bufferStr4);

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

    // 用当前时间初始化xLastWakeTime变量。
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // 等待下一个周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
        /* Get GPS data */
        while (Serial1.available())
            gps.encode(Serial1.read());

        // printInt(gps.satellites.value(), , 5);
        // printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);

        if (Serial2.available())
        {
            // Read out string from the serial monitor
            String input = Serial2.readStringUntil('\n');

            // Parse the user input into the CLI
            cli.parse(input);
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
    const TickType_t xFrequency = 1;

    // 用当前时间初始化xLastWakeTime变量。
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // 等待下一个周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        ArduinoOTA.handle(); //OTA必须运行的检测语句

        pcnt_get_counter_value(PCNT_FREQ_UNIT_SPD, &PulseCounter_SPD); // get pulse counter value - maximum value is 16 bits
        pcnt_get_counter_value(PCNT_FREQ_UNIT_RPM, &PulseCounter_RPM); // get pulse counter value - maximum value is 16 bits

        SPD_freq_in_mHz = (PulseCounter_SPD - lastPulseCounter_SPD) / (millis() - lastmSec);
        RPM_freq_in_mHz = (PulseCounter_RPM - lastPulseCounter_RPM) / (millis() - lastmSec);
        lastPulseCounter_RPM = PulseCounter_RPM;
        lastPulseCounter_SPD = PulseCounter_SPD;
        lastmSec = millis();

        SPD = SPD_freq_in_mHz * SPD_Calc_Factor;
        RPM = RPM_freq_in_mHz * RPM_Calc_Factor;

        if (BNO055isOK)
        {
            bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
            GFy = linearAccelData.acceleration.z;
            GFx = linearAccelData.acceleration.y;
            //(float)linearAccelData.acceleration.y 是车前后
            //(float)linearAccelData.acceleration.z 是车左右
        }

        vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
    }
}

void Task_UpdateTime(void *pvParameters) // 测速任务
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1500;

    // 用当前时间初始化xLastWakeTime变量。
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // 等待下一个周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint8_t hour;
        uint8_t min;
        uint8_t sec;
        uint8_t mday;
        uint8_t mon;
        uint16_t year;
        uint8_t wday;
        if (!rtc.getDateTime(&hour, &min, &sec, &mday, &mon, &year, &wday))
        {
            Serial.println(F("Read date/time failed"));
            vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
        }
        DEBUG_PRINT(hour)
        DEBUG_PRINT(min)
        DEBUG_PRINT(sec)
        DEBUG_PRINT(year)
        DEBUG_PRINT(mon)
        DEBUG_PRINTLN(mday)
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

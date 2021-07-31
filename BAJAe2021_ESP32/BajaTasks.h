//本文件为后部Tasks任务具体调用的函数。

void drawSignal(U8G2 u8g2, uint8_t x, uint8_t y, uint8_t strength)
{
    for (uint8_t i = 0; i < strength; i++)
    {
        u8g2.drawCircle(x, y, i * 3, U8G2_DRAW_UPPER_RIGHT);
    }
}

void drawGForce() //x:-100~100
{
    u8g2.drawFrame(0, 11, 53, 53); //GForce
    //长宽都是53，一半的长度是26
    //X:0->26->52
    //y:11->37->63
    //中心的点是x26,y37
    u8g2.drawLine(0, 37, 52, 37);  //横向中心线
    u8g2.drawLine(26, 11, 26, 63); //纵向中心线
    //u8g2.drawFrame(13, 24, 26, 26); //小圈,方的
    u8g2.drawCircle(26, 37, 13, U8G2_DRAW_ALL); //小圈，圆的
    int GFx_OLED = GFx * 2;
    int GFy_OLED = GFy * 2;
    if (GFx_OLED > 26)
    {
        GFx_OLED = 26;
    }
    else if (GFx_OLED < -26)
    {
        GFx_OLED = -26;
    }
    if (GFy_OLED > 26)
    {
        GFy_OLED = 26;
    }
    else if (GFy_OLED < -26)
    {
        GFy_OLED = -26;
    }

    u8g2.drawBox(GFx_OLED + 25, GFy_OLED + 36, 3, 3); //指示点 初始位置25,36,3,3
}

void drawSuspension() //减振器信息绘制任务
{
    //4096->40
    //=x* 0.009765625

    u8g2.drawFrame(87, 0, 7, 40);    //LF
    u8g2.drawFrame(75, 23, 10, 40);  //LR
    u8g2.drawFrame(158, 0, 7, 40);   //RF
    u8g2.drawFrame(167, 23, 10, 40); //RR

    int hLF = SUS_LF * 0.009765625;
    int hRF = SUS_RF * 0.009765625;
    int hLR = SUS_LR * 0.009765625;
    int hRR = SUS_RR * 0.009765625;

    u8g2.drawBox(87, 40 - hLF, 7, hLF);   //LF full
    u8g2.drawBox(75, 64 - hLR, 10, hLR);  //LR
    u8g2.drawBox(158, 40 - hRF, 7, hRF);  //RF
    u8g2.drawBox(167, 64 - hRR, 10, hRR); //RR
}

void PrintToOLED(void *pvParameters) // OLED 刷新任务
{

    //所有字体列表：https://github.com/olikraus/u8g2/wiki/fntlistall
    //模拟器：https://p3dt.net/u8g2sim/

    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 200;
    //设置为100时，大约为10fps
    //设置为16时，大约48fps
    //40约为40fps
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
            if (BNO055isOK)
            {
                bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
                char PrinterStr[20];
                GFy = linearAccelData.acceleration.z;
                GFx = linearAccelData.acceleration.y;
                sprintf(PrinterStr, "X%04.0f Y%04.0f", GFx, GFy);
                u8g2.setFont(u8g2_font_5x7_tr);
                u8g2.drawStr(0, 6, PrinterStr);
                drawGForce();
            }
            //(float)linearAccelData.acceleration.y 是车前后
            //(float)linearAccelData.acceleration.z 是车左右

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

            u8g2.setFont(u8g2_font_siji_t_6x10);
            // u8g2.drawGlyph(x, y, 0xe242);   //empty
            // u8g2.drawGlyph(x, y, 0xe250);   //half
            u8g2.drawGlyph(194, 12, 0xe254); //full

            // drawSuspension();

            fpsOLED = 1000.0 / (millis() - lastOLEDrefreshTime);
            lastOLEDrefreshTime = millis();
            u8g2.setFont(u8g2_font_6x10_mr);
            sprintf(bufferStr4, "%04d", fpsOLED);
            u8g2.drawStr(150, 64, bufferStr4);

            u8g2.sendBuffer(); //更新至屏幕
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

void UpdateLEDstrip(void *pvParameters) // LED灯条刷新任务
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 60;

    // 用当前时间初始化xLastWakeTime变量。
    xLastWakeTime = xTaskGetTickCount();
    FastLED.setBrightness(10);
    for (;;)
    {
        // 等待下一个周期。
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // 关闭灯条所有LED
        for (int i = 0; i < NUM_LEDS; i++)
            leds[i] = CRGB::Black;

        //开始计算速度
        if (setLEDtoSpeed == 1)
        {
            nShiftlightPos = intMapping(RPM, SPD_Display_MIN, SPD_Display_MAX, 0, NUM_LEDS);
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

        vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
    }
}

// void getBNO055Data(void *pvParameters) // BNO055姿态任务
// {
//     (void)pvParameters;
//     TickType_t xLastWakeTime;
//     const TickType_t xFrequency = 100;

//     // 用当前时间初始化xLastWakeTime变量。
//     xLastWakeTime = xTaskGetTickCount();

//     for (;;)
//     {
//         // 等待下一个周期。
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//         /* 获取一个新的传感器事件 */
//         if (BNO055isOK)
//         {

//         }

//         vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
//     }
// }

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
        while (Serial2.available())
            gps.encode(Serial2.read());
    }
}

void handleOTAtask(void *pvParameters) // OTA更新任务

{
    (void)pvParameters;
    for (;;)
    {
        ArduinoOTA.handle(); //OTA必须运行的检测语句
        vTaskDelay(1);       // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
    }
}

void updateRevving(void *pvParameters) // demo数据刷新
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100;

    // 用当前时间初始化xLastWakeTime变量。
    xLastWakeTime = xTaskGetTickCount();

    RPM = 1700;
    for (;;)
    {
        // 等待下一个周期。
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (enableFakeData)
        {
            /* 获取一个新的传感器事件 */
            RPM += 35;
            SPD++;

            SUS_LF += 22;
            SUS_RF += 41;
            SUS_LR += 35;
            SUS_RR += 19;

            if (RPM > 3900)
                RPM = 1700;
            if (SPD > 60)
                SPD = 0;

            if (SUS_LF > 4094)
                SUS_LF = 0;
            if (SUS_RF > 4096)
                SUS_RF = 0;
            if (SUS_LR > 4095)
                SUS_LR = 0;
            if (SUS_RR > 4090)
                SUS_RR = 0;
        }

        vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
    }
}

void writeToSDCard(void *pvParameters) // SD卡写入任务
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
    }
}

void updateSPDdata(void *pvParameters) // 测速任务
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

        // DEBUG_PRINTLN(digitalRead(33))
        pcnt_get_counter_value(PCNT_FREQ_UNIT_SPD, &PulseCounter_SPD); // get pulse counter value - maximum value is 16 bits
        pcnt_get_counter_value(PCNT_FREQ_UNIT_RPM, &PulseCounter_RPM); // get pulse counter value - maximum value is 16 bits
        DEBUG_PRINTLN(PulseCounter_SPD)

        SPD_freq_in_mHz = (PulseCounter_SPD - lastPulseCounter_SPD) / (millis() - lastmSec);
        RPM_freq_in_mHz = (PulseCounter_RPM - lastPulseCounter_RPM) / (millis() - lastmSec);
        lastPulseCounter_RPM = PulseCounter_RPM;
        lastPulseCounter_SPD = PulseCounter_SPD;
        lastmSec = millis();

        SPD = SPD_freq_in_mHz * SPD_Calc_Factor;
        RPM = RPM_freq_in_mHz * RPM_Calc_Factor;
        DEBUG_PRINTLN(SPD)

        vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
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

void updatetimetonow()
{
    if (Serial.available())
    {
        GetDateStuff(Year, Month, Date, DoW, Hour, Minute, Second);

        Clock.setClockMode(false); // set to 24h

        Clock.setYear(Year);
        Clock.setMonth(Month);
        Clock.setDate(Date);
        Clock.setDoW(DoW);
        Clock.setHour(Hour);
        Clock.setMinute(Minute);
        Clock.setSecond(Second);
    }
}

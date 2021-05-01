
void drawSignal(U8G2 u8g2, uint8_t x, uint8_t y, uint8_t strength)
{
    for (uint8_t i = 0; i < strength; i++)
    {
        u8g2.drawCircle(x, y, i * 3, U8G2_DRAW_UPPER_RIGHT);
    }
}

void drawGForce() //x:-100~100
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

    u8g2.drawBox(25, 36, 3, 3); //指示点
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

        if (isOTAing == 0)
        {

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

            drawSignal(u8g2, 180, 12, 4); //满格信号，三格

            drawGForce();

            u8g2.setFont(u8g2_font_siji_t_6x10);
            // u8g2.drawGlyph(x, y, 0xe242);   //empty
            // u8g2.drawGlyph(x, y, 0xe250);   //half
            u8g2.drawGlyph(194, 12, 0xe254); //full

            drawSuspension();

            u8g2.sendBuffer(); //更新至屏幕

            fpsOLED = 1000.0 / (millis() - lastOLEDrefreshTime);
            Serial.println(fpsOLED);
            lastOLEDrefreshTime = millis();
        }
        else
        {
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

    for (;;)
    {
        // 等待下一个周期。
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        /* 获取一个新的传感器事件 */
        RPM++;
        SPD++;

        SUS_LF += 22;
        SUS_RF += 41;
        SUS_LR += 35;
        SUS_RR += 19;

        if (RPM > 3800)
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

        vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
    }
}

void UpdateLEDstrip(void *pvParameters) // LED灯条刷新任务
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 500;

    // 用当前时间初始化xLastWakeTime变量。
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // 等待下一个周期。
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // Turn the LED on, then pause
        for (int i = 0; i < NUM_LEDS; i++)
        {
            leds[i] = CRGB::Red;
        }
        FastLED.show();
        vTaskDelay(500);
        for (int i = 0; i < NUM_LEDS; i++)
        {
            leds[i] = CRGB::Green;
        }
        FastLED.show();
        vTaskDelay(500);
        for (int i = 0; i < NUM_LEDS; i++)
        {
            leds[i] = CRGB::Blue;
        }
        FastLED.show();
        vTaskDelay(500);
        // Now turn the LED off, then pause
        for (int i = 0; i < NUM_LEDS; i++)
        {
            leds[i] = CRGB::Black;
        }
        FastLED.show();
        vTaskDelay(500);

        vTaskDelay(1); // 两次读取之间有一个刻度延迟（15毫秒），以确保稳定性
    }
}


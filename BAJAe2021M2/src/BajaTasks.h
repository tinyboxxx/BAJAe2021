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
            if (BNO055isOK == true && I2C_is_Busy == false)
            {
                I2C_is_Busy = true;
                bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
                GFy = linearAccelData.acceleration.z;
                GFx = linearAccelData.acceleration.y;
                I2C_is_Busy = false;
                //(float)linearAccelData.acceleration.y 是车前后
                //(float)linearAccelData.acceleration.z 是车左右
            }

            u8g2.clearBuffer(); //清空屏幕
            Serial.println("oled clearbuffer");
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
            u8g2.drawCircle(26, 37, 13, U8G2_DRAW_ALL); //小圈,圆的
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
            u8g2.print(&time_in_RAM, "%F");
            u8g2.setCursor(103, 64);
            u8g2.print(&time_in_RAM, "%T");

            // if (WiFi.waitForConnectResult() != WL_CONNECTE)
            //     drawSignal(u8g2, 180, 12, 4); //满格信号，三格

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
            Serial.println("oled sendBuffer");
            if (setLEDtoSpeed == 1) //开始计算LED灯
            {
                nShiftlightPos = intMapping(SPD, SPD_Display_MIN, SPD_Display_MAX, 0, 12);
            }
            else
            {
                nShiftlightPos = intMapping(RPM, RPM_Display_MIN, RPM_Display_MAX, 0, 12);
            }

            for (int i = 0; i < 12; i++) // Turn the LED on
            {
                if (i <= nShiftlightPos)
                {
                    mcp.digitalWrite(i, HIGH);
                }
                else
                {
                    mcp.digitalWrite(i, LOW);
                }
            }
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

    int mSec = 0;
    int lastmSec = 0;

    int PulseCounter_SPD;
    int PulseCounter_RPM;
    int lastPulseCounter_SPD = 0;
    float SPD_freq_in_mHz = 0.0;
    float SPD_Calc_Factor = 105.3; //频率换算系数，计算方法见excel表

    int lastPulseCounter_RPM = 0;
    float RPM_freq_in_mHz = 0.0;
    int RPM_Calc_Factor = 60000; //频率换算系数

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // 等待下一个周期
        // ArduinoOTA.handle();                         //OTA必须运行的检测语句

        // PulseCounter_SPD = (int32_t)encoder_speed.getCount();
        // PulseCounter_RPM = (int32_t)encoder_rpm.getCount();

        // DEBUG_PRINTLN((millis() - lastmSec))
        mSec = millis();
        SPD = SPD_Calc_Factor * (PulseCounter_SPD - lastPulseCounter_SPD) / (mSec - lastmSec);
        RPM = RPM_Calc_Factor * (PulseCounter_RPM - lastPulseCounter_RPM) / (mSec - lastmSec);
        lastPulseCounter_RPM = PulseCounter_RPM;
        lastPulseCounter_SPD = PulseCounter_SPD;
        lastmSec = mSec;

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
        // if (I2C_is_Busy == false && DS3231isOK && timeSyncedFromNTP == false && timeSyncedFromGPS == false && timeSyncedFromRTC == false) //读取RTC的时间
        // {
        //     RTCtoRAM();
        // }

        // if (wifiNeverConnected == true)
        // {
        //     // if (GPSconnected)
        //     // {
        //     //     //get time from gps
        //     // }
        // }
        // if (getLocalTime(&time_in_RAM)) // update time From ESP32 to RAM
        // {
        //     Serial.println(&time_in_RAM, "%F %T");
        // }
        // else
        // {
        //     Serial.println("Failed to obtain time");
        // }

        // BTRYvoltage=analogRead(35)/4095*3.3*2;
        BTRYvoltage = analogRead(35) * 0.0016117;
        BTRYpercentage = floatMapping(BTRYvoltage, 2.8, 3.6, 0, 100);
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

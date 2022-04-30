// CLI ====================================
#include <SimpleCLI.h>   // Inlcude Library
SimpleCLI cli;           // Create CLI Object
void turnoffwifi(cmd *c) // Callback function
{
    Command cmd(c); // Create wrapper object
    WiFi.mode(WIFI_OFF);
}
void turnonwifi(cmd *c) // Callback function
{
    Command cmd(c); // Create wrapper object
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    ArduinoOTA.begin();
}
void printip(cmd *c) // 输出IP
{
    Command cmd(c); // Create wrapper object
    TELL_EVERYONE_LN("IP:")
    TELL_EVERYONE(WiFi.localIP())
}

void loralowpower(cmd *c) // Callback function
{
    Command cmd(c); // Create wrapper object
    lora_power_mode = 2;
}
void lorahighpower(cmd *c) // Callback function
{
    Command cmd(c); // Create wrapper object
    lora_power_mode = 0;
}

void i2cscan(cmd *c) //扫描i2c设备
{
    Command cmd(c); // Create wrapper object
    Serial.println();
    Serial.println("I2C scanner. Scanning ...");
    byte count = 0;

    for (byte i = 8; i < 120; i++)
    {
        Wire.beginTransmission(i);       // Begin I2C transmission Address (i)
        if (Wire.endTransmission() == 0) // Receive 0 = success (ACK response)
        {
            Serial.print("Found address: ");
            Serial.print(i, DEC);
            Serial.print(" (0x");
            Serial.print(i, HEX); // PCF8574 7 bit address
            Serial.println(")");
            count++;
        }
    }
    Serial.print("Found ");
    Serial.print(count, DEC); // numbers of devices
    Serial.println(" device(s).");
}

void reboot(cmd *c)
{
    Command cmd(c); // Create wrapper object
    TELL_EVERYONE_LN("rebooting!")
    ESP.restart();
}
void cmd_gpstime(cmd *c)
{
    Command cmd(c); // Create wrapper object
    printGpsTime(); //读取GPS时间
}

void cmd_sendtele(cmd *c)
{
    Command cmd(c); // Create wrapper object
    sendtele = 1; // 发送信息
}
void cmd_teleoff(cmd *c)
{
    Command cmd(c); // Create wrapper object
    sendtele = 0; // 发送信息
}

void settime(cmd *c) // 格式为 settime 2022,04,29,22,27,24
{
    int ind1; // , locations
    int ind2;
    int ind3;
    int ind4;
    int ind5;
    int ind6;

    int year, mon, day, hour, min, sec = 0;

    Command cmd(c);                                     // Create wrapper object
    Argument arg = cmd.getArgument(0);                  // Get first (and only) Argument
    String argVal = arg.getValue();                     // Get value of argument
    ind1 = argVal.indexOf(',');                         //finds location of first ,
    year = argVal.substring(0, ind1).toInt();           //captures first data String
    ind2 = argVal.indexOf(',', ind1 + 1);               //finds location of second ,
    mon = argVal.substring(ind1 + 1, ind2 + 1).toInt(); //captures second data String
    ind3 = argVal.indexOf(',', ind2 + 1);
    day = argVal.substring(ind2 + 1, ind3 + 1).toInt();
    ind4 = argVal.indexOf(',', ind3 + 1);
    hour = argVal.substring(ind3 + 1, ind4 + 1).toInt();
    ind5 = argVal.indexOf(',', ind4 + 1);
    min = argVal.substring(ind4 + 1, ind5 + 1).toInt();
    ind6 = argVal.indexOf(',', ind5 + 1);
    sec = argVal.substring(ind5 + 1).toInt();

    rtc_builtin.setTime(sec, min, hour, day, mon, year);
    // DEBUG_PRINTLN(sec)
    // DEBUG_PRINTLN(min)
    // DEBUG_PRINTLN(hour)
    // DEBUG_PRINTLN(day)
    // DEBUG_PRINTLN(mon)
    // DEBUG_PRINTLN(year)
    // TELL_EVERYONE_LN("time set")
    printRAMtime();
    RAMtoRTC();
}

void errorCallback(cmd_error *e) // Callback in case of an error
{
    CommandError cmdError(e); // Create wrapper object
    Serial.print("ERROR: ");
    Serial2.print("ERROR: ");
    Serial.println(cmdError.toString());
    Serial2.println(cmdError.toString());
    if (cmdError.hasCommand())
    {
        Serial2.print("Did you mean \"");
        Serial2.print(cmdError.getCommand().toString());
        Serial2.println("\"?");

        Serial.print("Did you mean \"");
        Serial.print(cmdError.getCommand().toString());
        Serial.println("\"?");
    }
}

void cmd_fpson(cmd *c)
{
    Command cmd(c); // Create wrapper object
    fps_on = true;
}
void cmd_fpsoff(cmd *c)
{
    Command cmd(c); // Create wrapper object
    fps_on = false;
}

void cmd_calcpson(cmd *c)
{
    Command cmd(c); // Create wrapper object
    fps_calc_on = true;
}
void cmd_calcpsoff(cmd *c)
{
    Command cmd(c); // Create wrapper object
    fps_calc_on = false;
}

void cmd_setfps(cmd *c) //将传入的数据赋值给setfps
{
    Command cmd(c); // Create wrapper object
    Argument arg = cmd.getArgument(0); // Get first (and only) Argument
    String argVal = arg.getValue();    // Get value of argument
    setfps_tick = argVal.toInt();
}

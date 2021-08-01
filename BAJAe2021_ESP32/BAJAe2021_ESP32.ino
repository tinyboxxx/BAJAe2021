// todo: 切换到adafruit ds3231、测试GPS

#include "BajaHeaders.h" //引用和初始化部分。

// Stats 状态变量 ====================================

bool setLEDtoSpeed = false; // 1:SPEED 0:RPM
bool enableFakeData = true; //开启假数据

unsigned int fpsOLED = 0;
long lastOLEDrefreshTime = 0;
int fpsGPS = 0;

bool wifi_connected = false;
bool isOTAing = false;
unsigned int OTAprogress = 0;
unsigned int OTAtotal = 0;

bool debug_Using_USB = true;
bool debug_Using_LORA = true;

//行车数据变量=============================
unsigned int RPM = 0;
unsigned int SPD = 0; //千米每小时

unsigned int SUS_LF = 0; //减振器，ADC值，0->4096
unsigned int SUS_RF = 0;
unsigned int SUS_LR = 0;
unsigned int SUS_RR = 0;

float GFx = 0;
float GFy = 0;

void setup(void)
{
  pinMode(32, INPUT_PULLUP); //rpm 引脚
  pinMode(33, INPUT_PULLUP); //SPD 引脚
  initPulseCounter_RPM();
  initPulseCounter_SPD();
  Serial.begin(115200);
  Serial.println("Booting");

  cli.setOnError(errorCallback); // Set error Callback
  // Create the cowsay command with callback function
  // A single argument command has only one argument
  // in which the complete string is saved, that comes after the command name.
  // Spaces, which usually seperate arguments, will be ignored and any value will be accespted as valid.
  // For example: cowsay "Hello there!"
  //              cowsays Hello there!
  updatetime = cli.addSingleArgCmd("updatetime", update_time_to_given_time);

  // WIFIOTA ====================================

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    wifi_connected = 0;
    Serial.println("Connection Failed!");
  }
  else
  {
    wifi_connected = 1;
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
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }

  // I2C ====================================
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  // TIME ====================================

  // BTY
  lipo.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  // Set up the MAX17043 LiPo fuel gauge:
  if (lipo.begin() == false) // Connect to the MAX17043 using the default wire port
  {
    Serial.println(F("MAX17043 not detected. Please check wiring"));
  }

  // Quick start restarts the MAX17043 in hopes of getting a more accurate
  // guess for the SOC.
  lipo.quickStart();

  // We can set an interrupt to alert when the battery SoC gets too low.
  // We can alert at anywhere between 1% - 32%:
  lipo.setThreshold(20); // Set alert threshold to 20%.

  // GPS ====================================
  Serial1.begin(9600, SERIAL_8N1, Serial1_RXPIN, Serial1_TXPIN); // GPS com
  // LORA ====================================
  Serial2.begin(9600, SERIAL_8N1, Serial2_RXPIN, Serial2_TXPIN); // GPS com
  Serial2.println("hi???");

  // BNO055姿态 ====================================
  if (!bno.begin()) // 初始化传感器
  {
    Serial.print("ERR no BNO055 detected"); // 检测BNO055时出现问题...请检查您的连接
    BNO055isOK = false;
  }
  bno.setExtCrystalUse(true); //使用外部晶振以获得更好的精度

  // OLED屏幕 ====================================
  u8g2.begin();

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

  xTaskCreatePinnedToCore(PrintToOLED, "PrintToOLED" // 屏幕刷新任务
                          ,
                          30000, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(getGPSData, "getGPSData" // GPS数据更新任务
                          ,
                          16384, NULL, 0, NULL, 1);
  xTaskCreatePinnedToCore(handleOTAtask, "handleOTAtask" // 在线更新固件任务
                          ,
                          2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(updateRevving, "updateRevving" // 演示模式任务
                          ,
                          1024, NULL, 0, NULL, 1);
  xTaskCreatePinnedToCore(UpdateLEDstrip, "UpdateLEDstrip" // 灯条更新任务
                          ,
                          2048, NULL, 1, NULL, 1);

  xTaskCreatePinnedToCore(updateSPDdata, "updateSPDdata" // 测速任务
                          ,
                          4000, NULL, 1, NULL, 1);
  // xTaskCreatePinnedToCore(updateDateTime, "updateDateTime" // 时间更新任务
  //                         ,
  //                         2000, NULL, 1, NULL, 1);
}

void loop(void)
{
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
  vTaskDelay(1); //延时，或许以后删掉
}

#include "BajaTasks.h"

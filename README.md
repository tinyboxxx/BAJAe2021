# BAJAe2021

## ToDo

- [x] PCB尺寸调整，OLED下移
- [x] 手动RST按钮增加
- [x] 增加RTC及电池
- [x] 灯条，GPS代码
- [x] I2C引脚修改
- [x] GForce显示代码
- [x] USB Type-C下载
- [x] 锂电池充电管理
- [x] 优化Tasks
- [ ] 转速实测
- [x] 时速实测
- [ ] SD卡再次尝试，增加记录功能
- [ ] 电池，RTC再次尝试
- [ ] 继续增加串口命令
- [ ] 传动比计算并显示
- [ ] 45米加速计算功能

## 测量

转速、时速、里程、GPS（10Hz以上）、姿态传感器：BNO055、电池电压检测、RTC时间、~~减振器（4个模拟量）~~、~~制动压力传感器~~、~~温度~~

| 功能       | 模块/IC        | 实现    | 引脚          | 实测     | 理想更新频率 |
| ---------- | -------------- | ------- | ------------- | -------- | ------------ |
| 转速       | -              | Pulse   | IO32          | 还未测试 | 50hz       |
| 时速       | -              | Pulse   | IO33          | ✔️ | 50hz       |
| 姿态传感器 | BNO055/MPU6050 | i2c     |               | ✔️        | 50hz       |
| SD卡       | -              | SPI     | CS_25         | ❌        | 50hz     |
| 屏幕       | SSD1322        | SPI     | CS_IO5        | ✔️        | 30hz         |
| 灯条       | APA102C        | Digital | SDI_26,CKI_27 | ✔️        | 30hz         |
| GPS模块    | ATGM336H       | serial1 | TX22,RX21     | ✔️        | 10hz       |
| Lora       | E32-433T20DC   | Serial2 | TX_17,RX16    | ✔️        | 10hz         |
| 电池电量   | MAX17043       | i2c     |               | 还未测试    | 0.1hz        |
| RTC时间    | ds3231/ds1307  | i2c     |               | ❌        | 仅开机1次    |
| 电池充电   | IP5306         | -       | -             | ✔️        | -            |


|i2c设备|地址|手册中的最大通信速度|
|---|---|---|
|BNO055|41 (0x29)|400 kHz|
|MAX17043|54 (0x36)|400 kHz|
|ds3231|104 (0x68)|Fast mode 400 kHz, Standard 100 kHz|



## 时间

- [ ] RTC同步至内存
- [x] NTP服务器同步至内存
- [ ] GPS同步至内存
- [ ] 内存同步至RTC

## 计算 

-   [ ] 45米加速时间
-   [ ] 传动比计算。
-   [ ] ~~减振器最大最小范围。~~

## 显示

-   [x] 屏幕、灯条显示转速

## 设置

-   [ ] 充电管理

-   [ ] 屏幕显示内容设置

-   [ ] 按钮功能设置

## 通信

-   [x] Lora通信，上位机管理

## 记录

- [ ] SD卡记录

## 电池

锂聚合物电池。充放电管理、测量

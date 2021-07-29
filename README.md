# BAJAe2021

## ToDo

- [x] PCB尺寸调整，OLED下移
- [x] 手动RST按钮增加
- [x] 增加RTC及电池
- [x] 灯条，GPS代码
- [x] I2C引脚修改
- [ ] GForce显示代码
- [ ] 继续测试：USBC、下载、锂电池充放电管理

## 测量
转速、时速、里程、GPS（10Hz以上）、姿态传感器：BNO055、电池电压检测、RTC时间、减振器（4个模拟量）、制动压力传感器、温度

| 功能       | 模块/IC       | 实现    | 引脚 |
| ---------- | ------------- | ------- | --- |
| 转速       |               | Pulse   | IO32 |
| 时速       |               | Pulse   | IO33 |
| GPS模块    | ATGM336H      | serial1 | TX22,RX21 |
| 姿态传感器 | BNO055/MPU6050 | i2c     |
| 电池电量   | MAX17043      | i2c     |
| RTC时间    | ds3231/ds1307 | i2c     |
| 屏幕       | SSD1322       | SPI     | CS_IO5 |
| 灯条       | APA102C       | Digital | SDI_26,CKI_27 |
| SD卡       |               | SPI     | CS_25 |
| Lora       |               | Serial2 | TX_17,RX16 |

## 计算
45米加速时间、传动比计算。减振器最大最小范围。
## 显示
屏幕、灯条显示转速、
## 设置
充电管理、屏幕显示内容设置，按钮功能设置
## 通信
Lora通信，上位机管理
## 记录
SD卡记录
## 电池
锂聚合物电池。充放电管理、测量

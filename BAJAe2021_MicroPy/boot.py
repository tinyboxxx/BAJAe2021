# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
#import webrepl
#webrepl.start()

import ssd1322
import machine, sdcard, os
from machine import SPI, Pin, SoftI2C
from DS3231 import DS3231

spi = SPI(2, baudrate=16000000,polarity=0, phase=0, sck=Pin(18), mosi=Pin(23), miso=Pin(19))
dc=Pin(12,Pin.OUT)
cs_OLED=Pin(5,Pin.OUT)
res=Pin(14,Pin.OUT)
display=ssd1322.SSD1322_SPI(256,64,spi,dc,cs_OLED,res)
display.fill(0)
# display.rotate(True)
display.line(5,5,60,60,0xff)
display.text("HELLOWORLD1", 10, 10, col=0xff)

display.show()

# https://github.com/peterhinch/micropython-samples/tree/master/DS3231
i2c = SoftI2C(scl=Pin(4), sda=Pin(13))
ds3231 = DS3231(i2c)
ds3231.get_time()


#https://docs.micropython.org/en/latest/library/machine.SDCard.html

sd = sdcard.SDCard(spi, Pin(25))
#共用spi，25是CS的引脚

os.mount(sd, '/sd')




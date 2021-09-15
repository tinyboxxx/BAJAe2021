#include "Wire.h"

void setup()
{
#define I2C_SDA 33
#define I2C_SCL 4
  Wire.begin(I2C_SDA, I2C_SCL);
  // set I/O pins to outputs
  Wire.beginTransmission(0x20);
  Wire.write(0x00); // IODIRA register
  Wire.write(0x00); // set all of port A to outputs
  Wire.endTransmission();
}

void loop()
{
  Wire.beginTransmission(0x20);
  Wire.write(0x12); // address bank A
  Wire.write((byte)0xAA); // value to send - all HIGH
  Wire.endTransmission();
  delay(500);
  Wire.beginTransmission(0x20);
  Wire.write(0x12); // address bank A
  Wire.write((byte)0x00); // value to send - all HIGH
  Wire.endTransmission();
  delay(500);
}

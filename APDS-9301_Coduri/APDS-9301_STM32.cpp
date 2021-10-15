#include <Arduino.h>
#include <Wire.h>

class APDS9301 {
public:
  bool begin(int addr = 0x29) {
    Wire.begin();
    i2cAddr = addr;
    if (!setPower(true)) {
      return false;
    }
    if (!setTiming(true, 0)) {
      return false;
    }
    return true;
  }

  bool setPower(bool on) {
    byte regVal = (on ? REG_CONTROL_POWER_ON : 0);

    return setReg(REG_CONTROL, regVal);
  }

  bool setTiming(bool highGain, int integration) {
    byte regVal;

    if (highGain) {
      regVal = REG_TIMING_GAIN_16;
      gain = 1;
    }
    else {
      regVal = REG_TIMING_GAIN_1;
      gain = 1 / 16.0;
    }
    switch (integration) {
    case 0:
      regVal |= REG_TIMING_INTEGRATE_13_7MS;
      this->integration = TIMING_SCALE_13_7MS;
      break;
    case 1:
      regVal |= REG_TIMING_INTEGRATE_101MS;
      this->integration = TIMING_SCALE_101MS;
      break;
    default:
      regVal |= REG_TIMING_INTEGRATE_402MS;
      this->integration = TIMING_SCALE_402MS;
      break;
    }
    return setReg(REG_TIMING, regVal);
  }

  bool readLux(float *lux) {
    uint16_t ch0, ch1;
    if (!getReg16(REG_DATA0LOW, &ch0) || !getReg16(REG_DATA1LOW, &ch1)) {
      return false;
    }
    *lux = calcLux(ch0, ch1);
    return true;
  }

private:
  bool setReg(int reg, byte val) {
    Wire.beginTransmission(i2cAddr);
    int size = Wire.write(reg | REG_COMMAND_CMD);
    if (size == 1) {
      Wire.write(val);
    }
    if ((Wire.endTransmission() != 0) || (size != 1)) {
      return false;
    }
    return true;
  }

  bool getReg16(int reg, uint16_t *val) {
    Wire.beginTransmission(i2cAddr);
    int size = Wire.write(reg | REG_COMMAND_CMD | REG_COMMAND_WORD);
    if ((Wire.endTransmission() != 0) || (size != 1)) {
      return false;
    }
    if (Wire.requestFrom(i2cAddr, 2) != 2) {
      return false;
    }
    byte lsb = Wire.read();
    *val = Wire.read();
    *val = (*val << 8) | lsb;
    return true;
  }

  float calcLux(uint16_t ch0, uint16_t ch1) {
    float ch0f = ch0 / gain / integration;
    float ch1f = ch1 / gain / integration;

    if (ch0f == 0) {
      return 0;
    }
    float d = ch1f / ch0f;
    if (d <= 0.5) {
      return (0.0304 * ch0f - 0.062 * ch0f * pow(d, 1.4));
    }
    else if (d <= 0.61) {
      return (0.0224 * ch0f - 0.031 * ch1f);
    }
    else if (d <= 0.8) {
      return (0.0128 * ch0f - 0.0153 * ch1f);
    }
    else if (d <= 1.3) {
      return (0.00146 * ch0f - 0.00112 * ch1f);
    }
    else {
      return 0;
    }
  }

  static const byte REG_CONTROL = 0x00;
  static const byte REG_TIMING = 0x01;
  static const byte REG_THRESHLOWLOW = 0x02;
  static const byte REG_THRESHLOWHIGH = 0x03;
  static const byte REG_THRESHHIGHLOW = 0x04;
  static const byte REG_THRESHHIGHHIGH = 0x05;
  static const byte REG_INTERRUPT = 0x06;
  static const byte REG_CRC = 0x08;
  static const byte REG_ID = 0x0A;
  static const byte REG_DATA0LOW = 0x0C;
  static const byte REG_DATA0HIGH = 0x0D;
  static const byte REG_DATA1LOW = 0x0E;
  static const byte REG_DATA1HIGH = 0x0F;

  static const byte REG_COMMAND_CMD = 1 << 7;
  static const byte REG_COMMAND_CLEAR = 1 << 6;
  static const byte REG_COMMAND_WORD = 1 << 5;

  static const byte REG_CONTROL_POWER_ON = 0x03;

  static const byte REG_TIMING_GAIN_1 = 0 << 4;
  static const byte REG_TIMING_GAIN_16 = 1 << 4;
  static const byte REG_TIMING_START_CYCLE = 1 << 3;
  static const byte REG_TIMING_INTEGRATE_13_7MS = 0;
  static const byte REG_TIMING_INTEGRATE_101MS = 1;
  static const byte REG_TIMING_INTEGRATE_402MS = 2;

  static constexpr float TIMING_SCALE_13_7MS = 0.034;
  static constexpr float TIMING_SCALE_101MS = 0.252;
  static constexpr float TIMING_SCALE_402MS = 1;

  int i2cAddr;
  float gain;
  float integration;
};

APDS9301 apds9301;

void setup() {
  Serial.begin(115200);
  apds9301.begin();
}

void loop() {
  float lux;

  delay(1000);
  if (apds9301.readLux(&lux)) {
    Serial.println("Luminosity: " + String(lux) + " lux");
  }
  else {
    Serial.println("Cannot measure luminosity!");
  }
}

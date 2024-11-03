#include "../include/dfr_10_dof_pkg/VCM5883L.h"

VCM5883L::VCM5883L(uint8_t I2C_addr)
  : _i2cBus(I2C_addr)
  {
  minX  = 0;
  maxX  = 0;
  minY  = 0;
  maxY  = 0;
  minZ  = 0;
  maxZ  = 0;
  firstRun = true;
  this->_I2C_addr = I2C_addr;
}

void VCM5883L::init(void)
{
  _i2cBus.writemem(VCM5883L_CTR_REG1, 0X00);
  _i2cBus.writemem(VCM5883L_CTR_REG2, 0X4D);
}

sVector_t VCM5883L::readRaw(void)
{
  uint8_t vha, vla;
  _i2cBus.readmem(VCM5883L_REG_OUT_X_H, uint8_t(1), _buff);
  vha = _buff[0];
  _i2cBus.readmem(VCM5883L_REG_OUT_X_L, uint8_t(1), _buff);
  vla = _buff[0];
  v.XAxis = (((int16_t)vha) << 8) | vla;

  _i2cBus.readmem(VCM5883L_REG_OUT_Y_H, uint8_t(1), _buff);
  vha = _buff[0];
  _i2cBus.readmem(VCM5883L_REG_OUT_Y_L, uint8_t(1), _buff);
  vla = _buff[0];
  v.YAxis = (((int16_t)vha) << 8) | vla;

  _i2cBus.readmem(VCM5883L_REG_OUT_Z_H, uint8_t(1), _buff);
  vha = _buff[0];
  _i2cBus.readmem(VCM5883L_REG_OUT_Z_L, uint8_t(1), _buff);
  vla = _buff[0];
  v.ZAxis = (((int16_t)vha) << 8) | vla;

  // _i2cBus.readmem(VCM5883L_REG_OUT_X_L, uint8_t(2), _buff);
  // v.XAxis = (((int16_t)_buff[1]) << 8) | _buff[0];
  // _i2cBus.readmem(VCM5883L_REG_OUT_Y_L, uint8_t(2), _buff);
  // v.YAxis = (((int16_t)_buff[1]) << 8) | _buff[0];
  // _i2cBus.readmem(VCM5883L_REG_OUT_Z_L, uint8_t(2), _buff);
  // v.ZAxis = (((int16_t)_buff[1]) << 8) | _buff[0];
  // v.AngleXY = (atan2((double)v.YAxis,(double)v.XAxis) * (180 / 3.14159265) + 180);
  // v.AngleXZ = (atan2((double)v.ZAxis,(double)v.XAxis) * (180 / 3.14159265) + 180);
  // v.AngleYZ = (atan2((double)v.ZAxis,(double)v.YAxis) * (180 / 3.14159265) + 180);
  return v;
}


void VCM5883L::setSamples(eSamples_t samples)
{
  uint8_t value;

  _i2cBus.readmem(QMC5883_REG_CONFIG_1, uint8_t(1), _buff);
  value = _buff[0];
  value &= 0x3f;
  value |= (samples << 6);
  _i2cBus.writemem(QMC5883_REG_CONFIG_1, value);
}

eSamples_t VCM5883L::getSamples(void)
{
  uint8_t value;

  _i2cBus.readmem(QMC5883_REG_CONFIG_1, uint8_t(1), _buff);
  value = _buff[0];
  value &= 0x3f;
  value >>= 6;

  return (eSamples_t)value;
}

// Only returns QMC5883_RANGE_8GA
eRange_t VCM5883L::getRange(void)
{
  return static_cast<eRange_t>(QMC5883_RANGE_8GA);
}

void VCM5883L::setMeasurementMode(eMode_t mode)
{
  uint8_t value;
  _i2cBus.readmem(VCM5883L_CTR_REG2, uint8_t(1), _buff);
  value = _buff[0];
  value &= 0xFE;
  value |= mode;
  _i2cBus.writemem(VCM5883L_CTR_REG2, value);
}

eMode_t VCM5883L::getMeasurementMode(void)
{
  uint8_t value=0;
  _i2cBus.readmem(VCM5883L_CTR_REG2, uint8_t(1), _buff);
  value = _buff[0];
  value &= 0b00000001;  
  return (eMode_t)value;
}

void VCM5883L::setDataRate(eDataRate_t dataRate)
{
  uint8_t value;
  _i2cBus.readmem(VCM5883L_CTR_REG2, uint8_t(1), _buff);
  value = _buff[0];
  value &= 0xf3;
  value |= (dataRate << 2);
  _i2cBus.writemem(VCM5883L_CTR_REG2, value);
}

eDataRate_t VCM5883L::getDataRate(void)
{
  uint8_t value=0;
  _i2cBus.readmem(VCM5883L_CTR_REG2, uint8_t(1), _buff);
  value = _buff[0];
  value &= 0b00001100;
  value >>= 2;
  return (eDataRate_t)value;
}

void VCM5883L::setDeclinationAngle(float declinationAngle)
{
  this->ICdeclinationAngle = declinationAngle;
}

float VCM5883L::getHeadingDegrees(void)
{
  float heading = atan2(static_cast<float>(v.YAxis) - Y_offset_, static_cast<float>(v.XAxis) - X_offset_);
  heading += this->ICdeclinationAngle;
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  v.HeadingDegress = heading * 180/PI;
  return v.HeadingDegress;
}

// void VCM5883L::writeRegister8(uint8_t reg, uint8_t value)
// {
//   _pWire->beginTransmission(this->_I2C_addr);
//   #if ARDUINO >= 100
//     _pWire->write(reg);
//     _pWire->write(value);
//   #else
//     _pWire->send(reg);
//     _pWire->send(value);
//   #endif
//     _pWire->endTransmission();
// }

// uint8_t VCM5883L::fastRegister8(uint8_t reg)
// {
//   uint8_t value=0;
//   _pWire->beginTransmission(this->_I2C_addr);
//   #if ARDUINO >= 100
//     _pWire->write(reg);
//   #else
//     _pWire->send(reg);
//   #endif
//   _pWire->endTransmission();
//   _pWire->requestFrom((uint8_t)this->_I2C_addr, (uint8_t)1);
//   #if ARDUINO >= 100
//     value = _pWire->read();
//   #else
//     value = _pWire->receive();
//   #endif
//   _pWire->endTransmission();
//   return value;
// }


// uint8_t VCM5883L::readRegister8(uint8_t reg)
// {
//   uint8_t value=0;
//   _pWire->beginTransmission(this->_I2C_addr);
//   #if ARDUINO >= 100
//     _pWire->write(reg);
//   #else
//     _pWire->send(reg);
//   #endif
//   _pWire->endTransmission();
//   _pWire->requestFrom((uint8_t)this->_I2C_addr, (uint8_t)1);
//   while(!_pWire->available()) {};
//   #if ARDUINO >= 100
//     value = _pWire->read();
//   #else
//     value = _pWire->receive();
//   #endif
//   return value;
// }

// int16_t VCM5883L::readRegister16(uint8_t reg)
// {
//   int16_t value=0;
//   uint8_t vha,vla;
//   _pWire->beginTransmission(this->_I2C_addr);
//   #if ARDUINO >= 100
//     _pWire->write(reg);
//   #else
//     _pWire->send(reg);
//   #endif
//   _pWire->endTransmission();
//   _pWire->requestFrom((uint8_t)this->_I2C_addr, (uint8_t)2);
//   while(!_pWire->available()) {};
//   if(ICType == IC_HMC5883L){
//     #if ARDUINO >= 100
//       vha = _pWire->read();
//       vla = _pWire->read();
//     #else
//       vha = _pWire->receive();
//       vla = _pWire->receive();
//     #endif
//   }else{
//     #if ARDUINO >= 100
//       vla = _pWire->read();
//       vha = _pWire->read();
//     #else
//       vla = _pWire->receive();
//       vha = _pWire->receive();
//     #endif
//   }
//   value = vha << 8 | vla;
//   return value;
// }

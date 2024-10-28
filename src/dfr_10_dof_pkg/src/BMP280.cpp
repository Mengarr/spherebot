/*!
 * @file DFRobot_BMP280.cpp
 * @brief Provides an Arduino library for reading and interpreting Bosch BMP280 data over I2C. 
 * @n Used to read current temperature, air pressure and calculate altitude.
 *
 * @copyright   
 * @license     The MIT License (MIT)
 * @version     V1.0
 * @date        2019-03-12
 * @url         https://github.com/DFRobot/DFRobot_BMP280
 */

#include "../include/dfr_10_dof_pkg/BMP280.hpp"
#include <cstdint>

// CTRL_MEAS register bit masks
constexpr uint8_t CTRL_MEAS_MODE_MASK   = 0x03; // bits 1:0
constexpr uint8_t CTRL_MEAS_OSRS_P_MASK = 0x1C; // bits 4:2
constexpr uint8_t CTRL_MEAS_OSRS_T_MASK = 0xE0; // bits 7:5

// CONFIG register bit masks
constexpr uint8_t CONFIG_SPI3W_EN_MASK = 0x80; // bit 7
constexpr uint8_t CONFIG_RESERVED1_MASK = 0x40; // bit 6
constexpr uint8_t CONFIG_FILTER_MASK    = 0x38; // bits 5:3
constexpr uint8_t CONFIG_T_SB_MASK      = 0x07; // bits 2:0

// BMP280 Register default values
constexpr uint8_t BMP280_REG_CHIP_ID_DEFAULT = 0x58;

// Constructor
DFRobot_BMP280::DFRobot_BMP280(uint8_t dev_addr)
    : _dev_addr(dev_addr), _i2cBus(dev_addr), lastOperateStatus(eStatus_t::Error)
{
}

// Begin function
DFRobot_BMP280::eStatus_t DFRobot_BMP280::begin()
{
  uint8_t chip_id = getReg(static_cast<uint8_t>(BMP280_Register::CHIP_ID));
  if (chip_id == BMP280_REG_CHIP_ID_DEFAULT) {
    reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    getCalibrate();
    setCtrlMeasSamplingPress(eSampling_t::X8);
    setCtrlMeasSamplingTemp(eSampling_t::X8);
    setConfigFilter(eConfigFilter_t::Off);
    setConfigTStandby(eConfigTStandby_t::_125);
    setCtrlMeasMode(eCtrlMeasMode_t::Normal); // Apply settings
    lastOperateStatus = eStatus_t::OK;
  } else {
    lastOperateStatus = eStatus_t::DeviceNotDetected;
  }
  return lastOperateStatus;
}

// Get temperature
float DFRobot_BMP280::getTemperature()
{
  int32_t raw = getTemperatureRaw();
  float rslt = 0.0f;
  int32_t v1, v2;
  if (lastOperateStatus == eStatus_t::OK) {
    v1 = ((((raw >> 3) - ((int32_t)_sCalib.t1 << 1))) * ((int32_t)_sCalib.t2)) >> 11;
    v2 = (((((raw >> 4) - ((int32_t)_sCalib.t1)) * ((raw >> 4) - ((int32_t)_sCalib.t1))) >> 12) * ((int32_t)_sCalib.t3)) >> 14;
    _t_fine = v1 + v2;
    rslt = (_t_fine * 5 + 128) >> 8;
    return (rslt / 100.0f);
  }
  return 0.0f;
}

// Get pressure
uint32_t DFRobot_BMP280::getPressure()
{
  getTemperature(); // Update _t_fine
  int32_t raw = getPressureRaw();
  int64_t rslt = 0;
  int64_t v1, v2;
  if (lastOperateStatus == eStatus_t::OK) {
    v1 = ((int64_t)_t_fine) - 128000;
    v2 = v1 * v1 * (int64_t)_sCalib.p6;
    v2 += (v1 * (int64_t)_sCalib.p5) << 17;
    v2 += ((int64_t)_sCalib.p4) << 35;
    v1 = (((1LL << 47) + v1 * v1 * (int64_t)_sCalib.p3) >> 8) + ((v1 * (int64_t)_sCalib.p2) << 12);
    if (v1 == 0)
      return 0; // Avoid division by zero
    rslt = 1048576 - raw;
    rslt = (((rslt << 31) - v2) * 3125) / v1;
    v1 = (((int64_t)_sCalib.p9) * (rslt >> 13) * (rslt >> 13)) >> 25;
    v2 = (((int64_t)_sCalib.p8) * rslt) >> 19;
    rslt = ((rslt + v1 + v2) >> 8) + (((int64_t)_sCalib.p7) << 4);
    return static_cast<uint32_t>(rslt / 256);
  }
  return 0;
}

// Calculate altitude
float DFRobot_BMP280::calAltitude(float seaLevelPressure, uint32_t pressure)
{
  return 44330.0f * (1.0f - powf((pressure / 100.0f) / seaLevelPressure, 0.1903f));
}

// Reset sensor
void DFRobot_BMP280::reset()
{
  uint8_t reset_cmd = 0xB6;
  _i2cBus.writemem(static_cast<uint8_t>(BMP280_Register::RESET), reset_cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// Set control measurement mode
void DFRobot_BMP280::setCtrlMeasMode(eCtrlMeasMode_t eMode)
{
  uint8_t reg = static_cast<uint8_t>(BMP280_Register::CTRL_MEAS);
  uint8_t mask = CTRL_MEAS_MODE_MASK; // 0x03
  uint8_t value = static_cast<uint8_t>(eMode) << 0; // bits 1:0
  writeRegBits(reg, mask, value);
}

// Set temperature oversampling
void DFRobot_BMP280::setCtrlMeasSamplingTemp(eSampling_t eSampling)
{
  uint8_t reg = static_cast<uint8_t>(BMP280_Register::CTRL_MEAS);
  uint8_t mask = CTRL_MEAS_OSRS_T_MASK; // 0xE0
  uint8_t value = static_cast<uint8_t>(eSampling) << 5; // bits 7:5
  writeRegBits(reg, mask, value);
}

// Set pressure oversampling
void DFRobot_BMP280::setCtrlMeasSamplingPress(eSampling_t eSampling)
{
  uint8_t reg = static_cast<uint8_t>(BMP280_Register::CTRL_MEAS);
  uint8_t mask = CTRL_MEAS_OSRS_P_MASK; // 0x1C
  uint8_t value = static_cast<uint8_t>(eSampling) << 2; // bits 4:2
  writeRegBits(reg, mask, value);
}

// Set config filter
void DFRobot_BMP280::setConfigFilter(eConfigFilter_t eFilter)
{
  uint8_t reg = static_cast<uint8_t>(BMP280_Register::CONFIG);
  uint8_t mask = CONFIG_FILTER_MASK; // 0x38
  uint8_t value = static_cast<uint8_t>(eFilter) << 3; // bits 5:3
  writeRegBits(reg, mask, value);
}

// Set config standby time
void DFRobot_BMP280::setConfigTStandby(eConfigTStandby_t eT)
{
  uint8_t reg = static_cast<uint8_t>(BMP280_Register::CONFIG);
  uint8_t mask = CONFIG_T_SB_MASK; // 0x07
  uint8_t value = static_cast<uint8_t>(eT) << 0; // bits 2:0
  writeRegBits(reg, mask, value);
}

// Retrieve calibration data from sensor
void DFRobot_BMP280::getCalibrate()
{
  _i2cBus.readmem(static_cast<uint8_t>(BMP280_Register::CALIB), sizeof(_sCalib), reinterpret_cast<uint8_t*>(&_sCalib));
}

// Get raw temperature data
int32_t DFRobot_BMP280::getTemperatureRaw()
{
  sRegTemp_t sReg;
  _i2cBus.readmem(static_cast<uint8_t>(BMP280_Register::TEMP), sizeof(sReg), reinterpret_cast<uint8_t*>(&sReg));
  return (((int32_t)sReg.msb << 12) | ((int32_t)sReg.lsb << 4) | ((int32_t)sReg.xlsb));
}

// Get raw pressure data
int32_t DFRobot_BMP280::getPressureRaw()
{
  sRegPress_t sReg;
  _i2cBus.readmem(static_cast<uint8_t>(BMP280_Register::PRESS), sizeof(sReg), reinterpret_cast<uint8_t*>(&sReg));
  return (((int32_t)sReg.msb << 12) | ((int32_t)sReg.lsb << 4) | ((int32_t)sReg.xlsb));
}

// Read a single register
uint8_t DFRobot_BMP280::getReg(uint8_t reg)
{
  uint8_t temp;
  _i2cBus.readmem(reg, sizeof(temp), &temp);
  return temp;
}

// Write specific bits in a register
void DFRobot_BMP280::writeRegBits(uint8_t reg, uint8_t mask, uint8_t shifted_val)
{
  uint8_t temp;
  _i2cBus.readmem(reg, sizeof(temp), &temp);
  temp &= ~mask;               // Clear the bits specified by mask
  temp |= (shifted_val & mask); // Set the new value
  _i2cBus.writemem(reg, temp);
}

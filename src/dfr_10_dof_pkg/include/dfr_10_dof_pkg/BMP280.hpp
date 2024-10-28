/*!
 * @file DFRobot_BMP280.h
 * @brief Provides an Arduino library for reading and interpreting Bosch BMP280 data over I2C. 
 * @n Used to read current temperature, air pressure and calculate altitude.
 *
 * @copyright   
 * @license     The MIT License (MIT)
 * @version     V1.0
 * @date        2019-03-12
 * @url         https://github.com/DFRobot/DFRobot_BMP280
 */

#ifndef DFROBOT_BMP280_H
#define DFROBOT_BMP280_H

#include <cstdint>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>

#include "i2c_lib/i2c_interface.hpp"

#ifndef PROGMEM
#define PROGMEM
#endif

#pragma pack(push, 1) // Ensure no padding in structs

/**
 * @class DFRobot_BMP280
 * @brief Interface class for BMP280 sensor
 */
class DFRobot_BMP280 {

public:
  /**
   * @enum class eStatus_t
   * @brief Enum for global status
   */
  enum class eStatus_t : uint8_t {
    OK,
    Error,
    DeviceNotDetected,
    ParameterError
  };

  /**
   * @struct sCalibrateDig_t
   * @brief Structure to hold calibration data
   */
  typedef struct {
    uint16_t t1;
    int16_t  t2, t3;
    uint16_t p1;
    int16_t  p2, p3, p4, p5, p6, p7, p8, p9;
    uint16_t reserved0;
  } sCalibrateDig_t;

  /**
   * @struct sRegStatus_t
   * @brief Structure to hold status register bits
   */
  typedef struct {
    uint8_t im_update : 1;
    uint8_t reserved  : 2;
    uint8_t measuring  : 1;
  } sRegStatus_t;

  /**
   * @enum class eCtrlMeasMode_t
   * @brief Enum for control measurement modes
   */
  enum class eCtrlMeasMode_t : uint8_t {
    Sleep   = 0x00,
    Forced  = 0x01,
    Normal  = 0x03
  };

  /**
   * @enum class eSampling_t
   * @brief Enum for oversampling settings
   */
  enum class eSampling_t : uint8_t {
    None  = 0x00,
    X1    = 0x01,
    X2    = 0x02,
    X4    = 0x03,
    X8    = 0x04,
    X16   = 0x05
  };

  /**
   * @struct sRegCtrlMeas_t
   * @brief Structure to hold CTRL_MEAS register bits
   */
  typedef struct {
    uint8_t mode   : 2;
    uint8_t osrs_p : 3;
    uint8_t osrs_t : 3;
  } sRegCtrlMeas_t;

  /**
   * @enum class eConfigSpi3w_en_t
   * @brief Enum for SPI3W enable
   */
  enum class eConfigSpi3w_en_t : uint8_t {
    Disable = 0x00,
    Enable  = 0x01
  };

  /**
   * @enum class eConfigFilter_t
   * @brief Enum for filter settings
   */
  enum class eConfigFilter_t : uint8_t {
    Off  = 0x00,
    X2   = 0x01,
    X4   = 0x02,
    X8   = 0x03,
    X16  = 0x04
  };

  /**
   * @enum class eConfigTStandby_t
   * @brief Enum for standby time settings (unit: ms)
   */
  enum class eConfigTStandby_t : uint8_t {
    _0_5   = 0x00,    /**< 0.5 ms */
    _62_5  = 0x01,    /**< 62.5 ms */
    _125   = 0x02,    /**< 125 ms */
    _250   = 0x03,    /**< 250 ms */
    _500   = 0x04,    /**< 500 ms */
    _1000  = 0x05,    /**< 1000 ms */
    _2000  = 0x06,    /**< 2000 ms */
    _4000  = 0x07     /**< 4000 ms */
  };

  /**
   * @struct sRegConfig_t
   * @brief Structure to hold CONFIG register bits
   */
  typedef struct {
    uint8_t spi3w_en : 1;
    uint8_t reserved1 : 1;
    uint8_t filter    : 3;
    uint8_t t_sb      : 3;
  } sRegConfig_t;

  /**
   * @struct sRegPress_t
   * @brief Structure to hold pressure register bits
   */
  typedef struct {
    uint8_t msb, lsb;
    uint8_t reserved : 4;
    uint8_t xlsb     : 4;
  } sRegPress_t;

  /**
   * @struct sRegTemp_t
   * @brief Structure to hold temperature register bits
   */
  typedef struct {
    uint8_t msb, lsb;
    uint8_t reserved : 4;
    uint8_t xlsb     : 4;
  } sRegTemp_t;

  /**
   * @enum class BMP280_Register
   * @brief Enum for BMP280 register addresses
   */
  enum class BMP280_Register : uint8_t {
    CALIB     = 0x88,
    CHIP_ID   = 0xD0,
    RESET     = 0xE0,
    STATUS    = 0xF3,
    CTRL_MEAS = 0xF4,
    CONFIG    = 0xF5,
    PRESS     = 0xF7,
    TEMP      = 0xFA
  };

#pragma pack(pop) // End of struct packing
  uint8_t _dev_addr;               /**< I2C device address */
  
  int32_t _t_fine;                 /**< Internal temperature variable */
  sCalibrateDig_t _sCalib;         /**< Calibration data */

  I2C_INTERFACE _i2cBus;           /**< I2C interface instance */
  uint8_t _buff[6];                /**< Buffer for I2C operations */
public:
  /**
   * @brief Constructor
   * @param dev_addr I2C device address
   */
  DFRobot_BMP280(uint8_t dev_addr);

  /**
   * @brief Initialize the sensor
   * @return Status of initialization
   */
  eStatus_t begin();

  /**
   * @brief Get temperature in Celsius
   * @return Temperature value
   */
  float getTemperature();

  /**
   * @brief Get pressure in Pascal
   * @return Pressure value
   */
  uint32_t getPressure();

  /**
   * @brief Calculate altitude based on sea level pressure and current pressure
   * @param seaLevelPressure Sea level pressure in Pascal
   * @param pressure Current pressure in Pascal
   * @return Calculated altitude in meters
   */
  float calAltitude(float seaLevelPressure, uint32_t pressure);

  /**
   * @brief Reset the sensor
   */
  void reset();

  /**
   * @brief Set control measurement mode
   * @param eMode Desired mode
   */
  void setCtrlMeasMode(eCtrlMeasMode_t eMode);

  /**
   * @brief Set temperature oversampling
   * @param eSampling Desired oversampling rate
   */
  void setCtrlMeasSamplingTemp(eSampling_t eSampling);

  /**
   * @brief Set pressure oversampling
   * @param eSampling Desired oversampling rate
   */
  void setCtrlMeasSamplingPress(eSampling_t eSampling);

  /**
   * @brief Set filter configuration
   * @param eFilter Desired filter setting
   */
  void setConfigFilter(eConfigFilter_t eFilter);

  /**
   * @brief Set standby time configuration
   * @param eT Desired standby time
   */
  void setConfigTStandby(eConfigTStandby_t eT);

public:
  eStatus_t lastOperateStatus; /**< Last operation status */

private:

  /**
   * @brief Retrieve calibration data from sensor
   */
  void getCalibrate();

  /**
   * @brief Get raw temperature data
   * @return Raw temperature value
   */
  int32_t getTemperatureRaw();

  /**
   * @brief Get raw pressure data
   * @return Raw pressure value
   */
  int32_t getPressureRaw();

  /**
   * @brief Read a single register
   * @param reg Register address
   * @return Register value
   */
  uint8_t getReg(uint8_t reg);

  /**
   * @brief Write specific bits in a register
   * @param reg Register address
   * @param mask Bitmask indicating which bits to modify
   * @param shifted_val Value to set (already shifted)
   */
  void writeRegBits(uint8_t reg, uint8_t mask, uint8_t shifted_val);

};

#endif // DFROBOT_BMP280_H

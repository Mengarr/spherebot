/*!
 * @file VCM5883L.h
 * @url https://github.com/DFRobot/VCM5883L
 */

#ifndef DFROBOT_QMC5883_H
#define DFROBOT_QMC5883_H

#include "i2c_lib/i2c_interface.hpp"

#include <cmath>

#define PI  3.14159265
#define VCM5883L_ADDRESS             (0x0C)

#define VCM5883L_REG_OUT_X_L          (0x00)
#define VCM5883L_REG_OUT_X_H          (0x01)
#define VCM5883L_REG_OUT_Y_L          (0x02)
#define VCM5883L_REG_OUT_Y_H          (0x03)
#define VCM5883L_REG_OUT_Z_L          (0x04)
#define VCM5883L_REG_OUT_Z_H          (0x05)
#define VCM5883L_CTR_REG1             (0x0B)
#define VCM5883L_CTR_REG2             (0x0A)

typedef enum
{
  HMC5883L_SAMPLES_8    = 0b11,
  HMC5883L_SAMPLES_4    = 0b10,
  HMC5883L_SAMPLES_2    = 0b01,
  HMC5883L_SAMPLES_1    = 0b00,
  QMC5883_SAMPLES_8     = 0b11,
  QMC5883_SAMPLES_4     = 0b10,
  QMC5883_SAMPLES_2     = 0b01,
  QMC5883_SAMPLES_1     = 0b00
} eSamples_t;

typedef enum
{
  HMC5883L_DATARATE_75HZ       = 0b110,
  HMC5883L_DATARATE_30HZ       = 0b101,
  HMC5883L_DATARATE_15HZ       = 0b100,
  HMC5883L_DATARATE_7_5HZ      = 0b011,
  HMC5883L_DATARATE_3HZ        = 0b010,
  HMC5883L_DATARATE_1_5HZ      = 0b001,
  HMC5883L_DATARATE_0_75_HZ    = 0b000,
  QMC5883_DATARATE_10HZ        = 0b00,
  QMC5883_DATARATE_50HZ        = 0b01,
  QMC5883_DATARATE_100HZ       = 0b10,
  QMC5883_DATARATE_200HZ       = 0b11,
  VCM5883L_DATARATE_200HZ      = 0b00,
  VCM5883L_DATARATE_100HZ      = 0b01,
  VCM5883L_DATARATE_50HZ       = 0b10,
  VCM5883L_DATARATE_10HZ       = 0b11
} eDataRate_t;

typedef enum
{
  HMC5883L_RANGE_8_1GA     = 0b111,
  HMC5883L_RANGE_5_6GA     = 0b110,
  HMC5883L_RANGE_4_7GA     = 0b101,
  HMC5883L_RANGE_4GA       = 0b100,
  HMC5883L_RANGE_2_5GA     = 0b011,
  HMC5883L_RANGE_1_9GA     = 0b010,
  HMC5883L_RANGE_1_3GA    = 0b001,
  HMC5883L_RANGE_0_88GA    = 0b000,
  QMC5883_RANGE_2GA     = 0b00,
  QMC5883_RANGE_8GA     = 0b01,
  VCM5883L_RANGE_8GA    = 0b01,
} eRange_t;

typedef enum
{
  HMC5883L_IDLE         = 0b10,
  HMC5883_SINGLE        = 0b01,
  HMC5883L_CONTINOUS    = 0b00,
  QMC5883_SINGLE        = 0b00,
  QMC5883_CONTINOUS     = 0b01,
  VCM5883L_SINGLE       = 0b0,
  VCM5883L_CONTINOUS    = 0b1,
} eMode_t;

#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
typedef struct 
{
  int16_t XAxis;
  int16_t YAxis;
  int16_t ZAxis;
  float   AngleXY;
  float   AngleXZ;
  float   AngleYZ;
  float   HeadingDegress;
} sVector_t;
#endif // VECTOR_STRUCT_H

class VCM5883L
{
  public:
    VCM5883L(uint8_t I2C_addr = 0x0C);
    /**
     * @fn init
     * @brief Sensor init
     * @return void
     * @retval true init succeeded
     * @retval false init failed
     */
    void init(void);

    /**
     * @fn readRaw
     * @brief Get the data collected by the sensor
     * @return sVector_t The data collected by the sensor
     */
    sVector_t readRaw(void);

    /**
     * @fn getRange
     * @brief Get sensor signal gain range
     * @return eRange_t
     */
    eRange_t getRange(void);

    /**
     * @fn setMeasurementMode
     * @brief Set measurement mode
     * @param mode
     * @n     HMC5883L_IDLE
     * @n     HMC5883_SINGLE
     * @n     HMC5883L_CONTINOUS
     * @n     QMC5883_SINGLE
     * @n     QMC5883_CONTINOUS
     * @n     VCM5883L_SINGLE
     * @n     VCM5883L_CONTINOUS
     */
    void setMeasurementMode(eMode_t mode);

    /**
     * @fn  getMeasurementMode
     * @brief Get measurement mode
     * @return eMode_t
     */
    eMode_t getMeasurementMode(void);

    /**
     * @fn setDataRate
     * @brief Set the data collection rate of the sensor
     * @param dataRate
     * @n     HMC5883L_DATARATE_75HZ
     * @n     HMC5883L_DATARATE_30HZ
     * @n     HMC5883L_DATARATE_15HZ
     * @n     HMC5883L_DATARATE_7_5HZ
     * @n     HMC5883L_DATARATE_3HZ
     * @n     HMC5883L_DATARATE_1_5HZ
     * @n     HMC5883L_DATARATE_0_75_HZ
     * @n     QMC5883_DATARATE_10HZ
     * @n     QMC5883_DATARATE_50HZ
     * @n     QMC5883_DATARATE_100HZ
     * @n     QMC5883_DATARATE_200HZ
     * @n     VCM5883L_DATARATE_200HZ
     * @n     VCM5883L_DATARATE_100HZ
     * @n     VCM5883L_DATARATE_50HZ
     * @n     VCM5883L_DATARATE_10HZ
     */
    void  setDataRate(eDataRate_t dataRate);

    /**
     * @fn getDataRate
     * @brief Get the data collection rate of the sensor
     * @return eDataRate_t
     */
    eDataRate_t getDataRate(void);

    /**
     * @fn setSamples
     * @brief Set sensor status
     * @param samples
     * @n     HMC5883L_SAMPLES_8
     * @n     HMC5883L_SAMPLES_4
     * @n     HMC5883L_SAMPLES_2
     * @n     HMC5883L_SAMPLES_1
     * @n     QMC5883_SAMPLES_8
     * @n     QMC5883_SAMPLES_4
     * @n     QMC5883_SAMPLES_2
     * @n     QMC5883_SAMPLES_1
     */
    void  setSamples(eSamples_t samples);

    /**
     * @fn getSamples
     * @brief Get sensor status
     * @return eSamples_t
     */
    eSamples_t getSamples(void);

    /**
     * @fn  setDeclinationAngle
     * @brief Set sensor declination angle
     * @param declinationAngle
     */
    void setDeclinationAngle(float declinationAngle);

    void setHardIronOffsets(float x, float y, float z);

    /**
     * @fn getHeadingDegrees
     * @brief Set the sensor range
     */
    float getHeadingDegrees(void);

    /**
     * @fn getICType
     * @brief Get sensor type
     * @return int
     */
    int getICType(void);

  private:
    // void writeRegister8(uint8_t reg, uint8_t value);
    // uint8_t readRegister8(uint8_t reg);
    // uint8_t fastRegister8(uint8_t reg);
    // int16_t readRegister16(uint8_t reg);
    // Offsets
    
    float X_offset_ = 0.0;
    float Y_offset_ = 0.0;
    float Z_offset_ = 0.0;
    
    //
    uint8_t _I2C_addr;
    float ICdeclinationAngle;
    bool isHMC_;
    bool isQMC_;
    float mgPerDigit;
    float Gauss_LSB_XY = 1090.0;
    sVector_t v;
    float minX, maxX;
    float minY, maxY;
    float minZ, maxZ;
    bool firstRun;

    // I2C
    I2C_INTERFACE _i2cBus;
    uint8_t _buff[6];
};

#endif // DFROBOT_QMC5883_H

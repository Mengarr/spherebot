#include "i2c_lib/i2c_interface.hpp"

#ifndef AUX_ARDUINO_H
#define AUX_ARDUINO_H

#define AUX_ARDUINO_I2C_ADDR    0x09

#define CONTROL_REG             0x00 // 0x0F to initalise the motors, 0xFF to un-initialise and set the in1,in2 pins to 00 (motors wont move)
#define SOIL_MOISTURE_REG       0x01
#define HALL_EFFECT_REG         0x02
#define MOTOR_CONTROL_REG       0x03
#define LIMIT_SWITCH_REG        0x04 // Recieve 0x00 for none one, 0x01 for LHS on, 0x10 for RHS on (one hot encoding)


class AUX_ARDUINO
{   
    public:
        AUX_ARDUINO(uint8_t I2C_ADDR = AUX_ARDUINO_I2C_ADDR);
        void init();
        void reset();
        void readSoilMoisture(float* data);
        void readHallEffect(uint8_t* data);
        void readLimitSwitches(std::pair<bool, bool>* limit_switch_states);
        void setMotorSpeedDir(uint8_t data); // Eight bit speed value
    private:
        // I2C
        I2C_INTERFACE _i2cBus;
        uint8_t __buff[6];
};

#endif // AUX_ARDUINO_H
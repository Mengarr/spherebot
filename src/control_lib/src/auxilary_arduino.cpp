#include "../include/control_lib/auxilary_arduino.hpp"

AuxilaryArduino::AuxilaryArduino(uint8_t I2C_ADDR) 
    : _i2cBus(I2C_ADDR)
    {

    }

void AuxilaryArduino::init() {
    _i2cBus.writemem(CONTROL_REG, 0X0F);
}

void AuxilaryArduino::reset() {
    _i2cBus.writemem(CONTROL_REG, 0XFF);
}

void AuxilaryArduino::readSoilMoisture(float* soil_mositure) {
    _i2cBus.readmem(SOIL_MOISTURE_REG,  uint8_t(4), __buff);
    memcpy(soil_mositure, __buff, sizeof(float));  
}

void AuxilaryArduino::readHallEffect(uint8_t* data) {
    _i2cBus.readmem(HALL_EFFECT_REG,  uint8_t(1), __buff);
    memcpy(data, __buff, sizeof(uint8_t));  
}

void AuxilaryArduino::setMotorSpeedDir(uint8_t data) {
    _i2cBus.writemem(MOTOR_CONTROL_REG, data);
}

void AuxilaryArduino::readLimitSwitches(std::pair<bool, bool>* limit_switch_states) {
    uint8_t data;
    const uint8_t MASK1 = 0xF0;
    const uint8_t MASK2 = 0x0F;
    _i2cBus.readmem(LIMIT_SWITCH_REG, uint8_t(1), __buff);
    memcpy(&data, __buff, sizeof(uint8_t));
    
    limit_switch_states->first = ((data & MASK1) == MASK1 ? 1 : 0);
    limit_switch_states->second = ((data & MASK2) == MASK2 ? 1 : 0);
}
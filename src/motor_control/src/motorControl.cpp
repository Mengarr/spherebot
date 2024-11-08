#include "../include/motor_control/motorControl.hpp"

MotorControl::MotorControl(uint8_t I2C_ADDR) 
    : _i2cBus(I2C_ADDR)
    {

    }
void MotorControl::init() {
    _i2cBus.writemem(CONTROL_REG, 0X0F);
}

void MotorControl::readMotorASpeed(float* speed) {
    _i2cBus.readmem(MOTOR_A_SPEED_READ_REG,  uint8_t(4), __buff);
    memcpy(speed, __buff, sizeof(float));  
}

void MotorControl::readMotorACount(int32_t* val) {
    _i2cBus.readmem(MOTOR_A_COUNT_READ_REG,  uint8_t(4), __buff);
    memcpy(val, __buff, sizeof(int32_t));  
}

void MotorControl::readMotorBCount(int32_t* val) {
    _i2cBus.readmem(MOTOR_B_COUNT_READ_REG,  uint8_t(4), __buff);
    memcpy(val, __buff, sizeof(int32_t));  
}

void MotorControl::readMotorBSpeed(float* speed) {
    _i2cBus.readmem(MOTOR_B_SPEED_READ_REG,  uint8_t(4), __buff);
    memcpy(speed, __buff, sizeof(float));  
}

void MotorControl::setMotorASpeed(float speed) {
    _i2cBus.writemem_32f(MOTOR_A_SPEED_WRITE_REG, speed);
}

void MotorControl::setMotorBSpeed(float speed) {
    _i2cBus.writemem_32f(MOTOR_B_SPEED_WRITE_REG, speed);
}

void MotorControl::reset() {
    _i2cBus.writemem(CONTROL_REG, 0XFF);
}

std::pair<float, float> MotorControl::get_u_alpha() {
    int32_t motorACount, motorBCount;
    readMotorACount(&motorACount);
    readMotorBCount(&motorBCount);
    
    float Psi_L, Psi_R;
    Psi_L = motorACount / (CPR * MOTOR_GEAR_RATIO); 
    Psi_R = motorBCount / (CPR * MOTOR_GEAR_RATIO); 

    std::pair<float, float> u_alpha = computeJointVariablesInverse(Psi_L, Psi_R);
    return u_alpha;
}

// void MotorControl::test(float* speed){
//     _i2cBus.readmem(0x08,  uint8_t(4), __buff);

//     // Print individual buffer values using std::cout
//     std::cout << "Raw Bytes: ";
//     for (int i = 0; i < 4; ++i) {
//         std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(__buff[i]) << " ";
//     }
//     std::cout << std::dec << std::endl;  // Switch back to decimal format

//     memcpy(speed, __buff, sizeof(float)); 
// }


#include "i2c_lib/i2c_interface.hpp"
#include "kinematic_transforms.hpp"

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#define MOTORS_I2C_ADDR 0x08

#define CONTROL_REG             0x00 // 0x0F to initalise the motors, 0xFF to un-initialise and set the in1,in2 pins to 00 (motors wont move)
#define MOTOR_A_SPEED_READ_REG  0x01 // For motor speed read requests, includes direction as the sign
#define MOTOR_B_SPEED_READ_REG  0x02 // **
#define MOTOR_A_SPEED_WRITE_REG 0x03 // For motor speed write requests
#define MOTOR_B_SPEED_WRITE_REG 0x04
#define MOTOR_A_COUNT_READ_REG  0x05
#define MOTOR_B_COUNT_READ_REG  0x06

class MotorControl
{   
    public:
        MotorControl(uint8_t I2C_ADDR = MOTORS_I2C_ADDR);
        void init();
        void reset();
        void readMotorASpeed(float* speed);
        void readMotorBSpeed(float* speed);
        void readMotorACount(int32_t* val); // Reads the encoder count
        void readMotorBCount(int32_t* val); 
        void setMotorASpeed(float speed); // Eight bit speed value
        void setMotorBSpeed(float speed); 

        std::pair<float, float> get_u_alpha();

    private:
        // I2C
        I2C_INTERFACE _i2cBus;
        uint8_t __buff[6];
};

#endif // MotorControl_H
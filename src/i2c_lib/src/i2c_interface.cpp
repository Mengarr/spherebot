#include "../include/i2c_lib/i2c_interface.hpp"

I2C_INTERFACE::I2C_INTERFACE(uint8_t address) {
    _dev_address = address;
    _file = open("/dev/i2c-1", O_RDWR);
    if (_file < 0) {
        std::cerr << "Failed to open the I2C bus: " << strerror(errno) << std::endl;
        throw std::runtime_error("Failed to open the I2C bus");
    }
    if (ioctl(_file, I2C_SLAVE, address) < 0) {
        throw std::runtime_error("Failed to acquire bus access and/or talk to slave");
    }
}

I2C_INTERFACE::~I2C_INTERFACE() {
    if (_file >= 0) {
        close(_file);
    }
}

uint8_t I2C_INTERFACE::get_address() {
    return _dev_address;
}

// For writing a byte over i2c
void I2C_INTERFACE::writemem(uint8_t _addr, uint8_t _val) {
    std::lock_guard<std::mutex> lock(i2c_mutex);
    uint8_t buffer[2] = {_addr, _val};
    if (write(_file, buffer, 2) != 2) {
        throw std::runtime_error("Failed to write to the I2C device");
    }
}

void I2C_INTERFACE::writemem_32f(uint8_t _addr, float _val) {
    std::lock_guard<std::mutex> lock(i2c_mutex);
    uint8_t buffer[5];  // 1 byte for _addr, 4 bytes for _val

    // Copy the float value directly into the byte array (as a 4-byte float)
    memcpy(&buffer[1], &_val, sizeof(float));  // Copy 4 bytes of the float into buffer starting at index 1
    buffer[0] = _addr;  // First byte is the register address

    // // Print individual buffer values using std::cout
    // std::cout << "Raw Bytes: ";
    // for (int i = 0; i < 5; ++i) {
    //     std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer[i]) << " ";
    // }
    // std::cout << std::dec << std::endl;  // Switch back to decimal format

    if (write(_file, buffer, 5) != 5) {
        throw std::runtime_error("Failed to write to the I2C device");
    }
}
/*
    _addr is the register adress 
    _nbytes is the number of bytes to be read from the slave
    __buff is a buffer to write the read data into
*/
void I2C_INTERFACE::readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]) {
    std::lock_guard<std::mutex> lock(i2c_mutex);
    if (write(_file, &_addr, 1) != 1) {
        throw std::runtime_error("Failed to write to the I2C device");
    }
    if (read(_file, __buff, _nbytes) != _nbytes) {
        throw std::runtime_error("Failed to read from the I2C device");
    }
}
#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include <iostream>
#include <iomanip>

#include <mutex>
#include <fcntl.h>
#include <iostream>
#include <cstdint>
#include <stdexcept>
#include <sys/ioctl.h> // for linux i2c
#include <linux/i2c-dev.h> // for linux i2c
#include <unistd.h>
#include <cstdint>
#include <cerrno>    // For errno
#include <cstring>

class I2C_INTERFACE {
    public:
        I2C_INTERFACE(uint8_t address);
        ~I2C_INTERFACE();

        void writemem(uint8_t _addr, uint8_t _val);
        void readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]);
        void writemem_32f(uint8_t _addr, float _val); // writes 32 bit float over bus
        uint8_t get_address();

    private:
        uint8_t _dev_address;
        int _file;
        std::mutex i2c_mutex; 
};

#endif // I2C_INTERFACE_H
#ifndef BUS_H_
#define BUS_H_

#include <stdint.h>
#include <string>

namespace i2c {

class Bus {
   public:
    Bus(std::string i2c_dev);
    ~Bus();

    bool Write(uint8_t slave_addr, const uint8_t* data, uint32_t length);
    bool WriteRegister(uint8_t slave_addr, uint8_t offset, const uint8_t* data,
                       uint32_t length);
    bool Read(uint8_t slave_addr, uint8_t* data, uint32_t length);
    bool ReadRegister(uint8_t slave_addr, uint8_t offset, uint8_t* data,
                      uint32_t length);

   private:
    int bus_file_;
};
}

#endif

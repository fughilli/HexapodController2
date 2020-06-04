#ifndef BUS_H_
#define BUS_H_

#include <stdint.h>
#include <string>

#include "absl/types/span.h"

namespace i2c {

class Bus {
public:
  Bus(std::string i2c_dev);
  ~Bus();

  bool Write(uint8_t slave_addr, absl::Span<const uint8_t> data);
  bool WriteRegister(uint8_t slave_addr, uint8_t register_addr,
                     absl::Span<const uint8_t> data);
  bool Read(uint8_t slave_addr, absl::Span<uint8_t> *out_data);
  bool ReadRegister(uint8_t slave_addr, uint8_t register_addr,
                    absl::Span<uint8_t> *out_data);

private:
  int bus_file_;
};
} // namespace i2c

#endif

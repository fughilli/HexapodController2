#include "bus.h"

#include <assert.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace i2c {

Bus::Bus(std::string i2c_dev) {
  bus_file_ = open(i2c_dev.c_str(), O_RDWR);
  assert(bus_file_ >= 0);
}

Bus::~Bus() { close(bus_file_); }

bool Bus::Write(uint8_t slave_addr, absl::Span<const uint8_t> data) {
  struct i2c_rdwr_ioctl_data transaction_data;
  struct i2c_msg segment;
  transaction_data.msgs = &segment;
  transaction_data.nmsgs = 1;

  segment.addr = slave_addr;
  segment.flags = 0;
  segment.len = data.size();
  segment.buf = const_cast<uint8_t *>(data.data());

  return ioctl(bus_file_, I2C_RDWR, &transaction_data) >= 0;
}

bool Bus::WriteRegister(uint8_t slave_addr, uint8_t register_addr,
                        absl::Span<const uint8_t> data) {
  uint8_t reg_and_data[data.size() + 1];
  memcpy(reg_and_data + 1, data.data(), data.size());
  reg_and_data[0] = register_addr;
  return Write(slave_addr,
               absl::Span<const uint8_t>(reg_and_data, data.size() + 1));
}

bool Bus::Read(uint8_t slave_addr, absl::Span<uint8_t> *out_data) {
  struct i2c_rdwr_ioctl_data transaction_data;
  struct i2c_msg segment;
  transaction_data.msgs = &segment;
  transaction_data.nmsgs = 1;

  segment.addr = slave_addr;
  segment.flags = I2C_M_RD;
  segment.len = out_data->size();
  segment.buf = out_data->data();

  return ioctl(bus_file_, I2C_RDWR, &transaction_data) >= 0;
}

bool Bus::ReadRegister(uint8_t slave_addr, uint8_t register_addr,
                       absl::Span<uint8_t> *out_data) {
  struct i2c_rdwr_ioctl_data transaction_data;
  struct i2c_msg segments[2];
  transaction_data.msgs = segments;
  transaction_data.nmsgs = 2;

  segments[0].addr = slave_addr;
  segments[0].flags = 0;
  segments[0].len = 1;
  segments[0].buf = &register_addr;

  segments[1].addr = slave_addr;
  segments[1].flags = I2C_M_RD;
  segments[1].len = out_data->size();
  segments[1].buf = out_data->data();

  return ioctl(bus_file_, I2C_RDWR, &transaction_data) >= 0;
}
} // namespace i2c

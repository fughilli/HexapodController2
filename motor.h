#ifndef MOTOR_H_
#define MOTOR_H_

#include "bus.h"
#include "util.h"

#include <stdint.h>
#include <memory>
#include <utility>
#include <vector>

namespace motor {

using pos_t = uint16_t;

class Motor {
   public:
    Motor();
    Motor(std::pair<float, float> limits, std::pair<pos_t, pos_t> out_limits);
    void WriteToBuffer(void* buffer);
    // Calculates the output position for a given input position.
    pos_t GetOutPosition(float in_pos);
    void Write(float pos);
    void SetOutLimits(std::pair<pos_t, pos_t> out_limits) {
        out_limits_ = out_limits;
    }

   private:
    bool enabled_;
    std::pair<float, float> limits_;
    std::pair<pos_t, pos_t> out_limits_;
    pos_t position_;
};

class MotorController {
   public:
    MotorController(std::shared_ptr<i2c::Bus> bus, uint8_t address,
                    uint32_t num_motors);
    bool Flush();
    bool WriteNoFlush(uint32_t index, float angle);
    bool Reset();

   private:
    bool ReadLimits();
    std::pair<pos_t, pos_t> out_limits_;
    const uint8_t address_;
    std::vector<Motor> motors_;
    std::shared_ptr<i2c::Bus> bus_;
};
}

#endif

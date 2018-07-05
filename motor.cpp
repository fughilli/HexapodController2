#include "motor.h"

#include <chrono>
#include <thread>

namespace motor {

Motor::Motor(std::pair<float, float> limits, std::pair<pos_t, pos_t> out_limits)
    : limits_(std::move(limits)),
      out_limits_(std::move(out_limits)),
      position_(0) {}
Motor::Motor() : Motor({0, 0}, {0, 0}){};

void Motor::WriteToBuffer(void* buffer) {
    uint16_t value = (position_ & 0x7FFF) | ((enabled_ ? 1 : 0) << 15);
    *reinterpret_cast<uint16_t*>(buffer) = value;
}

pos_t Motor::GetOutPosition(float in_pos) {
    return util::Clamp(
        static_cast<pos_t>(util::Map<float>(in_pos, limits_, out_limits_)),
        out_limits_);
}

void Motor::Write(float pos) { position_ = GetOutPosition(pos); }

MotorController::MotorController(std::shared_ptr<i2c::Bus> bus, uint8_t address,
                                 uint32_t num_motors)
    : address_(address), motors_(num_motors), bus_(bus) {
    Reset();
    ReadLimits();
}

bool MotorController::Flush() {
    uint16_t servo_control_buffer[motors_.size()];
    uint16_t* current_servo = servo_control_buffer;
    for (auto& motor : motors_) {
        motor.WriteToBuffer(current_servo++);
    }
    return bus_->WriteRegister(address_, 0,
                               reinterpret_cast<uint8_t*>(servo_control_buffer),
                               sizeof(servo_control_buffer));
}

bool MotorController::ReadLimits() {
    // After the servo control section
    uint32_t servo_limits_offset = 2 * motors_.size();
    uint16_t out_limits[2];
    if (!bus_->ReadRegister(address_, servo_limits_offset,
                            reinterpret_cast<uint8_t*>(out_limits), 4)) {
        return false;
    }
    out_limits_.first = out_limits[0];
    out_limits_.second = out_limits[1];
    return true;
}

bool MotorController::Reset() {
    // After the limits
    uint32_t reset_offset = 2 * motors_.size() + 4;
    const uint8_t reset_data[] = {0xFF, 0xFF};
    if (bus_->WriteRegister(address_, reset_offset, reset_data, 2)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}
}

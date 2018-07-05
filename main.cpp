#include "bus.h"
#include "motor.h"

int main(int argc, char** argv) {
    std::shared_ptr<i2c::Bus> bus(new i2c::Bus("/dev/i2c-1"));
    motor::MotorController controller(bus, 0x40, 9);

    return 0;
}

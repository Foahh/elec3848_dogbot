#ifndef DOGBOT_HARDWARE_WHEEL_HPP
#define DOGBOT_HARDWARE_WHEEL_HPP

#include <string>
#include <cmath>


class Wheel {
public:
    std::string name;
    double cmd = 0;
    double pos = 0;
    long enc = 0;

    Wheel() = default;

    void setup(const std::string &wheel_name, int enc_counts_per_rev) {
        name = wheel_name;
        rad_per_counts_ = (2.0 * M_PI) / (double) enc_counts_per_rev;
    }

    void update() {
        pos = (double) enc * rad_per_counts_;
    }

    double calculate_command_speed() const {
        return cmd / rad_per_counts_ / 1000.0;
    }

private:
    double rad_per_counts_ = 0;
};


#endif // DOGBOT_HARDWARE_WHEEL_HPP
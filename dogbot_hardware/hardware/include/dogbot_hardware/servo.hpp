#ifndef DOGBOT_HARDWARE_SERVO_HPP_
#define DOGBOT_HARDWARE_SERVO_HPP_

#include <string>

namespace dogbot_hardware
{
   class Servo
   {
   public:
      std::string name;
      double cmd = 0;
      void setup(const std::string &wheel_name)
      {
         name = wheel_name;
      }

      int get_position() const
      {
         return (int)cmd;
      }

   };
} // namespace dogbot_hardware

#endif // DOGBOT_HARDWARE_SERVO_HPP_
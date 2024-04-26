#ifndef DOGBOT_HARDWARE_SONAR_HPP_
#define DOGBOT_HARDWARE_SONAR_HPP_

#include <string>

namespace dogbot_hardware
{
   class Sonar
   {
   public:
      std::string name;
      double range = 0.0;
      void setup(const std::string &sonar_name)
      {
         name = sonar_name;
      }
   };
} // namespace dogbot_hardware

#endif // DOGBOT_HARDWARE_SONAR_HPP_
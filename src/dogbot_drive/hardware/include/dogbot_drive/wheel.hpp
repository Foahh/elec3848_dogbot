// Copyright 2022 joshnewans
//
// Modified by Long Liangmao in 2024
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DOGBOT_DRIVE_WHEEL_HPP
#define DOGBOT_DRIVE_WHEEL_HPP

#include <cmath>
#include <string>

class Wheel
{
public:
  std::string name = "";
  long enc = 0;
  double cmd = 0;
  double pos = 0;
  double vel = 0;
  double rads_per_count = 0;

  Wheel() = default;

  Wheel(const std::string &wheel_name, int counts_per_rev)
  {
    setup(wheel_name, counts_per_rev);
  }

  void setup(const std::string &wheel_name, int counts_per_rev)
  {
    name = wheel_name;
    rads_per_count = (2 * M_PI) / counts_per_rev;
  }

  double calc_enc_angle()
  {
    return enc * rads_per_count;
  }
};

#endif // DOGBOT_DRIVE_WHEEL_HPP

// Copyright 2020 PAL Robotics S.L.
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

/*
 * Author: Enrique Fernández
 */

#include "dogbot_drive_controller/odometry.hpp"

namespace dogbot_drive_controller
{
  Odometry::Odometry(size_t velocity_rolling_window_size)
      : timestamp_(0.0),
        x_(0.0),
        y_(0.0),
        heading_(0.0),
        linear_x_(0.0),
        linear_y_(0.0),
        angular_(0.0),
        wheel_separation_x_(0.0),
        wheel_separation_y_(0.0),
        wheel_separation_k_(0.0),
        lf_wheel_radius_(0.0),
        rf_wheel_radius_(0.0),
        lb_wheel_radius_(0.0),
        rb_wheel_radius_(0.0),
        lf_wheel_old_pos_(0.0),
        rf_wheel_old_pos_(0.0),
        lb_wheel_old_pos_(0.0),
        rb_wheel_old_pos_(0.0),
        velocity_rolling_window_size_(velocity_rolling_window_size),
        linear_accumulator_x_(velocity_rolling_window_size),
        linear_accumulator_y_(velocity_rolling_window_size),
        angular_accumulator_(velocity_rolling_window_size)
  {
  }

  void Odometry::init(const rclcpp::Time &time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
  }

  // unit: meter
  bool Odometry::update(double lf_pos, double rf_pos, double lb_pos, double rb_pos, const rclcpp::Time &time)
  {
    // We cannot estimate the speed with very small time intervals:
    const double dt = time.seconds() - timestamp_.seconds();
    if (dt < 0.0001)
    {
      return false; // Interval too small to integrate with
    }

    // Get current wheel joint positions:
    const double lf_wheel_cur_pos = lf_pos;
    const double rf_wheel_cur_pos = rf_pos;
    const double lb_wheel_cur_pos = lb_pos;
    const double rb_wheel_cur_pos = rb_pos;

    // Estimate velocity of wheels using old and current position:
    const double lf_wheel_est_vel = (lf_wheel_cur_pos - lf_wheel_old_pos_) / dt;
    const double rf_wheel_est_vel = (rf_wheel_cur_pos - rf_wheel_old_pos_) / dt;
    const double lb_wheel_est_vel = (lb_wheel_cur_pos - lb_wheel_old_pos_) / dt;
    const double rb_wheel_est_vel = (rb_wheel_cur_pos - rb_wheel_old_pos_) / dt;

    // Update old position with current:
    lf_wheel_old_pos_ = lf_wheel_cur_pos;
    rf_wheel_old_pos_ = rf_wheel_cur_pos;
    lb_wheel_old_pos_ = lb_wheel_cur_pos;
    rb_wheel_old_pos_ = rb_wheel_cur_pos;

    updateFromVelocity(lf_wheel_est_vel, rf_wheel_est_vel, lb_wheel_est_vel, rb_wheel_est_vel, time);

    return true;
  }

  // unit: m
  bool Odometry::updateFromVelocity(double lf, double rf, double lb, double rb, const rclcpp::Time &time)
  {
    const double dt = time.seconds() - timestamp_.seconds();

    // Estimate linear and angular speed:
    const double linear_x = (lf + rf + lb + rb) / 4.0;
    const double linear_y = (-lf + rf + lb - rb) / 4.0;
    const double angular = (lf - rf + lb - rb) / (4.0 * wheel_separation_k_);

    // Integrate odometry:
    integrate(linear_x, linear_y, angular);

    timestamp_ = time;

    // Estimate speeds using a rolling mean to filter them out:
    linear_accumulator_x_.accumulate(linear_x / dt);
    linear_accumulator_y_.accumulate(linear_y / dt);
    angular_accumulator_.accumulate(angular / dt);

    linear_x_ = linear_accumulator_x_.getRollingMean();
    linear_y_ = linear_accumulator_y_.getRollingMean();
    angular_ = angular_accumulator_.getRollingMean();

    return true;
  }

  void Odometry::resetOdometry()
  {
    x_ = 0.0;
    y_ = 0.0;
    heading_ = 0.0;
  }

  void Odometry::setWheelParams(double wheel_separation_x, double wheel_separation_y,
                                double lf_wheel_radius, double rf_wheel_radius, double lb_wheel_radius, double rb_wheel_radius)
  {
    wheel_separation_x_ = wheel_separation_x;
    wheel_separation_y_ = wheel_separation_y;
    wheel_separation_k_ = (wheel_separation_x + wheel_separation_y) / 2.0;
    lf_wheel_radius_ = lf_wheel_radius;
    rf_wheel_radius_ = rf_wheel_radius;
    lb_wheel_radius_ = lb_wheel_radius;
    rb_wheel_radius_ = rb_wheel_radius;
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;
    resetAccumulators();
  }

  void Odometry::integrate(double linear_x, double linear_y, double angular)
  {
    x_ += linear_x;
    y_ += linear_y;
    heading_ += angular;
  }

  void Odometry::resetAccumulators()
  {
    linear_accumulator_x_ = RollingMeanAccumulator(velocity_rolling_window_size_);
    linear_accumulator_y_ = RollingMeanAccumulator(velocity_rolling_window_size_);
    angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  }

} // namespace dogbot_drive_controller

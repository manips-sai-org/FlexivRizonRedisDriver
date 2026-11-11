#pragma once

#include <Eigen/Core>

namespace Sai::Flexiv {

// Idle holding and controller shutdown handoff, updated at 1 kHz after startup.
// All returned torques are nominal: apply joint safeties and saturation afterward.
class IdleController {
public:
  using Vector = Eigen::Matrix<double, 7, 1>;
  using Matrix = Eigen::Matrix<double, 7, 7>;

  static constexpr int kNotTouchingCycles = 400;
  static constexpr int kHandoffCycles = 200;
  static constexpr double kHoldingKp = 200.0; // 1/s^2, before mass matrix
  static constexpr double kHoldingKv = 10.0; // 1/s, before mass matrix
  static constexpr double kHoldingVelocityLimit = 0.15; // rad/s, per joint
  static constexpr double kWristInertiaOffset = 0.15; // kg m^2, joints 5-7

  const Vector &hold_pose() const { return q_init_; }
  int not_touching_cycles() const { return not_touching_counter_; }
  bool handoff_active() const { return handoff_cycle_ < kHandoffCycles; }

  void reset_hold_pose(const Vector &q) {
    q_init_ = q;
    not_touching_counter_ = 0;
    was_active_ = false;
    handoff_cycle_ = kHandoffCycles;
    previous_torques_.setZero();
    handoff_start_torques_.setZero();
  }

  Vector update(bool active_command, bool touching, const Vector &q,
                const Vector &dq, const Matrix &mass_matrix,
                const Vector &controller_torques, const Vector &floating_torques) {
    if (active_command) {
      was_active_ = true;
      handoff_cycle_ = kHandoffCycles;
      // Include bias (in controller_torques) and floating damping in the handoff.
      previous_torques_ = controller_torques + floating_torques;
      return previous_torques_;
    }

    if (was_active_) {
      q_init_ = q;
      not_touching_counter_ = 0;
      handoff_start_torques_ = previous_torques_;
      handoff_cycle_ = 0;
      was_active_ = false;
    }

    Vector idle_acceleration = Vector::Zero();
    if (touching) {
      q_init_ = q;
      not_touching_counter_ = 0;
    } else if (not_touching_counter_ < kNotTouchingCycles) {
      // Follow the current pose during the release window, then hold that pose.
      ++not_touching_counter_;
      q_init_ = q;
      idle_acceleration = -kHoldingKv * dq;
    } else {
      const Vector goal_velocity =
          ((kHoldingKp / kHoldingKv) * (q_init_ - q))
              .cwiseMax(-kHoldingVelocityLimit)
              .cwiseMin(kHoldingVelocityLimit);
      idle_acceleration = kHoldingKv * (goal_velocity - dq);
    }

    Matrix idle_mass_matrix = mass_matrix;
    idle_mass_matrix.diagonal().tail<3>().array() += kWristInertiaOffset;
    Vector target = idle_mass_matrix * idle_acceleration + floating_torques;
    if (handoff_active()) {
      // Start exactly at the previous nominal command. Using a post-safety
      // command here would apply joint-limit avoidance a second time downstream.
      const double t = double(handoff_cycle_) / kHandoffCycles;
      const double blend = t * t * (3.0 - 2.0 * t);
      target = (1.0 - blend) * handoff_start_torques_ + blend * target;
      ++handoff_cycle_;
    }
    previous_torques_ = target;
    return target;
  }

private:
  Vector q_init_ = Vector::Zero();
  int not_touching_counter_ = 0;
  bool was_active_ = false;
  int handoff_cycle_ = kHandoffCycles;
  Vector previous_torques_ = Vector::Zero();
  Vector handoff_start_torques_ = Vector::Zero();
};

} // namespace Sai::Flexiv

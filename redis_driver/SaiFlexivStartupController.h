#pragma once

#include <Eigen/Core>
#include <algorithm>

namespace Sai::Flexiv {

// Hardware-independent startup sequencing; update() is called at 1 kHz.
class StartupController {
public:
  using Vector = Eigen::Matrix<double, 7, 1>;
  enum class Phase { ClearLimits, RampHold, HoldBeforeBias, EstimateBias, Ready };

  struct Limits {
    Vector hard_min;
    Vector hard_max;
    Vector stop_min;
    Vector stop_max;
    Vector torque_max;
  };

  // Counts are control cycles; at 1 kHz each cycle is one millisecond.
  static constexpr double kClearanceMargin = 0.5 * 3.14159265358979323846 / 180.0;
  static constexpr double kSettlingVelocity = 0.02; // rad/s, every joint
  static constexpr int kSettlingCycles = 200;
  static constexpr int kRampCycles = 500;
  static constexpr int kBiasSamples = 500;
  static constexpr double kHoldingKp = 1000.0; // N m/rad
  static constexpr double kHoldingKv = 10.0; // N m s/rad
  static constexpr double kHoldingVelocityLimit = 0.1; // rad/s, per joint

  explicit StartupController(int bias_delay_ms)
      : bias_delay_cycles_(std::max(0, bias_delay_ms)) {}

  struct Output {
    Vector torques = Vector::Zero(); // N m, excluding robot gravity compensation
    bool captured_pose = false;
    bool became_ready = false;
  };

  Phase phase() const { return phase_; }
  const Vector &hold_pose() const { return hold_pose_; }
  const Vector &bias() const { return bias_; }
  int bias_samples() const { return bias_samples_; }

  Output update(const Vector &q, const Vector &dq, const Limits &limits,
                const Vector &recovery_damping) {
    Output output;
    if (phase_ == Phase::Ready) {
      return output;
    }

    const Vector recovery = recovery_torques(q, dq, limits, recovery_damping);
    const bool clear =
        (q.array() >= limits.hard_min.array() + kClearanceMargin).all() &&
        (q.array() <= limits.hard_max.array() - kClearanceMargin).all();
    const bool settled = (dq.array().abs() <= kSettlingVelocity).all();

    // Losing clearance invalidates any captured pose and samples.
    if (!clear) {
      restart_clearance();
    }

    if (phase_ == Phase::ClearLimits) {
      settled_cycles_ = clear && settled ? settled_cycles_ + 1 : 0;
      output.torques = recovery;
      if (settled_cycles_ >= kSettlingCycles) {
        capture_pose(q, recovery);
        output.captured_pose = true;
      }
      return output;
    }

    // Saturate the goal velocity from position error, retaining full damping.
    // Below the velocity limit this is equivalent to the original position PD.
    const Vector goal_velocity =
        ((kHoldingKp / kHoldingKv) * (hold_pose_ - q))
            .cwiseMax(-kHoldingVelocityLimit)
            .cwiseMin(kHoldingVelocityLimit);
    const Vector hold_torques = kHoldingKv * (goal_velocity - dq);
    if (phase_ == Phase::RampHold) {
      const double t = std::min(1.0, double(++ramp_cycles_) / kRampCycles);
      const double blend = t * t * (3.0 - 2.0 * t);
      output.torques = (1.0 - blend) * ramp_start_torques_ + blend * hold_torques;
      if (ramp_cycles_ >= kRampCycles) {
        phase_ = bias_delay_cycles_ > 0 ? Phase::HoldBeforeBias : Phase::EstimateBias;
      }
      return output;
    }

    output.torques = hold_torques;
    if (phase_ == Phase::HoldBeforeBias) {
      // Keep full holding control active, but exclude this interval from bias.
      if (++hold_delay_cycles_ >= bias_delay_cycles_) {
        phase_ = Phase::EstimateBias;
      }
      return output;
    }

    if (!settled) {
      bias_sum_.setZero();
      bias_samples_ = 0;
    } else {
      // Average only the settled hold contribution, never recovery or ramp torques.
      bias_sum_ += hold_torques;
      if (++bias_samples_ == kBiasSamples) {
        bias_ = bias_sum_ / kBiasSamples;
        phase_ = Phase::Ready;
        output.became_ready = true;
      }
    }
    return output;
  }

private:
  static Vector recovery_torques(const Vector &q, const Vector &dq,
                                const Limits &limits, const Vector &damping) {
    Vector torques = -damping.cwiseProduct(dq);
    for (int i = 0; i < q.size(); ++i) {
      // Extend the existing quadratic avoidance profile past the hard boundary.
      // Its zero is beyond the clearance check, so recovery can cross the margin.
      const double lower_target = limits.hard_min(i) + 2.0 * kClearanceMargin;
      const double upper_target = limits.hard_max(i) - 2.0 * kClearanceMargin;
      if (q(i) < lower_target) {
        const double alpha = std::clamp(
            (lower_target - q(i)) / (lower_target - limits.stop_min(i)), 0.0, 1.0);
        torques(i) += alpha * alpha * limits.torque_max(i);
      } else if (q(i) > upper_target) {
        const double alpha = std::clamp(
            (q(i) - upper_target) / (limits.stop_max(i) - upper_target), 0.0, 1.0);
        torques(i) -= alpha * alpha * limits.torque_max(i);
      }
    }
    return torques;
  }

  void restart_clearance() {
    phase_ = Phase::ClearLimits;
    settled_cycles_ = 0;
    ramp_cycles_ = 0;
    hold_delay_cycles_ = 0;
    bias_samples_ = 0;
    bias_sum_.setZero();
    bias_.setZero();
  }

  void capture_pose(const Vector &q, const Vector &recovery) {
    hold_pose_ = q;
    ramp_start_torques_ = recovery;
    bias_sum_.setZero();
    bias_.setZero();
    bias_samples_ = 0;
    ramp_cycles_ = 0;
    hold_delay_cycles_ = 0;
    phase_ = Phase::RampHold;
  }

  Phase phase_ = Phase::ClearLimits;
  const int bias_delay_cycles_;
  int settled_cycles_ = 0;
  int ramp_cycles_ = 0;
  int hold_delay_cycles_ = 0;
  int bias_samples_ = 0;
  Vector hold_pose_ = Vector::Zero();
  Vector ramp_start_torques_ = Vector::Zero();
  Vector bias_sum_ = Vector::Zero();
  Vector bias_ = Vector::Zero();
};

} // namespace Sai::Flexiv

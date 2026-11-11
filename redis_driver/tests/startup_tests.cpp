#include "SaiFlexivStartupController.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

using Controller = Sai::Flexiv::StartupController;
using Phase = Controller::Phase;
using Vector = Controller::Vector;
constexpr int kDefaultBiasDelayMs = 1000;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

struct Fixture {
  Controller controller;
  Controller::Limits limits{Vector::Constant(-1.0), Vector::Constant(1.0),
                            Vector::Constant(-1.2), Vector::Constant(1.2),
                            Vector::Constant(100.0)};
  Vector q = Vector::Zero();
  Vector dq = Vector::Zero();
  Vector damping = Vector::Constant(10.0);

  explicit Fixture(int bias_delay_ms = kDefaultBiasDelayMs)
      : controller(bias_delay_ms) {}

  Controller::Output step() {
    return controller.update(q, dq, limits, damping);
  }

  Controller::Output cycles(int count) {
    Controller::Output output;
    for (int i = 0; i < count; ++i) {
      output = step();
    }
    return output;
  }
};

void hard_limit_recovers_and_captures_automatically(double direction) {
  Fixture f;
  f.q(2) = direction * 1.05;
  auto output = f.step();
  require(direction * output.torques(2) < 0, "hard-limit recovery must push inward");
  require(!output.captured_pose, "hard-limit recovery must precede pose capture");
  require(f.controller.bias_samples() == 0, "recovery must not collect bias");

  // Recovery remains active beyond the hard boundary to cross the clearance margin.
  f.q(2) = direction * (1.0 - Controller::kClearanceMargin / 2.0);
  require(direction * f.step().torques(2) < 0, "recovery must continue through the margin");
  require(f.controller.phase() == Phase::ClearLimits, "margin must be satisfied");

  f.q.setConstant(0.1);
  output = f.cycles(Controller::kSettlingCycles - 1);
  require(!output.captured_pose, "clearance must include a complete settling window");
  f.q.setConstant(0.2);
  output = f.step();
  require(output.captured_pose, "either cleared hard region must capture automatically");
  require((f.controller.hold_pose() - f.q).norm() == 0,
          "capture must use the current pose, not the initial hard-limit pose");
  require(f.controller.phase() == Phase::RampHold, "capture must begin ramp");
  require(f.controller.bias().norm() == 0, "capture must reset the bias");
}

void normal_and_mixed_limit_starts_capture_automatically() {
  Fixture normal;
  require(!normal.cycles(Controller::kSettlingCycles - 1).captured_pose,
          "clearance needs a complete settling window");
  require(normal.step().captured_pose, "normal starts capture automatically");

  Fixture mixed;
  mixed.q(0) = -1.05;
  mixed.q(6) = 1.05;
  const auto output = mixed.step();
  require(output.torques(0) > 0 && output.torques(6) < 0,
          "simultaneous upper and lower limits must both recover inward");
  mixed.q.setConstant(0.1);
  require(mixed.cycles(Controller::kSettlingCycles).captured_pose,
          "cleared simultaneous hard limits must capture automatically");
}

void settling_restarts_after_motion_or_limit_reentry(double direction) {
  Fixture f;
  f.q(0) = direction * 1.05;
  f.step();
  f.q.setZero();
  f.cycles(Controller::kSettlingCycles - 1);
  f.dq(1) = 2 * Controller::kSettlingVelocity;
  f.step();
  f.dq.setZero();
  f.cycles(Controller::kSettlingCycles - 1);
  require(f.controller.phase() == Phase::ClearLimits, "motion must reset settling time");
  require(f.step().captured_pose, "settling must automatically capture the pose");

  f.q(0) = direction * 1.05;
  require(!f.step().captured_pose, "loss of clearance must invalidate the capture");
  require(f.controller.phase() == Phase::ClearLimits, "limit re-entry must recover again");
  f.q.setZero();
  require(f.cycles(Controller::kSettlingCycles).captured_pose,
          "renewed clearance must automatically capture again");
}

void ramp_delay_and_bias_sampling_are_separate() {
  Fixture f;
  f.cycles(Controller::kSettlingCycles);
  f.q(3) = 0.001; // A stationary offset requiring -1 N m of startup hold torque.
  const auto first_ramp = f.step();
  require(first_ramp.torques(3) < 0 && first_ramp.torques(3) > -0.001,
          "hold torque must ramp from the recovery torque");
  f.cycles(Controller::kRampCycles - 1);
  require(f.controller.phase() == Phase::HoldBeforeBias, "ramp must be followed by a hold delay");
  require(f.controller.bias_samples() == 0, "ramp torques must not contaminate the bias");
  Vector expected = Vector::Zero();
  expected(3) = -1;
  for (int i = 0; i < kDefaultBiasDelayMs; ++i) {
    require(f.controller.phase() == Phase::HoldBeforeBias,
            "sampling must wait for the entire one-second delay");
    require((f.step().torques - expected).norm() < 1e-10,
            "full holding control must remain active throughout the delay");
    require(f.controller.bias_samples() == 0 && f.controller.bias().norm() == 0,
            "delay torques must not enter the bias estimate");
  }
  require(f.controller.phase() == Phase::EstimateBias, "delay must finish before sampling");
  f.q(3) = 0.0005; // Distinguish sampled torque from the -1 N m during the delay.
  f.cycles(100);
  f.dq(0) = 2 * Controller::kSettlingVelocity;
  f.step();
  require(f.controller.bias_samples() == 0, "motion must discard partial bias samples");
  f.dq.setZero();
  f.cycles(Controller::kBiasSamples - 1);
  require(f.controller.phase() == Phase::EstimateBias, "bias needs all settled samples");
  const auto output = f.step();
  require(output.became_ready, "completed bias sampling must finish startup");
  expected(3) = -0.5;
  require((f.controller.bias() - expected).norm() < 1e-10,
          "bias must be the mean settled hold torque");
}

void startup_hold_limits_goal_velocity_and_preserves_damping() {
  Fixture f;
  f.q.setConstant(0.2);
  f.cycles(Controller::kSettlingCycles);
  // Exercise both directions, small errors, and joints already at the goal.
  f.q(0) += 0.1;
  f.q(1) -= 0.1;
  f.q(2) += 0.00025;
  f.q(3) -= 0.00025;
  const double restoring_limit =
      Controller::kHoldingKv * Controller::kHoldingVelocityLimit;
  Controller::Output output;
  for (int i = 0; i < Controller::kRampCycles; ++i) {
    output = f.step();
    require(output.torques.cwiseAbs().maxCoeff() <= restoring_limit + 1e-10,
            "ramp must use the velocity-limited hold torque");
  }
  Vector expected = Vector::Zero();
  expected(0) = -restoring_limit;
  expected(1) = restoring_limit;
  expected(2) = -0.25;
  expected(3) = 0.25;
  require((output.torques - expected).norm() < 1e-10,
          "large errors must saturate independently; small errors retain PD stiffness");
  f.cycles(kDefaultBiasDelayMs);
  require(f.controller.phase() == Phase::EstimateBias, "holding test must reach bias estimation");

  f.q(0) += 0.1;
  f.q(1) -= 0.1;
  require((f.step().torques - expected).norm() < 1e-10,
          "larger displacements must not increase saturated restoring torque");

  f.dq(0) = 4 * Controller::kHoldingVelocityLimit;
  f.dq(1) = -4 * Controller::kHoldingVelocityLimit;
  f.dq(4) = 4 * Controller::kHoldingVelocityLimit;
  require((f.step().torques - (expected - Controller::kHoldingKv * f.dq)).norm() < 1e-10,
          "saturate only goal velocity, preserving full braking even at the goal");
  require(f.controller.bias_samples() == 0,
          "motion during velocity-limited holding must reset bias sampling");

  f.dq.setZero();
  require(f.cycles(Controller::kBiasSamples).became_ready,
          "velocity-limited holding must still complete settled bias estimation");
  require((f.controller.bias() - expected).norm() < 1e-10,
          "bias must use the saturated hold torque");
  require((f.controller.hold_pose() - Vector::Constant(0.2)).norm() < 1e-10,
          "velocity saturation must retain the captured goal pose");
}

void calibration_restarts_after_limit_reentry() {
  Fixture f;
  f.cycles(Controller::kSettlingCycles + Controller::kRampCycles + kDefaultBiasDelayMs + 50);
  require(f.controller.bias_samples() == 50, "test must reach bias estimation");
  f.q(4) = -1.05;
  const auto output = f.step();
  require(f.controller.phase() == Phase::ClearLimits, "unsafe calibration must restart recovery");
  require(f.controller.bias_samples() == 0 && f.controller.bias().norm() == 0,
          "unsafe calibration must discard its bias");
  require(output.torques(4) > 0, "recovery must replace the old hold torque");
  f.q.setConstant(0.2);
  require(f.cycles(Controller::kSettlingCycles).captured_pose,
          "renewed clearance must capture a fresh pose");
  require((f.controller.hold_pose() - f.q).norm() == 0, "old hold pose must not survive recovery");
}

void bias_delay_is_adjustable(int delay_ms) {
  Fixture f(delay_ms);
  f.cycles(Controller::kSettlingCycles + Controller::kRampCycles);
  f.q(0) = 0.0005;
  for (int i = 0; i < delay_ms; ++i) {
    require(f.controller.phase() == Phase::HoldBeforeBias, "configured delay must be respected");
    require(std::abs(f.step().torques(0) + 0.5) < 1e-10,
            "configured delay must apply holding torque");
    require(f.controller.bias_samples() == 0, "configured delay must exclude bias samples");
  }
  require(f.controller.phase() == Phase::EstimateBias, "zero or completed delay must allow sampling");
  f.step();
  require(f.controller.bias_samples() == 1, "first sample must follow the configured delay");
}

void delay_restarts_after_limit_reentry(double direction) {
  constexpr int delay_ms = 10;
  Fixture f(delay_ms);
  f.cycles(Controller::kSettlingCycles + Controller::kRampCycles + delay_ms / 2);
  require(f.controller.phase() == Phase::HoldBeforeBias, "test must interrupt the hold delay");
  f.q(4) = direction * 1.05;
  const auto output = f.step();
  require(f.controller.phase() == Phase::ClearLimits && direction * output.torques(4) < 0,
          "limit re-entry during the delay must resume inward recovery");
  f.q.setConstant(0.2);
  require(f.cycles(Controller::kSettlingCycles).captured_pose,
          "renewed clearance must automatically capture a fresh pose");
  f.cycles(Controller::kRampCycles + delay_ms - 1);
  require(f.controller.phase() == Phase::HoldBeforeBias && f.controller.bias_samples() == 0,
          "recovery must restart the full delay rather than reuse elapsed hold time");
  f.step();
  require(f.controller.phase() == Phase::EstimateBias && f.controller.bias_samples() == 0,
          "sampling must follow the restarted delay");
}

int main() {
  try {
    hard_limit_recovers_and_captures_automatically(1.0);
    hard_limit_recovers_and_captures_automatically(-1.0);
    normal_and_mixed_limit_starts_capture_automatically();
    settling_restarts_after_motion_or_limit_reentry(1.0);
    settling_restarts_after_motion_or_limit_reentry(-1.0);
    ramp_delay_and_bias_sampling_are_separate();
    startup_hold_limits_goal_velocity_and_preserves_damping();
    calibration_restarts_after_limit_reentry();
    bias_delay_is_adjustable(0);
    bias_delay_is_adjustable(7);
    bias_delay_is_adjustable(1750);
    delay_restarts_after_limit_reentry(1.0);
    delay_restarts_after_limit_reentry(-1.0);
    std::cout << "Startup sequence and bias-delay tests passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

#include "SaiFlexivIdleController.h"

#include <iostream>
#include <stdexcept>

using Controller = Sai::Flexiv::IdleController;
using Vector = Controller::Vector;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

bool near(const Vector &a, const Vector &b) { return (a - b).norm() < 1e-10; }

struct Fixture {
  Controller controller;
  Vector q = Vector::Zero();
  Vector dq = Vector::Zero();
  Controller::Matrix mass = Controller::Matrix::Identity();
  Vector command = Vector::Zero(); // Controller request including its bias.
  Vector floating = Vector::Zero();

  Vector step(bool active = false, bool touching = false) {
    return controller.update(active, touching, q, dq, mass, command, floating);
  }

  Vector cycles(int count, bool active = false, bool touching = false) {
    Vector output = Vector::Zero();
    for (int i = 0; i < count; ++i) {
      output = step(active, touching);
    }
    return output;
  }
};

void stopping_discards_the_old_hold_pose_and_expired_timer() {
  Fixture f;
  f.q.setConstant(-0.4);
  f.controller.reset_hold_pose(f.q);
  f.cycles(Controller::kNotTouchingCycles + 50);
  require(f.controller.not_touching_cycles() == Controller::kNotTouchingCycles,
          "test must begin with an expired idle timer");

  f.q.setConstant(0.5);
  f.command << 2.0, -1.0, 0.4, -0.3, 0.2, 0.1, -0.1;
  f.floating.setConstant(-0.2);
  const Vector previous = f.step(true);
  require(near(previous, f.command + f.floating), "active torques include bias and damping");

  f.q.setConstant(0.6);
  f.dq.setConstant(0.2);
  f.floating.setConstant(-0.4);
  require(near(f.step(), previous), "first zero cycle must retain the previous nominal torque");
  require(near(f.controller.hold_pose(), f.q), "stop must capture the current pose");
  require(f.controller.not_touching_cycles() == 1,
          "stop must reset the timer before counting the first idle cycle");

  f.q.setConstant(0.65);
  f.dq.setZero();
  f.floating.setZero();
  require(near(f.cycles(Controller::kHandoffCycles), Vector::Zero()),
          "handoff must release the previous controller torque and bias");
  require(near(f.cycles(Controller::kNotTouchingCycles), Vector::Zero()),
          "resuming hold must not pull toward the pose from before the controller ran");
  require(near(f.controller.hold_pose(), f.q), "release window must follow the new pose");
}

void handoff_is_continuous_and_finishes_on_repeated_zeros() {
  Fixture f;
  f.command.setConstant(4.0);
  f.floating.setConstant(-0.5);
  const Vector start = f.step(true);
  f.command.setConstant(0.3); // Bias must not remain in idle control.
  f.dq.setConstant(0.05);
  f.floating.setConstant(-0.1);
  Vector idle = Vector::Constant(-0.6); // -10*dq plus floating damping.
  idle.tail<3>().setConstant(-0.675); // Added wrist inertia also scales idle damping.
  Vector previous = f.step();
  require(near(previous, start), "handoff must start without a nominal torque step");
  const double maximum_step =
      1.5 * (start - idle).cwiseAbs().maxCoeff() / Controller::kHandoffCycles;
  for (int i = 0; i < Controller::kHandoffCycles; ++i) {
    const Vector output = f.step();
    require((output.array() >= idle.array() - 1e-10).all() &&
                (output.array() <= previous.array() + 1e-10).all(),
            "fixed-target handoff must decay monotonically without overshoot");
    require((output - previous).cwiseAbs().maxCoeff() <= maximum_step + 1e-10,
            "fixed-target handoff must spread the torque change over the ramp");
    previous = output;
  }
  require(near(previous, idle) && !f.controller.handoff_active(),
          "repeated zero commands must finish rather than restart the handoff");
  require(near(f.cycles(Controller::kNotTouchingCycles), idle),
          "idle control must not retain the external torque bias");
}

void a_new_controller_cancels_the_handoff_and_the_next_stop_recaptures() {
  Fixture f;
  f.command.setConstant(3.0);
  f.step(true);
  f.cycles(40);
  require(f.controller.handoff_active(), "test must interrupt a running handoff");

  f.q.setConstant(-0.3);
  f.command.setConstant(-2.0);
  f.floating.setConstant(0.2);
  const Vector restarted = f.step(true);
  require(near(restarted, f.command + f.floating) && !f.controller.handoff_active(),
          "a new active command must replace the interrupted handoff");
  f.q.setConstant(-0.5);
  require(near(f.step(), restarted), "the next stop must start from the new controller torque");
  require(near(f.controller.hold_pose(), f.q) && f.controller.not_touching_cycles() == 1,
          "every stop must capture a fresh pose and reset the timer");
}

void idle_hold_limits_goal_velocity_and_keeps_mass_weighted_damping() {
  Fixture f;
  f.q.setConstant(0.25);
  f.cycles(Controller::kNotTouchingCycles);
  require(!f.controller.handoff_active(), "initial zeros must not create a shutdown ramp");
  f.mass(0, 0) = 2.0;
  f.mass(1, 1) = 1.5;
  f.mass(0, 1) = f.mass(1, 0) = 0.25;
  f.mass(4, 4) = 0.4;
  f.mass(5, 5) = 0.6;
  f.mass(6, 6) = 0.8;
  f.mass(4, 5) = f.mass(5, 4) = 0.05;
  const Controller::Matrix original_mass = f.mass;
  f.q(0) += 0.002; // Below saturation: preserve the original stiffness.
  f.q(1) -= 0.1;
  f.q(2) += 0.1;
  f.q(4) += 0.002;
  f.q(5) -= 0.1;
  f.q(6) += 0.1;
  f.dq(0) = 0.02;
  f.dq(1) = -0.03;
  f.dq(3) = 0.4; // At the goal, retain braking beyond the goal velocity limit.
  f.dq(4) = 0.02;
  f.dq(6) = -0.02;
  f.floating.setConstant(0.1);
  const double restoring_limit =
      Controller::kHoldingKv * Controller::kHoldingVelocityLimit;
  Vector expected;
  // Wrist diagonal entries become 0.55, 0.75, 0.95; coupling stays unchanged.
  expected << -1.2 + 0.25 * (restoring_limit + 0.3),
      -0.15 + 1.5 * (restoring_limit + 0.3), -restoring_limit, -4.0,
      -0.33 + 0.05 * restoring_limit, -0.03 + 0.75 * restoring_limit,
      0.95 * (-restoring_limit + 0.2);
  expected += f.floating;
  require(near(f.step(), expected),
          "idle hold must saturate both directions and retain mass coupling and damping");
  f.q(1) -= 0.5;
  f.q(2) += 0.5;
  f.q(5) -= 0.5;
  f.q(6) += 0.5;
  require(near(f.step(), expected), "larger errors must not increase saturated restoring torque");
  require(f.mass.isApprox(original_mass), "idle inertia offsets must not modify the model matrix");
  require(near(f.controller.hold_pose(), Vector::Constant(0.25)),
          "hold must retain the pose captured at the end of the release window");

  require(near(f.step(false, true), f.floating), "touch must release the position hold");
  require(f.controller.not_touching_cycles() == 0 && near(f.controller.hold_pose(), f.q),
          "touch must capture the current pose and restart the release window");
  Vector release_torques = -Controller::kHoldingKv * f.mass * f.dq + f.floating;
  release_torques.tail<3>() -= 0.15 * Controller::kHoldingKv * f.dq.tail<3>();
  require(near(f.step(), release_torques),
          "release window must provide damping without position restoring torque");
}

int main() {
  try {
    stopping_discards_the_old_hold_pose_and_expired_timer();
    handoff_is_continuous_and_finishes_on_repeated_zeros();
    a_new_controller_cancels_the_handoff_and_the_next_stop_recaptures();
    idle_hold_limits_goal_velocity_and_keeps_mass_weighted_damping();
    std::cout << "Idle hold and controller shutdown tests passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

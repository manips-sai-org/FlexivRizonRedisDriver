/**
 * @file main.cpp
 * @brief Flexiv Rizon robot torque driver with joint safeties
 */
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "SaiFlexivDriverConfig.h"
#include "SaiFlexivRedisClientLocal.h"

#include <flexiv/gripper.h>
#include <flexiv/log.h>
#include <flexiv/model.h>
#include <flexiv/robot.h>
#include <flexiv/scheduler.h>
#include <flexiv/utility.h>

#include <atomic>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

// redis keys
// - read:
std::string JOINT_TORQUES_COMMANDED_KEY;
std::string GRIPPER_PARAMETERS_COMMANDED_KEY;
std::string GRIPPER_MODE_KEY;
// - write:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;
std::string WRIST_FORCE_SENSED_KEY;
std::string WRIST_MOMENT_SENSED_KEY;
std::string SAFETY_TORQUES_LOGGING_KEY;
std::string SENT_TORQUES_LOGGING_KEY;
std::string CONSTRAINED_NULLSPACE_KEY;
std::string GRIPPER_CURRENT_WIDTH_KEY;
std::string GRIPPER_SENSED_GRASP_FORCE_KEY;

// user options
const bool USING_4S =
    true; // set if using the Rizon 4s with wrist force-torque sensor
const bool VERBOSE = true; // print out safety violations

// globals
std::array<double, 7> joint_position_max_default;
std::array<double, 7> joint_position_min_default;
std::array<double, 7> joint_velocity_limits_default;
std::array<double, 7> joint_torques_limits_default;
std::array<double, 7> kv_safety;
std::array<double, 2> pos_zones;
std::array<double, 2> vel_zones;

// safety joint limits to trigger safety stop before hard limits
std::array<double, 7> joint_position_max;
std::array<double, 7> joint_position_min;
std::array<double, 7> joint_velocity_limits;
std::array<double, 7> joint_velocity_upper_limits;
std::array<double, 7> joint_velocity_lower_limits;
std::array<double, 7> joint_torques_limits;

// workspace safety monitoring
const Eigen::Vector3d monitoring_point_ee_frame =
    Eigen::Vector3d(0.0, 0.0, 0.15);
const double safety_plane_z_coordinate = 0.28;
const double safety_cylinder_radius = 0.28;
const double safety_cylinder_height = 0.53;
bool safety_mode_flag = false;
bool safety_enabled = false;
int safety_controller_count = 200;

enum Limit {
    SAFE = 0,
    MIN_SOFT, // soft lower position limit
    MIN_HARD, // hard lower position limit
    MAX_SOFT, // soft upper position limit
    MAX_HARD, // hard upper position limit
    MIN_SOFT_VEL,
    MIN_HARD_VEL,
    MAX_SOFT_VEL,
    MAX_HARD_VEL
};

// data
Eigen::MatrixXd MassMatrix;
std::array<double, 7> tau_cmd_array{};
std::array<double, 7> q_array{};
std::array<double, 7> dq_array{};
std::array<double, 7> tau_sensed_array{};
std::array<double, 6> wrist_ft_sensed_array{};
// std::array<double, 7> gravity_vector{};
// std::array<double, 7> coriolis{};
// std::array<double, 49> M_array{};
Eigen::VectorXd gravity_vector{};
Eigen::VectorXd coriolis{};
Eigen::MatrixXd M_array{};
std::vector<std::array<double, 7>> sensor_feedback;
std::array<double, 3> wrist_ft_sensed_force{};
std::array<double, 3> wrist_ft_sensed_moment{};
std::array<double, 1> gripper_current_width{};
std::array<double, 1> gripper_sensed_grasp_force{};
std::vector<std::string> key_names;
// bool fDriverRunning = true;
// void sighandler(int sig)
// { fDriverRunning = false; }

// gripper command storage
Eigen::Vector3d gripper_parameters =
    Eigen::Vector3d(0.06, 0.1, 10.0); // width in m, speed in m/s, force in N
Eigen::Vector3d last_gripper_parameters = gripper_parameters;
std::string gripper_mode = "o";
std::string last_gripper_mode = gripper_mode;
double gripper_width;
double gripper_speed;
double gripper_force;

// limit options
bool _pos_limit_opt = true;
bool _vel_limit_opt = true;

// setup joint limit avoidance
std::vector<int> _pos_limit_flag{7, SAFE};
std::vector<int> _vel_limit_flag{7, SAFE};
Eigen::MatrixXd
    _J_s; // constraint task jacobian for nullspace projection (n_limited x dof)
Eigen::MatrixXd _Lambda_s; // op-space matrix
Eigen::MatrixXd _Jbar_s;
Eigen::MatrixXd _N_s = Eigen::MatrixXd::Identity(
    7, 7); // nullspace matrix with the constraint task jacobian
Eigen::VectorXi
    _limited_joints(7); // 1 or 0 depending on whether the joint is limited
Eigen::VectorXd _tau_limited = Eigen::VectorXd::Zero(7);
Eigen::VectorXd _torque_scaling_vector =
    Eigen::VectorXd::Ones(7); // tau scaling based on the velocity signal

// override with new safety set (this set triggers software stop)
double default_sf = 0.98; // max violation safety factors

// zone 1 and 2 definitions subject to tuning
double soft_sf = 0.90;             // start of damping zone
double hard_sf = 0.95;             // start of feedback zone
double angle_tol = 1 * M_PI / 180; // rad
double vel_tol = 0.1;              // rad/s (0.1 = 5 deg/s)
double q_tol = 1e-1 * M_PI / 180;

Eigen::VectorXd soft_min_angles(7);
Eigen::VectorXd soft_max_angles(7);
Eigen::VectorXd hard_min_angles(7);
Eigen::VectorXd hard_max_angles(7);
Eigen::VectorXd soft_min_joint_velocity_limits(7);
Eigen::VectorXd hard_min_joint_velocity_limits(7);
Eigen::VectorXd soft_max_joint_velocity_limits(7);
Eigen::VectorXd hard_max_joint_velocity_limits(7);

// initial torque bias
Eigen::VectorXd init_torque_bias = Eigen::VectorXd::Zero(7);
int n_samples = 500;
int n_curr = 0;
bool initialized_torque_bias = false;
std::vector<double> kp_holding = {1000, 1000, 1000, 1000, 1000, 1000, 1000};
std::vector<double> kv_holding = {10, 10, 10, 10, 10, 10, 10};
Eigen::VectorXd q_init = Eigen::VectorXd::Zero(7);
bool first_loop = true;

// timing
std::clock_t start;
double duration;

unsigned long long counter = 0;

// driver config
Sai::Flexiv::DriverConfig driver_config;

// void redis_transfer(CDatabaseRedisClient* redis_client)
// {
//   while(fDriverRunning)
//   {
//     Eigen::Map<const Eigen::Matrix<double, 7, 7> >
//     MassMatrix(M_array.data()); redis_client->setGetBatchCommands(key_names,
//     tau_cmd_array, MassMatrix, sensor_feedback);
//   }
// }

const std::vector<std::string> limit_state{
    "Safe",         "Soft Min",     "Hard Min",     "Soft Max",    "Hard Max",
    "Min Soft Vel", "Min Hard Vel", "Max Soft Vel", "Max Hard Vel"};

/** Joint velocity damping gains for floating */
const std::array<double, flexiv::kJointDOF> kFloatingDamping = {
    10.0, 10.0, 5.0, 5.0, 1.0, 1.0, 1.0};

/** Atomic signal to stop scheduler tasks */
std::atomic<bool> g_stop_sched = {false};

double getBlendingCoeff(const double &val, const double &low,
                        const double &high) {
    return std::clamp((val - low) / (high - low), 0., 1.);
}

std::array<double, 7> getMaxJointVelocity(std::array<double, 7> &q) {
    std::array<double, 7> dq_max;
    dq_max[0] = std::min(
        joint_velocity_limits_default[0],
        std::max(0., -0.3 + sqrt(std::max(0., 12.0 * (2.75010 - q[0])))));
    dq_max[1] = std::min(
        joint_velocity_limits_default[1],
        std::max(0., -0.2 + sqrt(std::max(0., 5.17 * (1.79180 - q[1])))));
    dq_max[2] = std::min(
        joint_velocity_limits_default[2],
        std::max(0., -0.2 + sqrt(std::max(0., 7.00 * (2.90650 - q[2])))));
    dq_max[3] = std::min(
        joint_velocity_limits_default[3],
        std::max(0., -0.3 + sqrt(std::max(0., 8.00 * (-0.1458 - q[3])))));
    dq_max[4] = std::min(
        joint_velocity_limits_default[4],
        std::max(0., -0.35 + sqrt(std::max(0., 34.0 * (2.81010 - q[4])))));
    dq_max[5] = std::min(
        joint_velocity_limits_default[5],
        std::max(0., -0.35 + sqrt(std::max(0., 11.0 * (4.52050 - q[5])))));
    dq_max[6] = std::min(
        joint_velocity_limits_default[6],
        std::max(0., -0.35 + sqrt(std::max(0., 34.0 * (3.01960 - q[6])))));
    return dq_max;
}

std::array<double, 7> getMinJointVelocity(std::array<double, 7> &q) {
    std::array<double, 7> dq_min;
    dq_min[0] = std::max(
        -joint_velocity_limits_default[0],
        std::min(0., 0.3 - sqrt(std::max(0., 12.0 * (2.75010 + q[0])))));
    dq_min[1] = std::max(
        -joint_velocity_limits_default[1],
        std::min(0., 0.2 - sqrt(std::max(0., 5.17 * (1.79180 + q[1])))));
    dq_min[2] = std::max(
        -joint_velocity_limits_default[2],
        std::min(0., 0.2 - sqrt(std::max(0., 7.00 * (2.90650 + q[2])))));
    dq_min[3] = std::max(
        -joint_velocity_limits_default[3],
        std::min(0., 0.3 - sqrt(std::max(0., 8.00 * (3.04810 + q[3])))));
    dq_min[4] = std::max(
        -joint_velocity_limits_default[4],
        std::min(0., 0.35 - sqrt(std::max(0., 34.0 * (2.81010 + q[4])))));
    dq_min[5] = std::max(
        -joint_velocity_limits_default[5],
        std::min(0., 0.35 - sqrt(std::max(0., 11.0 * (-0.54092 + q[5])))));
    dq_min[6] = std::max(
        -joint_velocity_limits_default[6],
        std::min(0., 0.35 - sqrt(std::max(0., 34.0 * (3.01960 + q[6])))));
    return dq_min;
}

/** @brief Print program usage help */
void PrintHelp() {
    // clang-format off
    std::cout << "Required arguments: config_file_path" << std::endl;
    std::cout << "    For example: config_oberon.xml" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

/** @brief Callback function for realtime periodic task */
void PeriodicTask(flexiv::Robot &robot, flexiv::Gripper &gripper,
                  flexiv::Model &model, flexiv::Log &log,
                  Sai::Flexiv::CDatabaseRedisClient *redis_client) {

    try {

        redis_client->getEigenMatrixDerivedString(
            GRIPPER_PARAMETERS_COMMANDED_KEY, gripper_parameters);

        redis_client->getCommandIs(GRIPPER_MODE_KEY, gripper_mode);

        if ((gripper_parameters - last_gripper_parameters).norm() > 0.01) {
            gripper_width = gripper_parameters(0);
            gripper_speed = gripper_parameters(1);
            gripper_force = gripper_parameters(2);
            log.Info(
                "Moving Gripper - Width: " + std::to_string(gripper_width) +
                "m    Speed: " + std::to_string(gripper_speed) +
                "m/s    Force: " + std::to_string(gripper_force) + "N");
            gripper.Move(gripper_width, gripper_speed, gripper_force);
            last_gripper_parameters = gripper_parameters;
        }

        // if (gripper_mode != last_gripper_mode) {
        //     if (gripper_mode == "g") {
        //         log.Info("Closing Gripper");
        //         gripper.Move(0, 0.1, 60);
        //     } else if (gripper_mode == "o") {
        //         log.Info("Opening Gripper");
        //         gripper.Move(0.05, 0.1, 60);
        //     } else {
        //         log.Info("Invalid Gripper Command");
        //     }
        //     last_gripper_mode = gripper_mode;
        // }

        for (int i = 0; i < 7; ++i) {
            joint_position_max[i] = default_sf * joint_position_max_default[i];
            joint_position_min[i] = default_sf * joint_position_min_default[i];
            joint_velocity_limits[i] =
                default_sf * joint_velocity_limits_default[i];
            joint_torques_limits[i] =
                default_sf * joint_torques_limits_default[i];
        }

        for (int i = 0; i < 7; ++i) {
            soft_min_angles(i) =
                joint_position_min[i] + pos_zones[1] * angle_tol;
            hard_min_angles(i) =
                joint_position_min[i] + pos_zones[0] * angle_tol;
            soft_max_angles(i) =
                joint_position_max[i] - pos_zones[1] * angle_tol;
            hard_max_angles(i) =
                joint_position_max[i] - pos_zones[0] * angle_tol;
            soft_min_joint_velocity_limits(i) =
                -joint_velocity_limits[i] + vel_zones[1] * vel_tol;
            hard_min_joint_velocity_limits(i) =
                -joint_velocity_limits[i] + vel_zones[0] * vel_tol;
            soft_max_joint_velocity_limits(i) =
                joint_velocity_limits[i] - vel_zones[1] * vel_tol;
            hard_max_joint_velocity_limits(i) =
                joint_velocity_limits[i] - vel_zones[0] * vel_tol;

            // // specific joint offsets
            // if (i == 6) {
            //     soft_min_joint_velocity_limits(i) += 4.5 * vel_tol;
            //     hard_min_joint_velocity_limits(i) += 4.5 * vel_tol;
            //     soft_max_joint_velocity_limits(i) -= 4.5 * vel_tol;
            //     hard_max_joint_velocity_limits(i) -= 4.5 * vel_tol;
            // }
        }

        // Monitor fault on the connected robot
        if (robot.fault()) {
            throw std::runtime_error("PeriodicTask: Fault occurred on the "
                                     "connected robot, exiting ...");
        }

        auto robot_state = robot.states();
        model.Update(robot_state.q, robot_state.dq);
        auto gripper_state = gripper.states();

        // start = std::clock();
        sensor_feedback[0] = robot_state.q;
        sensor_feedback[1] = robot_state.dq; // non-filtered velocities
        // sensor_feedback[1] = dq_array;  // filtered velocities
        sensor_feedback[2] = robot_state.tau;
        wrist_ft_sensed_array = robot_state.ft_sensor_raw;
        gravity_vector = model.g();
        coriolis = model.c();
        MassMatrix = model.M();
        gripper_current_width[0] = gripper_state.width;
        gripper_sensed_grasp_force[0] = gripper_state.force;

        Eigen::Map<Eigen::Matrix<double, 7, 1>> _tau(tau_cmd_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> _sensed_torques(
            sensor_feedback[2].data()); // sensed torques
        Eigen::Map<Eigen::Matrix<double, 7, 1>> _coriolis(
            sensor_feedback[4].data());
        Eigen::MatrixXd MassMatrixInverse =
            MassMatrix.llt().solve(Eigen::MatrixXd::Identity(7, 7));

        redis_client->setGetBatchCommands(key_names, tau_cmd_array, MassMatrix,
                                          sensor_feedback);
        if (driver_config.robot_type == Sai::Flexiv::RobotType::RIZON_4S) {
            wrist_ft_sensed_force = {wrist_ft_sensed_array[0],
                                     wrist_ft_sensed_array[1],
                                     wrist_ft_sensed_array[2]};
            wrist_ft_sensed_moment = {wrist_ft_sensed_array[3],
                                      wrist_ft_sensed_array[4],
                                      wrist_ft_sensed_array[5]};
            redis_client->setDoubleArray(WRIST_FORCE_SENSED_KEY,
                                         wrist_ft_sensed_force, 3);
            redis_client->setDoubleArray(WRIST_MOMENT_SENSED_KEY,
                                         wrist_ft_sensed_moment, 3);
            redis_client->setDoubleArray(WRIST_FORCE_SENSED_KEY,
                                         wrist_ft_sensed_force, 3);
            redis_client->setDoubleArray(WRIST_MOMENT_SENSED_KEY,
                                         wrist_ft_sensed_moment, 3);
        }
        redis_client->setEigenMatrixDerived(ROBOT_GRAVITY_KEY, gravity_vector);
        redis_client->setEigenMatrixDerived(CORIOLIS_KEY, coriolis);
        redis_client->setDoubleArray(GRIPPER_CURRENT_WIDTH_KEY,
                                     gripper_current_width, 1);
        redis_client->setDoubleArray(GRIPPER_SENSED_GRASP_FORCE_KEY,
                                     gripper_sensed_grasp_force, 1);

        // reset containers
        _limited_joints.setZero(); // used to form the constraint jacobian
        _N_s.setIdentity();

        // position limit saturation
        if (_pos_limit_opt) {
            for (int i = 0; i < 7; ++i) {
                double curr_q = robot_state.q[i];
                if (curr_q > soft_min_angles(i) &&
                    curr_q < soft_max_angles(i)) {
                    _pos_limit_flag[i] = SAFE;
                    // std::cout << "joint" << i << " SAFE" << "\n";
                } else if (curr_q < hard_min_angles(i)) {
                    _pos_limit_flag[i] = MIN_HARD;
                    std::cout << "joint" << i << "MIN_HARD" << "\n";
                    _limited_joints(i) = 1;
                } else if (curr_q < soft_min_angles(i)) {
                    _pos_limit_flag[i] = MIN_SOFT;
                    std::cout << "joint" << i << "MIN_SOFT" << "\n";
                    _limited_joints(i) = 1;
                } else if (curr_q > hard_max_angles(i)) {
                    _pos_limit_flag[i] = MAX_HARD;
                    std::cout << "joint" << i << "MAX_HARD" << "\n";
                    _limited_joints(i) = 1;
                } else if (curr_q > soft_max_angles(i)) {
                    _pos_limit_flag[i] = MAX_SOFT;
                    std::cout << "joint" << i << "MAX_SOFT" << "\n";
                    _limited_joints(i) = 1;
                }
            }
        }

        // joint velocity limit saturation (lower priority than joint position
        // limit; _tau_unit_limited will be overwritten)
        if (_vel_limit_opt) {
            for (int i = 0; i < 7; ++i) {
                if (_pos_limit_flag[i] == SAFE) {
                    if (robot_state.dq[i] > soft_min_joint_velocity_limits[i] &&
                        robot_state.dq[i] < soft_max_joint_velocity_limits[i]) {
                        _vel_limit_flag[i] = SAFE;
                    } else if (robot_state.dq[i] >
                               hard_max_joint_velocity_limits[i]) {
                        _vel_limit_flag[i] = MAX_HARD_VEL;
                        _limited_joints(i) = 1;
                    } else if (robot_state.dq[i] >
                               soft_max_joint_velocity_limits[i]) {
                        _vel_limit_flag[i] = MAX_SOFT_VEL;
                        _limited_joints(i) = 1;
                    } else if (robot_state.dq[i] <
                               hard_min_joint_velocity_limits[i]) {
                        _vel_limit_flag[i] = MIN_HARD_VEL;
                        _limited_joints(i) = 1;
                    } else if (robot_state.dq[i] <
                               soft_min_joint_velocity_limits[i]) {
                        _vel_limit_flag[i] = MIN_SOFT_VEL;
                        _limited_joints(i) = 1;
                    }
                } else {
                    _vel_limit_flag[i] = SAFE;
                }
            }
        }

        // safety verbose output
        if (driver_config.verbose) {
            for (int i = 0; i < 7; ++i) {
                if (_pos_limit_flag[i] != SAFE) {
                    std::cout << counter << ": Joint " << i
                              << " State: " << limit_state[_pos_limit_flag[i]]
                              << "\n";
                    std::cout << "---\n";
                }
                if (_vel_limit_flag[i] != SAFE) {
                    std::cout << counter << ": Joint " << i
                              << " State: " << limit_state[_vel_limit_flag[i]]
                              << "\n";
                    std::cout << "---\n";
                }
            }
        }

        int n_limited_joints = _limited_joints.sum();
        _tau_limited = Eigen::VectorXd::Zero(7);
        _torque_scaling_vector = Eigen::VectorXd::Ones(7);

        if (n_limited_joints > 0) {
            for (int i = 0; i < 7; ++i) {
                if (_pos_limit_flag[i] == MIN_SOFT) {
                    // ramping kv damping proportional to violation difference
                    // up to max damping + command torques (goes from tau -> tau
                    // + max damping) double kv = kv_safety[i] *
                    // (robot_state.q[i] - soft_min_angles[i]) /
                    // (hard_min_angles[i] - soft_min_angles[i]);
                    double alpha =
                        getBlendingCoeff(robot_state.q[i], soft_min_angles[i],
                                         hard_min_angles[i]);
                    _tau_limited(i) =
                        _tau(i) - alpha * kv_safety[i] * robot_state.dq[i];

                } else if (_pos_limit_flag[i] == MAX_SOFT) {
                    // same as above, but for the upper soft limit
                    // double kv = kv_safety[i] * (robot_state.q[i] -
                    // soft_max_angles[i]) / (hard_max_angles[i] -
                    // soft_max_angles[i]);
                    double alpha =
                        getBlendingCoeff(robot_state.q[i], soft_max_angles[i],
                                         hard_max_angles[i]);
                    _tau_limited(i) =
                        _tau(i) - alpha * kv_safety[i] * robot_state.dq[i];

                } else if (_pos_limit_flag[i] == MIN_HARD) {
                    // max damping + command torques blend with quadratic
                    // ramping (goes from tau + max damping -> holding tau + max
                    // damping with blending from tau to holding tau) double
                    // dist = (robot_state.q[i] - hard_min_angles[i]) /
                    // (joint_position_min[i] - hard_min_angles[i]);
                    double alpha =
                        getBlendingCoeff(robot_state.q[i], hard_min_angles[i],
                                         joint_position_min[i]);
                    double tau_hold = joint_torques_limits_default[i];
                    _tau_limited(i) = (1 - std::pow(alpha, 2)) * _tau(i) +
                                      std::pow(alpha, 2) * tau_hold -
                                      kv_safety[i] * robot_state.dq[i];

                } else if (_pos_limit_flag[i] == MAX_HARD) {
                    // same as above, but for the upper hard limit
                    // double dist = (robot_state.q[i] - hard_max_angles[i]) /
                    // (joint_position_max[i] - hard_max_angles[i]);
                    double alpha =
                        getBlendingCoeff(robot_state.q[i], hard_max_angles[i],
                                         joint_position_max[i]);
                    double tau_hold = -joint_torques_limits_default[i];
                    _tau_limited(i) = (1 - std::pow(alpha, 2)) * _tau(i) +
                                      std::pow(alpha, 2) * tau_hold -
                                      kv_safety[i] * robot_state.dq[i];

                } else if (_vel_limit_flag[i] == MIN_SOFT_VEL) {
                    // command torques blend with holding torques to soft joint
                    // velocity (goes from tau -> tau hold with blending)
                    double alpha = getBlendingCoeff(
                        robot_state.dq[i], soft_min_joint_velocity_limits[i],
                        hard_min_joint_velocity_limits[i]);
                    _tau_limited(i) = (1 - std::pow(alpha, 1)) * _tau(i);
                    // _torque_scaling_vector(i) = (1 - std::pow(alpha, 4));
                    // _torque_scaling_vector(i) =
                    // soft_min_joint_velocity_limits[i] / robot_state.dq[i];
                    // _tau(i) = 0;

                } else if (_vel_limit_flag[i] == MAX_SOFT_VEL) {
                    // same as above, but for the upper soft limit
                    double alpha = getBlendingCoeff(
                        robot_state.dq[i], soft_max_joint_velocity_limits[i],
                        hard_max_joint_velocity_limits[i]);
                    _tau_limited(i) = (1 - std::pow(alpha, 1)) * _tau(i);
                    // _torque_scaling_vector(i) = (1 - std::pow(alpha, 4));
                    // _torque_scaling_vector(i) =
                    // soft_max_joint_velocity_limits[i] / robot_state.dq[i];
                    // _tau(i) = 0;

                } else if (_vel_limit_flag[i] == MIN_HARD_VEL) {
                    // full damping with soft joint velocity limits goal and a
                    // quadratic ramp to max torque
                    double alpha = getBlendingCoeff(
                        robot_state.dq[i], hard_min_joint_velocity_limits[i],
                        -joint_velocity_limits_default[i]);
                    // _tau_vel_limited(i) = - kv_safety[i] * std::pow(alpha, 2)
                    // * (robot_state.dq[i] - 0 *
                    // hard_min_joint_velocity_limits[i]);
                    // _torque_scaling_vector(i) =
                    // soft_min_joint_velocity_limits[i] / robot_state.dq[i];
                    // _torque_scaling_vector(i) = 0; _tau_limited(i) =
                    // - kv_safety[i] * std::pow(alpha, 4) * (robot_state.dq[i]
                    // - 0 * hard_min_joint_velocity_limits[i]); _tau_limited(i)
                    // = - kv_safety[i] * std::pow(alpha, 4) *
                    // (robot_state.dq[i] - 0 *
                    // hard_min_joint_velocity_limits[i]); _tau_limited(i) = -
                    // kv_safety[i] * std::pow(alpha, 4) * (robot_state.dq[i] -
                    // 0 * hard_min_joint_velocity_limits[i]);
                    _tau_limited(i) = std::pow(alpha, 4) *
                                      joint_torques_limits_default[i] * 1e-2;
                    // _tau(i) = 0;

                } else if (_vel_limit_flag[i] == MAX_HARD_VEL) {
                    // same as above, but for the upper hard limit
                    double alpha = getBlendingCoeff(
                        robot_state.dq[i], hard_max_joint_velocity_limits[i],
                        joint_velocity_limits_default[i]);
                    // _tau_vel_limited(i) = - kv_safety[i] * std::pow(alpha, 2)
                    // * (robot_state.dq[i] - 0 *
                    // hard_max_joint_velocity_limits[i]);
                    // _torque_scaling_vector(i) = 0;
                    // _torque_scaling_vector(i) =
                    // soft_max_joint_velocity_limits[i] / robot_state.dq[i];
                    // _tau_limited(i) = - kv_safety[i] * std::pow(alpha, 4) *
                    // (robot_state.dq[i] - 0 *
                    // hard_max_joint_velocity_limits[i]);
                    _tau_limited(i) = -std::pow(alpha, 4) *
                                      joint_torques_limits_default[i] * 1e-2;
                    // _tau(i) = 0;
                }
            }

            // compute revised torques
            _J_s = Eigen::MatrixXd::Zero(n_limited_joints, 7);
            int cnt = 0;
            for (int i = 0; i < 7; ++i) {
                if (_limited_joints(i)) {
                    _J_s(cnt, i) = 1;
                    cnt++;
                }
            }
            _Lambda_s = (_J_s * MassMatrixInverse * _J_s.transpose()).inverse();
            _Jbar_s = MassMatrixInverse * _J_s.transpose() * _Lambda_s;
            _N_s = Eigen::MatrixXd::Identity(7, 7) - _Jbar_s * _J_s;
            _tau = _tau_limited + _N_s.transpose() * _tau;

            for (int i = 0; i < 7; ++i) {
                tau_cmd_array[i] = _tau(i);
            }
        }

        // safey keys
        _N_s.setIdentity();
        redis_client->setEigenMatrixDerived(SAFETY_TORQUES_LOGGING_KEY,
                                            _tau_limited);
        redis_client->setEigenMatrixDerived(SENT_TORQUES_LOGGING_KEY, _tau);
        redis_client->setEigenMatrixDerived(CONSTRAINED_NULLSPACE_KEY, _N_s);

        // safety checks
        // joint torques, velocity and positions
        for (int i = 0; i < 7; ++i) {
            // torque saturation
            if (tau_cmd_array[i] > joint_torques_limits[i]) {
                std::cout << "WARNING : Torque commanded on joint " << i
                          << " too high (" << tau_cmd_array[i];
                std::cout << "), saturating to max value("
                          << joint_torques_limits[i] << ")" << std::endl;
                tau_cmd_array[i] = joint_torques_limits[i];
            }

            if (tau_cmd_array[i] < -joint_torques_limits[i]) {
                std::cout << "WARNING : Torque commanded on joint " << i
                          << " too low (" << tau_cmd_array[i];
                std::cout << "), saturating to min value("
                          << -joint_torques_limits[i] << ")" << std::endl;
                tau_cmd_array[i] = -joint_torques_limits[i];
            }
            // position limit
            if (robot_state.q[i] > joint_position_max[i]) {
                safety_mode_flag = true;
                if (safety_controller_count == 200) {
                    std::cout
                        << "WARNING : Soft joint upper limit violated on joint "
                        << i << ", engaging safety mode" << std::endl;
                }
            }
            if (robot_state.q[i] < joint_position_min[i]) {
                safety_mode_flag = true;
                if (safety_controller_count == 200) {
                    std::cout
                        << "WARNING : Soft joint lower limit violated on joint "
                        << i << ", engaging safety mode" << std::endl;
                }
            }
            // velocity limit
            if (abs(robot_state.dq[i]) > joint_velocity_limits[i]) {
                safety_mode_flag = true;
                if (safety_controller_count == 200) {
                    std::cout
                        << "WARNING : Soft velocity limit violated on joint "
                        << i << ", engaging safety mode" << std::endl;
                }
            }
        }

        // if (safety_enabled) {
        //     // cartesian checks
        //     Eigen::Affine3d
        //     T_EE(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        //     Eigen::Vector3d pos_monitoring_point =
        //     T_EE*monitoring_point_ee_frame; double radius_square =
        //     pos_monitoring_point(0)*pos_monitoring_point(0) +
        //     pos_monitoring_point(1)*pos_monitoring_point(1); double z_ee =
        //     pos_monitoring_point(2);

        //     // lower plane
        //     if (z_ee < safety_plane_z_coordinate) {
        //         safety_mode_flag = true;
        //         if (safety_controller_count == 200) {
        //             std::cout << "WARNING : End effector too low, engaging
        //             safety mode" << std::endl; std::cout << "position of
        //             monitoring point : " << pos_monitoring_point.transpose()
        //             << std::endl;
        //         }
        //     }
        //     // cylinder
        //     if (z_ee < safety_cylinder_height && radius_square <
        //     safety_cylinder_radius*safety_cylinder_radius) {
        //         safety_mode_flag = true;
        //         if (safety_controller_count == 200) {
        //             std::cout << "WARNING : End effector too close to center
        //             of workspace, engaging safety mode" << std::endl;
        //             std::cout << "position of monitoring point : " <<
        //             pos_monitoring_point.transpose() << std::endl;
        //         }
        //     }
        //     // std::cout << pos_monitoring_point.transpose() << std::endl;
        // }

        // safety mode
        if (safety_mode_flag) {
            for (int i = 0; i < 7; ++i) {
                tau_cmd_array[i] = -kv_safety[i] * robot_state.dq[i];
            }

            if (safety_controller_count == 0) {
                throw std::runtime_error(
                    "Stopping driver due to safety violation");
            }
            safety_controller_count--;
        }

        // Monitor fault on the connected robot
        if (robot.fault()) {
            throw std::runtime_error("PeriodicTask: Fault occurred on the "
                                     "connected robot, exiting ...");
        }

        // Set joint torques to command torques from Redis
        std::array<double, flexiv::kJointDOF> target_torque = tau_cmd_array;

        // Set 0 joint toruqes
        // std::array<double, flexiv::kJointDOF> target_torque = {};

        // Initialization at the start
        if (!initialized_torque_bias) {
            if (first_loop) {
                for (int i = 0; i < 7; ++i) {
                    q_init(i) = robot_state.q[i];
                }
            }
            first_loop = false;

            for (int i = 0; i < 7; ++i) {
                target_torque[i] =
                    -kp_holding[i] * (robot_state.q[i] - q_init(i)) -
                    kv_holding[i] * robot_state.dq[i];
                init_torque_bias(i) += (1. / n_samples) * target_torque[i];
            }

            if (n_curr > n_samples) {
                initialized_torque_bias = true;
                std::cout << "Initial torque bias: "
                          << init_torque_bias.transpose() << "\n";
            }
            n_curr++;

        } else {
            for (int i = 0; i < 7; ++i) {
                target_torque[i] += init_torque_bias(i);
            }
        }

        // Add some velocity damping
        for (size_t i = 0; i < flexiv::kJointDOF; ++i) {
            target_torque[i] += -kFloatingDamping[i] * robot.states().dtheta[i];
            // std::cout << target_torque[i] << "\n";
        }

        // Send target joint torque to RDK server, enable gravity compensation
        // and joint limits soft protection
        robot.StreamJointTorque(target_torque, true, true);

        counter++;
    } catch (const std::exception &e) {
        std::cout << "Error \n"
                  << "\n";
        log.Error(e.what());
        g_stop_sched = true;
    }
}

int main(int argc, char **argv) {

    // Program Setup
    // =============================================================================================
    // Logger for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse parameters
    if (argc < 2 ||
        flexiv::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
        PrintHelp();
        return 1;
    }
    // config file path
    std::string config_file = argv[1];
    std::string config_file_path =
        std::string(CONFIG_FOLDER) + "/" + config_file;
    driver_config = Sai::Flexiv::loadConfig(config_file_path);

    std::string redis_prefix = driver_config.redis_prefix.empty()
                                   ? ""
                                   : driver_config.redis_prefix + "::";

    JOINT_TORQUES_COMMANDED_KEY = redis_prefix +
                                  "commands::" + driver_config.robot_name +
                                  "::control_torques";
    JOINT_ANGLES_KEY = redis_prefix + "sensors::" + driver_config.robot_name +
                       "::joint_positions";
    JOINT_VELOCITIES_KEY = redis_prefix +
                           "sensors::" + driver_config.robot_name +
                           "::joint_velocities";
    JOINT_TORQUES_SENSED_KEY = redis_prefix +
                               "sensors::" + driver_config.robot_name +
                               "::joint_torques";
    MASSMATRIX_KEY = redis_prefix + "sensors::" + driver_config.robot_name +
                     "::model::mass_matrix";
    CORIOLIS_KEY = redis_prefix + "sensors::" + driver_config.robot_name +
                   "::model::coriolis";
    ROBOT_GRAVITY_KEY = redis_prefix + "sensors::" + driver_config.robot_name +
                        "::model::robot_gravity";
    SAFETY_TORQUES_LOGGING_KEY = redis_prefix +
                                 "redis_driver::" + driver_config.robot_name +
                                 "::safety_controller::safety_torques";
    SENT_TORQUES_LOGGING_KEY = redis_prefix +
                               "redis_driver::" + driver_config.robot_name +
                               "::safety_controller::sent_torques";
    CONSTRAINED_NULLSPACE_KEY = redis_prefix +
                                "redis_driver::" + driver_config.robot_name +
                                "::safety_controller::constraint_nullspace";

    GRIPPER_PARAMETERS_COMMANDED_KEY = redis_prefix +
                                       "commands::" + driver_config.robot_name +
                                       "::gripper::parameters";
    GRIPPER_MODE_KEY = redis_prefix + "commands::" + driver_config.robot_name +
                       "::gripper::mode";

    GRIPPER_CURRENT_WIDTH_KEY = redis_prefix +
                                "sensors::" + driver_config.robot_name +
                                "::gripper::width";
    GRIPPER_SENSED_GRASP_FORCE_KEY = redis_prefix +
                                     "sensors::" + driver_config.robot_name +
                                     "::gripper::grasp_force";

    if (driver_config.robot_type == Sai::Flexiv::RobotType::RIZON_4S) {
        WRIST_FORCE_SENSED_KEY = redis_prefix +
                                 "sensors::" + driver_config.robot_name +
                                 "::ft_sensor::force";
        WRIST_MOMENT_SENSED_KEY = redis_prefix +
                                  "sensors::" + driver_config.robot_name +
                                  "::ft_sensor::moment";
    }

    // start redis client
    Sai::Flexiv::CDatabaseRedisClient *redis_client;
    Sai::Flexiv::HiredisServerInfo info;
    info.hostname_ = "127.0.0.1";
    info.port_ = 6379;
    info.timeout_ = {1, 500000}; // 1.5 seconds
    redis_client = new Sai::Flexiv::CDatabaseRedisClient();
    redis_client->serverIs(info);

    for (int i = 0; i < 7; ++i) {
        tau_cmd_array[i] = 0;
    }

    redis_client->setDoubleArray(JOINT_TORQUES_COMMANDED_KEY, tau_cmd_array, 7);
    // safety to detect if controller is already running : wait 50 milliseconds
    usleep(50000);
    redis_client->getDoubleArray(JOINT_TORQUES_COMMANDED_KEY, tau_cmd_array, 7);
    for (int i = 0; i < 7; ++i) {
        if (tau_cmd_array[i] != 0) {
            std::cout << "Stop the controller before running the driver\n"
                      << "\n";
            return -1;
        }
    }

    redis_client->setEigenMatrixDerivedString(GRIPPER_PARAMETERS_COMMANDED_KEY,
                                              gripper_parameters);
    redis_client->setCommandIs(GRIPPER_MODE_KEY, "o");

    // prepare batch command
    key_names.push_back(JOINT_TORQUES_COMMANDED_KEY);
    key_names.push_back(MASSMATRIX_KEY);
    key_names.push_back(JOINT_ANGLES_KEY);
    key_names.push_back(JOINT_VELOCITIES_KEY);
    key_names.push_back(JOINT_TORQUES_SENSED_KEY);

    sensor_feedback.push_back(q_array);
    sensor_feedback.push_back(dq_array);
    sensor_feedback.push_back(tau_sensed_array);

    if (driver_config.robot_type == Sai::Flexiv::RobotType::RIZON_4S) {
        std::cout << "Using Rizon 4s specifications\n";
        // Rizon 4s specifications
        joint_position_max_default = {2.7925, 2.2689, 2.9670, 2.6878,
                                      2.9670, 4.5378, 2.9670};
        joint_position_min_default = {-2.7925, -2.2689, -2.9670, -1.8675,
                                      -2.9670, -1.3962, -2.9670};
        joint_velocity_limits_default = {2.0994, 2.0944, 2.4435, 2.4435,
                                         4.8869, 4.8869, 4.8869};
        joint_torques_limits_default = {123, 123, 64, 64, 39, 39, 39};

        // damping gains
        // kv_safety = {20.0, 20.0, 20.0, 15.0, 10.0, 10.0, 5.0};
        kv_safety = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 5.0};

        // zone definitions
        pos_zones = {6., 9.}; // hard, soft
        // vel_zones = {5., 7.};  // hard, soft
        vel_zones = {6., 8.}; // hard, soft  (8, 6)
    } else if (driver_config.robot_type == Sai::Flexiv::RobotType::RIZON_4) {
        std::cout << "Using Rizon 4 specifications\n";
        // Rizon 4 specifications
        joint_position_max_default = {2.7925, 2.2689, 2.9670, 2.6878,
                                      2.9670, 4.5378, 2.9670};
        joint_position_min_default = {-2.7925, -2.2689, -2.9670, -1.8675,
                                      -2.9670, -1.3962, -2.9670};
        joint_velocity_limits_default = {2.0994, 2.0944, 2.4435, 2.4435,
                                         4.8869, 4.8869, 4.8869};
        joint_torques_limits_default = {123, 123, 64, 64, 39, 39, 39};

        // damping gains
        // kv_safety = {20.0, 20.0, 20.0, 15.0, 10.0, 10.0, 5.0};
        kv_safety = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 5.0};

        // zone definitions
        pos_zones = {6., 9.}; // hard, soft
        // vel_zones = {5., 7.};  // hard, soft
        vel_zones = {6., 8.}; // hard, soft  (8, 6)
    } else {
        std::cout << "Robot serial number not recognized, exiting...\n"
                  << "\n";
        return -1;
    }

    try {

        // RDK Initialization
        // =========================================================================================
        // Instantiate robot interface
        flexiv::Robot robot(driver_config.serial_number);
        // load the kinematics and dynamics model
        flexiv::Model model(robot);

        // Clear fault on the connected robot if any
        if (robot.fault()) {
            log.Warn(
                "Fault occurred on the connected robot, trying to clear ...");
            // Try to clear the fault
            if (!robot.ClearFault()) {
                log.Error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.Info("Fault on the connected robot is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        log.Info("Enabling robot ...");
        robot.Enable();

        // Wait for the robot to become operational
        while (!robot.operational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.Info("Robot is now operational");

        // Zero Force-torque Sensors
        // =========================================================================================
        // IMPORTANT: must zero force/torque sensor offset for accurate
        // force/torque measurement
        robot.SwitchMode(flexiv::Mode::NRT_PRIMITIVE_EXECUTION);
        robot.ExecutePrimitive("ZeroFTSensor()");

        // WARNING: during the process, the robot must not contact anything,
        // otherwise the result will be inaccurate and affect following
        // operations
        log.Info(
            "Zeroing force/torque sensors, make sure nothing is in contact "
            "with the robot");

        // Wait for primitive completion
        while (robot.busy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.Info("Sensor zeroing complete");

        // Wait for the primitive to finish
        while (robot.busy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Instantiate gripper control interface
        flexiv::Gripper gripper(robot);

        // Manually initialize the gripper, not all grippers need this step
        log.Info(
            "Initializing gripper, this process takes about 10 seconds ...");
        gripper.Init();
        log.Info("Initialization complete");

        // Wait for the primitive to finish
        while (robot.busy()) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }

        // Real-time Control
        // =========================================================================================
        // Switch to real-time joint torque control mode
        robot.SwitchMode(flexiv::Mode::RT_JOINT_TORQUE);

        // Create real-time scheduler to run periodic tasks
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(std::bind(PeriodicTask, std::ref(robot),
                                    std::ref(gripper), std::ref(model),
                                    std::ref(log), std::ref(redis_client)),
                          "HP periodic", 1, scheduler.max_priority(), 2);
        // Start all added tasks
        scheduler.Start();

        // Block and wait for signal to stop scheduler tasks
        while (!g_stop_sched) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // Received signal to stop scheduler tasks
        scheduler.Stop();

    } catch (const std::exception &e) {
        log.Error(e.what());
        return 1;
    }

    return 0;
}

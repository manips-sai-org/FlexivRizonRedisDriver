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

#include <flexiv/rdk/gripper.hpp>
#include <flexiv/rdk/model.hpp>
#include <flexiv/rdk/robot.hpp>
#include <flexiv/rdk/scheduler.hpp>
#include <flexiv/rdk/tool.hpp>
#include <flexiv/rdk/utility.hpp>
#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

using namespace flexiv;

// redis keys
// - read:
std::string JOINT_TORQUES_COMMANDED_KEY;
std::string GRIPPER_PARAMETERS_COMMANDED_KEY;
// - write:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;
std::string RAW_WRIST_FORCE_SENSED_KEY;
std::string RAW_WRIST_MOMENT_SENSED_KEY;
std::string TCP_FORCE_SENSED_KEY;
std::string TCP_MOMENT_SENSED_KEY;
std::string SAFETY_TORQUES_LOGGING_KEY;
std::string SENT_TORQUES_LOGGING_KEY;
std::string CONSTRAINED_NULLSPACE_KEY;
std::string GRIPPER_CURRENT_WIDTH_KEY;
std::string GRIPPER_SENSED_GRASP_FORCE_KEY;
std::string SAI_DEBUG_KEY;

// user options
const bool USING_4S =
    true; // set if using the Rizon 4s with wrist force-torque sensor
const bool VERBOSE = true; // print out safety violations
const int K_DOF = 7;
const double FREE_DRIVE_THRESHOLD = 6; // n-m norm
int not_touching_counter = 0;
const int NOT_TOUCHING_WINDOW = 400; // ms
const auto kRedisExchangePeriod = std::chrono::microseconds(1000); // 1 kHz
const auto kGripperCommandPeriod = std::chrono::milliseconds(10); // 100 Hz
using Vector7d = Eigen::Matrix<double, K_DOF, 1>;
using Matrix7d = Eigen::Matrix<double, K_DOF, K_DOF>;

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
Matrix7d MassMatrix;
Vector7d gravity_vector = Vector7d::Zero();
Vector7d coriolis = Vector7d::Zero();
std::array<double, 7> tau_cmd_array{};
std::array<double, 7> redis_command_storage_array{};
std::array<double, 7> q_array{};
std::array<double, 7> dq_array{};
std::array<double, 7> tau_sensed_array{};
std::array<double, 7> gravity_vector_array{};
std::array<double, 7> coriolis_array{};
// std::array<double, 49> M_array{};
std::vector<std::array<double, 7>> robot_joint_sensor_feedback;
std::array<double, 6> wrist_ft_sensed_raw_array{};
std::array<double, 6> external_wrench_at_tcp_array{};
std::array<double, 6> external_wrench_at_tcp_unfiltered_array{};
std::array<double, 3> wrist_ft_sensed_raw_force{};
std::array<double, 3> wrist_ft_sensed_raw_moment{};
std::array<double, 3> tcp_sensed_force{};
std::array<double, 3> tcp_sensed_moment{};
std::vector<std::array<double, 3>> robot_wrist_ft_sensor_feedback;
std::array<double, 1>  gripper_current_width{};
std::array<double, 1>  gripper_sensed_grasp_force{};
std::vector<std::array<double, 1>> gripper_status_feedback;
std::vector<std::string> set_get_batch_key_names;
// bool fDriverRunning = true;
// void sighandler(int sig)
// { fDriverRunning = false; }

// gripper command storage
std::array<double, 3> gripper_parameters_array = {0.06, 0.1, 10.0}; // width in m, speed in m/s, force in N
std::array<double, 3> last_gripper_parameters_array = gripper_parameters_array;
Eigen::Vector3d gripper_parameters = Eigen::Map<Eigen::Vector3d>(gripper_parameters_array.data());
Eigen::Vector3d last_gripper_parameters = gripper_parameters;
double gripper_width;
double gripper_speed;
double gripper_force;

// limit options
bool _pos_limit_opt = true;
bool _vel_limit_opt = true;

// setup joint limit avoidance
std::vector<int> _pos_limit_flag{7, SAFE};
std::vector<int> _vel_limit_flag{7, SAFE};
Matrix7d
    _J_s = Matrix7d::Zero(); // constraint task jacobian storage
Matrix7d _Lambda_s = Matrix7d::Zero(); // op-space matrix storage
Matrix7d _Jbar_s = Matrix7d::Zero();
Matrix7d _N_s = Matrix7d::Identity(); // nullspace matrix with the constraint task jacobian
Eigen::VectorXi
    _limited_joints(7); // 1 or 0 depending on whether the joint is limited
Vector7d _tau_limited = Vector7d::Zero();
Vector7d _torque_scaling_vector =
    Vector7d::Ones(); // tau scaling based on the velocity signal

// override with new safety set (this set triggers software stop)
double default_sf = 0.98; // max violation safety factors

// zone 1 and 2 definitions subject to tuning
double soft_sf = 0.90;             // start of damping zone
double hard_sf = 0.95;             // start of feedback zone
double angle_tol = 1 * M_PI / 180; // rad
double vel_tol = 0.1;              // rad/s (0.1 = 5 deg/s)
double q_tol = 1e-1 * M_PI / 180;

Vector7d soft_min_angles;
Vector7d soft_max_angles;
Vector7d hard_min_angles;
Vector7d hard_max_angles;
Vector7d soft_min_joint_velocity_limits;
Vector7d hard_min_joint_velocity_limits;
Vector7d soft_max_joint_velocity_limits;
Vector7d hard_max_joint_velocity_limits;

// initial torque bias
Vector7d init_torque_bias = Vector7d::Zero();
int n_samples = 500;
int n_curr = 0;
bool initialized_torque_bias = false;
std::vector<double> kp_holding = {1000, 1000, 1000, 1000, 1000, 1000, 1000};
std::vector<double> kv_holding = {10, 10, 10, 10, 10, 10, 10};
std::vector<double> kp_holding_drive = {200, 200, 200, 200, 200, 200, 200};
std::vector<double> kv_holding_drive = {10, 10, 10, 10, 10, 10, 10};
Vector7d q_init = Vector7d::Zero();
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
//     MassMatrix(M_array.data()); redis_client->setGetBatchCommands(set_get_batch_key_names,
//     tau_cmd_array, MassMatrix, sensor_feedback);
//   }
// }

const std::vector<std::string> limit_state{
    "Safe",         "Soft Min",     "Hard Min",     "Soft Max",    "Hard Max",
    "Min Soft Vel", "Min Hard Vel", "Max Soft Vel", "Max Hard Vel"};

// clang-format off
/** Joint velocity damping gains for floating */
const std::array<double, K_DOF> kFloatingDamping = {
    10.0, 10.0, 5.0, 5.0, 1.0,  1.0,  1.0};
// clang-format on

// const std::array<double, K_DOF> kFloatingDamping = {
//     15.0, 15.0, 7.5, 7.5, 1.5, 1.5, 1.5};

// const std::array<double, K_DOF> kFloatingDamping = {
//     20.0, 20.0, 10, 10, 2, 2, 2};

template <typename T, std::size_t N>
std::vector<T> arrayToVector(const std::array<T, N> &arr) {
  return std::vector<T>(arr.begin(), arr.end());
}

template <typename T, std::size_t N>
std::array<T, N> vectorToArray(const std::vector<T> &vec) {
  if (vec.size() != N) {
    throw std::out_of_range("Vector size does not match array size.");
  }

  std::array<T, N> arr;
  std::copy_n(vec.begin(), N, arr.begin());
  return arr;
}

std::array<double, K_DOF * K_DOF> matrix7ToArray(const Matrix7d &matrix) {
  std::array<double, K_DOF * K_DOF> arr{};
  for (int row = 0; row < K_DOF; ++row) {
    for (int col = 0; col < K_DOF; ++col) {
      arr[row * K_DOF + col] = matrix(row, col);
    }
  }
  return arr;
}

Matrix7d arrayToMatrix7d(const std::array<double, K_DOF * K_DOF> &arr) {
  Matrix7d matrix;
  for (int row = 0; row < K_DOF; ++row) {
    for (int col = 0; col < K_DOF; ++col) {
      matrix(row, col) = arr[row * K_DOF + col];
    }
  }
  return matrix;
}

/** Atomic signal to stop scheduler tasks */
std::atomic<bool> g_stop_sched = {false};

template <typename T>
class DoubleBufferExchange {
public:
  DoubleBufferExchange() {
    busy_[0].store(false, std::memory_order_relaxed);
    busy_[1].store(false, std::memory_order_relaxed);
  }

  DoubleBufferExchange(const DoubleBufferExchange &) = delete;
  DoubleBufferExchange &operator=(const DoubleBufferExchange &) = delete;

  void initialize(const T &value) {
    busy_[0].store(false, std::memory_order_relaxed);
    busy_[1].store(false, std::memory_order_relaxed);
    buffers_[0] = value;
    buffers_[1] = value;
    sequence_[0] = 0;
    sequence_[1] = 0;
    next_sequence_.store(0, std::memory_order_relaxed);
    published_idx_.store(0, std::memory_order_release);
  }

  bool try_publish(const T &value) {
    const int idx = 1 - published_idx_.load(std::memory_order_acquire);
    if (!try_acquire(idx)) {
      return false;
    }

    buffers_[idx] = value;
    sequence_[idx] =
        next_sequence_.fetch_add(1, std::memory_order_acq_rel) + 1;
    release(idx);
    published_idx_.store(idx, std::memory_order_release);
    return true;
  }

  bool try_read(T &value, std::uint64_t *sequence = nullptr) {
    const int idx = published_idx_.load(std::memory_order_acquire);
    if (!try_acquire(idx)) {
      return false;
    }

    value = buffers_[idx];
    if (sequence != nullptr) {
      *sequence = sequence_[idx];
    }
    release(idx);
    return true;
  }

  bool try_read_if_new(T &value, std::uint64_t &last_sequence) {
    std::uint64_t sequence = 0;
    if (!try_read(value, &sequence) || sequence == last_sequence) {
      return false;
    }

    last_sequence = sequence;
    return true;
  }

private:
  bool try_acquire(const int idx) {
    bool expected = false;
    return busy_[idx].compare_exchange_strong(
        expected, true, std::memory_order_acquire,
        std::memory_order_relaxed);
  }

  void release(const int idx) {
    busy_[idx].store(false, std::memory_order_release);
  }

  T buffers_[2]{};
  std::uint64_t sequence_[2]{};
  std::atomic<bool> busy_[2];
  std::atomic<int> published_idx_{0};
  std::atomic<std::uint64_t> next_sequence_{0};
};

struct TorqueCommandData {
  std::array<double, K_DOF> command_torques{};
};

struct GripperCommandData {
  std::array<double, 3> gripper_parameters = {0.06, 0.1, 10.0};
};

struct RobotStateExchangeData {
  std::array<double, K_DOF> joint_positions{};
  std::array<double, K_DOF> joint_velocities{};
  std::array<double, K_DOF> sensed_torques{};
  std::array<double, K_DOF> gravity{};
  std::array<double, K_DOF> coriolis{};
  std::array<double, 3> wrist_ft_sensed_raw_force{};
  std::array<double, 3> wrist_ft_sensed_raw_moment{};
  std::array<double, 3> tcp_sensed_force{};
  std::array<double, 3> tcp_sensed_moment{};
  std::array<double, K_DOF * K_DOF> mass_matrix{};

  bool state_ready = false;
};

struct GripperStatusData {
  std::array<double, 1> gripper_current_width{};
  std::array<double, 1> gripper_sensed_grasp_force{};
  bool status_ready = false;
};

DoubleBufferExchange<TorqueCommandData> torque_command_exchange;
DoubleBufferExchange<GripperCommandData> gripper_command_exchange;
DoubleBufferExchange<RobotStateExchangeData> robot_state_exchange;
DoubleBufferExchange<GripperStatusData> gripper_status_exchange;

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
  dq_min[0] =
      std::max(-joint_velocity_limits_default[0],
               std::min(0., 0.3 - sqrt(std::max(0., 12.0 * (2.75010 + q[0])))));
  dq_min[1] =
      std::max(-joint_velocity_limits_default[1],
               std::min(0., 0.2 - sqrt(std::max(0., 5.17 * (1.79180 + q[1])))));
  dq_min[2] =
      std::max(-joint_velocity_limits_default[2],
               std::min(0., 0.2 - sqrt(std::max(0., 7.00 * (2.90650 + q[2])))));
  dq_min[3] =
      std::max(-joint_velocity_limits_default[3],
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

void UpdateSafetyLimits() {
  for (int i = 0; i < K_DOF; ++i) {
    joint_position_max[i] = default_sf * joint_position_max_default[i];
    joint_position_min[i] = default_sf * joint_position_min_default[i];
    joint_velocity_limits[i] = default_sf * joint_velocity_limits_default[i];
    joint_torques_limits[i] = default_sf * joint_torques_limits_default[i];

    soft_min_angles(i) = joint_position_min[i] + pos_zones[1] * angle_tol;
    hard_min_angles(i) = joint_position_min[i] + pos_zones[0] * angle_tol;
    soft_max_angles(i) = joint_position_max[i] - pos_zones[1] * angle_tol;
    hard_max_angles(i) = joint_position_max[i] - pos_zones[0] * angle_tol;
    soft_min_joint_velocity_limits(i) =
        -joint_velocity_limits[i] + vel_zones[1] * vel_tol;
    hard_min_joint_velocity_limits(i) =
        -joint_velocity_limits[i] + vel_zones[0] * vel_tol;
    soft_max_joint_velocity_limits(i) =
        joint_velocity_limits[i] - vel_zones[1] * vel_tol;
    hard_max_joint_velocity_limits(i) =
        joint_velocity_limits[i] - vel_zones[0] * vel_tol;
  }
}

/** @brief Thread that owns all Redis I/O. */
void RedisManagerThread(Sai::Flexiv::CDatabaseRedisClient *redis_client,
                        std::atomic<bool> &running) {
  std::array<double, K_DOF> command_torques{};
  std::array<double, 3> redis_gripper_parameters = {0.06, 0.1, 10.0};
  RobotStateExchangeData latest_robot_state;
  GripperStatusData latest_gripper_status;
  std::uint64_t robot_state_sequence = 0;
  std::uint64_t gripper_status_sequence = 0;
  auto next_wakeup = std::chrono::steady_clock::now();

  TorqueCommandData torque_command_snapshot;
  if (torque_command_exchange.try_read(torque_command_snapshot)) {
    command_torques = torque_command_snapshot.command_torques;
  }
  GripperCommandData gripper_command_snapshot;
  if (gripper_command_exchange.try_read(gripper_command_snapshot)) {
    redis_gripper_parameters =
        gripper_command_snapshot.gripper_parameters;
  }

  while (running.load(std::memory_order_acquire) &&
         !g_stop_sched.load(std::memory_order_acquire)) {
    next_wakeup += kRedisExchangePeriod;
    try {
      robot_state_exchange.try_read_if_new(latest_robot_state,
                                           robot_state_sequence);
      gripper_status_exchange.try_read_if_new(latest_gripper_status,
                                              gripper_status_sequence);

      if (latest_robot_state.state_ready) {
        const Matrix7d latest_mass_matrix =
            arrayToMatrix7d(latest_robot_state.mass_matrix);
        std::vector<std::array<double, K_DOF>> joint_sensor_feedback = {
            latest_robot_state.joint_positions,
            latest_robot_state.joint_velocities,
            latest_robot_state.sensed_torques, latest_robot_state.gravity,
            latest_robot_state.coriolis};
        std::vector<std::array<double, 1>> gripper_status_feedback = {
            latest_gripper_status.gripper_current_width,
            latest_gripper_status.gripper_sensed_grasp_force};
        std::vector<std::array<double, 3>> wrist_ft_sensor_feedback;

        if (driver_config.robot_type == Sai::Flexiv::RobotType::RIZON_4S) {
          wrist_ft_sensor_feedback.push_back(
              latest_robot_state.wrist_ft_sensed_raw_force);
          wrist_ft_sensor_feedback.push_back(
              latest_robot_state.wrist_ft_sensed_raw_moment);
          wrist_ft_sensor_feedback.push_back(
              latest_robot_state.tcp_sensed_force);
          wrist_ft_sensor_feedback.push_back(
              latest_robot_state.tcp_sensed_moment);
        }

        redis_client->setGetBatchRizon4S(
            set_get_batch_key_names, command_torques,
            redis_gripper_parameters, latest_mass_matrix,
            joint_sensor_feedback, gripper_status_feedback,
            wrist_ft_sensor_feedback);
      } else {
        redis_client->getDoubleArray(JOINT_TORQUES_COMMANDED_KEY,
                                     command_torques, K_DOF);
        redis_client->getDoubleArray(GRIPPER_PARAMETERS_COMMANDED_KEY,
                                     redis_gripper_parameters, 3);
      }

      TorqueCommandData torque_command_update;
      torque_command_update.command_torques = command_torques;
      torque_command_exchange.try_publish(torque_command_update);

      GripperCommandData gripper_command_update;
      gripper_command_update.gripper_parameters = redis_gripper_parameters;
      gripper_command_exchange.try_publish(gripper_command_update);
    } catch (const std::exception &e) {
      spdlog::error(std::string("Redis manager error: ") + e.what());
      running.store(false, std::memory_order_release);
      g_stop_sched.store(true, std::memory_order_release);
    }

    const auto now = std::chrono::steady_clock::now();
    if (now < next_wakeup) {
      std::this_thread::sleep_until(next_wakeup);
    } else {
      next_wakeup = now;
    }
  }
}

/** @brief Thread that owns all gripper device I/O. */
void GripperCommandThread(flexiv::rdk::Gripper &gripper,
                          std::atomic<bool> &running) {
  Eigen::Vector3d last_command = last_gripper_parameters;
  std::array<double, 3> command_array = gripper_parameters_array;
  auto next_wakeup = std::chrono::steady_clock::now();

  while (running.load(std::memory_order_acquire) &&
         !g_stop_sched.load(std::memory_order_acquire)) {
    next_wakeup += kGripperCommandPeriod;
    try {
      GripperCommandData command_snapshot;
      if (gripper_command_exchange.try_read(command_snapshot)) {
        command_array = command_snapshot.gripper_parameters;
      }

      auto gripper_state = gripper.states();
      GripperStatusData status_update;
      status_update.gripper_current_width[0] = gripper_state.width;
      status_update.gripper_sensed_grasp_force[0] = gripper_state.force;
      status_update.status_ready = true;
      gripper_status_exchange.try_publish(status_update);

      Eigen::Vector3d command =
          Eigen::Map<Eigen::Vector3d>(command_array.data());
      if ((command - last_command).norm() > 0.001) {
        double width = command(0);
        double speed = command(1);
        double force = command(2);
        spdlog::info("Moving Gripper - Width: " + std::to_string(width) +
                     "m    Speed: " + std::to_string(speed) +
                     "m/s    Force: " + std::to_string(force) + "N");
        gripper.Stop();
        gripper.Move(width, speed, force);
        last_command = command;
      }
    } catch (const std::exception &e) {
      spdlog::error(std::string("Gripper manager error: ") + e.what());
      running.store(false, std::memory_order_release);
      g_stop_sched.store(true, std::memory_order_release);
    }

    const auto now = std::chrono::steady_clock::now();
    if (now < next_wakeup) {
      std::this_thread::sleep_until(next_wakeup);
    } else {
      next_wakeup = now;
    }
  }
}

/** @brief Callback function for realtime periodic task */
void PeriodicTask(flexiv::rdk::Robot &robot, flexiv::rdk::Model &model) {

  try {

    TorqueCommandData torque_command_snapshot;
    if (torque_command_exchange.try_read(torque_command_snapshot)) {
      tau_cmd_array = torque_command_snapshot.command_torques;
      redis_command_storage_array =
          torque_command_snapshot.command_torques;
    }

    // Monitor fault on the connected robot
    if (robot.fault()) {
      throw std::runtime_error("PeriodicTask: Fault occurred on the "
                               "connected robot, exiting ...");
    }

    // Prepare variables for robot state update

    auto robot_state = robot.states();
    model.Update(robot_state.q, robot_state.dq);

    q_array = vectorToArray<double, K_DOF>(robot_state.q);
    dq_array = vectorToArray<double, K_DOF>(robot_state.dq); // non-filtered velocities
    // dq_array = dq_array;  // filtered velocities
    tau_sensed_array = vectorToArray<double, K_DOF>(robot_state.tau);
    gravity_vector = model.g();
    // std::cout << "Gravity vector: " << gravity_vector.transpose() << std::endl;
    coriolis = model.c();
    Eigen::Map<Vector7d>(gravity_vector_array.data()) = gravity_vector;
    Eigen::Map<Vector7d>(coriolis_array.data()) = coriolis;

    // fill joint sensor feedback vector
    robot_joint_sensor_feedback[0] = q_array;
    robot_joint_sensor_feedback[1] = dq_array;
    robot_joint_sensor_feedback[2] = tau_sensed_array;
    robot_joint_sensor_feedback[3] = gravity_vector_array;
    robot_joint_sensor_feedback[4] = coriolis_array;

    // std::cout << "Gravity vector array: " << gravity_vector_array[0] << ", "
    //           << gravity_vector_array[1] << ", " << gravity_vector_array[2]
    //           << ", " << gravity_vector_array[3] << ", "
    //           << gravity_vector_array[4] << ", " << gravity_vector_array[5]
    //           << ", " << gravity_vector_array[6] << std::endl;

    // store robot mass matrix
    MassMatrix = model.M();

    wrist_ft_sensed_raw_array = robot_state.ft_sensor_raw;
    external_wrench_at_tcp_array = robot_state.ext_wrench_in_tcp;
    // external_wrench_at_tcp_array = robot_state.ext_wrench_in_tcp_raw;
    wrist_ft_sensed_raw_force = {wrist_ft_sensed_raw_array[0],
                                  wrist_ft_sensed_raw_array[1],
                                  wrist_ft_sensed_raw_array[2]};
    wrist_ft_sensed_raw_moment = {wrist_ft_sensed_raw_array[3],
                                  wrist_ft_sensed_raw_array[4],
                                  wrist_ft_sensed_raw_array[5]};
    tcp_sensed_force = {external_wrench_at_tcp_array[0],
                        external_wrench_at_tcp_array[1],
                        external_wrench_at_tcp_array[2]};
    tcp_sensed_moment = {external_wrench_at_tcp_array[3],
                          external_wrench_at_tcp_array[4],
                           external_wrench_at_tcp_array[5]};

    // fill wrist force-torque sensor feedback vector
    robot_wrist_ft_sensor_feedback[0] = wrist_ft_sensed_raw_force;
    robot_wrist_ft_sensor_feedback[1] = wrist_ft_sensed_raw_moment;
    robot_wrist_ft_sensor_feedback[2] = tcp_sensed_force;
    robot_wrist_ft_sensor_feedback[3] = tcp_sensed_moment;

    Eigen::Map<Eigen::Matrix<double, 7, 1>> _tau(tau_cmd_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> _sensed_torques(
        tau_sensed_array.data()); // sensed torques
    Eigen::Map<Eigen::Matrix<double, 7, 1>> _coriolis(
        coriolis_array.data());
    Matrix7d MassMatrixInverse =
        MassMatrix.llt().solve(Matrix7d::Identity());

    RobotStateExchangeData state_update;
    state_update.joint_positions = q_array;
    state_update.joint_velocities = dq_array;
    state_update.sensed_torques = tau_sensed_array;
    state_update.gravity = gravity_vector_array;
    state_update.coriolis = coriolis_array;
    state_update.wrist_ft_sensed_raw_force = wrist_ft_sensed_raw_force;
    state_update.wrist_ft_sensed_raw_moment = wrist_ft_sensed_raw_moment;
    state_update.tcp_sensed_force = tcp_sensed_force;
    state_update.tcp_sensed_moment = tcp_sensed_moment;
    state_update.mass_matrix = matrix7ToArray(MassMatrix);
    state_update.state_ready = true;
    robot_state_exchange.try_publish(state_update);

    // reset containers
    _limited_joints.setZero(); // used to form the constraint jacobian
    _N_s.setIdentity();

    // position limit saturation
    if (_pos_limit_opt) {
      for (int i = 0; i < 7; ++i) {
        double curr_q = robot_state.q[i];
        if (curr_q > soft_min_angles(i) && curr_q < soft_max_angles(i)) {
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
          } else if (robot_state.dq[i] > hard_max_joint_velocity_limits[i]) {
            _vel_limit_flag[i] = MAX_HARD_VEL;
            _limited_joints(i) = 1;
          } else if (robot_state.dq[i] > soft_max_joint_velocity_limits[i]) {
            _vel_limit_flag[i] = MAX_SOFT_VEL;
            _limited_joints(i) = 1;
          } else if (robot_state.dq[i] < hard_min_joint_velocity_limits[i]) {
            _vel_limit_flag[i] = MIN_HARD_VEL;
            _limited_joints(i) = 1;
          } else if (robot_state.dq[i] < soft_min_joint_velocity_limits[i]) {
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
                    << " State: " << limit_state[_pos_limit_flag[i]] << "\n";
          std::cout << "---\n";
        }
        if (_vel_limit_flag[i] != SAFE) {
          std::cout << counter << ": Joint " << i
                    << " State: " << limit_state[_vel_limit_flag[i]] << "\n";
          std::cout << "---\n";
        }
      }
    }

    int n_limited_joints = _limited_joints.sum();
    _tau_limited = Vector7d::Zero();
    _torque_scaling_vector = Vector7d::Ones();

    if (n_limited_joints > 0) {
      for (int i = 0; i < 7; ++i) {
        if (_pos_limit_flag[i] == MIN_SOFT) {
          // ramping kv damping proportional to violation difference
          // up to max damping + command torques (goes from tau -> tau
          // + max damping) double kv = kv_safety[i] *
          // (robot_state.q[i] - soft_min_angles[i]) /
          // (hard_min_angles[i] - soft_min_angles[i]);
          double alpha = getBlendingCoeff(robot_state.q[i], soft_min_angles[i],
                                          hard_min_angles[i]);
          _tau_limited(i) = _tau(i) - alpha * kv_safety[i] * robot_state.dq[i];

        } else if (_pos_limit_flag[i] == MAX_SOFT) {
          // same as above, but for the upper soft limit
          // double kv = kv_safety[i] * (robot_state.q[i] -
          // soft_max_angles[i]) / (hard_max_angles[i] -
          // soft_max_angles[i]);
          double alpha = getBlendingCoeff(robot_state.q[i], soft_max_angles[i],
                                          hard_max_angles[i]);
          _tau_limited(i) = _tau(i) - alpha * kv_safety[i] * robot_state.dq[i];

        } else if (_pos_limit_flag[i] == MIN_HARD) {
          // max damping + command torques blend with quadratic
          // ramping (goes from tau + max damping -> holding tau + max
          // damping with blending from tau to holding tau) double
          // dist = (robot_state.q[i] - hard_min_angles[i]) /
          // (joint_position_min[i] - hard_min_angles[i]);
          double alpha = getBlendingCoeff(robot_state.q[i], hard_min_angles[i],
                                          joint_position_min[i]);
          double tau_hold = joint_torques_limits_default[i];
          _tau_limited(i) = (1 - std::pow(alpha, 2)) * _tau(i) +
                            std::pow(alpha, 2) * tau_hold -
                            kv_safety[i] * robot_state.dq[i];

        } else if (_pos_limit_flag[i] == MAX_HARD) {
          // same as above, but for the upper hard limit
          // double dist = (robot_state.q[i] - hard_max_angles[i]) /
          // (joint_position_max[i] - hard_max_angles[i]);
          double alpha = getBlendingCoeff(robot_state.q[i], hard_max_angles[i],
                                          joint_position_max[i]);
          double tau_hold = -joint_torques_limits_default[i];
          _tau_limited(i) = (1 - std::pow(alpha, 2)) * _tau(i) +
                            std::pow(alpha, 2) * tau_hold -
                            kv_safety[i] * robot_state.dq[i];

        } else if (_vel_limit_flag[i] == MIN_SOFT_VEL) {
          // command torques blend with holding torques to soft joint
          // velocity (goes from tau -> tau hold with blending)
          double alpha = getBlendingCoeff(robot_state.dq[i],
                                          soft_min_joint_velocity_limits[i],
                                          hard_min_joint_velocity_limits[i]);
          _tau_limited(i) = (1 - std::pow(alpha, 1)) * _tau(i);
          // _torque_scaling_vector(i) = (1 - std::pow(alpha, 4));
          // _torque_scaling_vector(i) =
          // soft_min_joint_velocity_limits[i] / robot_state.dq[i];
          // _tau(i) = 0;

        } else if (_vel_limit_flag[i] == MAX_SOFT_VEL) {
          // same as above, but for the upper soft limit
          double alpha = getBlendingCoeff(robot_state.dq[i],
                                          soft_max_joint_velocity_limits[i],
                                          hard_max_joint_velocity_limits[i]);
          _tau_limited(i) = (1 - std::pow(alpha, 1)) * _tau(i);
          // _torque_scaling_vector(i) = (1 - std::pow(alpha, 4));
          // _torque_scaling_vector(i) =
          // soft_max_joint_velocity_limits[i] / robot_state.dq[i];
          // _tau(i) = 0;

        } else if (_vel_limit_flag[i] == MIN_HARD_VEL) {
          // full damping with soft joint velocity limits goal and a
          // quadratic ramp to max torque
          double alpha = getBlendingCoeff(robot_state.dq[i],
                                          hard_min_joint_velocity_limits[i],
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
          _tau_limited(i) =
              std::pow(alpha, 4) * joint_torques_limits_default[i] * 1e-2;
          // _tau(i) = 0;

        } else if (_vel_limit_flag[i] == MAX_HARD_VEL) {
          // same as above, but for the upper hard limit
          double alpha = getBlendingCoeff(robot_state.dq[i],
                                          hard_max_joint_velocity_limits[i],
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
          _tau_limited(i) =
              -std::pow(alpha, 4) * joint_torques_limits_default[i] * 1e-2;
          // _tau(i) = 0;
        }
      }

      // compute revised torques
      _J_s.setZero();
      int cnt = 0;
      for (int i = 0; i < 7; ++i) {
        if (_limited_joints(i)) {
          _J_s(cnt, i) = 1;
          cnt++;
        }
      }
      _Lambda_s.setZero();
      _Lambda_s.topLeftCorner(n_limited_joints, n_limited_joints) =
          (_J_s.topRows(n_limited_joints) * MassMatrixInverse *
           _J_s.topRows(n_limited_joints).transpose())
              .inverse();
      _Jbar_s = MassMatrixInverse * _J_s.transpose() * _Lambda_s;
      _N_s = Matrix7d::Identity() - _Jbar_s * _J_s;
      _tau = _tau_limited + _N_s.transpose() * _tau;

      for (int i = 0; i < 7; ++i) {
        tau_cmd_array[i] = _tau(i);
      }
    }

    // safey keys
    _N_s.setIdentity();
    // redis_client->setEigenMatrixDerived(SAFETY_TORQUES_LOGGING_KEY,
    //                                     _tau_limited);
    // redis_client->setEigenMatrixDerived(SENT_TORQUES_LOGGING_KEY, _tau);
    // redis_client->setEigenMatrixDerived(CONSTRAINED_NULLSPACE_KEY, _N_s);

    // safety checks
    // joint torques, velocity and positions
    for (int i = 0; i < 7; ++i) {
      // torque saturation
      if (tau_cmd_array[i] > joint_torques_limits[i]) {
        std::cout << "WARNING : Torque commanded on joint " << i
                  << " too high (" << tau_cmd_array[i];
        std::cout << "), saturating to max value(" << joint_torques_limits[i]
                  << ")" << std::endl;
        tau_cmd_array[i] = joint_torques_limits[i];
      }

      if (tau_cmd_array[i] < -joint_torques_limits[i]) {
        std::cout << "WARNING : Torque commanded on joint " << i << " too low ("
                  << tau_cmd_array[i];
        std::cout << "), saturating to min value(" << -joint_torques_limits[i]
                  << ")" << std::endl;
        tau_cmd_array[i] = -joint_torques_limits[i];
      }
      // position limit
      if (robot_state.q[i] > joint_position_max[i]) {
        safety_mode_flag = true;
        if (safety_controller_count == 200) {
          std::cout << "WARNING : Soft joint upper limit violated on joint "
                    << i << ", engaging safety mode" << std::endl;
        }
      }
      if (robot_state.q[i] < joint_position_min[i]) {
        safety_mode_flag = true;
        if (safety_controller_count == 200) {
          std::cout << "WARNING : Soft joint lower limit violated on joint "
                    << i << ", engaging safety mode" << std::endl;
        }
      }
      // velocity limit
      if (abs(robot_state.dq[i]) > joint_velocity_limits[i]) {
        safety_mode_flag = true;
        if (safety_controller_count == 200) {
          std::cout << "WARNING : Soft velocity limit violated on joint " << i
                    << ", engaging safety mode" << std::endl;
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
        throw std::runtime_error("Stopping driver due to safety violation");
      }
      safety_controller_count--;
    }

    // Monitor fault on the connected robot
    if (robot.fault()) {
      throw std::runtime_error("PeriodicTask: Fault occurred on the "
                               "connected robot, exiting ...");
    }

    // Set joint torques to command torques from Redis
    std::array<double, K_DOF> target_torque = tau_cmd_array;

    bool active_command = false;
    for (int i = 0; i < 7; ++i) {
      if (redis_command_storage_array[i] != 0) {
        active_command = true;
      }
    }

    // Set 0 joint toruqes
    // std::array<double, K_DOF> target_torque = {};

    // Initialization at the start
    if (!initialized_torque_bias) {
      if (first_loop) {
        for (int i = 0; i < 7; ++i) {
          q_init(i) = robot_state.q[i];
        }
      }
      first_loop = false;

      for (int i = 0; i < 7; ++i) {
        target_torque[i] = -kp_holding[i] * (robot_state.q[i] - q_init(i)) -
                           kv_holding[i] * robot_state.dq[i];
        init_torque_bias(i) += (1. / n_samples) * target_torque[i];
      }

      if (n_curr > n_samples) {
        initialized_torque_bias = true;
        std::cout << "Initial torque bias: " << init_torque_bias.transpose()
                  << "\n";
      }
      n_curr++;

    } else if (!active_command) {

      if ((_sensed_torques - gravity_vector).norm() > FREE_DRIVE_THRESHOLD) {
        not_touching_counter = 0;

        // redis_client->setCommandIs(
        //     SAI_DEBUG_KEY, "From Driver - Robot Touch Detected - Floating");
        // active drive
        for (int i = 0; i < 7; ++i) {
          q_init(i) = robot_state.q[i];
          // target_torque[i] += init_torque_bias(i);
        }
      } else {

        not_touching_counter++;

        if (not_touching_counter > NOT_TOUCHING_WINDOW) {
          // position hold if not touching
          // redis_client->setCommandIs(
          //     SAI_DEBUG_KEY, "From Driver - No Touch Detected, Holding");
          for (int i = 0; i < 7; ++i) {
            target_torque[i] =
                -kp_holding_drive[i] * (robot_state.q[i] - q_init(i)) -
                kv_holding_drive[i] * robot_state.dq[i];
          }
        } else {
          for (int i = 0; i < 7; ++i) {
            q_init(i) = robot_state.q[i];
            // target_torque[i] += init_torque_bias(i);
            target_torque[i] = -kv_holding_drive[i] * robot_state.dq[i];
          }
        }
      }

      // multiply by mass matrix
      Vector7d holding_torques = Vector7d::Zero();
      for (int i = 0; i < 7; ++i) {
        holding_torques(i) = target_torque[i];
      }
      Vector7d decoupled_holding_torques = MassMatrix * holding_torques;
      for (int i = 0; i < 7; ++i) {
        target_torque[i] = decoupled_holding_torques(i);
      }

    } else {
      for (int i = 0; i < 7; ++i) {
        target_torque[i] += init_torque_bias(i);
      }
    }

    // Add some velocity damping
    for (size_t i = 0; i < K_DOF; ++i) {
      target_torque[i] += -kFloatingDamping[i] * robot_state.dtheta[i];
      // std::cout << target_torque[i] << "\n";
    }

    // Send target joint torque to RDK server, enable gravity
    // compensation and joint limits soft protection
    robot.StreamJointTorque(arrayToVector(target_torque), true, true);

    counter++;
  } catch (const std::exception &e) {
    std::cout << "Error \n"
              << "\n";
    spdlog::error(e.what());
    g_stop_sched = true;
  }
}

int main(int argc, char **argv) {

  // Program Setup
  // =============================================================================================
  // Logger for printing message with timestamp and coloring
  // flexiv::Log log;

  // Parse parameters
  if (argc < 2 ||
      flexiv::rdk::utility::ProgramArgsExistAny(argc, argv, {"-h", "--help"})) {
    PrintHelp();
    return 1;
  }
  // config file path
  std::string config_file = argv[1];
  std::string config_file_path = std::string(CONFIG_FOLDER) + "/" + config_file;
  driver_config = Sai::Flexiv::loadConfig(config_file_path);

  std::string redis_prefix = driver_config.redis_prefix.empty()
                                 ? ""
                                 : driver_config.redis_prefix + "::";

  JOINT_TORQUES_COMMANDED_KEY = redis_prefix +
                                "commands::" + driver_config.robot_name +
                                "::control_torques";
  JOINT_ANGLES_KEY = redis_prefix + "sensors::" + driver_config.robot_name +
                     "::joint_positions";
  JOINT_VELOCITIES_KEY = redis_prefix + "sensors::" + driver_config.robot_name +
                         "::joint_velocities";
  JOINT_TORQUES_SENSED_KEY =
      redis_prefix + "sensors::" + driver_config.robot_name + "::joint_torques";
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

  GRIPPER_CURRENT_WIDTH_KEY = redis_prefix +
                              "sensors::" + driver_config.robot_name +
                              "::gripper::width";
  GRIPPER_SENSED_GRASP_FORCE_KEY = redis_prefix +
                                   "sensors::" + driver_config.robot_name +
                                   "::gripper::grasp_force";

  if (driver_config.robot_type == Sai::Flexiv::RobotType::RIZON_4S) {
    RAW_WRIST_FORCE_SENSED_KEY = redis_prefix +
                                 "sensors::" + driver_config.robot_name +
                                 "::ft_sensor::force_raw";
    RAW_WRIST_MOMENT_SENSED_KEY = redis_prefix +
                                  "sensors::" + driver_config.robot_name +
                                  "::ft_sensor::moment_raw";
    TCP_FORCE_SENSED_KEY = redis_prefix +
                           "sensors::" + driver_config.robot_name +
                           "::ft_sensor::tcp_force";
    TCP_MOMENT_SENSED_KEY = redis_prefix +
                            "sensors::" + driver_config.robot_name +
                            "::ft_sensor::tcp_moment";
  }
  SAI_DEBUG_KEY = redis_prefix + "debug";

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
  // safety to detect if controller is already running : wait 50
  // milliseconds
  usleep(50000);
  redis_client->getDoubleArray(JOINT_TORQUES_COMMANDED_KEY, tau_cmd_array, 7);
  for (int i = 0; i < 7; ++i) {
    if (tau_cmd_array[i] != 0) {
      std::cout << "Stop the controller before running the driver\n"
                << "\n";
      return -1;
    }
  }

  redis_client->setDoubleArray(GRIPPER_PARAMETERS_COMMANDED_KEY, gripper_parameters_array, 3);

  TorqueCommandData initial_torque_command;
  initial_torque_command.command_torques = tau_cmd_array;
  torque_command_exchange.initialize(initial_torque_command);

  GripperCommandData initial_gripper_command;
  initial_gripper_command.gripper_parameters = gripper_parameters_array;
  gripper_command_exchange.initialize(initial_gripper_command);

  robot_state_exchange.initialize(RobotStateExchangeData{});
  gripper_status_exchange.initialize(GripperStatusData{});

  // prepare batch command
  set_get_batch_key_names.push_back(JOINT_TORQUES_COMMANDED_KEY);
  set_get_batch_key_names.push_back(GRIPPER_PARAMETERS_COMMANDED_KEY);
  set_get_batch_key_names.push_back(MASSMATRIX_KEY);

  set_get_batch_key_names.push_back(JOINT_ANGLES_KEY);
  set_get_batch_key_names.push_back(JOINT_VELOCITIES_KEY);
  set_get_batch_key_names.push_back(JOINT_TORQUES_SENSED_KEY);
  set_get_batch_key_names.push_back(ROBOT_GRAVITY_KEY);
  set_get_batch_key_names.push_back(CORIOLIS_KEY);

  set_get_batch_key_names.push_back(GRIPPER_CURRENT_WIDTH_KEY);
  set_get_batch_key_names.push_back(GRIPPER_SENSED_GRASP_FORCE_KEY);

  robot_joint_sensor_feedback.push_back(q_array);
  robot_joint_sensor_feedback.push_back(dq_array);
  robot_joint_sensor_feedback.push_back(tau_sensed_array);
  robot_joint_sensor_feedback.push_back(gravity_vector_array);
  robot_joint_sensor_feedback.push_back(coriolis_array);

  gripper_status_feedback.push_back(gripper_current_width);
  gripper_status_feedback.push_back(gripper_sensed_grasp_force);

  // Add additional keys and data if rizon model is 4S
  if (driver_config.robot_type == Sai::Flexiv::RobotType::RIZON_4S) {
    set_get_batch_key_names.push_back(RAW_WRIST_FORCE_SENSED_KEY);
    set_get_batch_key_names.push_back(RAW_WRIST_MOMENT_SENSED_KEY);
    set_get_batch_key_names.push_back(TCP_FORCE_SENSED_KEY);
    set_get_batch_key_names.push_back(TCP_MOMENT_SENSED_KEY);

    robot_wrist_ft_sensor_feedback.push_back(wrist_ft_sensed_raw_force);
    robot_wrist_ft_sensor_feedback.push_back(wrist_ft_sensed_raw_moment);
    robot_wrist_ft_sensor_feedback.push_back(tcp_sensed_force);
    robot_wrist_ft_sensor_feedback.push_back(tcp_sensed_moment);
  }


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
  UpdateSafetyLimits();

  std::atomic<bool> redis_thread_running{false};
  std::thread redis_thread;
  std::atomic<bool> gripper_thread_running{false};
  std::thread gripper_thread;

  try {

    // RDK Initialization
    // =========================================================================================
    // Instantiate robot interface
    // flexiv::rdk::Robot robot(driver_config.serial_number,
    //                          {"192.168.100.11"});
    flexiv::rdk::Robot robot(driver_config.serial_number,
                             {driver_config.computer_ip_address});
    // flexiv::rdk::Robot robot(driver_config.serial_number);
    // load the kinematics and dynamics model
    flexiv::rdk::Model model(robot);

    // Clear fault on the connected robot if any
    if (robot.fault()) {
      spdlog::warn("Fault occurred on the connected robot, trying to "
                   "clear ...");
      // Try to clear the fault
      if (!robot.ClearFault()) {
        spdlog::error("Fault cannot be cleared, exiting ...");
        return 1;
      }
      spdlog::info("Fault on the connected robot is cleared");
    }

    // Enable the robot, make sure the E-stop is released before
    // enabling
    spdlog::info("Enabling robot ...");
    robot.Enable();

    // Wait for the robot to become operational
    while (!robot.operational()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    spdlog::info("Robot is now operational");

    // Switch Mode to Primitive Execution
    robot.SwitchMode(flexiv::rdk::Mode::NRT_PRIMITIVE_EXECUTION);

    // Zero Force-torque Sensors
    // =========================================================================================
    // IMPORTANT: must zero force/torque sensor offset for accurate
    // force/torque measurement
    robot.ExecutePrimitive("ZeroFTSensor",
                           std::map<std::string, rdk::FlexivDataTypes>{});

    // WARNING: during the process, the robot must not contact anything,
    // otherwise the result will be inaccurate and affect following
    // operations
    spdlog::info(
        "Zeroing force/torque sensors, make sure nothing is in contact "
        "with the robot");

    // Wait for primitive completion
    // while (robot.busy()) {
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    // }
    while (!std::get<int>(robot.primitive_states()["terminated"])) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    spdlog::info("Sensor zeroing complete");

    // // Wait for the primitive to finish
    // while (robot.busy()) {
    //     std::this_thread::sleep_for(std::chrono::seconds(5));
    // }

    // Gripper Control
    // =========================================================================================
    // Instantiate gripper control interface
    rdk::Gripper gripper(robot);

    // Instantiate tool interface. Gripper is categorized as both a device and a
    // tool. The device attribute allows a gripper to be interactively
    // controlled by the user; whereas the tool attribute tells the robot to
    // account for its mass properties and TCP location.
    rdk::Tool tool(robot);

    // Enable the specified gripper as a device. This is equivalent to enabling
    // the specified gripper in Flexiv Elements -> Settings -> Device
    spdlog::info("Enabling gripper [{}]", driver_config.gripper_name);
    gripper.Enable(driver_config.gripper_name);

    // Print parameters of the enabled gripper
    spdlog::info("Gripper params:");
    std::cout << std::fixed << std::setprecision(3) << "{\n"
              << "name: " << gripper.params().name
              << "\nmin_width: " << gripper.params().min_width
              << "\nmax_width: " << gripper.params().max_width
              << "\nmin_force: " << gripper.params().min_force
              << "\nmax_force: " << gripper.params().max_force
              << "\nmin_vel: " << gripper.params().min_vel
              << "\nmax_vel: " << gripper.params().max_vel << "\n}"
              << std::endl;

    // User needs to determine if this gripper requires manual initialization
    int choice = 0;
    spdlog::info("Manually trigger initialization for the gripper now?");
    std::cout << "[1] Skip Gripper Initialization" << std::endl;
    std::cout << "[2] Perform Gripper Initialization Now" << std::endl;
    std::cin >> choice;

    // Trigger manual initialization based on choice
    if (choice == 1) {
      spdlog::info("Skipped manual initialization");
    } else if (choice == 2) {
      gripper.Init();
      // User determines if the manual initialization is finished
      spdlog::info("Triggered manual initialization, press Enter when the "
                   "initialization is finished "
                   "to continue");
      std::cin.get();
      std::cin.get();
    } else {
      spdlog::error("Invalid choice");
      return 1;
    }

    // Real-time Control
    // =========================================================================================
    // Switch to real-time joint torque control mode
    robot.SwitchMode(flexiv::rdk::Mode::RT_JOINT_TORQUE);

    // Create real-time scheduler to run periodic tasks
    flexiv::rdk::Scheduler scheduler;
    redis_thread_running.store(true, std::memory_order_release);
    redis_thread = std::thread(RedisManagerThread, redis_client,
                               std::ref(redis_thread_running));
    gripper_thread_running.store(true, std::memory_order_release);
    gripper_thread = std::thread(GripperCommandThread, std::ref(gripper),
                                 std::ref(gripper_thread_running));
    // Add periodic task with 1ms interval and highest applicable
    // priority
    scheduler.AddTask(std::bind(PeriodicTask, std::ref(robot), std::ref(model)),
                      "HP periodic", 1, driver_config.process_priority,
                      driver_config.cpu_affinity);
    // Start all added tasks
    scheduler.Start();

    // Block and wait for signal to stop scheduler tasks
    while (!g_stop_sched) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // Received signal to stop scheduler tasks
    scheduler.Stop();
    gripper_thread_running.store(false, std::memory_order_release);
    redis_thread_running.store(false, std::memory_order_release);
    if (gripper_thread.joinable()) {
      gripper_thread.join();
    }
    if (redis_thread.joinable()) {
      redis_thread.join();
    }

  } catch (const std::exception &e) {
    gripper_thread_running.store(false, std::memory_order_release);
    redis_thread_running.store(false, std::memory_order_release);
    if (gripper_thread.joinable()) {
      gripper_thread.join();
    }
    if (redis_thread.joinable()) {
      redis_thread.join();
    }
    spdlog::error(e.what());
    return 1;
  }

  return 0;
}

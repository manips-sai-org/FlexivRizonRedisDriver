/*
 * redis_thread_example.cpp
 *
 * Focused example: move Redis network I/O into a dedicated thread while
 * providing realtime access to small, lock-free snapshots.
 *
 * Integration notes:
 * - Include this file alongside the drivers and call/instantiate the
 *   redis manager from main before starting the realtime Scheduler.
 * - Use DriverConfig values for Redis key names (shown here as example keys).
 * - This example uses Sai::Flexiv::CDatabaseRedisClient (SaiFlexivRedisClientLocal.h)
 *   defined in this repo; adjust includes/paths as needed.
 *
 * Compile (example):
 *  - g++ -std=c++17 -I. -I<eigen_include> redis_thread_example.cpp -o redis_thread_example
 *  - Link with hiredis/jsoncpp if you run it against a real Redis server.
 */

#include <array>
#include <atomic>
#include <chrono>
#include <thread>
#include <iostream>
#include <cstring>

#include <Eigen/Dense>

#include "SaiFlexivRedisClientLocal.h"

using namespace Sai::Flexiv;

// Example Redis keys (in real integration build these from DriverConfig)
static const std::string JOINT_ANGLES_KEY = "sai::sensors::rizon::joint_positions";
static const std::string JOINT_VELOCITIES_KEY = "sai::sensors::rizon::joint_velocities";
static const std::string JOINT_TORQUES_CMD_KEY = "sai::commands::rizon::control_torques";

// Small POD containers used for lock-free copying
struct SharedState {
    std::array<double, 7> joint_pos{};
    std::array<double, 7> joint_vel{};
    std::array<double, 7> joint_tau{}; // sensed
    Eigen::Vector3d gripper{}; // small extra example
};

struct SharedCmd {
    std::array<double, 7> torques{};
    Eigen::Vector3d gripper{};
    uint64_t seq = 0; // monotonic sequence for published commands
};

// Double buffers and atomics
static SharedState g_sensor_buf[2];
static std::atomic<int> g_sensor_idx{0}; // index of the currently visible buffer

static SharedCmd g_cmd_buf[2];
static std::atomic<uint64_t> g_cmd_seq{0}; // realtime increments when publishing a new command

// Redis manager thread: performs blocking network I/O and publishes snapshots
void redisManagerThread(CDatabaseRedisClient *redis_client, std::atomic<bool> &running, int poll_ms = 5) {
    // last command sequence observed by redis thread
    uint64_t last_sent_seq = 0;

    // Temporary local structures for reads
    SharedState tmp;

    while (running.load(std::memory_order_acquire)) {
        try {
            // Read sensor keys from Redis into tmp (blocking operations allowed here)
            // These calls throw on error in the project's CDatabaseRedisClient
            redis_client->getDoubleArray(JOINT_ANGLES_KEY, tmp.joint_pos, 7);
            redis_client->getDoubleArray(JOINT_VELOCITIES_KEY, tmp.joint_vel, 7);
            // (Optionally) read sensed torques & gripper state if present
            // redis_client->getDoubleArray(JOINT_TORQUES_SENSED_KEY, tmp.joint_tau, 7);

            // Publish snapshot with atomic index flip (publish side)
            int write_idx = 1 - g_sensor_idx.load(std::memory_order_acquire);
            // copy is cheap (POD arrays + small Eigen vector)
            g_sensor_buf[write_idx] = tmp;
            // Release: make the written data visible before flipping the index
            g_sensor_idx.store(write_idx, std::memory_order_release);

            // Check for outgoing commands published by realtime loop
            uint64_t seq = g_cmd_seq.load(std::memory_order_acquire);
            if (seq != last_sent_seq && seq > 0) {
                // Copy the command to send
                SharedCmd to_send = g_cmd_buf[seq % 2];

                // Serialize and send to Redis (batching suggested)
                // Here we use setDoubleArray to post torques as JSON array
                redis_client->setDoubleArray(JOINT_TORQUES_CMD_KEY, to_send.torques, 7);

                // Update last_sent_seq when the network sends succeed
                last_sent_seq = seq;
            }

        } catch (const std::exception &e) {
            std::cerr << "Redis manager error: " << e.what() << "\n";
            // In production, decide whether to stop running or continue
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
    }
}

// Simulated realtime periodic task: reads latest snapshot atomically and publishes commands
void realtimeLoopSimulation(std::atomic<bool> &running, int iterations = 2000) {
    for (int it = 0; it < iterations && running.load(); ++it) {
        // Acquire currently visible snapshot index
        int idx = g_sensor_idx.load(std::memory_order_acquire);
        // Copy small snapshot for use in realtime control logic (single cheap memcpy-like copy)
        SharedState snapshot = g_sensor_buf[idx];

        // --- Realtime control logic goes here ---
        // For demonstration, compute simple damping torques: -K * vel
        std::array<double, 7> computed_torques{};
        for (size_t i = 0; i < 7; ++i) {
            computed_torques[i] = -2.0 * snapshot.joint_vel[i];
        }

        // Publish command using sequence-numbered double buffer (non-blocking)
        uint64_t next_seq = g_cmd_seq.fetch_add(1, std::memory_order_acq_rel) + 1;
        SharedCmd &slot = g_cmd_buf[next_seq % 2];
        slot.torques = computed_torques;
        // publish gripper example
        slot.gripper = snapshot.gripper; // pass-through in this example
        slot.seq = next_seq;

        // Do NOT perform any Redis/network I/O here
        // The redisManagerThread will observe g_cmd_seq and send the latest command

        // Simulate a 1ms periodic realtime loop
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    running.store(false);
}

int main() {
    // Example startup: create and configure a hiredis client (same as drivers)
    CDatabaseRedisClient *redis_client = new CDatabaseRedisClient();
    HiredisServerInfo info;
    info.hostname_ = "127.0.0.1";
    info.port_ = 6379;
    info.timeout_ = {1, 500000}; // 1.5 sec
    try {
        redis_client->serverIs(info);
    } catch (const std::exception &e) {
        std::cerr << "Failed to connect to redis server: " << e.what() << "\n";
        return 1;
    }

    // Initialize shared buffers to known safe values
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 7; ++j) {
            g_sensor_buf[i].joint_pos[j] = 0.0;
            g_sensor_buf[i].joint_vel[j] = 0.0;
            g_sensor_buf[i].joint_tau[j] = 0.0;
            g_cmd_buf[i].torques[j] = 0.0;
        }
        g_sensor_buf[i].gripper = Eigen::Vector3d::Zero();
        g_cmd_buf[i].gripper = Eigen::Vector3d::Zero();
        g_cmd_buf[i].seq = 0;
    }

    std::atomic<bool> running{true};

    // Start Redis manager thread (non-realtime)
    std::thread redis_thread(redisManagerThread, redis_client, std::ref(running), 5);

    // Start realtime simulation (replace with actual PeriodicTask in driver)
    realtimeLoopSimulation(running, 2000);

    // Join redis thread and cleanup
    if (redis_thread.joinable()) redis_thread.join();

    delete redis_client;
    std::cout << "Example finished" << std::endl;
    return 0;
}

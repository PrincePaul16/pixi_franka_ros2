#pragma once

#include <atomic>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

namespace crisp_mujoco_sim
{
class MuJoCoSimulator
{
private:
  MuJoCoSimulator();

  // Lock the mutex for these calls
  void syncStates();

public:
  // Modern singleton approach
  MuJoCoSimulator(const MuJoCoSimulator &) = delete;
  MuJoCoSimulator & operator=(const MuJoCoSimulator &) = delete;
  MuJoCoSimulator(MuJoCoSimulator &&) = delete;
  MuJoCoSimulator & operator=(MuJoCoSimulator &&) = delete;

  // Use this in ROS2 code
  static MuJoCoSimulator & getInstance()
  {
    static MuJoCoSimulator simulator;
    return simulator;
  }

  // MuJoCo data structures
  mjModel * m = NULL;  // MuJoCo model
  mjData * d = NULL;   // MuJoCo data

  // Buffers for data exchange with ROS2-control
  std::vector<double> eff_cmd;
  std::vector<double> pos_state;
  std::vector<double> vel_state;
  std::vector<double> eff_state;

  // Safety guards for buffers
  std::mutex state_mutex;
  std::mutex command_mutex;

  // True once the model is loaded, the home keyframe applied, and the state
  // buffers first populated. Lets the hardware interface wait before exposing
  // joint states, so controllers never activate from an uninitialized pose.
  std::atomic<bool> sim_ready{false};
  bool ready() const { return sim_ready.load(); }

  // Control input callback for the solver
  static void controlCB(const mjModel * m, mjData * d);
  void controlCBImpl(const mjModel * m, mjData * d);
  void controlCBImplTorque(const mjModel * m, mjData * d);

  // Call this in a separate thread
  static int simulate(const std::string & model_xml);
  int simulateImpl(const std::string & model_xml);

  // Non-blocking
  void read(std::vector<double> & pos, std::vector<double> & vel, std::vector<double> & eff);
  void write(const std::vector<double> & eff);

  void readFullState(std::vector<double> &qpos, std::vector<double> &qvel);
};

}  // namespace crisp_mujoco_sim

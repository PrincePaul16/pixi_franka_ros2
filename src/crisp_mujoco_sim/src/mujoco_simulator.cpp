#include "crisp_mujoco_sim/mujoco_simulator.h"


#include <iostream>
#include <memory>
#include <ostream>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>


namespace crisp_mujoco_sim
{
MuJoCoSimulator::MuJoCoSimulator() {}

void MuJoCoSimulator::controlCB(const mjModel * m, mjData * d)
{
  getInstance().controlCBImplTorque(m, d);
}

void MuJoCoSimulator::controlCBImplTorque([[maybe_unused]] const mjModel * m, mjData * d)
{
  command_mutex.lock();

  for (size_t i = 0; i < eff_cmd.size(); ++i)
  {
    d->ctrl[i] = eff_cmd[i];  // torque control
  }
  command_mutex.unlock();
}

int MuJoCoSimulator::simulate(const std::string & model_xml)
{
  return getInstance().simulateImpl(model_xml);
}

int MuJoCoSimulator::simulateImpl(const std::string & model_xml)
{
  // Make sure that the ROS2-control system_interface only gets valid data in read().
  // We lock until we are done with simulation setup.
  state_mutex.lock();
  rclcpp::Logger logger = rclcpp::get_logger("MuJoCoSimulator");

  // load and compile model
  char error[1000] = "Could not load binary model";
  m = mj_loadXML(model_xml.c_str(), nullptr, error, 1000);
  if (!m)
  {
    RCLCPP_ERROR_STREAM(logger, "Could not start the simulation: "  << error);
    /*mju_error_s("Load model error: %s", error);*/
    return 1;
  }

  RCLCPP_INFO_STREAM(logger, "Creating data for simulation");
  // Set initial state with the keyframe mechanism from xml
  d = mj_makeData(m);
  mju_copy(d->qpos, m->key_qpos, m->nq);

  // Initialize buffers for ROS2-control.
  pos_state.resize(m->nu);
  vel_state.resize(m->nu);
  eff_state.resize(m->nu);

  // Size the command buffer to the number of actuators (d->ctrl is nu-sized),
  // NOT nv: with unactuated DOF (e.g. free-joint tangram pieces) nv > nu, and
  // writing eff_cmd into d->ctrl would overrun the ctrl array.
  eff_cmd.resize(m->nu);

  RCLCPP_INFO_STREAM(logger, "Syncing states.");
  syncStates();
  state_mutex.unlock();

  // Model loaded, home keyframe applied, state buffers populated: safe for the
  // hardware interface to start exposing joint states now.
  sim_ready = true;

  // Connect our specific control input callback for MuJoCo's engine.
  mjcb_control = MuJoCoSimulator::controlCB;

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  RCLCPP_INFO(logger, "Starting simulation");
  /*auto previous_time = clock->now();*/
  /*auto dt = (double)m->opt.timestep;*/

  auto starting_time = clock->now();

  // Simulate in realtime
  while (true)
  {
    mj_step1(m, d);
    /*mj_step(m, d);*/

    // Provide fresh data for ROS2-control
    state_mutex.lock();
    syncStates();
    RCLCPP_DEBUG_STREAM_THROTTLE(logger, *clock, 1000, "Control: " << d->ctrl[0] << ", " << d->ctrl[1] << ", " << d->ctrl[2] << ", " << d->ctrl[3] << ", " << d->ctrl[4] << ", " << d->ctrl[5] << ", " << d->ctrl[6]);
    if (std::any_of(eff_cmd.begin(), eff_cmd.end(), [](double value) { return value > 0.0; }))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(logger, *clock, 1000, "Command: " << eff_cmd[0] << ", " << eff_cmd[1] << ", " << eff_cmd[2] << ", " << eff_cmd[3] << ", " << eff_cmd[4] << ", " << eff_cmd[5] << ", " << eff_cmd[6]);
    }

    RCLCPP_DEBUG_STREAM_THROTTLE(logger, *clock, 1000, "State: " << d->qpos[0] << ", " << d->qpos[1] << ", " << d->qpos[2] << ", " << d->qpos[3] << ", " << d->qpos[4] << ", " << d->qpos[5] << ", " << d->qpos[6]);

    state_mutex.unlock();

    // Sync time
    while (clock->now() < starting_time + rclcpp::Duration::from_seconds(d->time))
    {
      rclcpp::sleep_for(100 * rclcpp::nanoseconds(1));
    }


    RCLCPP_DEBUG_STREAM_THROTTLE(logger, *clock, 1000, "Time: " << d->time);

    mj_step2(m, d);
    /*previous_time = clock->now();*/
  }

  return 0;
}

void MuJoCoSimulator::read(std::vector<double> & pos, std::vector<double> & vel,
                           std::vector<double> & eff)
{
  if (state_mutex.try_lock())
  {
    // Only publish once the sim thread has actually populated the buffers.
    // During startup the sim thread may have begun but not yet loaded the model
    // (pos_state still empty); copying it would clobber the caller's home-seeded
    // state with garbage, which the active controller can latch as its target
    // -> intermittent "wrong pose on launch". Skip until sizes match.
    if (pos_state.size() == pos.size())
    {
      pos = pos_state;
      vel = vel_state;
      eff = eff_state;
    }
    state_mutex.unlock();
  }
}

void MuJoCoSimulator::write(const std::vector<double> & eff)
{
  if (command_mutex.try_lock())
  {
    eff_cmd = eff;
    command_mutex.unlock();
  }
}

void MuJoCoSimulator::syncStates()
{
  for (auto i = 0; i < m->nu; ++i)
  {
    pos_state[i] = d->qpos[i];
    vel_state[i] = d->qvel[i];
    eff_state[i] = d->qfrc_actuator[i];
  }
}

}  // namespace crisp_mujoco_sim

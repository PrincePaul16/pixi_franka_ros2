#include "crisp_mujoco_sim/mujoco_simulator.h"


#include <iostream>
#include <memory>
#include <ostream>
#include <thread>
#include <GLFW/glfw3.h>
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

  eff_cmd.resize(m->nv);

  // Full-state snapshot for the independent render thread.
  qpos_render.resize(m->nq);

  RCLCPP_INFO_STREAM(logger, "Syncing states.");
  syncStates();
  state_mutex.unlock();

  // Spawn the viewer on its own thread with its own GL context + mjData copy.
  // It only reads qpos_render (under state_mutex), so it never disturbs the
  // realtime control loop below.
  std::thread(&MuJoCoSimulator::renderLoop, this).detach();

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
    pos = pos_state;
    vel = vel_state;
    eff = eff_state;
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
  // Full-state snapshot so the render thread can reconstruct all body poses.
  mju_copy(qpos_render.data(), d->qpos, m->nq);
}

void MuJoCoSimulator::renderLoop()
{
  rclcpp::Logger logger = rclcpp::get_logger("MuJoCoSimulator");

  if (!glfwInit())
  {
    RCLCPP_ERROR(logger, "Could not initialize GLFW; running headless.");
    return;
  }

  GLFWwindow * window = glfwCreateWindow(1200, 900, "crisp_mujoco_sim", nullptr, nullptr);
  if (!window)
  {
    RCLCPP_ERROR(logger, "Could not create GLFW window; running headless.");
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  // vsync is fine: this thread is independent of control.

  // Own visualization structures and an own mjData so we never touch the sim's.
  mjvCamera cam;
  mjvOption opt;
  mjvScene scn;
  mjrContext con;
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  mjData * d_render = mj_makeData(m);

  while (!glfwWindowShouldClose(window))
  {
    // Grab the latest joint state from the sim thread.
    state_mutex.lock();
    mju_copy(d_render->qpos, qpos_render.data(), m->nq);
    state_mutex.unlock();

    // Reconstruct world-frame body/geom poses (xpos/xmat/geom_xpos) from qpos.
    // Use mj_kinematics (NOT mj_forward): mj_forward would re-run the global
    // mjcb_control callback + full constraint solver on this thread, contending
    // for command_mutex with the 1000 Hz control loop. Kinematics is all the
    // renderer needs and touches nothing the sim thread uses.
    mj_kinematics(m, d_render);

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjv_updateScene(m, d_render, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  mj_deleteData(d_render);
  mjr_freeContext(&con);
  mjv_freeScene(&scn);
  glfwDestroyWindow(window);
  glfwTerminate();
}

}  // namespace crisp_mujoco_sim

# Install `franka_ros2` with `pixi`

## Get Started

### Default franka_ros2 setup
Get started with  [`franka_ros2`](https://github.com/frankarobotics/franka_ros2) faster using pixi:
```bash
pixi run setup
pixi run -e jazzy ros2 launch franka_brigup franka.launch.py robot_ip:=XXX.XXX.XXX.XXX load_gripper:=true
```

#### Use CRISP for control

Control the robots [CRISP](https://utiasdsl.github.io/crisp_controllers/) controllers (cartesian_impedance_controller, joint_impedance_controller ...)
```bash
# Add the CRISP controllers to the installation
pixi run crisp-clone  
pixi run build

pixi run -e jazzy franka robot_ip:=XXX.XXX.XXX.XXX controllers_yaml:=config/controllers.yaml load_gripper:=true
```

In a different terminal test following a figure-eight with the example file:
```bash
pixi run crisp-figure-eight
```

Or simple teleop with fixed orientation:
```bash
# Terminal A
pixi run -e jazzy franka robot_ip:=XXX.XXX.XXX.XXX controllers_yaml:=config/controllers.yaml load_gripper:=true namespace:=left

# Terminal B
pixi run -e jazzy franka robot_ip:=YYY.YYY.YYY.YYY controllers_yaml:=config/controllers.yaml load_gripper:=true namespace:=right

# Terminal C runs a crisp_py example where teleop is enabled
pixi run crisp-teleop
```

### Simulate in MuJoCo (no real robot)

Run the **exact same CRISP controller stack** against a MuJoCo simulation instead of the
real FR3, so the whole ROS pipeline can be verified before deploying on hardware. The
sim is a `ros2_control` `SystemInterface` plugin (`crisp_mujoco_sim`) that backs the
`effort` command / `position,velocity,effort` state interfaces with MuJoCo — so nothing
above the hardware layer (controllers, topics, `crisp_py` scripts) changes between sim
and real. The switch is the `use_fake_hardware` xacro flag: `true` → MuJoCo,
`false` → real robot.

Everything runs in the `jazzy-crisp` environment (adds the `crisp` feature on top of
`jazzy`).

#### Initial setup (from scratch)

Exact command sequence used to get here, starting from an empty checkout:

```bash
# 1. Clone the repo on the MuJoCo-sim branch
git clone -b feat-add-mujoco-sim https://github.com/danielsanjosepro/pixi_franka_ros2.git
cd pixi_franka_ros2

# 2. Clone deps (franka_ros2 + franka_description + libfranka) + first build.
#    Pick the `jazzy-crisp` environment when pixi prompts.
pixi run -e jazzy-crisp setup

# 3. The build stops on mobile_fr3_duo_trajectory_controller (API-incompatible with
#    Jazzy ros2_control) — skip it permanently, then continue.
touch src/franka_ros2/mobile_fr3_duo_trajectory_controller/COLCON_IGNORE

# 4. Add the CRISP controllers and (re)build the workspace with them included
pixi run -e jazzy-crisp clone-crisp
pixi run -e jazzy-crisp build
```

> `config/fr3.urdf.xacro` has already been updated to the `franka_description` 2.8.1
> `franka_robot` macro signature (`robot_type` instead of `arm_id`); no manual edit needed.

#### Run the simulation

```bash
# Terminal A — launch the MuJoCo-backed robot (rviz + a live MuJoCo viewer window)
pixi run -e jazzy-crisp franka-sim

# Terminal B — command the arm exactly as you would the real robot
pixi run -e jazzy-crisp crisp-figure-eight     # example figure-eight
# ...or any crisp_py script publishing to the cartesian_impedance_controller
```

The MuJoCo viewer runs on its own render thread (own GL context + `mjData`, updated via
`mj_kinematics`), so it is display-only and never perturbs the 1000 Hz control loop — the
arm holds at its home pose on launch and tracks commands with the window open. Stop the
launch with `Ctrl+C` in Terminal A.

**Notes / gotchas**
- The scene (`src/franka_ros2_pixi/config/scene.xml`) currently sets `gravity="0 0 0"` so
  the un-gravity-compensated arm holds. This is fine for arm-only motion; once free-body
  objects (e.g. tangram pieces) are added, re-enable gravity and either turn on CRISP
  gravity compensation or set `gravcomp="1"` on the arm bodies to match the real FR3.


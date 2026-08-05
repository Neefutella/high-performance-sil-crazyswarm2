# Real-Time Vectorized Crazyflie Swarm Simulation

This repository is a fork of [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) that adds a high-performance software-in-the-loop simulation path for Crazyflie swarms.

The optimized simulation uses:

* vectorized NumPy rigid-body dynamics;
* a batched cascaded position-attitude controller;
* cached Crazyflie firmware trajectory-planner output;
* reduced visualization and firmware-state update rates;
* burst stepping to reduce ROS 2 Python executor overhead.

The current validated configuration simulates **nine Crazyflie drones with 1000 Hz control and physics while running faster than real time**.

> [!NOTE]
> This is a research and development fork. It is not an official IMRCLab or Bitcraze release.

---- [Real-Time Vectorized Crazyflie Swarm Simulation](#real-time-vectorized-crazyflie-swarm-simulation)
  - [Performance](#performance)
  - [Main Changes](#main-changes)
    - [1. Vectorized NumPy physics](#1-vectorized-numpy-physics)
    - [2. Vectorized cascaded controller](#2-vectorized-cascaded-controller)
    - [3. Cached firmware planner](#3-cached-firmware-planner)
    - [4. Burst stepping](#4-burst-stepping)
  - [Tested Environment](#tested-environment)
  - [Installation](#installation)
    - [1. Create a ROS 2 workspace](#1-create-a-ros-2-workspace)
    - [2. Clone this fork](#2-clone-this-fork)
    - [3. Install the upstream dependencies](#3-install-the-upstream-dependencies)
    - [4. Build the workspace](#4-build-the-workspace)
    - [5. Source the workspace](#5-source-the-workspace)
  - [Simulation Configuration](#simulation-configuration)
  - [Recommended Runtime Settings](#recommended-runtime-settings)
  - [Running the Simulation](#running-the-simulation)
  - [Example: Formation Hover and Square Motion](#example-formation-hover-and-square-motion)
    - [Start the formation keeper](#start-the-formation-keeper)
    - [Run the square animation](#run-the-square-animation)
  - [Measuring Real-Time Factor](#measuring-real-time-factor)
  - [Profiling](#profiling)
  - [Runtime Configuration](#runtime-configuration)
    - [Controller selection](#controller-selection)
    - [Planner rate](#planner-rate)
    - [Visualization rate](#visualization-rate)
    - [Burst size](#burst-size)
  - [Controller Parameters](#controller-parameters)
  - [Architecture](#architecture)
  - [Important Limitations](#important-limitations)
    - [No proximity-based target assignment](#no-proximity-based-target-assignment)
    - [Square trajectory overshoot](#square-trajectory-overshoot)
    - [Custom controller mode is not full firmware-controller SIL](#custom-controller-mode-is-not-full-firmware-controller-sil)
  - [Stable Checkpoint](#stable-checkpoint)
  - [Keeping the Fork Updated](#keeping-the-fork-updated)
  - [Planned Work](#planned-work)
  - [Upstream Project](#upstream-project)
  - [License](#license)
  - [Acknowledgements](#acknowledgements)


## Performance

The following configuration has been tested successfully:

| Parameter                         |                             Value |
| --------------------------------- | --------------------------------: |
| Number of drones                  |                                 9 |
| Physics rate                      |                           1000 Hz |
| Controller rate                   |                           1000 Hz |
| Firmware planner sampling rate    |                            100 Hz |
| Visualization rate                |                             50 Hz |
| Simulated `/clock` rate           |                            200 Hz |
| Simulation steps per ROS callback |                                 4 |
| Measured real-time factor         |                         1.25–1.34 |
| Motor-allocation saturation       |                  Approximately 0% |
| Tested behavior                   | Stable hover and formation motion |

A real-time factor greater than `1.0` means the simulation is running faster than real time.

Example result:

```text
Messages:              2639
Wall elapsed:        10.001 s
Simulation elapsed:  13.370 s
/clock rate:          263.76 Hz
RTF:                   1.337
Result: real time or faster
```

---

## Main Changes

### 1. Vectorized NumPy physics

The `np_vectorized` backend stores the state of the entire swarm in batched NumPy arrays.

It evaluates the following operations for all drones simultaneously:

* motor RPM-to-force conversion;
* thrust and torque calculation;
* translational acceleration;
* angular acceleration;
* quaternion integration;
* ground constraints;
* state updates.

The default timestep is:

```text
dt = 0.001 s
```

This corresponds to a physics rate of:

```text
1000 Hz
```

The backend also publishes `/clock` at a lower rate to reduce ROS serialization and DDS overhead.

---

### 2. Vectorized cascaded controller

The optimized simulation path uses a custom cascaded position-attitude controller instead of the firmware Mellinger controller.

The controller is evaluated for the complete swarm in one NumPy batch.

```text
Position and velocity errors
            ↓
Bounded acceleration command
            ↓
Desired thrust direction
            ↓
Desired attitude
            ↓
Geometric attitude controller
            ↓
Bounded body torque
            ↓
Motor-force allocation
            ↓
Motor RPM commands
```

The controller includes:

* horizontal and vertical position feedback;
* velocity damping;
* horizontal acceleration limits;
* maximum tilt-angle limits;
* roll, pitch, and yaw torque limits;
* motor-force limits;
* force-preserving motor allocation;
* Crazyflie-compatible RPM conversion.

The original firmware-controller path remains available as a fallback.

---

### 3. Cached firmware planner

The Crazyflie firmware high-level planner remains the source of desired trajectories.

Calling the firmware planner independently for every drone at 1000 Hz created a significant performance bottleneck. The optimized path samples the firmware planner at a lower rate and reconstructs the desired state between samples.

The default planner rate is:

```text
100 Hz
```

Between planner updates, the desired position is reconstructed using:

```text
p_des(t) = p_sample + v_sample × elapsed_time
```

The desired position, velocity, and quaternion are then passed directly to the vectorized controller as NumPy arrays.

This reduced average setpoint-processing time from approximately:

```text
0.74 ms → 0.08 ms per simulation step
```

---

### 4. Burst stepping

Even after the simulation kernel became faster than real time, ROS 2 Python timer dispatch introduced considerable overhead.

Burst stepping executes multiple complete simulation steps inside one ROS timer callback.

The default burst size is:

```text
4 simulation steps per ROS callback
```

Each internal step still performs:

* desired-state reconstruction;
* controller execution;
* motor allocation;
* physics integration;
* scheduled firmware-state synchronization;
* scheduled visualization;
* scheduled `/clock` publication.

Control and physics are not decimated.

---

## Tested Environment

| Component                 | Tested configuration                       |
| ------------------------- | ------------------------------------------ |
| Operating system          | Ubuntu 24.04                               |
| ROS                       | ROS 2 Jazzy                                |
| Python                    | 3.12                                       |
| CPU                       | AMD Ryzen 9, 8 cores / 16 threads          |
| GPU                       | NVIDIA RTX 5050 Laptop GPU                 |
| Simulation backend        | `np_vectorized`                            |
| Crazyflie firmware commit | `16c5630707e92830dcd16455a57b5560e5838735` |

The optimized NumPy backend runs on the CPU.

A CuPy backend was tested but was not retained in the stable configuration because GPU launch and synchronization overhead was too high for small swarms.

---

## Installation

### 1. Create a ROS 2 workspace

```bash
mkdir -p ~/workspaces/cf_accel_ws/src
cd ~/workspaces/cf_accel_ws/src
```

### 2. Clone this fork

Using SSH:

```bash
git clone git@github.com:Neefutella/crazyswarm2.git
```

Using HTTPS:

```bash
git clone https://github.com/Neefutella/crazyswarm2.git
```

### 3. Install the upstream dependencies

Follow the standard Crazyswarm2 installation instructions for:

* ROS 2;
* Crazyflie firmware bindings;
* Python dependencies;
* Crazyswarm2 packages;
* optional visualization tools.

The Crazyflie firmware Python bindings must be available as:

```python
import cffirmware
```

### 4. Build the workspace

```bash
cd ~/workspaces/cf_accel_ws

source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --allow-overriding crazyflie_sim
```

### 5. Source the workspace

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspaces/cf_accel_ws/install/setup.bash
```

---

## Simulation Configuration

The vectorized backend is selected in:

```text
crazyflie/config/server.yaml
```

Relevant configuration:

```yaml
sim:
  max_dt: 0
  backend: np_vectorized
  visualizations:
    rviz:
      enabled: true
```

The tested setup uses nine Crazyflie entries in the swarm configuration.

---

## Recommended Runtime Settings

Run these commands before launching the simulator:

```bash
export CF_SIM_VECTOR_CONTROLLER=1

export CF_SIM_MAX_XY_ACCEL=1.0
export CF_SIM_MAX_TILT_DEG=10
export CF_SIM_MAX_TORQUE_XY=0.0025

export CF_SIM_PLANNER_RATE_HZ=100
export CF_SIM_VIS_RATE_HZ=50
export CF_SIM_BURST_STEPS=4
```

These values correspond to the validated nine-drone real-time configuration.

---

## Running the Simulation

Source ROS 2 and the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspaces/cf_accel_ws/install/setup.bash
```

Set the optimized simulation parameters:

```bash
export CF_SIM_VECTOR_CONTROLLER=1

export CF_SIM_MAX_XY_ACCEL=1.0
export CF_SIM_MAX_TILT_DEG=10
export CF_SIM_MAX_TORQUE_XY=0.0025

export CF_SIM_PLANNER_RATE_HZ=100
export CF_SIM_VIS_RATE_HZ=50
export CF_SIM_BURST_STEPS=4
```

Launch the simulation:

```bash
ros2 launch crazyflie_test launch.py \
  backend:=sim \
  mocap:=False \
  gui:=False
```

Expected startup output includes lines similar to:

```text
Vectorized NumPy physics backend started for 9 drones at dt=0.0010 s
Vectorized cascaded controller enabled
Vectorized planner cache enabled: requested=100.0Hz actual=100.0Hz
Vectorized visualization rate: 50.0Hz
Simulation burst size: 4 complete step(s) per ROS timer dispatch
Controller path: vectorized cascaded controller
```

---

## Example: Formation Hover and Square Motion

Open another terminal and source the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspaces/cf_accel_ws/install/setup.bash
```

### Start the formation keeper

```bash
ros2 run crazyflie_test formation_keeper_file_sil \
  --ros-args -p use_sim_time:=True
```

The drones should take off and settle into a stable hover formation.

### Run the square animation

```bash
ros2 run crazyflie_test animate_square_file
```

The drones should move through the square sequence and recover to their assigned formation targets.

Controller diagnostics are periodically printed:

```text
VECTOR_CTRL max_pos_error=0.334m allocation_saturation=0.0% max_rpm=19454
```

The diagnostic fields are:

| Field                   | Description                                                   |
| ----------------------- | ------------------------------------------------------------- |
| `max_pos_error`         | Largest position error in the swarm                           |
| `allocation_saturation` | Percentage of controller steps with reduced torque allocation |
| `max_rpm`               | Largest commanded motor RPM                                   |

A temporary increase in position error during a formation transition is expected. During stable operation, the error should decrease after the transition.

---

## Measuring Real-Time Factor

A simple benchmark can calculate the real-time factor from the ROS `/clock` topic.

Example command:

```bash
python3 ~/workspaces/cf_accel_ws/benchmark_rtf.py --window 10
```

Example output:

```text
Messages:              2639
Wall elapsed:        10.001 s
Simulation elapsed:  13.370 s
/clock rate:          263.76 Hz
RTF:                   1.337
Result: real time or faster
```

Interpretation:

|     RTF | Meaning                             |
| ------: | ----------------------------------- |
| `< 1.0` | Simulation is slower than real time |
|   `1.0` | Simulation matches real time        |
| `> 1.0` | Simulation is faster than real time |

The wall-time `/clock` message rate increases when the simulation runs faster than real time.

For example:

```text
200 Hz simulated clock × 1.337 RTF ≈ 267 Hz wall-time message rate
```

---

## Profiling

The simulation server prints timing information for each internal simulation stage.

Example:

```text
SIM_PROFILE N=9 steps=1000 mean=0.740ms callback_rate=1351.4Hz |
setpoint=0.086ms/11.6%
controller=0.346ms/46.7%
physics=0.277ms/37.4%
setstate=0.021ms/2.9%
visualization=0.009ms/1.2%
```

Typical optimized timing:

| Stage                                      | Approximate time |
| ------------------------------------------ | ---------------: |
| Cached planner and setpoint reconstruction |     0.08–0.09 ms |
| Vectorized controller                      |     0.34–0.36 ms |
| Vectorized physics                         |     0.27–0.28 ms |
| Firmware-state synchronization             |          0.02 ms |
| Visualization                              |         0.009 ms |
| Total internal simulation step             |     0.72–0.76 ms |

The burst-stepping optimization reduces ROS executor overhead outside the measured internal simulation step.

---

## Runtime Configuration

### Controller selection

Enable the custom vectorized controller:

```bash
export CF_SIM_VECTOR_CONTROLLER=1
```

Use the firmware-controller fallback:

```bash
export CF_SIM_VECTOR_CONTROLLER=0
```

### Planner rate

Validated default:

```bash
export CF_SIM_PLANNER_RATE_HZ=100
```

Higher-fidelity planner sampling:

```bash
export CF_SIM_PLANNER_RATE_HZ=200
```

Increasing the planner rate may reduce performance.

### Visualization rate

Validated default:

```bash
export CF_SIM_VIS_RATE_HZ=50
```

Higher visualization rate:

```bash
export CF_SIM_VIS_RATE_HZ=100
```

### Burst size

Validated default:

```bash
export CF_SIM_BURST_STEPS=4
```

Disable burst stepping:

```bash
export CF_SIM_BURST_STEPS=1
```

Alternative values:

```bash
export CF_SIM_BURST_STEPS=2
export CF_SIM_BURST_STEPS=8
```

Large burst values can reduce ROS callback responsiveness.

---

## Controller Parameters

| Environment variable       | Description                                |
| -------------------------- | ------------------------------------------ |
| `CF_SIM_VECTOR_CONTROLLER` | Enables the vectorized controller          |
| `CF_SIM_KP_XY`             | Horizontal position proportional gain      |
| `CF_SIM_KD_XY`             | Horizontal velocity damping gain           |
| `CF_SIM_KP_Z`              | Vertical position proportional gain        |
| `CF_SIM_KD_Z`              | Vertical velocity damping gain             |
| `CF_SIM_KR_XY`             | Roll and pitch attitude gain               |
| `CF_SIM_KW_XY`             | Roll and pitch angular-rate damping        |
| `CF_SIM_KR_Z`              | Yaw attitude gain                          |
| `CF_SIM_KW_Z`              | Yaw-rate damping                           |
| `CF_SIM_MAX_XY_ACCEL`      | Maximum horizontal acceleration            |
| `CF_SIM_MAX_Z_FB`          | Maximum vertical feedback acceleration     |
| `CF_SIM_MAX_TILT_DEG`      | Maximum commanded tilt                     |
| `CF_SIM_MAX_TORQUE_XY`     | Maximum roll and pitch torque              |
| `CF_SIM_MAX_TORQUE_Z`      | Maximum yaw torque                         |
| `CF_SIM_PLANNER_RATE_HZ`   | Firmware planner sampling rate             |
| `CF_SIM_VIS_RATE_HZ`       | Visualization update rate                  |
| `CF_SIM_BURST_STEPS`       | Internal simulation steps per ROS callback |

Default controller gains are defined in:

```text
crazyflie_sim/crazyflie_sim/vectorized_cascaded_controller.py
```

---

## Architecture

```text
ROS formation command
          ↓
Crazyflie firmware high-level planner
          ↓
100 Hz planner sampling
          ↓
1000 Hz vectorized setpoint reconstruction
          ↓
Vectorized cascaded controller
          ↓
Batched motor-force allocation
          ↓
Motor RPM commands
          ↓
Vectorized Newton-Euler physics
          ↓
Drone state arrays
          ↓
Decimated firmware-state and visualization updates
```

Four complete internal simulation steps are executed during each ROS timer callback by default.

---

## Important Limitations

### No proximity-based target assignment

Formation targets are assigned using drone identity or list order.

The current implementation does not include:

* nearest-neighbor target assignment;
* Hungarian assignment;
* collision-aware assignment;
* trajectory deconfliction.

When an animation is restarted, drones may travel across the formation to reach targets associated with their identifiers.

This can produce:

* crossing trajectories;
* large temporary position errors;
* an outward bloom-like pattern;
* temporary overshoot;
* extended formation rotation.

The local controller remains stable and returns the drones to their assigned targets, but collision avoidance is not implemented.

---

### Square trajectory overshoot

The current square animation can assign the next target while a drone still has significant lateral velocity.

This can produce:

* rounded corners;
* temporary movement outside the intended square;
* fast transitions;
* overshoot followed by immediate recovery.

Potential improvements include:

* zero terminal velocity at each corner;
* quintic minimum-jerk segments;
* short corner dwell times;
* velocity-aware trajectory restarts;
* reduced segment acceleration;
* proximity-based assignment.

---

### Custom controller mode is not full firmware-controller SIL

When the following setting is enabled:

```bash
export CF_SIM_VECTOR_CONTROLLER=1
```

the optimized simulation bypasses the Crazyflie firmware Mellinger controller.

The firmware high-level planner is still used, but the low-level controller and motor allocator are replaced with the custom vectorized controller.

The optimized mode therefore uses:

* firmware-generated high-level trajectories;
* Crazyflie mass and inertia values;
* Crazyflie motor-force relationships;
* Crazyflie-compatible four-motor geometry;
* custom batched flight control;
* vectorized rigid-body dynamics.

Use the following setting to return to the firmware-controller path:

```bash
export CF_SIM_VECTOR_CONTROLLER=0
```

---

## Stable Checkpoint

The validated real-time baseline is tagged as:

```text
phase5c-stable-realtime
```

Validated commit:

```text
05e0485 Stable real-time vectorized SIL controller and burst stepping
```

Check out the stable tag:

```bash
git switch --detach phase5c-stable-realtime
```

Create a new development branch from it:

```bash
git switch -c experiment-name phase5c-stable-realtime
```

To restore the current branch exactly to the stable checkpoint:

```bash
git reset --hard phase5c-stable-realtime
```

> [!WARNING]
> `git reset --hard` deletes uncommitted changes.

---

## Keeping the Fork Updated

Configure the original repository as `upstream`:

```bash
git remote add upstream https://github.com/IMRCLab/crazyswarm2.git
```

Fetch upstream changes:

```bash
git fetch upstream
```

View the configured remotes:

```bash
git remote -v
```

Expected setup:

```text
origin    git@github.com:Neefutella/crazyswarm2.git
upstream  https://github.com/IMRCLab/crazyswarm2.git
```

Review upstream changes carefully before merging because the simulation-server and backend files have been substantially modified in this fork.

---

## Planned Work

Potential next steps include:

1. testing scalability with 25, 50, and 100 drones;
2. passing NumPy RPM arrays directly from the controller to physics;
3. removing remaining per-drone `Action` object creation;
4. proximity-based formation assignment;
5. collision-aware target allocation;
6. minimum-jerk square trajectories;
7. velocity-aware trajectory restarts;
8. configurable yaw alignment;
9. comparison against real Crazyflie flight data;
10. deployment to physical multi-Crazyflie experiments.

---

## Upstream Project

This repository is based on the official Crazyswarm2 project:

* [IMRCLab/Crazyswarm2](https://github.com/IMRCLab/crazyswarm2)

The upstream project provides:

* the ROS 2 swarm framework;
* Crazyflie interfaces;
* simulation architecture;
* launch files;
* firmware bindings;
* high-level command infrastructure.

---

## License

This fork retains the licensing terms of the upstream Crazyswarm2 repository.

See the repository license files for details.

---

## Acknowledgements

This work builds on:

* [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2);
* [Bitcraze Crazyflie Firmware](https://github.com/bitcraze/crazyflie-firmware);
* ROS 2;
* NumPy;
* Rowan quaternion utilities.

The vectorized physics backend, cascaded controller, planner cache, profiling instrumentation, and burst-stepping implementation were developed as part of an MSc swarm-drone simulation project.

# Project Explanation — ROS 2 Navigation Assignment (Testbed-T1.0.0)

This document explains what this repository is, how its parts fit together, what has been done
so far, and what comes next. It is updated as the project moves forward.

_Last updated: 14 Oct 2026 — Phase 1 (starter-code fixes) done and tested in simulation._

---

## 1. The goal in one paragraph

We have a small two-wheeled robot (**Testbed-T1.0.0**) in a Gazebo simulation. The goal is to make
it **navigate on its own**: load a map, work out where it is on that map, plan a path to a goal you
click, and drive there without hitting anything. The catch is that we may **not** use Nav2's
ready-made `nav2_bringup` launch files. We have to start each Nav2 component ourselves, which proves
we understand how the navigation stack is built. The starter code also contains **deliberate bugs**
that we have to find, fix and document (see `BUGS_FIXED.md`).

---

## 2. What is in the repository

```
ERIC_ASSINGMENT_WS_Om_Bhagwat/
├── testbed_description/   The robot itself: shape (URDF/xacro), 3D meshes, sensors, RViz configs
├── testbed_gazebo/        The simulated world and the launch files that start Gazebo and spawn the robot
├── testbed_bringup/       One launch file that starts everything above + RViz; also holds the map
├── testbed_navigation/    (to be created) OUR package: map loading, localization, navigation
├── BUGS_FIXED.md          Table of every bug/issue and how we fixed it
├── PROJECT_EXPLANATION.md This file
├── README.md / help.md    The assignment brief and FAQ from ERIC Robotics
```

None of the starter packages contain compiled code. They are "data" packages: launch files, URDF,
meshes, worlds and maps. Building them just **copies** those folders into `install/`, where ROS can
find them by package name. This is why bugs #3 and #4 mattered: a folder that isn't listed in
`CMakeLists.txt`'s `install(...)` rule simply doesn't exist at runtime.

---

## 3. How the simulation starts

```
ros2 launch testbed_bringup testbed_full_bringup.launch.py
        │
        ├── robot_description.launch.py   → runs xacro on testbed.xacro → robot_state_publisher
        │                                   publishes /robot_description and the robot's fixed TFs
        ├── spawn_playground.launch.py    → starts Gazebo with worlds/testbed_playground.world
        ├── spawn_testbed.launch.py       → reads /robot_description and spawns the robot at (0, 5)
        └── rviz2                         → visualisation (full_bringup.rviz)
```

Inside Gazebo, plugins declared in `testbed.gazebo` make the robot behave like real hardware:

| Plugin | What it does | ROS interface |
|---|---|---|
| `gazebo_ros_diff_drive` | Turns velocity commands into wheel motion; reports where the robot moved | subscribes `/cmd_vel`, publishes `/odom` and the `odom → base_footprint` TF |
| `gazebo_ros_ray_sensor` | Simulated 2D lidar, 300 rays over 360° | publishes `/scan` (frame `lidar_link_1`) |
| `gazebo_ros_imu_sensor` | Simulated IMU | publishes `/imu` |

### The TF tree (coordinate frames)

```
map ──(AMCL, our job)──► odom ──(diff_drive plugin)──► base_footprint ──► base_link ──► lidar_link_1
```

- **`odom → base_footprint`** is smooth but drifts over time (wheel odometry).
- **`map → odom`** is AMCL's correction. It compares the lidar scan with the map and shifts the
  odometry so the robot's position on the map is right.
- Nav2 needs this whole chain to work. If any link is missing, nothing moves.

---

## 4. What we have done so far (Phase 1 — fixing the starter code)

We read every file in the three starter packages, compared them with how ROS 2 Humble and
Gazebo actually work, and fixed the problems. Highlights (full table in `BUGS_FIXED.md`):

1. **The build was broken.** One missing `()` in `testbed_description/CMakeLists.txt`
   (`ament_package` → `ament_package()`) made the whole workspace fail to build.
2. **The map couldn't load**, for two reasons: the YAML pointed to a non-existent image
   (`wrong_path_testbed_world.pgm`), and the `maps` folder was never installed.
3. **Timing mismatch.** `robot_state_publisher` didn't use simulation time, while everything else
   did. In simulation that causes TF timestamp errors that break AMCL and Nav2.
4. **The lidar was nearly blind.** Its maximum range was 1.5 m in a 20 m × 20 m world. AMCL
   localizes by matching walls in the scan to walls on the map; with 1.5 m it sees almost no walls.
   We raised it to 12 m.
5. **Leftovers from ROS 1.** The ROS 1 `gazebo_ros_control` plugin (doesn't exist in ROS 2) was
   removed, plus an XML typo in the IMU plugin.
6. **Missing dependencies** in every `package.xml`, so the project would fail on a fresh machine.

7. **The robot turned too little.** The drive plugin assumed the wheels were 0.35 m apart. Measured
   from the robot's 3D meshes they are 0.376 m apart, so every turn came out about 9% short
   (78° instead of 86°). Fixed to 0.376 m; the same turn now gives 84°.

**How we tested it (Phase 1 run, 14 Oct 2026):** we launched the full simulation and checked:

| Check | Result |
|---|---|
| Build, URDF parse, dependencies | All packages build; `check_urdf` OK; `rosdep` all satisfied |
| Robot spawns | `Successfully spawned entity [testbed]`, no errors in the launch log |
| Lidar | 298 of 300 rays hit walls (3.4–11.4 m). With the old 1.5 m range it would have been 0 |
| TF chain | `odom → base_footprint → base_link → lidar_link_1` resolves, using sim time |
| Drive forward/back 1 m | Moved exactly 1.00 m both ways: no wheel slip |
| Turn 86° | 78° before the fix, 84° after |

We suspected the wheel friction too, but the drive test showed it's fine, so we left it unchanged.
One thing to remember for later: when the robot spins in place, `base_footprint` moves about 8 cm,
because it isn't exactly at the centre of the wheel axle. That's why Nav2 will get a footprint polygon.

---

## 5. What comes next — the `testbed_navigation` package

We will build navigation in three layers and test each one before adding the next:

| Layer | Launch file | Nav2 component(s) | Config | How we know it works |
|---|---|---|---|---|
| 1. Map | `map_loader.launch.py` | `map_server` | (map YAML from `testbed_bringup`) | Map appears in RViz on `/map` |
| 2. Localization | `localization.launch.py` | `amcl` | `config/amcl_params.yaml` | Scan lines up with walls; particle cloud tightens |
| 3. Navigation | `navigation.launch.py` | `planner_server`, `controller_server`, `bt_navigator`, `behavior_server` (+ optional `velocity_smoother`, `collision_monitor`) | `config/nav2_params.yaml` | Robot reaches a "Nav2 Goal" clicked in RViz |

**Important concept: lifecycle nodes.** Every Nav2 server starts in an "unconfigured" state and does
nothing until it's told to *configure* and then *activate*. `nav2_bringup` normally does this for
you. Because we aren't using it, each of our launch files starts a `nav2_lifecycle_manager` with
`autostart: true` and a list of the nodes it manages.

**Plugin choices (kept simple, as the brief asks for basic navigation):**
- Global planner: **NavFn** (Dijkstra/A* on the costmap)
- Local controller: **Regulated Pure Pursuit** (follows the path smoothly; easy to tune)
- Localization: **AMCL** with the differential-drive motion model
- Robot footprint: a **polygon** built from the measured mesh size. The robot's body is off-centre
  relative to `base_footprint`, so a simple circle would be inaccurate.

---

## 6. How to run it (current state)

```bash
cd ~/assignment_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch testbed_bringup testbed_full_bringup.launch.py
```

Tip from `help.md`: if topics drop or navigation times out, switch to Cyclone DDS:
`sudo apt install ros-humble-rmw-cyclonedds-cpp && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

---

## 7. Glossary

| Term | Meaning |
|---|---|
| **URDF / xacro** | XML description of the robot's links (parts) and joints; xacro adds macros/includes |
| **TF** | ROS's system for tracking where every coordinate frame is relative to the others |
| **Odometry** | Estimating motion from wheel rotation. Smooth but drifts |
| **AMCL** | Adaptive Monte Carlo Localization: a particle filter that matches the lidar scan to the map |
| **Costmap** | A grid that marks where the robot can and can't go (walls, obstacles + safety padding) |
| **Behaviour tree (BT)** | The "decision logic" of Nav2: plan → follow → if stuck, recover → replan |
| **Lifecycle node** | A node that must be explicitly configured and activated before it runs |
| **use_sim_time** | Makes a node use Gazebo's clock instead of the computer's clock |

# Project Explanation — ROS 2 Navigation Assignment (Testbed-T1.0.0)

A plain-language walkthrough of what was done in this repository and why. The technical
details are in [`testbed_navigation/README.md`](testbed_navigation/README.md), and every
bug and issue is in [`BUGS_FIXED.md`](BUGS_FIXED.md).

_Last updated: 14 Oct 2026. All phases are done, including the screen recordings; only
opening the pull request is left._

## 1. The goal

Make a small two-wheeled robot in a Gazebo simulation navigate on its own: load a map,
work out where it is, plan a path to a goal and drive there without hitting anything.
The assignment doesn't allow Nav2's ready-made `nav2_bringup` launch files, so every Nav2
component is started by hand from our own `testbed_navigation` package. The starter code
also contained deliberate bugs to find and fix.

**Result:** the robot reaches 5 of 5 goals on a 57.4 m tour of the whole building, with no
recoveries, each within 16.4 cm of its target.

## 2. What is in the repository

| Package | What it holds |
|---|---|
| `testbed_description` | The robot: URDF/xacro, meshes, Gazebo plugins (wheels, lidar, IMU) |
| `testbed_gazebo` | The simulated building and the launch files that start Gazebo and spawn the robot |
| `testbed_bringup` | One launch file for the whole simulation, plus the map |
| `testbed_navigation` | **Ours:** map loading, localization, navigation, RViz layout, test tools, result images |

None of the packages contain compiled code. Building them copies their folders into
`install/`, which is why a folder missing from a `CMakeLists.txt` install rule doesn't
exist at runtime (bugs 3 and 4).

## 3. How it works

```
RViz "Nav2 Goal" ─► bt_navigator ─► planner_server (NavFn path on the global costmap)
                         │       └─► controller_server (Pure Pursuit, local costmap)
                         │                 └─► velocity_smoother ─► /cmd_vel ─► Gazebo wheels
                         └─► behavior_server (spin / wait / back up when stuck)

Gazebo lidar ─► /scan ─► amcl ◄─ /map ◄─ map_server      amcl publishes map → odom
```

Frames: `map → odom` (AMCL) `→ base_footprint` (Gazebo diff drive) `→ base_link → lidar_link_1`.

Every Nav2 server is a *lifecycle node*: it does nothing until it is configured and
activated. Each of our launch files runs a lifecycle manager that does this.

## 4. What we did, phase by phase

1. **Fixed the starter code: 13 bugs.** Ten were visible in the code: the build was broken,
   the map path was wrong, folders weren't installed, the lidar saw only 1.5 m, and more.
   Three only appeared once the robot moved:
   - the wheel spacing was wrong (turns came out 9% short);
   - `base_footprint` sat 6 cm off the wheel axle, so every spin looked to AMCL like sliding
     sideways (error up to 0.51 m);
   - the brick walls had two positions in the world file, and Gazebo's Reset World moved
     them 1.25 m away from the map.
2. **Created `testbed_navigation`** inside the repo, with `launch/`, `config/`, `rviz/`,
   `scripts/` and `media/`.
3. **Map loading** (`map_loader.launch.py`): `map_server` publishes the 405 × 400 map.
4. **Localization** (`localization.launch.py`, `amcl_params.yaml`): AMCL's error is 2–7 cm
   while driving and spinning.
5. **Navigation** (`navigation.launch.py`, `nav2_params.yaml`): NavFn planner, Regulated Pure
   Pursuit controller, the default behaviour tree, spin/back-up/wait recoveries and a velocity
   smoother. The robot uses its real outline (0.35 m × 0.41 m) as the footprint.
6. **Tuning:** switching NavFn from A* to Dijkstra removed every planner failure; a
   one-command launch file and an RViz layout were added.

## 5. How we tested

In this simulation, `/odom` is the robot's true position and the map frame equals the world
frame. So we could measure exactly how far the robot moved compared with what was commanded,
and exactly how wrong AMCL was. The test tools in `testbed_navigation/scripts/` do this and
are part of the submission.

## 6. Media

`testbed_navigation/media/` holds two screen recordings of the same run (RViz and an overhead
Gazebo view, 71 s each), side-by-side and full-window screenshots, the route plot of the full
tour, and before/after images of the wall bug.

## 7. Still open

- **Upgrade one system library** (`sudo apt install --only-upgrade ros-humble-diagnostic-updater`)
  on this machine. The code is verified: with the fixed library loaded for a test run, the
  one-command launch started every node by itself and the tour passed 5 of 5.
- **Open the pull request** from the `testbed-navigation` branch to `main`.

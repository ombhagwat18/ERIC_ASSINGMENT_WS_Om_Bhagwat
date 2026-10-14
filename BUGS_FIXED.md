# Bugs Found and Fixed — Testbed-T1.0.0 Starter Code

This file lists every bug I found in the starter code and how I fixed it. Part B logs the
issues I hit while building the `testbed_navigation` package, and how I solved each one.

**Environment:** Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11.10.2

**Status legend:** ✅ Fixed and verified  ·  🔧 Fixed, still to be checked in simulation  ·  🔍 Open / under investigation

**How bugs were verified:** a build check (`colcon build`), a URDF parse (`xacro` + `check_urdf`), dependency checks (`rosdep check`), and a simulation drive test: a script sends fixed `cmd_vel` commands and compares them with the robot's actual motion on `/odom` (the diff-drive plugin reports the true world pose by default).

---

## Part A — Bugs in the starter code

| # | Package / File | Problem (what we saw) | Root cause | Fix | How it was found / verified | Status |
|---|---|---|---|---|---|---|
| 1 | `testbed_description/CMakeLists.txt` | `colcon build` fails: `CMake Error at CMakeLists.txt:24: Parse error. Expected "(", got newline`. `testbed_gazebo` is then skipped because it depends on it. | Last line was `ament_package` with no parentheses, so it is not a valid CMake command call. | Changed to `ament_package()`. | Build log `log/latest_build/testbed_description/stderr.log`. After the fix, all packages build. | ✅ |
| 2 | `testbed_bringup/maps/testbed_world.yaml` | The map server cannot load the map image. | `image: wrong_path_testbed_world.pgm` points to a file that doesn't exist. The real file is `testbed_world.pgm`. | `image: testbed_world.pgm` (relative to the YAML file, so it works from any install location). | Listed `maps/`; only `testbed_world.pgm` exists. | ✅ |
| 3 | `testbed_bringup/CMakeLists.txt` | The map can't be found by package path (`get_package_share_directory('testbed_bringup')/maps/...`). | The `install(DIRECTORY ...)` rule installed only `launch`, so `maps/` was never copied to `install/`. | Added `maps` to the install directories. | After rebuilding, `install/testbed_bringup/share/testbed_bringup/maps/` contains the `.yaml` and `.pgm`. | ✅ |
| 4 | `testbed_gazebo/CMakeLists.txt` | `spawn_playground.launch.py` adds `share/testbed_gazebo/models` to `GAZEBO_MODEL_PATH`, but that folder doesn't exist after install. | `models` was missing from the install rule. | Added `models` to the install directories. | After rebuilding, `install/testbed_gazebo/share/testbed_gazebo/models/` exists. | ✅ |
| 5 | `testbed_description/launch/robot_description.launch.py` | TF from `robot_state_publisher` is stamped with wall-clock time while Gazebo, AMCL and Nav2 use simulation time. This causes TF "extrapolation into the future/past" errors and dropped transforms. | `robot_state_publisher` was started without `use_sim_time`. | Added `"use_sim_time": True` to its parameters. | Code review. In the sim, `tf2_echo odom lidar_link_1` resolves with sim-time stamps (e.g. `At time 188.54`), with no extrapolation errors. | ✅ |
| 6 | `testbed_description/urdf/testbed.gazebo` (lidar) | The lidar only sees 1.5 m. In a room about 20 m × 20 m, most scans hit nothing, so AMCL has almost no walls to match against the map and localization fails or drifts. | `<range><max>1.5</max>` | Set `<max>12.0</max>` (typical for a 2D indoor lidar, and enough to see the walls of this world). | Compared against the map size: 405 × 400 px at 0.05 m/px ≈ 20 m × 20 m. **Sim test:** from the spawn point the nearest wall is 3.39 m away, so a 1.5 m lidar would see **0 of 300** rays. With 12 m, 298 of 300 rays hit walls (up to 11.4 m). | ✅ |
| 7 | `testbed_description/urdf/testbed.gazebo` | Gazebo tries to load a plugin that doesn't exist in ROS 2 and logs an error at spawn. | `libgazebo_ros_control.so` is the ROS 1 (`gazebo_ros_control`) plugin. Humble doesn't ship it. Motion already comes from `libgazebo_ros_diff_drive.so`. | Removed the `<plugin name="control" filename="libgazebo_ros_control.so"/>` block. | Not present in `/opt/ros/humble/lib`. | ✅ |
| 8 | `testbed_description/urdf/testbed.gazebo` (IMU) | Malformed XML: an extra `>` after `</initial_orientation_as_reference>` leaves stray text inside the plugin element. | Typo. | Removed the extra `>`. | Code review. `xacro` + `check_urdf` parse cleanly. | ✅ |
| 9 | `package.xml` (all three packages) | Packages don't declare what they need at runtime, so `rosdep install` on a fresh machine won't install `gazebo_ros`, `xacro`, `robot_state_publisher`, etc., and the launch files fail. | `exec_depend` entries were missing. `testbed_bringup` also calls `find_package(rclpy)` without declaring it. | Added `exec_depend`s: **description** → `xacro`, `robot_state_publisher`, `joint_state_publisher`, `rviz2`. **gazebo** → `gazebo_ros`, `gazebo_plugins`. **bringup** → `rclpy` (depend), `testbed_description`, `testbed_gazebo`, `rviz2`. | `rosdep check --from-paths src --ignore-src` → "All system dependencies have been satisfied". | ✅ |
| 10 | `.vscode/settings.json` | The editor's CMake extension points to a folder on another machine. | Hard-coded `cmake.sourceDirectory: /home/eric-ros1/level01_ros_assignment/testbed_bringup`. | Removed the machine-specific path. | Code review (doesn't affect the robot). | ✅ |
| 11 | `testbed_description/urdf/testbed.gazebo` (`wheel_separation`) | The robot turns less than commanded: a commanded 86° turn (0.5 rad/s × 3 s) gives only **78°**. | The plugin uses `wheel_separation = 0.35`. Measured from the wheel meshes, the wheel centres are at y = +0.150 m and y = −0.226 m, which is **0.376 m** apart. | `wheel_separation = 0.376`. (`wheel_diameter = 0.1` was checked and is correct.) | Mesh bounding boxes computed from the STL files. **Drive test after the fix:** the same command gives **84°** of 86°; the small remainder is the wheels speeding up and slowing down. | ✅ |

### Checked and found OK (not bugs)
- **Wheel/caster friction** (all μ = 0.2) looked suspicious, because the four casters are fixed links that slide rather than roll. The drive test showed no slip, though: commanding 0.2 m/s for 5 s moved the robot exactly 1.00 m forwards and 1.00 m backwards. Left unchanged.
- **Rotation centre:** turning in place shifts `base_footprint` by about 8 cm, because `base_footprint` is ~6 cm away from the wheel-axle midpoint. This isn't a bug, but Nav2 must use a footprint polygon (not a circle centred on `base_footprint`).
- `wheel_diameter = 0.1`: the wheel mesh spans z = 0.0 → 0.1 m.
- The lidar sits above the body (lidar z ≈ 0.228 m, top of body ≈ 0.2225 m), so it doesn't see the robot itself.
- The map frame matches the Gazebo world frame: the map covers x −10.2 → 10.05 m and y −9.94 → 10.06 m. The robot spawns at **(0, 5, yaw 0)**, which becomes AMCL's initial pose.
- `testbed.trans` uses ROS 1 transmissions. It isn't used in ROS 2 but does no harm, so I left it unchanged.

---

## Part B — Issues hit while building `testbed_navigation`

_New issues are added here as they come up during development._

| # | Phase | Problem (what we saw) | Root cause | How we solved it | Status |
|---|---|---|---|---|---|
| B1 | Setup | `colcon build` builds a `testbed_navigation` package that isn't in the git repo. | The package was created with `ros2 pkg create` from `~/assignment_ws` (the workspace root) instead of inside the repo, so it would never be part of the pull request. | Recreate it inside the repo: `src/ERIC_ASSINGMENT_WS_Om_Bhagwat/testbed_navigation/`, then delete the stray copy. | 🔍 |
| B2 | Setup | Editing `testbed.gazebo` with a script turned a 7-line change into a 316-line diff. | The file uses Windows (CRLF) line endings and the script rewrote them as LF. | Restored the CRLF endings, so the diff shows only the real changes. Watch for this in files that use CRLF (`git ls-files --eol`). | ✅ |

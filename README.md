# RB2301 Robotics Projects

ROS 2 (Jazzy, Gazebo Harmonic) workspace for the RB2301 assignments, run on the Bingda robot in the lab and in simulation. The ROS 2 workspace is the `rb2301/` folder: every path below (`src/...`, the `.sh` scripts) is relative to it. Each project has its own package under `rb2301/src/` and its own detailed write-up here.

| Project | Package | What the robot does | Details |
| --- | --- | --- | --- |
| CA1: Obstacle avoidance | `src/rb2301_ca1` | Drives out of a random field of cans to a finish line using only the LiDAR. Moves sideways and forwards without turning. Two controllers (`nhien`, `max`) can be swapped by a parameter. | [CA1.md](CA1.md) |
| CA2: Path planning | `src/rb2301_ca2` | Visits a list of goals in a known maze. Plans with RRT* on the occupancy grid and follows the path with a carrot point and a heading PID, driving like a car (forward, reverse and turning only). | [CA2.md](CA2.md) |
| FP: Obstacle course | `src/rb2301_fp` | Crosses the final-project course toward north-east: avoids obstacles with a front corridor and side cones, finds and drives through the gate, then crosses two swinging boards. | [FP.md](FP.md) |

Shared packages:

- `src/rb2301_gz`: Gazebo worlds, robot model and launch files for all three projects.
- `src/rb2301_tutorial`, `src/rb2301_p1`: course tutorial and practical packages.

## Build

```bash
cd rb2301
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Run the scripts in `rb2301/` (`ca1.sh`, `ca2.sh`, `gz_ca2.sh` and so on) from that folder, after the build. [HOW_TO_RUN.md](HOW_TO_RUN.md) shows how to run every project in simulation, including pocket_test for FP. Each project's page covers the real robot too.

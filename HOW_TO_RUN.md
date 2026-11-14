# How to Run (Simulation)

This page covers running each project in Gazebo, and the FP controller in pocket_test, our 2-D simulator. For the real robot, see each project's own page ([CA1](CA1.md), [CA2](CA2.md), [FP](FP.md)).

## 1. One-time setup

You need Ubuntu with ROS 2 Jazzy, Gazebo Harmonic and `ros-jazzy-ros-gz`, plus the Python packages `numpy`, `pillow` and `matplotlib` (matplotlib is only for pocket_test).

Build the workspace. It is the `rb2301/` folder of this repo:

```bash
cd rb2301
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
chmod +x *.sh src/rb2301_fp/*.sh
```

With `--symlink-install`, edits to the Python files take effect straight away. Rebuild only after adding or removing files.

All commands below are run from `rb2301/`, each in its own terminal. Start Gazebo first, and launch the controller only once the robot is in the world. `Ctrl+C` stops either one; the Gazebo scripts also kill leftover Gazebo processes on exit (`./kill.sh` does it by hand).

## 2. CA1: Obstacle avoidance

```bash
./gz_ca1.sh        # terminal 1: Gazebo, a new random can field every launch
./ca1.sh           # terminal 2: the controller (nhien by default)
```

To run the other controller in terminal 2:

```bash
source install/setup.bash
ros2 run rb2301_ca1 obstacle_avoidance --ros-args -p algo:=max
```

Or start Gazebo, the controller and the grader (time and path length to the finish line) together:

```bash
source install/setup.bash
ros2 launch rb2301_ca1 ca1.launch.py algo:=nhien random_seed:=0
```

`algo` (`nhien` or `max`) and `random_seed` are both required. The same seed gives the same can field, so you can compare the two controllers fairly. Add `scenario:=front_left` (or `front_right`, `diagonal_left`) for a fixed edge case instead of a random field.

## 3. CA2: Path planning

```bash
./gz_ca2.sh        # terminal 1: Gazebo, CA2 maze; the robot starts at (0, 0)
./ca2.sh           # terminal 2: plans with RRT* and drives the goals in order
```

Planning the 2,000-node tree takes a moment ("Progress: n/2000"). The terminal then prints the map with the route (`*`) for each goal and `DONE ALL GOAL` at the end. To start elsewhere, change `x:=` and `y:=` in `gz_ca2.sh`.

## 4. FP: Obstacle course

### Gazebo

```bash
bash src/rb2301_fp/gz_fp.sh     # terminal 1: Gazebo, a newly randomised course every launch
bash src/rb2301_fp/fp.sh        # terminal 2: the controller
```

To start past the gate (for testing the boards), use the commented line in `gz_fp.sh` (`x:=3.8 y:=-2.6`). Twice a second the controller logs the nearest obstacle, the turn, the gate phase, the goal heading and the board phase.

### pocket_test (2-D simulator, no ROS needed)

pocket_test runs the exact same controller (`avoidance_command` in `obstacle_course.py`) against a fake LiDAR in 2-D worlds, and animates the robot, its front corridor, its side cones and the scan points. It is the quickest way to try a change.

```bash
cd src/rb2301_fp
python3 pocket_test.py                  # every scenario, one after another
python3 pocket_test.py zigzag           # one scenario
python3 pocket_test.py random           # a new randomised final-project course
python3 pocket_test.py random --seed 7  # the same random course every time
python3 pocket_test.py random --swing   # boards swing like hand-pushed pendulums (decaying, bent)
python3 pocket_test.py random --seed 0 --trials 20 --headless   # success rate over 20 courses, no plots
```

| Scenario | What it tests |
| --- | --- |
| `box_right`, `box_left` | A box on one side of the corridor; should turn away from it |
| `perpendicular`, `slash`, `backslash` | A wall across the path, square or angled |
| `pocket`, `pocket_deep` | Reverse-C pockets (dead ends) |
| `course` | A mixed course heading north-east |
| `zigzag` | A narrow zigzag corridor |
| `fp_course` | The course from the last Gazebo `gz_fp.sh` launch |
| `random` | A randomised final-project course (gate, cylinders, boards) |

| Option | Effect |
| --- | --- |
| `--seed N` | Fix the random course (with `random`) |
| `--trials N` | Run N random courses from the seed |
| `--swing` | Hand-swung, decaying boards instead of motorised ones |
| `--real` | Real-robot speed caps and a 720-beam scan |
| `--no-anim` | Skip the animation, only draw the final paths |
| `--headless` | Print results only, no windows |

**Animation controls.** A slider and Play/Pause, Rewind, Stop, step and speed buttons. Keys: `space` play/pause, `←`/`→` step, `r` rewind, `s` stop. Close the window to move on to the next scenario.

**Result line.** Each run prints `FINISHED in Ns`, `COLLISION`, `STUCK` or `ok`, with the distance travelled and the first turn.

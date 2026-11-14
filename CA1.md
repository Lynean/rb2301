# CA1: Obstacle Avoidance

Paths on this page are relative to the ROS workspace, the `rb2301/` folder of this repo.

The robot starts at one edge of a random field of cans (Coke cans in Gazebo) and has to reach the finish line on the far side without touching any. It uses only the LiDAR scan: no pose and no map. The robot can move sideways, so both controllers keep the heading fixed and steer by choosing a direction of travel (x forward, y sideways). Neither of them turns.

- Package: `src/rb2301_ca1`
- Controller: `rb2301_ca1/obstacle_avoidance.py`, node `obstacle_avoidance`
- Speed limit: 0.4 m/s (`max_translate_velocity`)

## Files

| File | Purpose |
| --- | --- |
| `rb2301_ca1/obstacle_avoidance.py` | The node. Holds both controllers; the ROS parameter `algo` picks one each control step (20 Hz). |
| `rb2301_ca1/obstacle_generator.py` | Generates the can layout (random with a seed, or a fixed edge case) and writes the Gazebo world. |
| `rb2301_ca1/algo_grader.py` | Measures a run from the first odometry message: time and path length to the finish line, just past the last row of cans. |
| `launch/ca1.launch.py` | Starts Gazebo, the controller and the grader together. |

## Running

All commands run from the workspace root after `colcon build --symlink-install`.

Full evaluation run (Gazebo, controller and grader):

```bash
source install/setup.bash
ros2 launch rb2301_ca1 ca1.launch.py algo:=nhien random_seed:=7
```

| Argument | Values | Meaning |
| --- | --- | --- |
| `algo` | `nhien`, `max` | Which controller to run |
| `scenario` | `random` (default), `front_left`, `front_right`, `diagonal_left` | Random field, or a fixed edge case around the spawn point |
| `random_seed` | integer | Seed for the random field (required here). Use the same seed to compare the two controllers on the same layout. |

`ca1_gazebo.launch.py` (used by both this launch and `gz_ca1.sh`) writes the layout into `obstacles_world_ca1.sdf` just before Gazebo loads it. On its own it takes the same `scenario` and `random_seed` arguments; with no seed it makes a new random layout every launch.

Separately, with Gazebo in one terminal and the controller in another:

```bash
./gz_ca1.sh      # terminal 1: Gazebo with the CA1 world
./ca1.sh         # terminal 2: obstacle_avoidance (algo defaults to nhien)
```

To pick the controller here: `ros2 run rb2301_ca1 obstacle_avoidance --ros-args -p algo:=max`.

## Controller 1: `nhien` (gap scoring over the whole scan)

This controller looks all the way round the robot, finds the best open gap and drives straight toward its centre. The scan is downsampled to 360 readings (one per degree), and no return is clipped to 20 m.

1. **Open boundary.** Outside the can field the LiDAR sees nothing. Any sector of 90° or more with no return is given a pretend distance that shrinks the wider the sector is (20 m at 90°, a quarter of that for every extra 30°). A huge empty sector off to the side, outside the field, then scores lower than a real gap ahead. Left and right halves are processed separately so one sector never wraps across the back.
2. **Masks.** Every fifth reading over the back is set to 0.1 m, which blocks backward gaps. When something comes within 0.15 m on one side, that side is masked for a few steps (a "blind" timer), so the robot does not try to slide back into it.
3. **Gap search.** The scan is doubled end to end so a gap can wrap around 0°. A gap is a run of at least 20° where every reading is at least 0.5 m. Its score is (width in degrees + mean distance) / 2, counted only if its mean distance is at least 2 m. The best gap's centre becomes the travel direction.
4. **Motion.** The robot moves at full speed along that direction, split into forward and sideways velocity.
5. **Emergency rules.** These override the gap direction:
    - Something within 0.15 m on the left: slide right. On the right: slide left.
    - Something within 0.15 m in front: no forward motion.

## Controller 2: `max` (front gaps between cans)

This controller looks only ahead and threads between neighbouring cans. It uses the front half of the scan, up to 0.75 m away.

1. **Clear check.** The robot drives straight at 0.3 m/s until something is inside a strip 0.24 m wide and 0.3 m deep in front of it.
2. **Cans.** Points are grouped into cans wherever neighbouring readings are within 0.12 m of each other.
3. **Gap targets.** For each pair of neighbouring cans, the midpoint between their facing edges is a target if:
    - the sideways opening is at least the robot width plus a margin on each side (0.33 m);
    - it is within 85° of straight ahead;
    - the strip the body sweeps on the way there is free of points.
4. **Choosing a target.** A gap almost straight ahead wins. Otherwise it takes the gap closest to straight ahead on the preferred side (right first). The robot moves toward it at 0.18 m/s.
5. **No gap: sidestep.** It slides at 0.25 m/s toward the side with more room (median range), and holds that side unless the other becomes clearly better. If that side is blocked close by, it swaps sides; if both are, it backs up. It also backs up if a can is within 0.20 m in front.
6. **Leaving avoidance.** After the front clears, it keeps the last avoiding command for 8 readings before going straight again.

## Comparing the two

Run both controllers with `ca1.launch.py` on the same `random_seed` and compare the grader's output: time and path length to the finish line.

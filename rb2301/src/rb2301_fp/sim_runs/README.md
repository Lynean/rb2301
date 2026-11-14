# pocket_test: 20 randomised runs

Randomised final-project courses (same randomisation as `obstacle_modifier.py`) with hand-swung boards, run by the real controller in `pocket_test.py`. Time limit 240 s per run.

![All runs](all_runs.png)

*Final frame of every run: green frame = COMPLETE, orange = TIMEOUT, red = COLLISION.*

| Result | Runs |
| --- | --- |
| COMPLETE | 15 |
| TIMEOUT | 5 |
| COLLISION | 0 |

Completed runs took 40–66 s (mean 51 s).

Rejected (robot spawned on top of an obstacle): seeds 10, 12, 15, 20.

| Run | Seed | Result | Time (s) | Distance (m) | Boards reached at (s) | Animation |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0 | **TIMEOUT** | 239.9 | 0.9 | - | [run_01_seed_0_TIMEOUT.gif](run_01_seed_0_TIMEOUT.gif) |
| 2 | 1 | **TIMEOUT** | 239.9 | 45.0 | - | [run_02_seed_1_TIMEOUT.gif](run_02_seed_1_TIMEOUT.gif) |
| 3 | 2 | **COMPLETE** | 42.7 | 7.6 | 24.4 | [run_03_seed_2_COMPLETE.gif](run_03_seed_2_COMPLETE.gif) |
| 4 | 3 | **TIMEOUT** | 239.9 | 38.3 | - | [run_04_seed_3_TIMEOUT.gif](run_04_seed_3_TIMEOUT.gif) |
| 5 | 4 | **COMPLETE** | 55.9 | 8.2 | 37.8 | [run_05_seed_4_COMPLETE.gif](run_05_seed_4_COMPLETE.gif) |
| 6 | 5 | **COMPLETE** | 64.5 | 7.8 | 45.9 | [run_06_seed_5_COMPLETE.gif](run_06_seed_5_COMPLETE.gif) |
| 7 | 6 | **COMPLETE** | 50.6 | 7.0 | 29.8 | [run_07_seed_6_COMPLETE.gif](run_07_seed_6_COMPLETE.gif) |
| 8 | 7 | **COMPLETE** | 57.4 | 8.1 | 39.7 | [run_08_seed_7_COMPLETE.gif](run_08_seed_7_COMPLETE.gif) |
| 9 | 8 | **TIMEOUT** | 239.9 | 17.9 | - | [run_09_seed_8_TIMEOUT.gif](run_09_seed_8_TIMEOUT.gif) |
| 10 | 9 | **COMPLETE** | 39.8 | 6.7 | 20.3 | [run_10_seed_9_COMPLETE.gif](run_10_seed_9_COMPLETE.gif) |
| 11 | 11 | **COMPLETE** | 46.5 | 8.2 | 27.2 | [run_11_seed_11_COMPLETE.gif](run_11_seed_11_COMPLETE.gif) |
| 12 | 13 | **COMPLETE** | 44.0 | 6.6 | 26.4 | [run_12_seed_13_COMPLETE.gif](run_12_seed_13_COMPLETE.gif) |
| 13 | 14 | **COMPLETE** | 47.7 | 7.1 | 28.3 | [run_13_seed_14_COMPLETE.gif](run_13_seed_14_COMPLETE.gif) |
| 14 | 16 | **COMPLETE** | 42.7 | 7.3 | 22.8 | [run_14_seed_16_COMPLETE.gif](run_14_seed_16_COMPLETE.gif) |
| 15 | 17 | **TIMEOUT** | 239.9 | 22.7 | - | [run_15_seed_17_TIMEOUT.gif](run_15_seed_17_TIMEOUT.gif) |
| 16 | 18 | **COMPLETE** | 58.8 | 7.2 | 38.9 | [run_16_seed_18_COMPLETE.gif](run_16_seed_18_COMPLETE.gif) |
| 17 | 19 | **COMPLETE** | 60.8 | 10.0 | 40.1 | [run_17_seed_19_COMPLETE.gif](run_17_seed_19_COMPLETE.gif) |
| 18 | 21 | **COMPLETE** | 66.4 | 9.0 | 46.8 | [run_18_seed_21_COMPLETE.gif](run_18_seed_21_COMPLETE.gif) |
| 19 | 22 | **COMPLETE** | 45.2 | 7.4 | 25.5 | [run_19_seed_22_COMPLETE.gif](run_19_seed_22_COMPLETE.gif) |
| 20 | 23 | **COMPLETE** | 42.7 | 7.3 | 21.7 | [run_20_seed_23_COMPLETE.gif](run_20_seed_23_COMPLETE.gif) |

Re-run: `python3 sim_runs/make_sim_runs.py` from `rb2301/src/rb2301_fp` (see the script for options).

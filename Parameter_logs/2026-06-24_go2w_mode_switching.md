# Go2W Simulation/Real Mode Switching Notes

## Goal

Add additive mode switching for `go2w` between:

- `robot_lab_line_run`
- `robot_lab_handstand`

while preserving the existing startup behavior based on `config_name`, and without breaking other models or other robot FSMs.

## Final Behavior

### Startup behavior

The old startup path is preserved:

- `Num1` / `RB_DPadUp`
  - still enters the original `RLFSMStateRLLocomotion`
  - this state still honors `requested_config_name`
  - if startup `config_name` is a known go2w mode alias, it now forwards to an explicit mode state

This means:

- launching with `robot_lab_line_run` and then entering locomotion still starts line-run
- launching with `robot_lab_handstand` and then entering locomotion still starts handstand
- unknown/custom go2w configs can still be loaded by the old generic locomotion state

### New explicit runtime switching

Added explicit mode-switch keys:

- `Num2` / `RB_DPadRight`
  - switch to `robot_lab_handstand`
- `Num3` / `RB_DPadLeft`
  - switch to `robot_lab_line_run`

This works:

- from `GetUp`
- from the generic `RLFSMStateRLLocomotion`
- while already running in `line_run`
- while already running in `handstand`

## Modified Files

### 1. `src/rl_sar/fsm_robot/fsm_go2w.hpp`

Core change location for mode routing.

Added:

- config aliases/constants
  - `kGo2WHandstandConfig`
  - `kGo2WLineRunConfig`
- mode-state resolver
  - `ResolveGo2WModeStateNameIfKnown(...)`
- switch-key helpers
  - `IsGo2WHandstandSwitch(...)`
  - `IsGo2WLineRunSwitch(...)`
- shared policy load helper
  - `LoadGo2WPolicy(...)`
- shared run helper
  - `RunGo2WPolicy(...)`
- shared transition helper
  - `CheckGo2WPolicyTransition(...)`

Added new explicit FSM states:

- `RLFSMStateRLLineRun`
- `RLFSMStateRLHandstand`

Updated old state:

- `RLFSMStateRLLocomotion`
  - now acts as a compatibility bridge for known go2w mode aliases
  - if `requested_config_name` resolves to line-run or handstand, it requests a switch to the explicit mode state
  - if `requested_config_name` is unknown/custom, it keeps the old generic loading behavior

Updated `GetUp`:

- preserved old `Num1` / `RB_DPadUp` behavior
- added direct entry to the explicit mode states through the new keys

Updated factory registration:

- registered `RLFSMStateRLLineRun`
- registered `RLFSMStateRLHandstand`

## 2. `src/rl_sar/library/core/rl_sdk/rl_sdk.hpp`

Added a new additive hook to `RL`:

- `virtual void OnPolicyActivated(const std::string &config_name)`

Purpose:

- avoid hard-coding simulation/real special logic inside generic `InitRL()`
- allow each runtime backend to reset mode-dependent state when a policy becomes active

Default implementation is a no-op, so other robots/models are unaffected.

## 3. `src/rl_sar/include/rl_sim_mujoco.hpp`
## 4. `src/rl_sar/src/rl_sim_mujoco.cpp`

Added simulation-side activation handling:

- override `OnPolicyActivated(...)`

What it does:

- resets `root_origin_initialized`
- clears `obs.root_pos_rel_xy_b`

Why:

- handstand policy uses `root_pos_rel_xy_b`
- after switching into handstand, the corridor/drift reference should start from the switch moment, not from the very first simulation spawn position

Result:

- switching into handstand in MuJoCo now re-centers the relative drift observation

## 5. `src/rl_sar/include/rl_real_go2.hpp`
## 6. `src/rl_sar/src/rl_real_go2.cpp`

Added a minimal real-robot observation estimator for handstand deployment.

### Added members

- `estimated_root_pos_w`
- `estimated_root_origin_w`
- `estimated_root_origin_initialized`

### Added functions

- `OnPolicyActivated(...)`
- `UpdateEstimatedBaseMotion()`

### Real-side observation estimation logic

Inside `RunModel()`:

- keep original observation updates
- after `base_quat` is updated, call `UpdateEstimatedBaseMotion()`

`UpdateEstimatedBaseMotion()` computes:

1. `obs.lin_vel`

- estimate body-frame forward velocity from wheel angular velocity
- formula:
  - `avg(selected wheel dq) * real_odom_wheel_radius * real_odom_velocity_sign`

2. integrated root position in world frame

- rotate estimated body velocity into world frame
- integrate with:
  - `dt * decimation`

3. `obs.root_pos_rel_xy_b`

- subtract the mode-switch origin
- rotate world drift back into body frame
- store body-frame XY

### Why this was needed

`robot_lab_handstand` is a 62-dim model and needs:

- `lin_vel`
- `root_pos_rel_xy_b`

Before this change, real `go2w` only filled:

- `ang_vel`
- `commands`
- `base_quat`
- `dof_pos`
- `dof_vel`

So the handstand model could be loaded, but not fed with the full observation structure.

### Important limitation

This is a minimal deployment estimator, not a full odometry/state-estimation stack.

Current real estimate quality depends on:

- wheel radius correctness
- wheel velocity sign correctness
- which wheel indices are used
- slip/drift

If later you want stronger real stability, the best upgrade path is:

- replace this wheel-based estimate with a real odometry/state estimator
- or subscribe to a higher-level state source that exposes base position/velocity directly

## 7. `policy/go2w/robot_lab_handstand/config.yaml`

Added deployment-only tuning parameters:

- `real_odom_wheel_indices: [14, 15]`
- `real_odom_wheel_radius: 0.086`
- `real_odom_velocity_sign: 1.0`

### Why `[14, 15]`

From the training task config:

- `robot_lab/source/robot_lab/robot_lab/tasks/locomotion/velocity/config/wheeled/unitree_go2w_handstand/rough_env_cfg.py`
- `handstand_type = "front"`

In this setup:

- front wheels are the air wheels
- rear wheels are the support wheels

So for real deployment, rear wheels are the better default source for handstand drift estimation.

Index mapping in current `go2w` order:

- `12`: `FR_foot_joint`
- `13`: `FL_foot_joint`
- `14`: `RR_foot_joint`
- `15`: `RL_foot_joint`

## 8. `src/rl_sar/test/test_go2w_handstand_config.cpp`

Extended regression coverage for:

- mode alias resolution
- explicit mode-state resolution
- explicit handstand/line-run switch key helpers
- independent loading of the 57-dim and 62-dim go2w configs

This protects the main compatibility surface introduced by the new switching logic.

## Verification Performed

### Built

- `test_go2w_handstand_config`
- `rl_real_go2`
- `rl_sim_mujoco`

### Ran

- `./bin/test_go2w_handstand_config`

Observed result:

- config loading passed
- alias assertions passed

## Parameters You Can Continue Tuning

### A. Key mapping

File:

- `src/rl_sar/fsm_robot/fsm_go2w.hpp`

Functions:

- `IsGo2WHandstandSwitch(...)`
- `IsGo2WLineRunSwitch(...)`

If you want different hot-switch keys, change them here.

### B. Real wheel source for odometry

File:

- `policy/go2w/robot_lab_handstand/config.yaml`

Parameters:

- `real_odom_wheel_indices`

Current meaning:

- which wheel joints are used to estimate real handstand forward drift

If later you change the handstand support side:

- front support: use `[12, 13]`
- rear support: use `[14, 15]`
- if you want to average all wheels: use `[12, 13, 14, 15]`

### C. Wheel radius

File:

- `policy/go2w/robot_lab_handstand/config.yaml`

Parameter:

- `real_odom_wheel_radius`

Current value:

- `0.086`

If real estimated drift is consistently too large or too small, this is the first parameter to retune.

### D. Velocity sign

File:

- `policy/go2w/robot_lab_handstand/config.yaml`

Parameter:

- `real_odom_velocity_sign`

Current value:

- `1.0`

If the real robot switches into handstand and the estimated drift direction is mirrored, this is the first sign-flip parameter to check.

### E. Real observation integration logic

File:

- `src/rl_sar/src/rl_real_go2.cpp`

Function:

- `UpdateEstimatedBaseMotion()`

This is the main place to upgrade later if you want:

- IMU + wheel fusion
- contact-aware wheel selection
- direct external odometry
- better XY estimation than the current forward-wheel integration

### F. Simulation drift-reset behavior

File:

- `src/rl_sar/src/rl_sim_mujoco.cpp`

Function:

- `OnPolicyActivated(...)`

If you later want different switch semantics in simulation, this is the place to adjust whether the relative origin is reset on every policy activation or only on handstand activation.

## Recommended Future Improvements

### 1. Add a dedicated transition pose state

Current switching is direct policy-to-policy switching.

Because current `default_dof_pos` is the same, this is acceptable, but if future line-run and handstand postures diverge more, a dedicated transition pose state will be safer.

Suggested file:

- `src/rl_sar/fsm_robot/fsm_go2w.hpp`

### 2. Replace minimal real odometry with a stronger estimator

Best future improvement for real deployment quality.

Suggested replacement point:

- `src/rl_sar/src/rl_real_go2.cpp`
- `UpdateEstimatedBaseMotion()`

### 3. Add a separate test target for go2w FSM mode switching

Current regression coverage is merged into the config test.

If this logic keeps growing, split it into:

- config loading test
- FSM mode routing test

## Summary

This implementation is intentionally additive:

- preserved existing startup/config behavior
- added explicit runtime mode switching
- made simulation handstand switching re-center drift observations
- made real handstand deployment receive the two missing observation channels through a minimal estimator

The most important future tuning points are:

- `real_odom_wheel_indices`
- `real_odom_wheel_radius`
- `real_odom_velocity_sign`
- `UpdateEstimatedBaseMotion()` implementation

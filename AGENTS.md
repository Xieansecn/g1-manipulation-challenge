# G1 Pick & Place Challenge

MuJoCo simulation of a Unitree G1 humanoid robot performing a pick-and-place task: grab a red cylinder from a brown table, place it on a blue table. This is a **Lucky Robots** challenge repo — the goal is to automate the task beyond the provided keyboard teleoperation.

## Quick Start

```bash
pip install mujoco onnxruntime numpy opencv-python
python run.py                 # full viewer + 2 camera windows
python run.py --no-cameras    # viewer only (faster)
```

## Architecture

Single self-contained script (`run.py`, ~700 lines). No build system, tests, or CI.

**Key classes in `run.py`:**

| Class | Role |
|-------|------|
| `ONNXPolicy` | Wraps ONNX Runtime CPU inference |
| `G1Controller` | Keyboard input → walker/reacher policy calls → PD target positions |
| `CameraRenderer` | Offscreen rendering via `mujoco.Renderer` |

**Policies (pre-trained ONNX, single-threaded CPU):**

| Model | Input | Output | Purpose |
|-------|-------|--------|---------|
| `walker.onnx` | 99D obs | 29D action | Bipedal locomotion from velocity commands |
| `right_reacher.onnx` | 36D obs | 7D action | Right-arm reaching to pelvis-frame target |
| `croucher.onnx` | 101D obs | 29D action | Crouching (loaded but not used in current code) |
| `rotator.onnx` | 99D obs | 29D action | Rotation (loaded but not used in current code) |

## Critical Gotchas

- **Timestep must be 0.005s** (200 Hz physics). `run.py` overrides the XML timestep at runtime.
- **Control decimation = 4** (50 Hz control). PD targets update every 4 physics steps.
- **Reacher target is in pelvis frame**, not world frame. The reacher observation includes pelvis-frame palm position/orientation.
- **Walker velocity commands are in pelvis frame** (vx=forward, vy=left, yaw_rate).
- **ONNX external data**: Each `.onnx` file has a corresponding `.onnx.data` file — both are required.
- **PD gains are set twice**: in `g1.xml` actuator definitions (used by MuJoCo's implicit PD) and in `run.py`'s `_compute_pd_gains()` (not directly applied — the XML actuators use position servos). The Python-side gains are loaded but `apply_pd_control()` only writes position targets to `data.ctrl`.
- **Grip**: Hardcoded closed-joint positions for the right hand's 7 finger actuators. Toggled by `,` key. No force feedback or grasp detection.
- **Arm freeze**: When switching from REACH to WALK mode, the right arm holds its last position. When switching back, it resumes from there.
- **Robot spawn**: `data.qpos[0] = -0.6, qpos[2] = 0.76` (behind the brown table at x=0.351). The pelvis `pos` in g1.xml is also `-0.6 0 0.79`.
- **Camera rendering** uses a separate `mujoco.Renderer` instance (not the viewer). Camera windows run at a lower FPS (default 10) via `cv2.imshow`. If `opencv-python` is missing, cameras are silently disabled.

## File Map

| File | Purpose |
|------|---------|
| `run.py` | Entrypoint: simulation loop, keyboard control, policy orchestration |
| `scene.xml` | MuJoCo world: ground, brown table, blue table, red cylinder (freejoint), cameras |
| `g1.xml` | G1 robot model: 29 joint DoF + 14 finger DoF, actuators, IMU sensors, meshes |
| `model_config.json` | Joint names, default positions, action scales, obs mean/std for walker/croucher |
| `walker.onnx` / `.data` | Walking policy |
| `right_reacher.onnx` / `.data` | Right-arm reaching policy |
| `croucher.onnx` / `.data` | Crouching policy (unused) |
| `rotator.onnx` / `.data` | Rotation policy (unused) |
| `assets/` | OBJ/STL mesh files for robot links and hands |

## Robot Summary

- 29 actuated DoF: legs (12), waist (3), arms (14)
- 14 finger DoF: 7 per hand (Dex3-1 3-finger hand: thumb 3 DoF, index/middle 2 DoF each)
- Joint order is fixed in `model_config.json["joint_names"]` (31 entries matching the 29 body joints)

## Simulation Loop Flow

1. `G1Controller.step()` — builds observation, calls walker policy, overlays reacher if active
2. `G1Controller.apply_pd_control(target_pos)` — writes position targets to `data.ctrl` for all 29 joints + finger actuators
3. `mujoco.mj_step()` — physics step (4× per control update, decimation=4)
4. `viewer.sync()` — render main window
5. `CameraRenderer.render()` — render head/wrist cam at lower rate (default 10 FPS)

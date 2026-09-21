# Simulation Robotics

PyBullet physics demos for robot and rigid-body simulation. Scripts live in `robots/` and are meant to be run with the **`robots` conda environment**, which already includes PyBullet, OpenCV, and NumPy.

## Setup

```bash
conda activate robots
cd simulationRobotics
```

## Demos

| Script                                               | Description                                                               | Run                              |
| ---------------------------------------------------- | ------------------------------------------------------------------------- | -------------------------------- |
| [`robots/hello_world.py`](robots/hello_world.py)     | Connect to PyBullet, load a ground plane and R2-D2, print connection info | `python robots/hello_world.py`   |
| [`robots/robot_arm.py`](robots/robot_arm.py)         | Load a Franka Panda arm and randomly move joints 2 and 4                  | `python robots/robot_arm.py`     |
| [`robots/robot_fingers.py`](robots/robot_fingers.py) | Smooth interpolated motion of joint 4; tracks finger link position        | `python robots/robot_fingers.py` |
| [`robots/cube_creating.py`](robots/cube_creating.py) | Create a 0.5 m box from collision/visual shapes (no URDF)                 | `python robots/cube_creating.py` |
| [`robots/cube_sliding.py`](robots/cube_sliding.py)   | Push a cube horizontally with initial velocity and external force         | `python robots/cube_sliding.py`  |
| [`robots/cube_rolling.py`](robots/cube_rolling.py)   | Launch a cube with diagonal velocity so it tumbles across the plane       | `python robots/cube_rolling.py`  |

Press `Ctrl+C` to stop any demo that runs a simulation loop.

## What changed in this commit

- Added three cube demos: **creating**, **sliding**, and **rolling**
- Moved scripts out of `robots/PyBullet/` into `robots/`
- Renamed `sample.py` to `robot_fingers.py` and documented camera/debug-visualizer settings

## Simulation previews

Headless MP4 recordings of each demo are in [`docs/videos/`](docs/videos/). Regenerate them with:

```bash
conda activate robots
python scripts/record_simulations.py
```

Record a single demo:

```bash
python scripts/record_simulations.py --only cube_sliding
```

### hello_world

[docs/videos/hello_world.mp4](docs/videos/hello_world.mp4)

R2-D2 spawned above the ground plane with gravity enabled.

### robot_arm

[docs/videos/robot_arm.mp4](docs/videos/robot_arm.mp4)

Franka Panda joints 2 and 4 receive random position targets.

### robot_fingers

[docs/videos/robot_fingers.mp4](docs/videos/robot_fingers.mp4)

Joint 4 moves smoothly between random targets while the camera stays focused on the arm.

### cube_creating

[docs/videos/cube_creating.mp4](docs/videos/cube_creating.mp4)

A teal 0.5 m cube created programmatically with `createCollisionShape` / `createMultiBody`.

### cube_sliding

[docs/videos/cube_sliding.mp4](docs/videos/cube_sliding.mp4)

Orange cube given forward velocity and a short horizontal push.

### cube_rolling

[docs/videos/cube_rolling.mp4](docs/videos/cube_rolling.mp4)

Cube launched with diagonal velocity and pushed so it tumbles.

## Project layout

```text
simulationRobotics/
├── README.md
├── docs/
│   └── videos/          # MP4 previews
├── robots/
│   ├── hello_world.py
│   ├── robot_arm.py
│   ├── robot_fingers.py
│   ├── cube_creating.py
│   ├── cube_sliding.py
│   └── cube_rolling.py
└── scripts/
    └── record_simulations.py
```

## Notes

- All interactive demos use `p.connect(p.GUI)` and open a PyBullet window.
- `scripts/record_simulations.py` uses `p.connect(p.DIRECT)` for headless capture, so you can batch-generate videos without a display.
- PyBullet data assets (`plane.urdf`, `cube.urdf`, `franka_panda/panda.urdf`, etc.) come from the `pybullet_data` package bundled with PyBullet.

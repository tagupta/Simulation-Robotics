"""Record lightweight GIF previews for each PyBullet demo in robots/."""

from __future__ import annotations

import argparse
from collections.abc import Callable
from pathlib import Path

import cv2
import numpy as np
import pybullet as p  # type: ignore
import pybullet_data
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
URDF_DIR = ROOT / "URDFS"
OUTPUT_DIR = ROOT / "docs" / "previews"

WIDTH = 640
HEIGHT = 480
FPS = 12
GRAVITY = (0, 0, -9.81)
TIME_STEP = 1.0 / 240.0


def connect() -> None:
    p.connect(p.DIRECT)
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(*GRAVITY)
    p.setTimeStep(TIME_STEP)


def camera(
    target: list[float],
    distance: float = 3.0,
    yaw: float = 45.0,
    pitch: float = -35.0,
) -> list[float]:
    return p.computeViewMatrixFromYawPitchRoll(target, distance, yaw, pitch, 0, 2)


def projection() -> list[float]:
    return p.computeProjectionMatrixFOV(60, WIDTH / HEIGHT, 0.1, 100.0)


def capture(view_matrix: list[float]) -> np.ndarray:
    _, _, rgba, _, _ = p.getCameraImage(
        WIDTH,
        HEIGHT,
        view_matrix,
        projection(),
        renderer=p.ER_BULLET_HARDWARE_OPENGL,
    )
    frame = np.array(rgba, dtype=np.uint8).reshape(HEIGHT, WIDTH, 4)
    return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)


def save_gif(frames: list[np.ndarray], output_path: Path) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    pil_frames = [
        Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        for frame in frames
    ]
    duration_ms = int(1000 / FPS)
    pil_frames[0].save(
        output_path,
        save_all=True,
        append_images=pil_frames[1:],
        duration=duration_ms,
        loop=0,
        optimize=True,
        disposal=2,
    )
    return output_path


def render_preview(
    name: str,
    steps: int,
    view_matrix: list[float],
    setup_fn: Callable[[], None],
    step_hook: Callable[[int], None] | None = None,
) -> Path:
    frames: list[np.ndarray] = []
    capture_every = max(1, round((1.0 / FPS) / TIME_STEP))

    connect()
    setup_fn()
    try:
        for step in range(steps):
            if step_hook is not None:
                step_hook(step)
            p.stepSimulation()
            if step % capture_every == 0:
                frames.append(capture(view_matrix))
    finally:
        if p.isConnected():
            p.disconnect()

    return save_gif(frames, OUTPUT_DIR / f"{name}.gif")


def record_hello_world() -> Path:
    def setup() -> None:
        p.setGravity(0, 0, -10)
        p.loadURDF("plane.urdf")
        p.loadURDF("r2d2.urdf", [0, 0, 3], p.getQuaternionFromEuler([0, 0, 0]))

    return render_preview(
        "hello_world",
        steps=360,
        view_matrix=camera([0, 0, 0.5]),
        setup_fn=setup,
    )


def record_robot_arm() -> Path:
    def setup() -> None:
        p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        panda_id = p.loadURDF(
            "franka_panda/panda.urdf",
            [0, 0, 0],
            [0, 0, 0, 1],
            useFixedBase=True,
        )
        record_robot_arm.panda_id = panda_id
        record_robot_arm.limits = {
            2: (
                p.getJointInfo(panda_id, 2)[8],
                p.getJointInfo(panda_id, 2)[9],
            ),
            4: (
                p.getJointInfo(panda_id, 4)[8],
                p.getJointInfo(panda_id, 4)[9],
            ),
        }

    def step_hook(step: int) -> None:
        if step % 8 != 0:
            return
        panda_id = record_robot_arm.panda_id
        limits = record_robot_arm.limits
        p.setJointMotorControl2(
            panda_id,
            2,
            p.POSITION_CONTROL,
            targetPosition=np.random.uniform(*limits[2]),
        )
        p.setJointMotorControl2(
            panda_id,
            4,
            p.POSITION_CONTROL,
            targetPosition=np.random.uniform(*limits[4]),
        )

    return render_preview(
        "robot_arm",
        steps=300,
        view_matrix=camera([0.0, 0.0, 0.5], distance=1.8, yaw=55, pitch=-25),
        setup_fn=setup,
        step_hook=step_hook,
    )


def record_robot_fingers() -> Path:
    def setup() -> None:
        p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        panda_id = p.loadURDF(
            "franka_panda/panda.urdf",
            [0, 0, 0],
            [0, 0, 0, 1],
            useFixedBase=True,
        )
        record_robot_fingers.panda_id = panda_id
        record_robot_fingers.jlower = p.getJointInfo(panda_id, 4)[8]
        record_robot_fingers.jupper = p.getJointInfo(panda_id, 4)[9]
        record_robot_fingers.start = p.getJointState(panda_id, 4)[0]
        record_robot_fingers.target = record_robot_fingers.start
        record_robot_fingers.move_steps = 0

    def step_hook(step: int) -> None:
        panda_id = record_robot_fingers.panda_id
        if record_robot_fingers.move_steps == 0 and step % 80 == 0:
            record_robot_fingers.start = p.getJointState(panda_id, 4)[0]
            record_robot_fingers.target = np.random.uniform(
                record_robot_fingers.jlower,
                record_robot_fingers.jupper,
            )
            record_robot_fingers.move_steps = 80

        if record_robot_fingers.move_steps > 0:
            alpha = 1.0 - (record_robot_fingers.move_steps / 80.0)
            interpolated = (
                record_robot_fingers.start
                + alpha * (record_robot_fingers.target - record_robot_fingers.start)
            )
            p.setJointMotorControl2(
                panda_id,
                4,
                p.POSITION_CONTROL,
                targetPosition=interpolated,
                force=200,
                maxVelocity=0.6,
            )
            record_robot_fingers.move_steps -= 1

    return render_preview(
        "robot_fingers",
        steps=560,
        view_matrix=camera([0.35, 0.0, 0.45], distance=0.9, yaw=45, pitch=-25),
        setup_fn=setup,
        step_hook=step_hook,
    )


def record_cube_creating() -> Path:
    def setup() -> None:
        p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        half = 0.25
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half, half, half])
        vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[half, half, half],
            rgbaColor=[0.0, 0.5, 0.5, 1.0],
        )
        p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=[0, 0, half],
            baseOrientation=[0, 0, 0, 1],
        )

    return render_preview(
        "cube_creating",
        steps=120,
        view_matrix=camera([0, 0, 0.25]),
        setup_fn=setup,
    )


def record_cube_rolling() -> Path:
    def setup() -> None:
        p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        cube_id = p.loadURDF(
            "cube.urdf",
            [0, 0, 1],
            [0, 0, 0, 1],
            globalScaling=0.5,
        )
        p.changeVisualShape(cube_id, -1, rgbaColor=[1, 0.5, 0.1, 1])
        p.resetBaseVelocity(
            cube_id,
            linearVelocity=[1, 1, 1],
            angularVelocity=[0, 0, 0],
        )
        record_cube_rolling.cube_id = cube_id
        record_cube_rolling.step_count = 0

    def step_hook(_step: int) -> None:
        if record_cube_rolling.step_count < 50:
            p.applyExternalForce(
                record_cube_rolling.cube_id,
                -1,
                [4, 0, 0],
                [0, 0, 0],
                p.WORLD_FRAME,
            )
        record_cube_rolling.step_count += 1

    return render_preview(
        "cube_rolling",
        steps=360,
        view_matrix=camera([0, 0, 0.25]),
        setup_fn=setup,
        step_hook=step_hook,
    )


def record_aruco_markers() -> Path:
    def setup() -> None:
        p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        p.loadURDF(
            str(URDF_DIR / "simple_camera.urdf"),
            [0, 0, 0.5],
            p.getQuaternionFromEuler([1.57, 0, 0]),
            useFixedBase=True,
        )
        ar_marker_box_id = p.loadURDF(
            str(URDF_DIR / "ar_marker_box.urdf"),
            [0, -1.07, 0.5],
            p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=True,
        )
        texture_id = p.loadTexture(str(URDF_DIR / "images" / "ar_marker_box.png"))
        p.changeVisualShape(ar_marker_box_id, -1, textureUniqueId=texture_id)
        record_aruco_markers.marker_id = ar_marker_box_id

    def step_hook(step: int) -> None:
        t = step / 240.0
        x = 0.35 * np.sin(0.7 * t)
        y = -1.07 + 0.25 * np.cos(0.5 * t)
        z = 0.5 + 0.15 * np.sin(0.9 * t)
        roll = 0.4 * np.sin(0.6 * t)
        pitch = 0.5 * np.cos(0.4 * t)
        yaw = 0.6 * np.sin(0.8 * t)
        p.resetBasePositionAndOrientation(
            record_aruco_markers.marker_id,
            [x, y, z],
            p.getQuaternionFromEuler([roll, pitch, yaw]),
        )

    return render_preview(
        "aruco_markers",
        steps=480,
        view_matrix=camera([0, 0, 0.1], distance=1.5, yaw=-150, pitch=-20),
        setup_fn=setup,
        step_hook=step_hook,
    )


def record_cube_sliding() -> Path:
    def setup() -> None:
        p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        cube_id = p.loadURDF(
            "cube.urdf",
            [0, 0, 0.25],
            [0, 0, 0, 1],
            globalScaling=0.5,
        )
        p.changeVisualShape(cube_id, -1, rgbaColor=[1, 0.5, 0.1, 1])
        p.resetBaseVelocity(
            cube_id,
            linearVelocity=[3, 0, 0],
            angularVelocity=[0, 0, 0],
        )
        record_cube_sliding.cube_id = cube_id
        record_cube_sliding.step_count = 0

    def step_hook(_step: int) -> None:
        if record_cube_sliding.step_count < 50:
            p.applyExternalForce(
                record_cube_sliding.cube_id,
                -1,
                [4, 0, 0],
                [0, 0, 0],
                p.WORLD_FRAME,
            )
        record_cube_sliding.step_count += 1

    return render_preview(
        "cube_sliding",
        steps=360,
        view_matrix=camera([0, 0, 0.25]),
        setup_fn=setup,
        step_hook=step_hook,
    )


RECORDERS = {
    "hello_world": record_hello_world,
    "robot_arm": record_robot_arm,
    "robot_fingers": record_robot_fingers,
    "cube_creating": record_cube_creating,
    "cube_rolling": record_cube_rolling,
    "cube_sliding": record_cube_sliding,
    "aruco_markers": record_aruco_markers,
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--only",
        nargs="+",
        choices=sorted(RECORDERS),
        help="Record a subset of demos (default: all).",
    )
    args = parser.parse_args()
    selected = args.only or sorted(RECORDERS)

    for name in selected:
        path = RECORDERS[name]()
        print(f"Wrote {path}")


if __name__ == "__main__":
    main()

"""Record headless MP4 previews for each PyBullet demo in robots/."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import pybullet as p # type: ignore
import pybullet_data

ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "docs" / "videos"

WIDTH = 960
HEIGHT = 720
FPS = 30
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


def record(name: str, steps: int, view_matrix: list[float], setup_fn) -> Path:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output_path = OUTPUT_DIR / f"{name}.mp4"
    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        FPS,
        (WIDTH, HEIGHT),
    )

    connect()
    setup_fn()
    capture_every = max(1, round((1.0 / FPS) / TIME_STEP))

    try:
        for step in range(steps):
            p.stepSimulation()
            if step % capture_every == 0:
                writer.write(capture(view_matrix))
    finally:
        writer.release()
        if p.isConnected():
            p.disconnect()

    return output_path


def record_hello_world() -> Path:
    def setup() -> None:
        p.setGravity(0, 0, -10)
        p.loadURDF("plane.urdf")
        p.loadURDF("r2d2.urdf", [0, 0, 3], p.getQuaternionFromEuler([0, 0, 0]))

    return record("hello_world", steps=480, view_matrix=camera([0, 0, 0.5]), setup_fn=setup)


def record_robot_arm() -> Path:
    def setup() -> None:
        p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        panda_id = p.loadURDF(
            "franka_panda/panda.urdf",
            [0, 0, 0],
            [0, 0, 0, 1],
            useFixedBase=True,
        )
        joint_limits = {
            joint_id: (
                p.getJointInfo(panda_id, joint_id)[8],
                p.getJointInfo(panda_id, joint_id)[9],
            )
            for joint_id in (2, 4)
        }
        record_robot_arm.joint_limits = joint_limits
        record_robot_arm.panda_id = panda_id

    def step_hook(step: int) -> None:
        panda_id = record_robot_arm.panda_id
        limits = record_robot_arm.joint_limits
        if step % 8 == 0:
            joint_2 = np.random.uniform(*limits[2])
            joint_4 = np.random.uniform(*limits[4])
            p.setJointMotorControl2(
                panda_id,
                2,
                p.POSITION_CONTROL,
                targetPosition=joint_2,
            )
            p.setJointMotorControl2(
                panda_id,
                4,
                p.POSITION_CONTROL,
                targetPosition=joint_4,
            )

    return record_with_hook(
        "robot_arm",
        steps=360,
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
        record_robot_fingers.target = p.getJointState(panda_id, 4)[0]
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

    return record_with_hook(
        "robot_fingers",
        steps=640,
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

    return record("cube_creating", steps=240, view_matrix=camera([0, 0, 0.25]), setup_fn=setup)


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

    def step_hook(step: int) -> None:
        if record_cube_rolling.step_count < 50:
            p.applyExternalForce(
                record_cube_rolling.cube_id,
                -1,
                [4, 0, 0],
                [0, 0, 0],
                p.WORLD_FRAME,
            )
        record_cube_rolling.step_count += 1

    return record_with_hook(
        "cube_rolling",
        steps=480,
        view_matrix=camera([0, 0, 0.25]),
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

    def step_hook(step: int) -> None:
        if record_cube_sliding.step_count < 50:
            p.applyExternalForce(
                record_cube_sliding.cube_id,
                -1,
                [4, 0, 0],
                [0, 0, 0],
                p.WORLD_FRAME,
            )
        record_cube_sliding.step_count += 1

    return record_with_hook(
        "cube_sliding",
        steps=480,
        view_matrix=camera([0, 0, 0.25]),
        setup_fn=setup,
        step_hook=step_hook,
    )


def record_with_hook(
    name: str,
    steps: int,
    view_matrix: list[float],
    setup_fn,
    step_hook,
) -> Path:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    output_path = OUTPUT_DIR / f"{name}.mp4"
    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        FPS,
        (WIDTH, HEIGHT),
    )
    capture_every = max(1, round((1.0 / FPS) / TIME_STEP))

    connect()
    setup_fn()
    try:
        for step in range(steps):
            step_hook(step)
            p.stepSimulation()
            if step % capture_every == 0:
                writer.write(capture(view_matrix))
    finally:
        writer.release()
        if p.isConnected():
            p.disconnect()

    return output_path


RECORDERS = {
    "hello_world": record_hello_world,
    "robot_arm": record_robot_arm,
    "robot_fingers": record_robot_fingers,
    "cube_creating": record_cube_creating,
    "cube_rolling": record_cube_rolling,
    "cube_sliding": record_cube_sliding,
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

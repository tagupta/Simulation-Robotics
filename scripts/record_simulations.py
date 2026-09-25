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

ARUCO_FOV = 60
ARUCO_NEAR = 0.05
ARUCO_FAR = 10.0
ARUCO_MARKER_SIZE = 0.1
ARUCO_CAMERA_LINK_IDX = 0
ARUCO_CAMERA_TARGET_LINK_IDX = 1
_ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
_ARUCO_DETECTOR = cv2.aruco.ArucoDetector(_ARUCO_DICT, cv2.aruco.DetectorParameters())


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


def _aruco_camera_matrices() -> tuple[np.ndarray, np.ndarray, list[float], list[float]]:
    fov_rad = np.deg2rad(ARUCO_FOV)
    aspect = WIDTH / HEIGHT
    f = HEIGHT / (2 * np.tan(fov_rad / 2))
    camera_matrix = np.array(
        [
            [f, 0, WIDTH / 2],
            [0, f, HEIGHT / 2],
            [0, 0, 1],
        ],
        dtype=np.float32,
    )
    half = ARUCO_MARKER_SIZE / 2
    object_points = np.array(
        [
            [half, half, 0],
            [-half, half, 0],
            [-half, -half, 0],
            [half, -half, 0],
        ],
        dtype=np.float32,
    )
    dist_coeff = np.zeros((1, 4), dtype=np.float32)
    proj = p.computeProjectionMatrixFOV(ARUCO_FOV, aspect, ARUCO_NEAR, ARUCO_FAR)
    return camera_matrix, object_points, dist_coeff, proj


def _detect_aruco_pose(
    gray: np.ndarray,
    camera_matrix: np.ndarray,
    object_points: np.ndarray,
    dist_coeff: np.ndarray,
) -> np.ndarray | None:
    corners, ids, _rejected = _ARUCO_DETECTOR.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return None

    success, rvec, tvec = cv2.solvePnP(
        object_points,
        corners[0][0],
        camera_matrix,
        dist_coeff,
    )
    if not success:
        return None

    x, y, z = tvec.flatten()
    rotation, _ = cv2.Rodrigues(rvec)
    sy = np.hypot(rotation[0, 0], rotation[1, 0])
    if sy < 1e-6:
        roll = np.arctan2(-rotation[1, 2], rotation[1, 1])
        pitch = np.arctan2(-rotation[2, 0], sy)
        yaw = 0.0
    else:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        pitch = np.arctan2(-rotation[2, 0], sy)
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])

    roll, pitch, yaw = np.degrees([roll, pitch, yaw])
    return np.array([x, y, z, roll, pitch, yaw])


def _aruco_pose_overlay_lines(marker_pose: np.ndarray | None) -> tuple[str, str]:
    if marker_pose is None:
        return ("marker not visible", "")

    x = marker_pose[0]
    y = marker_pose[2]
    z = -marker_pose[1]
    roll = -marker_pose[3]
    pitch = -marker_pose[5]
    yaw = -marker_pose[4]
    line1 = f"x: {x:.2f}, y: {y:.2f}, z: {z:.2f},"
    line2 = f"roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f}"
    return (line1, line2)


def _draw_aruco_pose_text(frame: np.ndarray, marker_pose: np.ndarray | None) -> np.ndarray:
    out = frame.copy()
    line1, line2 = _aruco_pose_overlay_lines(marker_pose)
    color = (0, 0, 255)
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.62
    thickness = 2
    x0, y0 = 12, 28
    cv2.putText(out, line1, (x0, y0), font, scale, color, thickness, cv2.LINE_AA)
    if line2:
        cv2.putText(out, line2, (x0, y0 + 26), font, scale, color, thickness, cv2.LINE_AA)
    return out


def _grab_gray_from_view(view_matrix: list[float], proj_matrix: list[float]) -> np.ndarray:
    _, _, rgba, _, _ = p.getCameraImage(
        WIDTH,
        HEIGHT,
        view_matrix,
        proj_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL,
    )
    rgba = np.asarray(rgba, dtype=np.uint8).reshape(HEIGHT, WIDTH, 4)
    return cv2.cvtColor(rgba, cv2.COLOR_RGBA2GRAY)


def record_aruco_markers() -> Path:
    steps = 480
    preview_view = camera([0, 0, 0.1], distance=1.5, yaw=-150, pitch=-20)
    capture_every = max(1, round((1.0 / FPS) / TIME_STEP))
    frames: list[np.ndarray] = []

    connect()
    try:
        p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        camera_id = p.loadURDF(
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

        camera_link_pose = p.getLinkState(camera_id, ARUCO_CAMERA_LINK_IDX)[0]
        camera_target_pose = p.getLinkState(camera_id, ARUCO_CAMERA_TARGET_LINK_IDX)[0]
        camera_matrix, object_points, dist_coeff, sim_proj = _aruco_camera_matrices()
        sim_view = p.computeViewMatrix(camera_link_pose, camera_target_pose, [0, 0, 1])

        for step in range(steps):
            t = step / 240.0
            mx = 0.35 * np.sin(0.7 * t)
            my = -1.07 + 0.25 * np.cos(0.5 * t)
            mz = 0.5 + 0.15 * np.sin(0.9 * t)
            roll = 0.4 * np.sin(0.6 * t)
            pitch = 0.5 * np.cos(0.4 * t)
            yaw = 0.6 * np.sin(0.8 * t)
            p.resetBasePositionAndOrientation(
                ar_marker_box_id,
                [mx, my, mz],
                p.getQuaternionFromEuler([roll, pitch, yaw]),
            )
            p.stepSimulation()
            if step % capture_every != 0:
                continue

            gray = _grab_gray_from_view(sim_view, sim_proj)
            marker_pose = _detect_aruco_pose(
                gray, camera_matrix, object_points, dist_coeff
            )
            frame = capture(preview_view)
            frames.append(_draw_aruco_pose_text(frame, marker_pose))
    finally:
        if p.isConnected():
            p.disconnect()

    return save_gif(frames, OUTPUT_DIR / "aruco_markers.gif")


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

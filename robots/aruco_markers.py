import time
from pathlib import Path

import cv2 as cv
import numpy as np
import pybullet as p  # type: ignore
import pybullet_data

ROOT = Path(__file__).resolve().parent.parent
URDF_DIR = ROOT / "URDFS"

# Default simulated camera intrinsics (matches commented setup / simple_camera use case).
DEFAULT_FOV = 60
DEFAULT_IMAGE_WIDTH = 640
DEFAULT_IMAGE_HEIGHT = 480
DEFAULT_NEAR_PLANE = 0.05
DEFAULT_FAR_PLANE = 10.0
DEFAULT_DIST_COEFF = np.zeros((1, 4), dtype=np.float32)
CAMERA_LINK_IDX = 0
CAMERA_TARGET_LINK_IDX = 1
MARKER_SIZE = 0.1
ARUCO_DICT = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
PARAMETERS = cv.aruco.DetectorParameters()
# Physics stays at 240 Hz. Detection only needs to run when the marker moves.
VISION_HZ = 30


class ArucoMarkerDetection:
    def __init__(self):
        self.init_camera(
            DEFAULT_FOV,
            DEFAULT_IMAGE_WIDTH,
            DEFAULT_IMAGE_HEIGHT,
            DEFAULT_NEAR_PLANE,
            DEFAULT_FAR_PLANE,
            DEFAULT_DIST_COEFF,
            CAMERA_LINK_IDX,
            CAMERA_TARGET_LINK_IDX,
            MARKER_SIZE,
            ARUCO_DICT,
            PARAMETERS,
        )
        self.state = self.init_state()
        self._slider_pose = None
        self._pose_text_id = -1
        self._next_vision_time = 0.0

    def init_state(self):
        p.connect(p.GUI)
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        time_step = 1.0 / 240.0
        p.setTimeStep(time_step)
        # getCameraImage otherwise redraws these extra panes every frame.
        # p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        # p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        # p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        self.zoom_camera()
        self.configure_camera_aruco_marker()
        self.slider_ids = self.add_camera_position_and_orientation_parameters()
        return {"time_step": time_step, "vision_period": 1.0 / VISION_HZ}

    def init_camera(
        self,
        fov,
        image_width,
        image_height,
        near_plane,
        far_plane,
        dist_coeff,
        camera_link_idx,
        camera_target_link_idx,
        marker_size,
        aruco_dict,
        parameters,
    ):
        self.camera_link_idx = camera_link_idx
        self.camera_target_link_idx = camera_target_link_idx
        self.fov_rad = np.deg2rad(fov)
        self.image_width = image_width
        self.image_height = image_height
        self.near_plane = near_plane
        self.far_plane = far_plane
        self.aspect_ratio = image_width / image_height
        self.projection_matrix = p.computeProjectionMatrixFOV(
            fov, self.aspect_ratio, self.near_plane, self.far_plane
        )
        self.f = self.image_height / (2 * np.tan(self.fov_rad / 2))
        self.camera_matrix = np.array(
            [
                [self.f, 0, self.image_width / 2],
                [0, self.f, self.image_height / 2],
                [0, 0, 1],
            ],
            dtype=np.float32,
        )
        self.dist_coeff = dist_coeff
        self.marker_size = marker_size
        half = marker_size / 2
        self.object_points = np.array(
            [
                [half, half, 0],
                [-half, half, 0],
                [-half, -half, 0],
                [half, -half, 0],
            ],
            dtype=np.float32,
        )
        self.aruco_dict = aruco_dict
        self.parameters = parameters
        self.detector = cv.aruco.ArucoDetector(aruco_dict, parameters)

    def zoom_camera(self):
        # Smaller cameraDistance = closer zoom. Aim at the arm, not the floor.
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=-150.0,
            cameraPitch=-20.0,
            cameraTargetPosition=[0, 0, 0.1],
        )

    def configure_camera_aruco_marker(self):
        p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        self.camera_id = p.loadURDF(
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

        # Move the light so the box shadow does not wash out the marker.
        p.configureDebugVisualizer(lightPosition=[0, 0, 100])
        self.ar_marker_box_id = ar_marker_box_id
        # Camera body never moves, so the view matrix is constant.
        camera_link_pose = p.getLinkState(self.camera_id, self.camera_link_idx)[0]
        camera_target_link_pose = p.getLinkState(
            self.camera_id, self.camera_target_link_idx
        )[0]
        # Eye is the camera link. The target link sits 1 cm along the optical axis.
        self.view_matrix = p.computeViewMatrix(
            camera_link_pose, camera_target_link_pose, [0, 0, 1]
        )

    def detect_aruco_marker(self, gray_image):
        corners, ids, _rejected = self.detector.detectMarkers(gray_image)
        if ids is None or len(ids) == 0:
            return None

        success, rvec, tvec = cv.solvePnP(
            self.object_points,
            corners[0][0],
            self.camera_matrix,
            self.dist_coeff
            # flags=cv.SOLVEPNP_IPPE,
        )
        if not success:
            return None

        x, y, z = tvec.flatten()
        rotation, _ = cv.Rodrigues(rvec)
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

    def reset(self):
        p.disconnect()
        self.state = self.init_state()
        self._slider_pose = None
        self._pose_text_id = -1
        self._next_vision_time = 0.0

    def add_camera_position_and_orientation_parameters(self):
        return (
            p.addUserDebugParameter("Aruco_Marker_X", -4, 4, 0),
            p.addUserDebugParameter("Aruco_Marker_Y", -4, 4, -1.07),
            p.addUserDebugParameter("Aruco_Marker_Z", -4, 8, 0.5),
            p.addUserDebugParameter("Aruco_Marker_roll", -3.14, 3.14, 0),
            p.addUserDebugParameter("Aruco_Marker_pitch", -3.14, 3.14, 0),
            p.addUserDebugParameter("Aruco_Marker_yaw", -3.14, 3.14, 0),
        )

    def _read_sliders(self):
        return tuple(p.readUserDebugParameter(slider_id) for slider_id in self.slider_ids)

    def _grab_gray(self):
        _w, _h, rgba, _depth, _seg = p.getCameraImage(
            self.image_width,
            self.image_height,
            self.view_matrix,
            self.projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
            # flags=p.ER_NO_SEGMENTATION_MASK,
        )
        rgba = np.asarray(rgba, dtype=np.uint8).reshape(
            self.image_height, self.image_width, 4
        )
        return cv.cvtColor(rgba, cv.COLOR_RGBA2GRAY)

    def _publish_pose(self, marker_pose):
        if marker_pose is None:
            text = "marker not visible"
        else:
            x = marker_pose[0]
            y = marker_pose[2]
            z = -marker_pose[1]
            roll = -marker_pose[3]
            pitch = -marker_pose[5]
            yaw = -marker_pose[4]
            text = (
                f"x: {x:.2f}, y: {y:.2f}, z: {z:.2f}, "
                f"roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f}"
            )
        self._pose_text_id = p.addUserDebugText(
            text,
            [1, 0, 0],
            textColorRGB=[1, 0, 0],
            textSize=1.5,
            lifeTime=0,
            replaceItemUniqueId=self._pose_text_id,
        )

    def run(self):
        self.main()

    def main(self):
        try:
            while p.isConnected():
                slider_pose = self._read_sliders()
                if slider_pose != self._slider_pose:
                    ar_x, ar_y, ar_z, ar_roll, ar_pitch, ar_yaw = slider_pose
                    p.resetBasePositionAndOrientation(
                        self.ar_marker_box_id,
                        [ar_x, ar_y, ar_z],
                        p.getQuaternionFromEuler([ar_roll, ar_pitch, ar_yaw]),
                    )
                    self._slider_pose = slider_pose

                now = time.perf_counter()
                if now >= self._next_vision_time:
                    marker_pose = self.detect_aruco_marker(self._grab_gray())
                    self._publish_pose(marker_pose)
                    self._next_vision_time = now + self.state["vision_period"]

                p.stepSimulation()
                time.sleep(self.state["time_step"])
        except (KeyboardInterrupt, p.error):
            # Closing the GUI makes the next slider read raise p.error.
            pass
        finally:
            if p.isConnected():
                p.disconnect()


aruco_marker_detection = ArucoMarkerDetection()
aruco_marker_detection.run()

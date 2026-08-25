#!/usr/bin/env python3
"""
OpenPose-based 2D pose detector.
"""

import sys
import os
import numpy as np
from typing import List

from social_context.pose_estimation.src.pose_detectors.abstract import (
    AbstractPoseDetector,
    PersonDetection
)

# BODY_25 keypoint indices
NOSE = 0
RSHOULDER = 2
LSHOULDER = 5
REYE = 15
LEYE = 16

FACE_VISIBLE_THRESHOLD = 0.3
MIN_SHOULDER_CONFIDENCE_FOR_ORIENTATION = 0.1

class OpenPosePoseDetector(AbstractPoseDetector):
    """OpenPose implementation of 2D human pose detection"""

    def __init__(
            self,
            logger=None,
            openpose_path: str = None,
            model_pose: str = "BODY_25",
            num_gpu: int = 0,
            confidence_threshold: float = 0.1
    ):
        """
        Initialize OpenPose detector.

        Args:
            logger: Optional ROS logger
            openpose_path: Path to OpenPose installation (defaults to OPENPOSE_PATH env var)
            model_pose: Model to use ("BODY_25", "COCO", "MPI")
            num_gpu: Number of GPUs to use (0 = CPU only)
            confidence_threshold: Minimum confidence for detection
        """
        super().__init__(logger)
        self.openpose_path = openpose_path or os.environ.get('OPENPOSE_PATH', '')
        self.model_folder = f"{self.openpose_path}/models/"
        self.model_pose = model_pose
        self.num_gpu = num_gpu
        self.confidence_threshold = confidence_threshold
        self.op_wrapper = None
        self.op = None

    def initialize(self) -> bool:
        """Initialize OpenPose."""
        try:
            if not self.openpose_path:
                self.log_error("OPENPOSE_PATH not set! Set environment variable.")
                return False

            # Import OpenPose Python module
            sys.path.append(f'{self.openpose_path}/build/python/openpose')
            import pyopenpose as op
            self.op = op

            params = {
                "model_folder": self.model_folder,
                "model_pose": self.model_pose,
            }

            if self.num_gpu > 0:
                params["num_gpu"] = self.num_gpu
                params["num_gpu_start"] = 0

            self.op_wrapper = op.WrapperPython()
            self.op_wrapper.configure(params)
            self.op_wrapper.start()

            self.is_ready = True
            self.log_info(
                f"OpenPose initialized: "
                f"model={self.model_pose}, "
                f"GPU={self.num_gpu}, "
                f"conf_threshold={self.confidence_threshold}"
            )
            return True

        except Exception as e:
            self.log_error(f"OpenPose initialization failed: {e}")
            return False

    def detect(self, image: np.ndarray) -> List[PersonDetection]:
        """
        Detect people in image using OpenPose.

        Args:
            image: BGR image as numpy array (H, W, 3)

        Returns:
            List of PersonDetection objects (one per detected person)
        """
        if not self.is_ready:
            self.log_error("Detector not initialized!")
            return []

        try:
            # Process image with OpenPose
            datum = self.op.Datum()
            datum.cvInputData = image
            self.op_wrapper.emplaceAndPop(self.op.VectorDatum([datum]))

            detections = []

            if (datum.poseKeypoints is not None and
                    not isinstance(datum.poseKeypoints, (tuple, float, int)) and
                    len(datum.poseKeypoints) > 0):

                # Process each detected person
                for person in datum.poseKeypoints:
                    # Get keypoint 8 (MidHip in BODY_25 model)
                    if len(person) > 8 and len(person[8]) >= 3:
                        confidence = float(person[8, 2])

                        if confidence > self.confidence_threshold:
                            facing = self._compute_facing_heuristic(person)
                            detections.append(PersonDetection(
                                pixel_x=float(person[8, 0]),
                                pixel_y=float(person[8, 1]),
                                confidence=confidence,
                                orientation_facing=facing
                            ))

            return detections

        except Exception as e:
            self.log_error(f"Error during detection: {e}")
            return []

    @staticmethod
    def _keypoint_conf(person, idx: int) -> float:
        """
        Confidence of a BODY_25 keypoint, or 0.0 if not present.

        Args:
            person: Numpy array of shape (25, 3) for BODY_25 keypoints
            idx: index of the keypoint in BODY_25 model
        
        Returns:
            Confidence value (float) of the keypoint, or 0.0 if not present
        """
        if len(person) > idx and len(person[idx]) >= 3:
            return float(person[idx, 2])
        return 0.0

    def _compute_facing_heuristic(self, person):
        """
        Approximate a facing direction from 2D-only OpenPose BODY_25 keypoints.

        Args:
            person: Numpy array of shape (25, 3) for BODY_25 keypoints

        Returns:
            (dx, dy, dz) in camera optical frame -- (0, 0, -1) facing the
            camera, (0, 0, 1) facing away (same convention/default as
            MediaPipe's detector).
        """
        face_conf = (
            self._keypoint_conf(person, NOSE)
            + self._keypoint_conf(person, LEYE)
            + self._keypoint_conf(person, REYE)
        ) / 3.0
        face_toward_camera = face_conf >= FACE_VISIBLE_THRESHOLD

        r_conf = self._keypoint_conf(person, RSHOULDER)
        l_conf = self._keypoint_conf(person, LSHOULDER)
        shoulder_toward_camera = None
        if r_conf > MIN_SHOULDER_CONFIDENCE_FOR_ORIENTATION and l_conf > MIN_SHOULDER_CONFIDENCE_FOR_ORIENTATION:
            shoulder_toward_camera = person[RSHOULDER, 0] < person[LSHOULDER, 0]

        if shoulder_toward_camera is None:
            toward_camera = face_toward_camera
        elif shoulder_toward_camera == face_toward_camera:
            toward_camera = face_toward_camera
        else:
            return (0.0, 0.0, 1.0)

        return (0.0, 0.0, -1.0) if toward_camera else (0.0, 0.0, 1.0)

    def cleanup(self):
        """Clean up OpenPose resources."""
        # OpenPose doesn't have explicit cleanup
        # The wrapper is cleaned up automatically when Python exits
        self.is_ready = False
        self.log_info("OpenPose cleaned up")

    def get_name(self) -> str:
        """Return detector name."""
        return "OpenPose"



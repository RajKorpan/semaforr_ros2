"""
Implements Kalman Filtering for 1 person
Remembers where a person was
Predicts where they will next frame
Needs position and velocity - [x, y, vx, vy]

"""

import numpy as np
import math
from collections import deque
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import time


ORIENTATION_MEDIAN_WINDOW = 7
FLIP_SUSPECT_THRESHOLD = math.radians(130)
MIN_SPEED_FOR_VELOCITY_HEADING = 0.2

MAX_SPEED = 3.0

# Multiply velocity by this on each missed frame, so a coasting
# (uncorrected) track's extrapolation settles toward "stayed put" instead
# of continuing to fly off in whatever direction it last inferred.
COASTING_VELOCITY_DECAY = 0.85

class KalmanPersonTracker():
    count = 0

    def __init__(self, initial_position, timestamp=None):
        """
        New person is detected
        Set up Kalman Filter for this person and 

        Args: 
            initial-position: (x, y)
            timestamp: Time of detection
        """

        self.kf = KalmanFilter(dim_x=4, dim_z=2)

        # Initialize state [x, y, vx=0, vy=0]
        x, y = initial_position
        self.kf.x = np.array([[x], [y], [0.], [0.]])
        
        # State transition matrix
        # next_x  = x + vx
        # next_y  = y + vy
        # next_vx = vx
        # next_vy = vy
        self.kf.F = np.eye(4)
        
        # Measurement matrix - we only measure position (x, y)
        # measured_x = x
        # measured_y = y
        self.kf.H = np.array([[1., 0., 0., 0.],
                                [0., 1., 0., 0.]])
        

        
        # Process noise covariance
        self.kf.Q = np.eye(4) * 0.1
        
        # Measurement noise covariance
        self.kf.R = np.eye(2) * 0.1
        
        # Initial state covariance
        self.kf.P = np.eye(4) * 1.0

        if timestamp is not None:
            self.last_predict_time = timestamp
        else:
            self.last_predict_time = time.time()  # Use current time if no timestamp is provided
        self.q_scale = 0.5
        self.max_dt = 2.0
        self.min_dt = 1e-3
        self.last_dt = 0.1  # defensive default, overwritten on first predict()
        self.time_since_update = 0.0  # accumulated real time since the last real detection matched

        self.initial_position = initial_position

        KalmanPersonTracker.count += 1
        self.id = KalmanPersonTracker.count

        self.history = [initial_position]
        self.missed = 0
        self.hits = 1
        self.state = 'TENTATIVE'  # Can be 'TENTATIVE', 'CONFIRMED' (3 hits for now), 'DELETED'
        self.confidence = 0.0
        self.appearance_feature = None
        self.orientation = (0.0, 0.0, 0.0, 1.0)  # Default orientation as a quaternion
        self._orientation_yaw_history = deque(maxlen=ORIENTATION_MEDIAN_WINDOW)


    def update_appearance_feature(self, feature, ema_alpha=0.8):
        """
        Update the appearance feature using Exponential Moving Average (EMA)

        Args:
            feature: New appearance feature vector
            ema_alpha: EMA smoothing factor (0 < alpha < 1)
        """
        if feature is None:
            return 

        if self.appearance_feature is None:
            self.appearance_feature = feature #first time -> initialize
        else: #update using EMA
            self.appearance_feature = ema_alpha * self.appearance_feature + (1 - ema_alpha) * feature


    def predict(self, timestamp=None):
        """
        Predict the next position of the person based on the Kalman Filter

        Returns:
            predicted_position: (x, y)
        """

        if timestamp is not None:
            now = timestamp
        else:
            now = time.time()
        df = now - self.last_predict_time
        self.last_predict_time = now
        df = max(self.min_dt, min(df, self.max_dt))  # Clamp df to avoid extreme values
        self.last_dt = df  # exposed for SortTracker.match_radius() -- see there for why
        self.time_since_update += df  # accumulates across consecutive misses; reset in update()

        self.kf.F = np.array([[1., 0., df, 0.],
                              [0., 1., 0., df],
                              [0., 0., 1., 0.],
                              [0., 0., 0., 1.]])

        self.kf.Q = Q_discrete_white_noise(dim=2, dt=df, var=self.q_scale, block_size=2, order_by_dim=False)


        self.kf.predict()
        predicted_x = self.kf.x[0, 0]
        predicted_y = self.kf.x[1, 0]

        return (predicted_x, predicted_y)

    def update (self, new_position, confidence = 0.0, hits_threshold = 3, orientation = None):
        """
        Update the Kalman Filter with a new position measurement

        Args:
            new_position: (x, y)
            confidence: Detection confidence from MediaPipe (0.0 to 1.0)
            hits_threshold: Number of consecutive hits before a tracker is confirmed
        """
        self.kf.R = np.eye(2) * 0.1* (1.0 - confidence) + np.eye(2)*1e-6 # Adjust measurement noise based on confidence
        self.kf.update(np.array([[new_position[0]], [new_position[1]]]))
        self._clamp_velocity()
        self.time_since_update = 0.0  # a real detection just matched -- gap is closed
        self.history.append(new_position)
        if len(self.history) > 30:  # Keep only the last 30 positions
            self.history.pop(0)


        self.missed = 0
        self.hits += 1
        self.confidence = confidence

        # Threshold to confirm a track - if we have 3 hits, we consider it confirmed (COULD BE CHANGED)
        if self.state == 'TENTATIVE' and self.hits >= hits_threshold:
            self.state = 'CONFIRMED'
        
        if orientation is not None:
            self.orientation = self._filter_orientation(orientation)

    def _clamp_velocity(self):
        """Bound the Kalman filter's velocity estimate to MAX_SPEED"""
        vx, vy = self.kf.x[2, 0], self.kf.x[3, 0]
        speed = math.hypot(vx, vy)
        if speed > MAX_SPEED:
            scale = MAX_SPEED / speed
            self.kf.x[2, 0] = vx * scale
            self.kf.x[3, 0] = vy * scale

    def _filter_orientation(self, orientation):
        """
        Filter the orientation using a circular median filter to reduce noise and sudden jumps

        Args:
            orientation: Quaternion (x, y, z, w)
        
        Returns:
            Filtered orientation as a quaternion (x, y, z, w)
        """
        _, _, oz, ow = orientation
        raw_yaw = 2.0 * math.atan2(oz, ow)
        raw_yaw = self._disambiguate_flip(raw_yaw)
        self._orientation_yaw_history.append(raw_yaw)
        filtered_yaw = self._circular_median(self._orientation_yaw_history)
        return (0.0, 0.0, math.sin(filtered_yaw / 2.0), math.cos(filtered_yaw / 2.0))

    def _disambiguate_flip(self, raw_yaw):
        """
        Disambiguate a potential shoulder-labeling flip in the yaw measurement

        Args:
            raw_yaw: Raw yaw angle in radians from the current orientation measurement

        Returns:
            Disambiguated yaw angle in radians
        """
        _, _, prev_oz, prev_ow = self.orientation
        prev_yaw = 2.0 * math.atan2(prev_oz, prev_ow)

        if abs(self._angle_diff(raw_yaw, prev_yaw)) < FLIP_SUSPECT_THRESHOLD:
            return raw_yaw  # not a suspected flip, nothing to disambiguate

        vx, vy = self.kf.x[2, 0], self.kf.x[3, 0]
        speed = math.hypot(vx, vy)
        if speed < MIN_SPEED_FOR_VELOCITY_HEADING:
            return raw_yaw  # not moving fast enough to trust a velocity heading

        velocity_heading = math.atan2(vy, vx)
        flipped_yaw = raw_yaw + math.pi
        flipped_yaw = (flipped_yaw + math.pi) % (2 * math.pi) - math.pi  # wrap to (-pi, pi]

        if abs(self._angle_diff(flipped_yaw, velocity_heading)) < abs(self._angle_diff(raw_yaw, velocity_heading)):
            return flipped_yaw
        return raw_yaw

    @staticmethod
    def _angle_diff(a, b):
        """
        Compute the difference between two angles (in radians), wrapped to (-pi, pi]
        Args:
            a: Angle a in radians
            b: Angle b in radians
        Returns:
            Difference a - b, wrapped to (-pi, pi]
        """
        return (a - b + math.pi) % (2 * math.pi) - math.pi

    @classmethod
    def _circular_median(cls, angles):
        """
        Compute the median of a small set of angles (radians), robust to wraparound.
        Args:
            angles: List of angles in radians
        Returns:
            Median angle in radians, wrapped to (-pi, pi]"""
        angles = list(angles)
        ref = angles[0]
        unwrapped = sorted(ref + cls._angle_diff(a, ref) for a in angles)
        n = len(unwrapped)
        med = unwrapped[n // 2] if n % 2 == 1 else (unwrapped[n // 2 - 1] + unwrapped[n // 2]) / 2
        return (med + math.pi) % (2 * math.pi) - math.pi


    def mark_missed(self, missed_threshold = 15, tentative_missed_threshold = 7):
        """
        Mark the person as missed

        Args:
            missed_threshold: Number of consecutive misses before a tracker is deleted
            tentative_missed_threshold: Number of consecutive misses before a tentative tracker is deleted
        """
        self.missed += 1
        self.hits = 0  # reset hits if we miss a detection
        self.kf.x[2, 0] *= COASTING_VELOCITY_DECAY
        self.kf.x[3, 0] *= COASTING_VELOCITY_DECAY
        if self.state == 'TENTATIVE' and self.missed >= tentative_missed_threshold:
            self.state = 'DELETED'
        elif self.missed >= missed_threshold:
            self.state = 'DELETED'

    def get_position(self):
        """
        Get the current position of the person

        Returns:
            position: (x, y)
        """
        x = self.kf.x[0, 0]
        y = self.kf.x[1, 0]
        return (x, y)

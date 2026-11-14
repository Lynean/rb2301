"""PID control for a car-like robot from heading and front-distance errors."""

from dataclasses import dataclass
import math
import time
from typing import Optional, Tuple


def wrap_axis_angle(angle: float) -> float:
    """Wrap an angle to [-pi/2, pi/2] for an unoriented heading axis."""
    return 0.5 * math.atan2(math.sin(2.0 * angle), math.cos(2.0 * angle))


@dataclass
class PIDController:
    """Turn and drive using heading error (radians) and distance error (m).

    Heading error is the smallest rotation needed to align the robot's heading
    axis with the goal vector. It is limited to [-pi/2, pi/2], where positive
    means counter-clockwise and negative means clockwise. Distance error is
    signed along that axis (positive ahead, negative behind). Outputs are
    forward speed (m/s) and yaw rate (rad/s); lateral speed is always zero.
    """

    kp_distance: float = 0.8
    ki_distance: float = 0.1
    kd_distance: float = 0.05
    kp_heading: float = 0.8
    ki_heading: float = 0.1
    kd_heading: float = 0.05
    max_speed: float = 0.3
    max_yaw_rate: float = 1
    minimum_turn_speed: float = 0.1
    max_linear_acceleration: float = 0.4
    max_yaw_acceleration: float = 1.2
    nominal_dt: float = 0.05
    heading_tolerance: float = math.radians(8.0)
    integral_limit: float = 1.0

    def __post_init__(self):
        self.reset()

    def reset(self) -> None:
        self._previous_distance_error = None
        self._previous_heading_error = None
        self._distance_integral = 0.0
        self._heading_integral = 0.0
        self._previous_time = None
        self._previous_speed = 0.0
        self._previous_yaw_rate = 0.0

    def compute(
        self,
        error_heading: float,
        error_distance: float,
        dt: Optional[float] = None,
    ) -> Tuple[float, float]:
        """Return ``(linear_x, angular_z)`` from heading and distance errors.

        ``error_heading`` is in radians, positive counter-clockwise, and is
        folded into [-pi/2, pi/2] because forward and backward axis alignment
        are equivalent;
        ``error_distance`` is in meters along the front axis.
        """
        heading_error = wrap_axis_angle(float(error_heading))
        distance_error = float(error_distance)

        now = time.monotonic()
        if dt is None:
            dt = 0.0 if self._previous_time is None else now - self._previous_time
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 1e-6 or dt > 1.0:
            dt = 0.0

        if dt:
            self._distance_integral = self._clamp(
                self._distance_integral + distance_error * dt,
                self.integral_limit,
            )
            self._heading_integral = self._clamp(
                self._heading_integral + heading_error * dt,
                self.integral_limit,
            )
            distance_derivative = (
                0.0 if self._previous_distance_error is None else
                (distance_error - self._previous_distance_error) / dt
            )
            heading_derivative = (
                0.0 if self._previous_heading_error is None else
                wrap_axis_angle(heading_error - self._previous_heading_error) / dt
            )
        else:
            distance_derivative = heading_derivative = 0.0

        speed = (
            self.kp_distance * distance_error
            + self.ki_distance * self._distance_integral
            + self.kd_distance * distance_derivative
        )
        # A car-like robot must translate while steering. Keep the signed
        # projected-distance command and provide a small crawl speed when the
        # goal is strongly off-axis. Negative distance selects reverse.
        if abs(heading_error) > self.heading_tolerance:
            travel_direction = -1.0 if distance_error < 0.0 else 1.0
            if abs(speed) < self.minimum_turn_speed:
                speed = travel_direction * self.minimum_turn_speed
        speed = self._clamp(speed, self.max_speed)

        yaw_rate = (
            self.kp_heading * heading_error
            + self.ki_heading * self._heading_integral
            + self.kd_heading * heading_derivative
        )
        yaw_rate = self._clamp(yaw_rate, self.max_yaw_rate)

        # Slew-rate limits avoid instantaneous starts and steering changes.
        # Use the node's nominal 20 Hz period on the first controller update.
        ramp_dt = dt if dt > 0.0 else self.nominal_dt
        speed = self._rate_limit(
            speed,
            self._previous_speed,
            self.max_linear_acceleration,
            ramp_dt,
        )
        yaw_rate = self._rate_limit(
            yaw_rate,
            self._previous_yaw_rate,
            self.max_yaw_acceleration,
            ramp_dt,
        )

        self._previous_distance_error = distance_error
        self._previous_heading_error = heading_error
        self._previous_time = now
        self._previous_speed = speed
        self._previous_yaw_rate = yaw_rate
        return speed, yaw_rate

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-abs(limit), min(abs(limit), value))

    @staticmethod
    def _rate_limit(target: float, previous: float, rate: float, dt: float) -> float:
        maximum_change = abs(rate) * dt
        change = target - previous
        change = max(-maximum_change, min(maximum_change, change))
        return previous + change

"""
Landing Evaluation Module
Defines metrics to evaluate landing success or failure for the car jump simulation.
Metrics include: uprightness (pitch/roll), velocity at impact, and stability after landing.
"""

import pybullet as p
import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Tuple


@dataclass
class LandingMetrics:
    """Data class to store landing evaluation metrics."""
    timestamp: float
    is_landed: bool
    uprightness_score: float  # 0-100 (100 is perfectly upright)
    pitch: float  # radians
    roll: float  # radians
    yaw: float  # radians
    velocity_at_impact: float  # m/s
    vertical_velocity: float  # m/s (downward component)
    stability_score: float  # 0-100 (100 is completely stable)
    impact_smoothness: float  # 0-100 (100 is smoothest)
    landing_success: bool  # Overall success (meets thresholds)
    
    def __str__(self):
        status = "✓ SUCCESS" if self.landing_success else "✗ FAILED"
        return f"""
Landing Evaluation Report {status}
{'='*50}
Timestamp:           {self.timestamp:.2f}s
Uprightness Score:   {self.uprightness_score:.1f}/100
  Pitch:             {np.degrees(self.pitch):.2f}°
  Roll:              {np.degrees(self.roll):.2f}°
  Yaw:               {np.degrees(self.yaw):.2f}°
Velocity Metrics:
  Total Speed:       {self.velocity_at_impact:.2f} m/s
  Vertical Speed:    {self.vertical_velocity:.2f} m/s
Stability Score:     {self.stability_score:.1f}/100
Impact Smoothness:   {self.impact_smoothness:.1f}/100
{'='*50}
"""


class LandingEvaluator:
    """
    Evaluates landing performance based on multiple metrics.
    Tracks metrics before, during, and after landing.
    """
    
    def __init__(self, cfg: Dict = None):
        """
        Initialize the landing evaluator.
        
        Args:
            cfg: Configuration dict with landing thresholds
        """
        self.cfg = cfg or {}
        
        # Landing success thresholds
        self.uprightness_threshold = self.cfg.get('uprightness_threshold', 60.0)  # degrees
        self.velocity_threshold = self.cfg.get('velocity_threshold', 5.0)  # m/s
        self.vertical_velocity_threshold = self.cfg.get('vertical_velocity_threshold', 3.0)  # m/s
        self.stability_threshold = self.cfg.get('stability_threshold', 50.0)  # score out of 100
        
        # Historical tracking
        self.velocity_history: List[float] = []
        self.angular_velocity_history: List[Tuple[float, float, float]] = []
        self.orientation_history: List[Tuple[float, float, float]] = []
        self.max_history_size = 100
        
        # Landing event tracking
        self.is_landing_phase = False
        self.landing_velocity = None
        self.landing_time = None
        
    def update_history(self, car: int, current_time: float):
        """
        Update motion history for trend analysis.
        
        Args:
            car: PyBullet car body ID
            current_time: Current simulation time
        """
        car_vel, car_ang_vel = p.getBaseVelocity(car)
        car_pos, car_orn = p.getBasePositionAndOrientation(car)
        
        # Track velocity magnitude
        velocity_mag = np.linalg.norm(car_vel)
        self.velocity_history.append(velocity_mag)
        
        # Track angular velocity
        self.angular_velocity_history.append(tuple(car_ang_vel))
        
        # Track orientation
        pitch, roll, yaw = p.getEulerFromQuaternion(car_orn)
        self.orientation_history.append((pitch, roll, yaw))
        
        # Limit history size
        if len(self.velocity_history) > self.max_history_size:
            self.velocity_history.pop(0)
        if len(self.angular_velocity_history) > self.max_history_size:
            self.angular_velocity_history.pop(0)
        if len(self.orientation_history) > self.max_history_size:
            self.orientation_history.pop(0)
    
    def calculate_uprightness_score(self, pitch: float, roll: float) -> float:
        """
        Calculate uprightness score based on pitch and roll angles.
        
        Args:
            pitch: Pitch angle in radians (rotation about Y-axis)
            roll: Roll angle in radians (rotation about X-axis)
        
        Returns:
            Score from 0-100 (100 = perfectly upright)
        """
        # Convert to degrees for easier interpretation
        pitch_deg = np.degrees(pitch)
        roll_deg = np.degrees(roll)
        
        # Maximum acceptable angles
        max_pitch = 45.0  # degrees
        max_roll = 45.0   # degrees
        
        # Calculate deviation from upright (0, 0)
        pitch_deviation = min(abs(pitch_deg), max_pitch)
        roll_deviation = min(abs(roll_deg), max_roll)
        
        # Score: 100 when both are 0, 0 when either exceeds threshold
        pitch_score = 100 * (1 - pitch_deviation / max_pitch)
        roll_score = 100 * (1 - roll_deviation / max_roll)
        
        # Combined score (average)
        uprightness_score = (pitch_score + roll_score) / 2.0
        
        return max(0, uprightness_score)
    
    def calculate_velocity_score(self, velocity: float) -> float:
        """
        Calculate impact smoothness based on velocity magnitude.
        
        Args:
            velocity: Total velocity magnitude in m/s
        
        Returns:
            Score from 0-100 (100 = zero velocity/softest landing)
        """
        # Ideal landing velocity is 0 m/s
        # Score decreases as velocity increases
        max_acceptable_velocity = 10.0  # m/s
        
        if velocity <= 0.5:  # Nearly zero
            return 100.0
        elif velocity >= max_acceptable_velocity:
            return 0.0
        else:
            # Linear interpolation
            score = 100.0 * (1 - velocity / max_acceptable_velocity)
            return max(0, score)
    
    def calculate_vertical_velocity_component(self, car_vel: np.ndarray) -> float:
        """
        Extract and return the vertical (Z) component of velocity.
        
        Args:
            car_vel: 3D velocity vector [vx, vy, vz]
        
        Returns:
            Vertical velocity in m/s (positive = upward, negative = downward)
        """
        return car_vel[2]
    
    def calculate_stability_score(self, 
                                  angular_velocity: Tuple[float, float, float],
                                  vehicle_state: str = "during") -> float:
        """
        Calculate stability based on angular velocity.
        Lower angular velocities indicate more stable landings.
        
        Args:
            angular_velocity: Angular velocity tuple (wx, wy, wz) in rad/s
            vehicle_state: "before", "during", or "after" landing
        
        Returns:
            Score from 0-100 (100 = no rotation/most stable)
        """
        ang_vel_mag = np.linalg.norm(angular_velocity)
        
        # Define thresholds based on landing phase
        if vehicle_state == "before":
            max_acceptable_rotation = 5.0  # rad/s
        elif vehicle_state == "during":
            max_acceptable_rotation = 3.0  # rad/s (stricter)
        else:  # "after"
            max_acceptable_rotation = 1.0  # rad/s (very strict)
        
        if ang_vel_mag <= 0.5:
            return 100.0
        elif ang_vel_mag >= max_acceptable_rotation:
            return 0.0
        else:
            score = 100.0 * (1 - ang_vel_mag / max_acceptable_rotation)
            return max(0, score)
    
    def calculate_smoothness_score(self, velocity_history: List[float] = None) -> float:
        """
        Calculate impact smoothness based on velocity variation.
        Smooth landing = minimal acceleration/deceleration during impact.
        
        Args:
            velocity_history: List of velocity values over time (uses self.velocity_history if None)
        
        Returns:
            Score from 0-100 (100 = smoothest impact)
        """
        if velocity_history is None:
            velocity_history = self.velocity_history
        
        if len(velocity_history) < 2:
            return 100.0
        
        # Calculate acceleration (velocity changes)
        velocities = np.array(velocity_history[-10:])  # Last 10 samples
        if len(velocities) < 2:
            return 100.0
        
        accelerations = np.diff(velocities)
        mean_acceleration = np.mean(np.abs(accelerations))
        
        # Score based on smoothness (lower acceleration = higher score)
        max_acceptable_acceleration = 5.0  # m/s²
        
        if mean_acceleration <= 0.5:
            return 100.0
        elif mean_acceleration >= max_acceptable_acceleration:
            return 0.0
        else:
            score = 100.0 * (1 - mean_acceleration / max_acceptable_acceleration)
            return max(0, score)
    
    def evaluate_landing(self, 
                        car: int,
                        plane: int,
                        current_time: float) -> LandingMetrics:
        """
        Comprehensive landing evaluation at the moment of impact.
        
        Args:
            car: PyBullet car body ID
            plane: PyBullet plane body ID
            current_time: Current simulation time in seconds
        
        Returns:
            LandingMetrics object with all evaluation scores
        """
        # Get current state
        car_pos, car_orn = p.getBasePositionAndOrientation(car)
        car_vel, car_ang_vel = p.getBaseVelocity(car)
        pitch, roll, yaw = p.getEulerFromQuaternion(car_orn)
        
        # Check if car is in contact with ground
        contacts = p.getContactPoints(car, plane)
        is_landed = len(contacts) > 0
        
        # Calculate metrics
        uprightness = self.calculate_uprightness_score(pitch, roll)
        velocity_mag = np.linalg.norm(car_vel)
        velocity_score = self.calculate_velocity_score(velocity_mag)
        vertical_vel = self.calculate_vertical_velocity_component(car_vel)
        stability = self.calculate_stability_score(car_ang_vel, vehicle_state="during")
        smoothness = self.calculate_smoothness_score()
        
        # Determine success (meets all thresholds)
        uprightness_ok = uprightness >= self.uprightness_threshold
        velocity_ok = velocity_mag <= self.velocity_threshold
        vertical_vel_ok = abs(vertical_vel) <= self.vertical_velocity_threshold
        stability_ok = stability >= self.stability_threshold
        
        landing_success = uprightness_ok and velocity_ok and vertical_vel_ok and stability_ok
        
        # Create metrics object
        metrics = LandingMetrics(
            timestamp=current_time,
            is_landed=is_landed,
            uprightness_score=uprightness,
            pitch=pitch,
            roll=roll,
            yaw=yaw,
            velocity_at_impact=velocity_mag,
            vertical_velocity=vertical_vel,
            stability_score=stability,
            impact_smoothness=smoothness,
            landing_success=landing_success
        )
        
        return metrics
    
    def evaluate_landing_sequence(self,
                                 car: int,
                                 plane: int,
                                 current_time: float,
                                 pre_landing_metrics: List[LandingMetrics] = None) -> Dict:
        """
        Evaluate the entire landing sequence (before, during, after).
        
        Args:
            car: PyBullet car body ID
            plane: PyBullet plane body ID
            current_time: Current simulation time
            pre_landing_metrics: List of metrics captured before landing
        
        Returns:
            Dictionary with sequence evaluation details
        """
        current_metrics = self.evaluate_landing(car, plane, current_time)
        
        evaluation = {
            'current_metrics': current_metrics,
            'landing_success': current_metrics.landing_success,
            'improvement_score': self._calculate_improvement_score(pre_landing_metrics, current_metrics),
            'recommendations': self._generate_recommendations(current_metrics)
        }
        
        return evaluation
    
    def _calculate_improvement_score(self,
                                    pre_metrics: List[LandingMetrics],
                                    landing_metrics: LandingMetrics) -> float:
        """
        Calculate overall improvement/performance score.
        Compares landing metrics against pre-landing trajectory.
        
        Args:
            pre_metrics: List of metrics before landing
            landing_metrics: Metrics at landing
        
        Returns:
            Score from 0-100
        """
        if not pre_metrics or len(pre_metrics) == 0:
            return landing_metrics.uprightness_score
        
        # Average of key metrics
        score = (
            landing_metrics.uprightness_score * 0.35 +
            landing_metrics.impact_smoothness * 0.35 +
            landing_metrics.stability_score * 0.30
        )
        
        return score
    
    def _generate_recommendations(self, metrics: LandingMetrics) -> List[str]:
        """
        Generate recommendations for improving landing.
        
        Args:
            metrics: Landing metrics
        
        Returns:
            List of recommendation strings
        """
        recommendations = []
        
        if metrics.uprightness_score < self.uprightness_threshold:
            pitch_deg = np.degrees(metrics.pitch)
            roll_deg = np.degrees(metrics.roll)
            if abs(pitch_deg) > abs(roll_deg):
                recommendations.append(f"Reduce pitch tilt (currently {pitch_deg:.1f}°)")
            else:
                recommendations.append(f"Reduce roll tilt (currently {roll_deg:.1f}°)")
        
        if metrics.velocity_at_impact > self.velocity_threshold:
            recommendations.append(f"Reduce impact velocity (currently {metrics.velocity_at_impact:.2f} m/s)")
        
        if abs(metrics.vertical_velocity) > self.vertical_velocity_threshold:
            if metrics.vertical_velocity > 0:
                recommendations.append("Reduce upward velocity at impact")
            else:
                recommendations.append(f"Reduce downward velocity (currently {abs(metrics.vertical_velocity):.2f} m/s)")
        
        if metrics.stability_score < self.stability_threshold:
            recommendations.append("Reduce rotational velocity for more stable landing")
        
        if metrics.impact_smoothness < 50:
            recommendations.append("Smooth the landing trajectory - reduce abrupt changes")
        
        if not recommendations:
            recommendations.append("Landing within acceptable parameters!")
        
        return recommendations


def print_landing_report(metrics: LandingMetrics, evaluator: LandingEvaluator = None):
    """
    Print a formatted landing evaluation report.
    
    Args:
        metrics: LandingMetrics object
        evaluator: LandingEvaluator instance for thresholds (optional)
    """
    print(metrics)
    
    if evaluator:
        recommendations = evaluator._generate_recommendations(metrics)
        print("Recommendations for improvement:")
        for i, rec in enumerate(recommendations, 1):
            print(f"  {i}. {rec}")

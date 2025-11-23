
# ============================================================
#  PID CONTROLLER
# ============================================================
import math


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def reset(self):
        self.integral = 0
        self.prev_error = 0

    def step(self, error):
        self.integral += error
        deriv = error - self.prev_error
        self.prev_error = error
        return self.kp*error + self.ki*self.integral + self.kd*deriv


class CubeController:
    def __init__(self, pid_cfg, limit_front, limit_back, local_center_x):
        self.pid = PID(pid_cfg['kp'], pid_cfg['ki'], pid_cfg['kd'])
        self.target_pitch = pid_cfg['target_pitch']
        self.limit_front = limit_front
        self.limit_back = limit_back
        self.local_center_x = local_center_x
        self.angle_threshold = math.radians(pid_cfg.get('angle_threshold_deg', 10.0))
        self.max_pitch = math.radians(pid_cfg.get('max_pitch_deg', 35.0))
        if self.max_pitch <= self.angle_threshold:
            self.max_pitch = self.angle_threshold + math.radians(5.0)
        print(f"Cube limits - Front: {limit_front:.3f}, Back: {limit_back:.3f}, Center: {local_center_x:.3f}")

    def get_control_action(self, current_pitch, time_step):
        pitch_error = self.target_pitch - current_pitch

        if abs(current_pitch) <= self.angle_threshold:
            shift_cmd = self.pid.step(pitch_error)
        else:
            self.pid.reset()
            sign = 1 if current_pitch > 0 else -1
            span = self.max_pitch - self.angle_threshold
            over = min(abs(current_pitch) - self.angle_threshold, span)
            ratio = over / span if span > 1e-6 else 1.0
            limit = self.limit_front if sign > 0 else self.limit_back
            shift_cmd = sign * limit * (0.5 + 0.5 * ratio)

        cube_shift = max(-self.limit_back, min(self.limit_front, shift_cmd))

        return cube_shift, self.local_center_x
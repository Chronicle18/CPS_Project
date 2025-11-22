
# ============================================================
#  PID CONTROLLER
# ============================================================
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
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

    def get_control_action(self, current_pitch, time_step):
        # Normal PID Control
        # Error = Target - Current (Nose Up is Negative Pitch -> Needs Positive Shift (Front))
        pitch_error = self.target_pitch - current_pitch
        raw_shift = self.pid.step(pitch_error)
        cube_shift = max(-self.limit_back, min(self.limit_front, raw_shift))
            
        return cube_shift, self.local_center_x
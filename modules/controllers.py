

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


class PID2D:
    """2D PID controller for pitch (X-axis) and roll (Y-axis) control."""
    def __init__(self, kp_pitch, ki_pitch, kd_pitch, kp_roll, ki_roll, kd_roll):
        self.pid_pitch = PID(kp_pitch, ki_pitch, kd_pitch)
        self.pid_roll = PID(kp_roll, ki_roll, kd_roll)
    
    def step(self, pitch_error, roll_error):
        """Returns (x_shift, y_shift) for cube positioning."""
        x_shift = self.pid_pitch.step(pitch_error)
        y_shift = self.pid_roll.step(roll_error)
        return x_shift, y_shift
    
    def reset(self):
        self.pid_pitch.integral = 0
        self.pid_pitch.prev_error = 0
        self.pid_roll.integral = 0
        self.pid_roll.prev_error = 0
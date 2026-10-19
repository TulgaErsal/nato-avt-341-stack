#################################################
# pid.py                                        #
# Author: Evan Vandermate (evanderm@mtu.edu)    #
# --------------------------------------------- #
# PID controller implementation                 #
#################################################
import time

class PID():
    def __init__(self, p, i, d, max_i=None):
        self.Kp = p
        self.Ki = i
        self.Kd = d
        self.max_i = max_i
        self.last_error = 0
        self.integral = 0
        self.last_time = None
        self.is_initialized = False

    def initialize(self):
        self.last_time = time.time()
        self.is_initialized = True

    def update(self, setpoint, pv):
        t = time.time()
        dt = 0.0
        if not self.is_initialized:
            self.initialize()
        else:
            # Calculate time step
            dt = t - self.last_time
        
        # Calculate control output
        error = setpoint - pv
        self.integral += error * dt
        if self.max_i is not None:
            self.integral = max(min(self.integral, self.max_i), -self.max_i)
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        control = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Save state values for next iteration
        self.last_time = t
        self.last_error = error

        return control
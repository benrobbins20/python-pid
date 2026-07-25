import time

import numpy as np
from sim.elevator import sim_run
import json
import atexit

# Simulator Options
options = {}
options['FIG_SIZE'] = [8, 8] # [Width, Height]
options['PID_DEBUG'] = False
# Physics Options
options['GRAVITY'] = True
options['FRICTION'] = False
options['ELEVATOR_MASS'] = 1000
options['COUNTERWEIGHT_MASS'] = 1000
options['PEOPLE_MASS'] = 300
# Controller Options
options['CONTROLLER'] = True
options['START_LOC'] = 3
options['SET_POINT'] = 27
options['OUTPUT_GAIN'] = 2000

class Controller:
    def __init__(self, reference):
        self.r = reference # fixed halfway reference point
        self.prev_time = 0
        self.prev_pos = 0
        self.prev_velo = 0
        self.output = 0
        
        self.p_out = 0
        self.i_out = 0
        self.d_out = 0
        
        self.raw_accel = 0
        self.kp = 4
        self.ki = 0
        self.kd = 6.7
        self.max_output = 5
        self.anti_windup = 0.5
        self.integral = 0
        self.cancel_gravity = (9.8 * options["PEOPLE_MASS"]) / options['OUTPUT_GAIN']
        self.gravity_ff = self.max_output * np.arctanh(self.cancel_gravity / self.max_output)
         
    
        self.start_error = options['SET_POINT'] - options['START_LOC']
        
    def run(self, t, x, v):
        position_error = self.r - x
        dt = t - self.prev_time
        self.prev_time = t
        self.p_out = self.kp * position_error
        self.integral += position_error * dt
        self.integral = np.clip(self.integral, -self.anti_windup, self.anti_windup)
        self.i_out = self.ki * self.integral

        self.d_out = -self.kd * v
        raw_output = self.p_out + self.i_out + self.d_out + self.gravity_ff
        self.output = self.max_output * np.tanh(raw_output / self.max_output)
        
        return self.output, raw_output, self.p_out, self.i_out, self.d_out

sim_run(options, Controller)


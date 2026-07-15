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
options['FRICTION'] = True
options['ELEVATOR_MASS'] = 1000
options['COUNTERWEIGHT_MASS'] = 1000
options['PEOPLE_MASS'] = 0
# Controller Options
options['CONTROLLER'] = True
options['START_LOC'] = 0
options['SET_POINT'] = 20 # transition point
options['OUTPUT_GAIN'] = 1000 # elevator plus counterweight to start

class Controller:
    def __init__(self, reference):
        self.r = reference # fixed halfway reference point
        self.prev_time = 0
        self.prev_pos = 0
        self.prev_velo = 0
        self.output = 0
        
        # use these for in between 20hz/4hz calls
        self.p_out = 0
        self.d_out = 0
        
        self.raw_accel = 0
        
        
        self.start_error = options['SET_POINT'] - options['START_LOC']
        
    def run(self, t, x, v):
        position_error = self.r - x
        kp = 2
        kd = 3.2
        max_power = 5.0
        self.p_out = kp * position_error
        self.d_out = -kd * v
        raw_output = self.p_out + self.d_out
        self.output = max_power * np.tanh(raw_output / max_power)
        return self.output, self.p_out, self.d_out

sim_run(options, Controller)


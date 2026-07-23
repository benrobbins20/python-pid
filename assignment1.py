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
        self.kp = 1.6
        self.kd = 3.4
        self.max_output = 5.0
        
        
        self.start_error = options['SET_POINT'] - options['START_LOC']
        
    def run(self, t, x, v):
        position_error = self.r - x
        self.p_out = self.kp * position_error
        self.d_out = -self.kd * v
        raw_output = self.p_out + self.d_out
        self.output = self.max_output * np.tanh(raw_output / self.max_output)
        
        return self.output, self.p_out, self.d_out

sim_run(options, Controller)


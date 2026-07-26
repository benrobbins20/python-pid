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
options['PEOPLE_MASS'] = 200
# Controller Options
options['CONTROLLER'] = True
options['START_LOC'] = 1
options['SET_POINT'] = 27
options['OUTPUT_GAIN'] = 2000

class Controller:
    Kp = 5
    Ki = 0.25
    Kd = 6.7
    g = 9.8
    
    def __init__(self, starting_ref):
        self.r = starting_ref
        self.output = 0
        self.max_output = 5
        self.max_windup = 5
        self.integral = 0
        
        self.weight_ratio = options["ELEVATOR_MASS"] - options["COUNTERWEIGHT_MASS"]
        self.payload_mass = self.weight_ratio + options['PEOPLE_MASS']
        
        # hold force required for the entire payload
        self.gravity_hold = (self.payload_mass * (self.g)) / options['OUTPUT_GAIN'] # normalize output to controller output range
        # clip the gravity hold output to max output of the system
        # this is when all other PID outputs are zero and system needs to hold payload in place
        self.hold_ratio = np.clip(self.gravity_hold / self.max_output, -1, 1)
        # # inverse tanh for ff, reduces math error for tanh saturation
        self.g_feed_forward = self.max_output * np.arctanh(self.gravity_hold / self.max_output)

        self.start_error = options['SET_POINT'] - options['START_LOC']
        
    def run(self, t, x, v, dt=None):
        position_error = self.r - x
        
        # after solver.integrate(), run(is called with dt, which is the signal to update the state)
        if dt is not None:
            self.integral += position_error * dt
            # hard limit to windup bounds
            self.integral = np.clip(self.integral, -self.max_windup, self.max_windup)
    
        self.p_out = self.Kp * position_error
        self.i_out = self.Ki * self.integral
        self.d_out = -self.Kd * v
        
        # complete output system, stuff all outputs and holding gravity output into a single output
        raw_output = self.p_out + self.i_out + self.d_out + self.g_feed_forward
        # soft saturate the controller output
        self.output = self.max_output * np.tanh(raw_output / self.max_output)
        
        return self.output, raw_output, self.p_out, self.i_out, self.d_out

sim_run(options, Controller)


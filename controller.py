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
options['START_LOC'] = 27
options['SET_POINT'] = 3
options['OUTPUT_GAIN'] = 2000

class Controller:
    Kp = 4
    Ki = 0.25
    Kd = 6.7
    g = -9.8
    
    def __init__(self, reference):
        self.r = reference # fixed halfway reference point
        self.output = 0
        
        self.raw_accel = 0
        self.max_output = 5
        self.anti_windup = 0.5
        self.integral = 0
        self.cancel_gravity = (9.8 * options["PEOPLE_MASS"]) / options['OUTPUT_GAIN']
        self.gravity_ff = self.max_output * np.arctanh(self.cancel_gravity / self.max_output)
         
    
        self.start_error = options['SET_POINT'] - options['START_LOC']
        
    # def get_output(self, x, v):
    #     """
    #     pure function for getting output for specific position and velocity, does not update integral, dt, or any state variables
    #     this is done becuase dopri5 will call the physics function many times per evaluation/time step
    #     """
    #     error = self.r - x
        
    #     self.p
        
        
    def run(self, t, x, v, dt=None):
        position_error = self.r - x
        
        # after solver.integrate(), run(is called with dt, which is the signal to update the state)
        if dt is not None:
            self.integral += position_error * dt
            self.integral = np.clip(self.integral, -self.anti_windup, self.anti_windup)
    
        self.p_out = self.Kp * position_error
        self.i_out = self.Ki * self.integral
        self.d_out = -self.Kd * v
        
        raw_output = self.p_out + self.i_out + self.d_out + self.gravity_ff
        self.output = self.max_output * np.tanh(raw_output / self.max_output)
        
        return self.output, raw_output, self.p_out, self.i_out, self.d_out

sim_run(options, Controller)


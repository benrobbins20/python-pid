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
        # block to control only 20hz
        # continuously polled but only if time diff is > 0.05 
        # return 4
        if t - self.prev_time < 0.01:
            return self.output, self.p_out, self.d_out
        else:
            # distance away, +error is under set point, -error is over set point 
            position_error = self.r - x
            direction = 1 if position_error > 0 else -1
            dist = abs(position_error)
            dt = t - self.prev_time
            actual_velo = (x - self.prev_pos) / dt
            desired_velo = (position_error/self.start_error)
            velocity_error = desired_velo - actual_velo
            self.raw_accel = (actual_velo - self.prev_velo) / dt
            max_power = 5 # ceiling at least
        
            kp = 1.5
            kd = 2.5
            
            p_out = position_error * kp
            
            # de/dt (ref - pos), ref rate of change is 0, so dr/dt - dx/dt = -dx/dt = -actual_velo
            # use the constant to store the negative velo 
            d_out = -kd * actual_velo
            self.output = p_out + d_out
            
            # no I for now
            I_out = 0
            
            # tanh is a range -1:1, it will provide granually scaled output(eg. 0.9->0.999999) for max_power
            # "soft saturation"
            self.output = max_power * np.tanh(self.output / max_power)
            
            # print(
            #     f"x={x:.2f} t={t:.2f} desired_v={desired_velo:.2f} "
            #     f"act_v={actual_velo:.2f} "
            #     f"des_v={desired_velo:.2f} "
            #     f"v_err={velocity_error:.2f} "
            #     f"a={accel:.2f}"
            # )
            
            
            self.prev_time = t
            self.prev_pos = x
            self.prev_velo = actual_velo
            self.p_out = p_out
            self.d_out = d_out
            
            return self.output, self.p_out, self.d_out

sim_run(options, Controller)


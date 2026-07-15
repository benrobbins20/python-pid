
import numpy as np
from sim.elevator import sim_run
# Simulator Options
options = {}
options['FIG_SIZE'] = [8, 8] # [Width, Height]
options['PID_DEBUG'] = False
# Physics Options
options['GRAVITY'] = True
options['FRICTION'] = False
options['ELEVATOR_MASS'] = 500
options['COUNTERWEIGHT_MASS'] = 500
options['PEOPLE_MASS'] = 0
# Controller Options
options['CONTROLLER'] = True
options['START_LOC'] = 0
options['SET_POINT'] = 15 # transition point
options['OUTPUT_GAIN'] = 1000 # elevator plus counterweight to start

class Controller:
    def __init__(self, reference):
        self.r = reference # fixed halfway reference point
        self.prev_time = 0
        self.prev_x = 0
        self.output = 0
    def run(self, x, t):
        # block to control only 20hz
        # continuously polled but only if time diff is > 0.05 
        if t - self.prev_time < 0.05:
            return self.output
        else:
            if (abs(x-self.r) < 0.5):
                print(f"hit setpoint {self.r}:{x}:{t}")
                
            # distance away, +error is under set point, -error is over set point 
            error = self.r - x
            direction = 1 if error > 0 else -1
            self.prev_time = t
            self.prev_x = x
            
            # speed gain
            fast = 4
            slow = 1
            # relative distance from setpoint
            far = 20
            close = 10
            stop = 2
            
            
            slope = (fast - slow) / (far - close)
            
            # furthest away, full output
            if abs(error) > far:
                self.output = fast * direction
            elif abs(error) > close:
                # linear acceleration from 19-10
                _y = (abs(error) - close) * slope + slow
                self.output = (_y * direction)
            elif abs(error) > stop:
                self.output = slow * direction
            else:
                # no power, just let gravity move elevator
                self.output = 0
            
            
            # if x < self.r:
            #     self.output = 10
            # else:
            #     self.output = -10
                
            # self.output = 4
            # INSERT CODE ABOVE.
            return self.output
sim_run(options, Controller)

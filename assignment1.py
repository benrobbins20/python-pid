
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
options['SET_POINT'] = 20 # transition point
options['OUTPUT_GAIN'] = 1000 # elevator plus counterweight to start

class Controller:
    def __init__(self, reference):
        self.r = reference # fixed halfway reference point
        self.prev_time = 0
        self.prev_pos = 0
        self.prev_velo = 0
        self.output = 0
    def run(self, x, t):
        # block to control only 20hz
        # continuously polled but only if time diff is > 0.05 
        if t - self.prev_time < 0.05:
            return self.output
        else:
            # distance away, +error is under set point, -error is over set point 
            position_error = self.r - x
            direction = 1 if position_error > 0 else -1
            dist = abs(position_error)
            
            # speed gain
            fast = 4
            slow = 1
            # relative distance from setpoint
            far = 10
            close = 5
            stop = 1
            
            # linear slow down in the slowing range
            slope = (fast - slow) / (far - close)
            
            # 
            if dist > far:
                desired_velo = fast * direction
            elif dist > close:
            
                target_speed = (dist - close) * slope + slow
                desired_velo = (target_speed * direction)
            elif dist > stop:
                desired_velo = slow * direction
            elif dist > 0.01:
                desired_velo = 0.1 * direction
            else:
                desired_velo = 0
                
            # dt = 0.05?
            DT = 0.05
            actual_velo = (x - self.prev_pos) / (DT)
            
            velocity_error = desired_velo - actual_velo
            self.output = velocity_error
            
            accel = (actual_velo - self.prev_velo) / DT
            
            self.prev_time = t
            self.prev_pos = x
            self.prev_velo = actual_velo
            
            # print(f"t:{t:.2f} x:{x:.2f} v:{actual_velo:.2f} a:{accel:.2f}")
            print(
                f"x={x:.2f} desired_v={desired_velo:.2f} "
                f"actual_v={actual_velo:.2f} "
                f"v_err={velocity_error:.2f} "
                f"return_desired={desired_velo:.2f} "
                f"return_error={velocity_error:.2f}"
            )
            # return self.output
            return desired_velo
sim_run(options, Controller)

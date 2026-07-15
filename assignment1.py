import time

from mcap.writer import Writer
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

class McapWriter:
    def __init__(self, path):
        self.file = open(path, "wb")
        self.writer: Writer = Writer(self.file)
        self.writer.start()
        self.channels = {}
        
    def register(self, topic, fields):
        schema_id = self.writer.register_schema(
            name=topic.strip("/"),
            encoding="jsonschema",
            # 
            data=json.dumps({
                "type": "object", # 'object' = dict
                "properties": {k: {"type": v} for k, v in fields.items()} # dict{str,dict} properties is k-type:v-'number'
            }).encode()
        )
        self.channels[topic] = self.writer.register_channel(
            topic=topic,
            message_encoding="json",
            schema_id=schema_id
        )
    
    def write(self, topic, time, data):
        ns = int(time*1e9)
        self.writer.add_message(
            channel_id=self.channels[topic],
            log_time=ns,
            publish_time=ns,
            data=json.dumps(data).encode(),
        )
    def close(self):
        self.writer.finish()
        self.file.close()
    
    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
class Controller:
    def __init__(self, reference):
        self.r = reference # fixed halfway reference point
        self.prev_time = 0
        self.prev_pos = 0
        self.prev_velo = 0
        self.output = 0
        timestamp = time.time()
        self.file_path = f"des_vel_{timestamp}.mcap"
        self.mcap_writer = McapWriter(self.file_path)
        atexit.register(self.mcap_writer.close)
        self.start_error = options['SET_POINT'] - options['START_LOC']
        self.output_data = np.array([[0,0,0,0]])
        
        self.mcap_writer.register(
            topic="/elevator",
            fields={
                "time": "number",
                "position": "number",
                "position_err": "number",
                "desired_velo": "number",
                "actual_velo": "number",
                "velocity_err": "number",
                "acceleration": "number",
                "output": "number",
                "p_out": "number",
                "d_out": "number"
                
            }
        )
        
        
    def run(self, x, t):
        # block to control only 20hz
        # continuously polled but only if time diff is > 0.05 
        # return 4
        if t - self.prev_time < 0.01:
            return self.output
        else:
            # distance away, +error is under set point, -error is over set point 
            position_error = self.r - x
            direction = 1 if position_error > 0 else -1
            dist = abs(position_error)
            dt = t - self.prev_time
            actual_velo = (x - self.prev_pos) / dt
            desired_velo = (position_error/self.start_error)
            velocity_error = desired_velo - actual_velo
            accel = (actual_velo - self.prev_velo) / dt
            max_power = 5 # ceiling at least
        
            kp = 1.5
            kd = 2.5
            
            P_out = position_error * kp
            
            # de/dt (ref - pos), ref rate of change is 0, so dr/dt - dx/dt = -dx/dt = -actual_velo
            # use the constant to store the negative velo 
            D_out = -kd * actual_velo
            self.output = P_out + D_out
            
            # no I for now
            I_out = 0
            
            # tanh is a range -1:1, it will provide granually scaled output(eg. 0.9->0.999999) for max_power
            # "soft saturation"
            self.output = max_power * np.tanh(self.output / max_power)
            
            print(
                f"x={x:.2f} t={t:.2f} desired_v={desired_velo:.2f} "
                f"act_v={actual_velo:.2f} "
                f"des_v={desired_velo:.2f} "
                f"v_err={velocity_error:.2f} "
                f"a={accel:.2f}"
            )
            
            self.mcap_writer.write(
                topic="/elevator",
                time=t,
                data={
                    "time": t,
                    "position": x,
                    "position_err": position_error,
                    "desired_velo": desired_velo,
                    "actual_velo": actual_velo,
                    "velocity_err": velocity_error,
                    "acceleration": accel,
                    "output": self.output,
                    "p_out": P_out,
                    "d_out": D_out
                }
            )
            
            self.output_data = np.concatenate((
                self.output_data,
                np.array([[t, P_out, I_out, D_out]])
            ))
            
            self.prev_time = t
            self.prev_pos = x
            self.prev_velo = actual_velo
            
            return self.output

sim_run(options, Controller)


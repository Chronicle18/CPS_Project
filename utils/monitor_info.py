import pybullet as p
import numpy as np
import math

class MonitorInfo:
    def __init__(self, cfg):
        self.cfg = cfg
        self.total_airtime = 0.0
        self.task_states = {"ascend" : False, "launched" : False, "landed" : False}

        # Airtime tracking
        self.airborne_start_time = None
        self.is_airborne = False

        # landing info
        self.wheel_landing_times = {} # {"front_left" : None, "front_right" : None, "rear_left" : None, "rear_right" : None}
        self.landing_pitch = None
        self.landing_roll = None
    
    def hasLanded(self, car, plane, ramp, curr_timestep):
        car_pos, _ = p.getBasePositionAndOrientation(car)
        ramp_end = (self.cfg['ramp']['length'] * np.cos(self.cfg['ramp']['angle_rad'])) + self.cfg['ramp']['position'][0]  # (x only)
        # print(ramp_end)

        on_ramp = len(p.getContactPoints(car, ramp)) > 0
        on_plane = len(p.getContactPoints(car, plane)) > 0
        
        wheel_touch_plane = False
        for wheel_joint in (self.cfg['car']['front_whls'] + self.cfg['car']['rear_whls']):
            if len(p.getContactPoints(car, plane, linkIndexA=wheel_joint)) > 0:
                wheel_touch_plane = True
                break

        # Car is still climbing the ramp
        if on_ramp:
            self.task_states["ascend"] = True
            self.task_states["launched"] = False
            self.no_ramp_contact_counter = 0

        # Check for launch AFTER crossing ramp end
        if self.task_states["ascend"] and not self.task_states["launched"]:
            # Car front has passed ramp edge
            passed_ramp_end = car_pos[0] > ramp_end  

            no_ramp_contact = not on_ramp
            if no_ramp_contact:
                self.no_ramp_contact_counter += 1
            else:
                self.no_ramp_contact_counter = 0

            # Launch confirmed only when both conditions hold for 3+ frames
            if passed_ramp_end and self.no_ramp_contact_counter > 3:
                self.task_states["launched"] = True

        # Landing detection
        if self.task_states["ascend"] and self.task_states["launched"] and wheel_touch_plane:
            if not self.task_states["landed"]:
                self.task_states["landed"] = True
                # Record landing times for all wheels touching the plane
                for wheel_joint in (self.cfg['car']['front_whls'] + self.cfg['car']['rear_whls']):
                    contacts = p.getContactPoints(car, plane, linkIndexA=wheel_joint)
                    if len(contacts) > 0 and self.wheel_landing_times.get(wheel_joint) is None:
                        self.wheel_landing_times[wheel_joint] = curr_timestep
                        print(f"[{curr_timestep}] Wheel {wheel_joint} landed.")
                return True
        
        if self.task_states["landed"] == True:
            for wheel_joint in (self.cfg['car']['front_whls'] + self.cfg['car']['rear_whls']):
                contacts = p.getContactPoints(car, plane, linkIndexA=wheel_joint)
                if len(contacts) > 0 and self.wheel_landing_times.get(wheel_joint) is None:
                    self.wheel_landing_times[wheel_joint] = curr_timestep
                    print(f"[{curr_timestep}] Wheel {wheel_joint} landed.")

        return False
    
    @staticmethod
    def get_pos_orn(car):
        car_pos, car_orn = p.getBasePositionAndOrientation(car)
        roll, pitch, yaw = p.getEulerFromQuaternion(car_orn)
        return car_pos, roll, pitch, yaw
    
    @staticmethod
    def get_velocity(car):
        car_vel, car_ang_vel = p.getBaseVelocity(car)
        return np.linalg.norm(car_vel), car_ang_vel
    
    def isWheelOnGround(self, car, plane):
        pass
    
    def check_airborne_status(self, car, plane, curr_timestep):
        current_time = curr_timestep * self.cfg['simulation']['time_step']

        wheel_contacts = []
        for wheel_joint in (self.cfg['car']['front_whls'] + self.cfg['car']['rear_whls']): 
            contacts = p.getContactPoints(car, plane, linkIndexA=wheel_joint)
            if contacts is not None:  # Safety check
                wheel_contacts.extend(contacts)

        was_airborne = self.is_airborne
        self.is_airborne = len(wheel_contacts) == 0

        # Track airtime
        if self.is_airborne and not was_airborne:
            self.airborne_start_time = current_time
            # print(f"[{current_time:.2f}s] Airborne!")
        elif not self.is_airborne and was_airborne:
            if self.airborne_start_time is not None:
                flight_duration = current_time - self.airborne_start_time
                self.total_airtime += flight_duration
                # print(f"[{current_time:.2f}s] Landed! Flight: {flight_duration:.2f}s")
                self.airborne_start_time = None
        
        # Calculate current airtime
        if self.is_airborne and self.airborne_start_time is not None:
            current_airtime = current_time - self.airborne_start_time
        else:
            current_airtime = 0.0
        
        return current_airtime 

    def record_wheel_landings(self, car, plane, curr_timestep):
        pass
    
    def evaluate_episode(self):
        # orientation while landing
        pitch_error = abs(self.landing_pitch) # ground pitch should be zero
        roll_error = abs(self.landing_roll)
        pitch_score = 2 if pitch_error < np.radians(5) else 1 if pitch_error < np.radians(15) else 0
        roll_score = 2 if roll_error < np.radians(4) else 1 if roll_error < np.radians(10) else 0
        orientation_score = (pitch_score + roll_score) / 4
        print(f"Landing orientation score: {orientation_score*100:.2f}%")

        # time step difference between landing of all wheels
        	# •	Small ∆t ⇒ smooth, even landing
	        # •	Large ∆t ⇒ one wheel slams first (bad)
        if len(self.wheel_landing_times) == 4:
            delta_t = max(self.wheel_landing_times.values()) - min(self.wheel_landing_times.values())
            k = 0.3
            wheel_touch_score = math.exp(-delta_t*k) # keeps score between 0 and 1 (higher is greater)
        else:
            print("All wheels did not land")
            wheel_touch_score = -2
        
        print(f"Wheel touch score: {wheel_touch_score*100:.2f}")


        # normal forces on wheels

        # total airtime
        
        pass
import time
import math
import threading
from pymavlink import mavutil

class DroneState:
    """Snapshot of drone position/orientation at a specific time."""
    def __init__(self, lat, lon, alt, roll, pitch, yaw, timestamp):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.timestamp = timestamp

class MavlinkDrone:
    """
    Unified hardware abstraction layer for Drone control via PyMAVLink.
    Compatible with Pixhawk and Crossflight FCs.
    """
    def __init__(self, connection_string, baud=57600):
        self.lock = threading.Lock()
        print(f"-- [DRONE] Connecting to {connection_string} at {baud} baud...")
        try:
            self.master = mavutil.mavlink_connection(connection_string, baud=baud)
            self.master.wait_heartbeat(timeout=10)
            print(f"-- [DRONE] Connected! System ID: {self.master.target_system}")
            self.connected = True
        except Exception as e:
            print(f"-- [DRONE] Connection failed: {e}")
            self.connected = False
            return

        # State storage
        self.running = True
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0      # Relative altitude (AGL)
        self.abs_alt = 0.0  # AMSL
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0
        
        self.mission_current = 0
        self.mission_total = 0
        self.armed = False

        # Start telemetry thread
        self.t = threading.Thread(target=self._update_loop, daemon=True)
        self.t.start()

    def _update_loop(self):
        while self.running:
            try:
                msg = self.master.recv_match(blocking=True, timeout=1.0)
                if not msg:
                    continue
                
                type_ = msg.get_type()
                
                with self.lock:
                    if type_ == 'GLOBAL_POSITION_INT':
                        self.lat = msg.lat / 1e7
                        self.lon = msg.lon / 1e7
                        self.alt = msg.relative_alt / 1000.0
                        self.abs_alt = msg.alt / 1000.0
                        self.vx = msg.vx / 100.0
                        self.vy = msg.vy / 100.0
                        self.vz = msg.vz / 100.0
                        
                    elif type_ == 'ATTITUDE':
                        self.roll = math.degrees(msg.roll)
                        self.pitch = math.degrees(msg.pitch)
                        self.yaw = math.degrees(msg.yaw)
                        self.yaw_rate = math.degrees(msg.yawspeed)
                        
                    elif type_ == 'HEARTBEAT':
                        self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                        
                    elif type_ == 'MISSION_CURRENT':
                        self.mission_current = msg.seq

            except Exception as e:
                time.sleep(0.1)

    def get_state(self):
        """Returns the current DroneState, velocity vector, yaw rate, and absolute altitude."""
        with self.lock:
            state = DroneState(self.lat, self.lon, self.alt, self.roll, self.pitch, self.yaw, time.time())
            return state, (self.vx, self.vy, self.vz), self.yaw_rate, self.abs_alt

    def set_mode(self, mode_name):
        mode_id = self.master.mode_mapping().get(mode_name.upper())
        if mode_id is None:
            print(f"-- [DRONE] Unknown mode: {mode_name}")
            return False

        print(f"-- [DRONE] Switching to {mode_name.upper()} mode...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        return True

    def arm(self):
        print("-- [DRONE] Sending ARM command...")
        self.master.arducopter_arm()
        return self.master.motors_armed_wait()

    def disarm(self):
        print("-- [DRONE] Sending DISARM command...")
        self.master.arducopter_disarm()

    def takeoff(self, altitude):
        print(f"-- [DRONE] Taking off to {altitude}m...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, altitude
        )

    def upload_mission(self, waypoints, flight_altitude=12.0):
        """
        Uploads a list of (lon, lat) waypoints as a mission.
        """
        print(f"-- [DRONE] Uploading Mission with {len(waypoints)} items...")
        
        self.master.mav.mission_clear_all_send(self.master.target_system, self.master.target_component)
        self.master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=2) 

        count = len(waypoints)
        self.master.mav.mission_count_send(self.master.target_system, self.master.target_component, count)

        for i, wp in enumerate(waypoints):
            msg = self.master.recv_match(type=['MISSION_REQUEST'], blocking=True, timeout=5)
            if not msg or msg.seq != i:
                print(f"-- [DRONE] Error: Expected request for seq {i}, got {msg}")
                return False

            lon, lat = wp 
            
            self.master.mav.mission_item_int_send(
                self.master.target_system,
                self.master.target_component,
                i,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1, # current, autocontinue
                0, 0, 0, float('nan'),
                int(lat * 1e7),
                int(lon * 1e7),
                flight_altitude
            )

        ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print("-- [DRONE] Mission Uploaded Successfully!")
            self.mission_total = count
            return True
        else:
            print(f"-- [DRONE] Mission Upload Failed! ACK: {ack}")
            return False

    def start_mission_cmd(self):
        print("-- [DRONE] Starting Mission (AUTO Mode)...")
        self.set_mode("AUTO")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
    
    def wait_gps(self):
        print("-- [DRONE] Waiting for GPS lock...")
        while self.lat == 0 and self.lon == 0:
            time.sleep(0.5)
        print("-- [DRONE] GPS Lock Acquired.")

    def return_to_launch(self):
        self.set_mode("RTL")

    def set_servo(self, servo_instance, pwm_value):
        """Actuate a specific servo to a PWM value."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            183, # MAV_CMD_DO_SET_SERVO
            0,   
            servo_instance,
            pwm_value,
            0, 0, 0, 0, 0
        )

    def play_tune(self, tune_string):
        """Play an MML string on the drone buzzer."""
        try:
            if hasattr(self.master.mav, 'play_tune_v2_send'):
                self.master.mav.play_tune_v2_send(
                    self.master.target_system,
                    self.master.target_component,
                    1, # MML_MODERN
                    tune_string.encode('ascii')
                )
        except Exception as e:
            print(f"-- [DRONE] Buzzer Error: {e}")

    def close(self):
        self.running = False
        self.master.close()

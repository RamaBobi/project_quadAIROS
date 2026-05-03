# gcs/telemetry.py
import time
import csv
from pymavlink import mavutil
from pathlib import Path

class Telemetry:
    def __init__(self, drone_id, conn, shared_state):
        self.drone_id = drone_id
        self.conn = conn
        self.shared_state = shared_state
        self.running = True
        
        # Setup CSV Logging for data extraction
        drone_int = int(drone_id.split('_')[1])
        self.log_file = Path(f"run/agent_{drone_int}/{drone_id}_telemetry.csv")
        # Write headers if file doesn't exist
        if not self.log_file.exists():
            with open(self.log_file, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "timestamp", "mode", "armed",
                    "lat", "lon", "alt", "heading",
                    "x", "y", "z", "vx", "vy", "vz",
                    "gps_fix", "gps_sats", "battery"
                ])
        
        print(f"[{drone_id}] Telemetry logger initialized → {self.log_file}")
        


    def listen(self):
        while self.running:
            try:
                # blocking=False means if no packet is here, it instantly drops the GIL token
                msg = self.conn.recv_match(blocking=False)
                if not msg:
                    time.sleep(0.01) # Sleep to yield CPU token
                    continue
                    
                msg_type = msg.get_type()
                
                # Extract and update shared memory
                if msg_type == 'HEARTBEAT':
                    self.shared_state[self.drone_id]['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.shared_state[self.drone_id]['mode'] = mavutil.mode_string_v10(msg)
                    self._log_data()

                elif msg_type == 'GLOBAL_POSITION_INT':
                    self.shared_state[self.drone_id]['lat']     = msg.lat / 1e7
                    self.shared_state[self.drone_id]['lon']     = msg.lon / 1e7
                    self.shared_state[self.drone_id]['alt']     = msg.relative_alt / 1000.0
                    self.shared_state[self.drone_id]['heading'] = msg.hdg / 100.0
                    self._log_data()

                elif msg_type == 'LOCAL_POSITION_NED':
                    self.shared_state[self.drone_id]['x']  = msg.x
                    self.shared_state[self.drone_id]['y']  = msg.y
                    self.shared_state[self.drone_id]['z']  = msg.z
                    self.shared_state[self.drone_id]['vx'] = msg.vx
                    self.shared_state[self.drone_id]['vy'] = msg.vy
                    self.shared_state[self.drone_id]['vz'] = msg.vz

                elif msg_type == 'GPS_RAW_INT':
                    self.shared_state[self.drone_id]['gps_fix']  = msg.fix_type
                    self.shared_state[self.drone_id]['gps_sats'] = msg.satellites_visible

                elif msg_type == 'BATTERY_STATUS':
                    self.shared_state[self.drone_id]['battery'] = msg.battery_remaining  # percent                    
            except Exception as e:
                print(f"\n[{self.drone_id}] Telemetry error: {e}")
                break

    def _log_data(self):
        """Writes the current state to the hard drive for extraction."""
        s = self.shared_state[self.drone_id]
        with open(self.log_file, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                time.strftime('%H:%M:%S', time.localtime()),
                s['mode'], s['armed'],
                s['lat'], s['lon'], s['alt'], s['heading'],
                s['x'], s['y'], s['z'],
                s['vx'], s['vy'], s['vz'],
                s['gps_fix'], s['gps_sats'],
                s['battery']
            ])

    def stop(self):
        self.running = False

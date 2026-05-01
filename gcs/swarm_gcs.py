import time
import threading
import cmd
import subprocess
from pymavlink import mavutil

class SwarmGCS(cmd.Cmd):
    intro = "\033[92mDynamic Swarm GCS Initialized. Type 'help' to see commands.\033[0m"
    prompt = "(Swarm) "

    def __init__(self):
        super().__init__()
        # Dynamic Storage
        self.drones = {}         # Stores the mavutil connections
        self.state = {}          # Stores the live telemetry data
        self.thread_flags = {}   # Thread kill-switches
        self.drone_counter = 0   # Keeps labels unique (drone_0, drone_1...)
        self.mav_processes = {}
        self.drone_list = {}

    def telemetry_listener(self, drone_id, conn):
        """Background thread. Exits cleanly when thread_flags[drone_id] is False."""
        while self.thread_flags.get(drone_id, False):
            try:
                msg = conn.recv_match(blocking=False)
                if not msg:
                    time.sleep(0.01)
                    continue
                    
                msg_type = msg.get_type()
                if msg_type == 'HEARTBEAT':
                    self.state[drone_id]['armed'] = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    self.state[drone_id]['mode'] = mavutil.mode_string_v10(msg)
                elif msg_type == 'GLOBAL_POSITION_INT':
                    self.state[drone_id]['alt'] = msg.relative_alt / 1000.0
            except Exception:
                # Socket was closed or thread was killed
                break
                
        print(f"\n[{drone_id}] Telemetry thread terminated.")

    def _register_drone(self, port):
        """Internal function to safely bind, verify, and register a new drone."""
        # Prevent double-connecting to the same port
        for d_id, state in self.state.items():
            if state.get('port') == port:
                print(f"Error: Already connected to port {port} as {d_id}")
                return False

        print(f"Testing UDP port {port}...")
        try:
            conn = mavutil.mavlink_connection(f'udp:127.0.0.1:{port}')
            msg = conn.wait_heartbeat(timeout=3.0)
            
            if msg:
                drone_id = f"drone_{self.drone_counter}"
                self.drone_counter += 1
                
                self.drones[drone_id] = conn
                self.state[drone_id] = {"mode": "UNKNOWN", "armed": False, "alt": 0.0, "port": port}
                self.thread_flags[drone_id] = True
                
                # Boot the isolated telemetry thread
                t = threading.Thread(target=self.telemetry_listener, args=(drone_id, conn), daemon=True)
                t.start()
                
                self.add_mav(drone_id, port)

                print(f"\033[92mSuccess:\033[0m Registered {drone_id} on port {port}")
                return True
            else:
                print(f"Timeout: No MAVLink heartbeat detected on port {port}.")
                conn.close()
                return False
        except Exception as e:
            print(f"Failed to connect to {port}: {e}")
            return False

    # --- MAVProxy Wrapper ---
    def add_mav(self, drone_id, port):

        process = subprocess.Popen(
            ["mavproxy.py", f"--master=udp:127.0.0.1:{port}", "--cmd=set echo off"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        self.mav_processes[drone_id] = process
        print(f"[{drone_id}] Headless MAVProxy connected on {port}")

    # --- CLI COMMANDS ---
    def do_MAVcommand(self, arg):
    
        target  = arg.split()[0]
        command_string = " ".join(arg.split()[1:])  

        if target == "all":
            drone_list = list(self.mav_processes.keys())
        else:
            drone_list = [target]

        for drone_id in drone_list:
            if drone_id in self.mav_processes:

                process = self.mav_processes[drone_id]

                try:
                    process.stdin.write(command_string + "\n")
                    process.stdin.flush()
                    print(f"[{drone_id}] Executed: {command_string}")
                except Exception as e:
                    print(f"{drone_id} Failed to send command: {e}")

            else:
                print(f"Error: {drone_id} not found")


    def do_add(self, arg):
        """
        Add a drone to the swarm.
        Usage 1: add auto  (Scans standard SITL ports 14550 - 14600)
        Usage 2: add 14560 (Connects to a specific UDP port)
        """
        if arg.strip() == "auto":
            print("Initiating Auto-Discovery sweep...")
            found = 0
            # Sweep ports 14550, 14560, 14570, 14580, 14590, 14600
            for port in range(14550, 14601, 10):
                if self._register_drone(port):
                    found += 1
            print(f"Auto-Discovery complete. Added {found} new drones.")
        
        elif arg.isdigit():
            self._register_drone(int(arg))
        
        else:
            print("Error: Invalid argument. Use 'add auto' or 'add [port_number]'")

        

    def do_remove(self, arg):
        """
        Safely remove a drone and kill its background processes.
        Usage: remove drone_0
        """
        target = arg.strip()
        if target not in self.drones:
            print(f"Error: {target} not found in active swarm.")
            return

        print(f"Initiating shutdown sequence for {target}...")
        
        # 1. Flip the kill-switch for the background thread
        self.thread_flags[target] = False
        
        # 2. Close the socket connection
        self.drones[target].close()
        
        # 3. Purge from memory dictionaries
        del self.drones[target]
        del self.state[target]
        del self.thread_flags[target]
        
        print(f"{target} successfully purged from swarm logic.")

    def do_status(self, arg):
        """View live swarm status."""
        if not self.state:
            print("Swarm is empty. Use 'add auto' to find drones.")
            return
            
        print("-" * 50)
        for d_id, state in self.state.items():
            arm_str = "\033[91mARMED\033[0m" if state['armed'] else "DISARMED"
            print(f"[{d_id}] Port: {state['port']} | Mode: {state['mode']:<10} | {arm_str:<15} | Alt: {state['alt']:.1f}m")
        print("-" * 50)

    def do_exit(self, arg):
        """Safely exit the GCS and kill all threads."""
        print("Executing global swarm disconnect...")
        for drone_id in list(self.drones.keys()):
            self.do_remove(drone_id)
        return True

if __name__ == '__main__':
    SwarmGCS().cmdloop()

# gcs/cli.py
import cmd
from pymavlink import mavutil
import subprocess

class SwarmCLI(cmd.Cmd):
    intro = "\033[92mSwarm GCS Modular CLI. Type 'help'.\033[0m"
    prompt = "(Swarm) "

    def __init__(self, gcs):
        super().__init__()
        # It receives pointers to the memory created in main.py
        self.gcs = gcs
        
        self.mav_processes = gcs.mav_processes
        self.thread_flags = gcs.thread_flags
        self.connections = gcs.connections 
        self.state = gcs.shared_state

        self.drone_counter = 0


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
               
                self.add_mav(drone_id, port)

                conn = mavutil.mavlink_connection(f'udp:127.0.0.1:{port+100}')

                self.connections[drone_id] = conn
                self.state[drone_id] = {
                        "mode": "UNKNOWN", "armed": False, 
                        "lat":0.0, "lon":0.0, "alt": 0.0, "heading":0.0,
                        "x": 0.0, "y": 0.0, "z": 0.0,
                        "vx": 0.0, "vy": 0.0, "vz": 0.0,
                        "gps_fix": 0, "gps_sats": 0,
                        "battery": -1,
                        "port": port}
                self.thread_flags[drone_id] = True
                
                self.gcs.start_telemetry(drone_id)
                

                print(f"\033[92mSuccess:\033[0m Registered {drone_id} on port {port}")
                return True
            else:
                print(f"Timeout: No MAVLink heartbeat detected on port {port}.")
                conn.close()
                return False
        except Exception as e:
            print(f"Failed to connect to {port}: {e}")
            return False

    def add_mav(self, drone_id, port):

        process = subprocess.Popen(
            ["mavproxy.py", 
             f"--master=udp:127.0.0.1:{port}", 
             f"--out=udp:127.0.0.1:{port+100}",
             "--cmd=set echo off"],
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


    def do_status(self, arg):
        """View live swarm status extracted by background threads."""
        if not self.state:
            print("Swarm state is empty.")
            return
            
        print("-" * 50)
        for d_id, state in self.state.items():
            arm_str = "\033[91mARMED\033[0m" if state['armed'] else "DISARMED"
            print(f"[{d_id}] Mode: {state['mode']:<10} | {arm_str:<15} | Alt: {state['alt']:.1f}m")
        print("-" * 50)

    def do_arm(self, arg):
        """Arm a drone. Usage: arm drone_0 or arm all"""
        targets = list(self.connections.keys()) if arg == "all" else [arg]
        
        for d_id in targets:
            if d_id in self.connections:
                conn = self.connections[d_id]
                conn.mav.command_long_send(
                    conn.target_system, conn.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 0, 0, 0, 0, 0, 0
                )
                print(f"[{d_id}] ARM command sent.")

    def do_exit(self, arg):
        """Safely exit."""
        return True


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
        if target not in self.connections:
            print(f"Error: {target} not found in active swarm.")
            return

        print(f"Initiating shutdown sequence for {target}...")
        self.thread_flags[target] = False
        self.connections[target].close()
        del self.connections[target]
        del self.state[target]
        del self.thread_flags[target]
        print(f"{target} successfully purged from swarm logic.")



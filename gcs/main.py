# gcs/main.py
import threading
from pymavlink import mavutil
from gcs.telemetry import Telemetry
from gcs.cli import SwarmCLI
from scripts.swarm_spawner import SwarmSpawner

class SwarmGCS:
    def __init__(self):
        self.connections = {}
        self.shared_state = {}
        self.thread_flags = {}
        self.mav_processes = {}

        self.Telemetry_all = {}

        #print("Connecting to Swarm Infrastructure...")
        #for i in range(num_drones):
            #port = 14550 + (i * 10)
            #drone_id = f"drone_{i}"
            #
            ## Connect UDP Socket
            #conn = mavutil.mavlink_connection(f'udp:127.0.0.1:{port}')
            #self.connections[drone_id] = conn
            #
            ## Initialize shared state dictionary
            #self.shared_state[drone_id] = {"mode": "UNKNOWN", "armed": False, "alt": 0.0}
            #
            ## 2. Spawn the Extraction Threads
            #extractor = TelemetryExtractor(drone_id, conn, self.shared_state)
            #self.extractors[drone_id] = extractor
            #
            #t = threading.Thread(target=extractor.listen, daemon=True)
            #self.threads.append(t)
            #t.start()
            #print(f"[{drone_id}] Extraction thread active on port {port}.")

    def start_telemetry(self, drone_id):
        if self.thread_flags[drone_id]:
            Telemetry_id = Telemetry(drone_id, self.connections[drone_id], self.shared_state)
            self.Telemetry_all[drone_id] = Telemetry_id

            t = threading.Thread(target= Telemetry_id.listen, daemon=True)
            t.start()
            

    def start_cli(self):
        try:
            cli = SwarmCLI(self)
            cli.cmdloop()
        except KeyboardInterrupt:
            print("\nForce quitting...")
        finally:
            print("Shutting down extraction threads...")
            for ext in self.Telemetry_all.values():
                ext.stop()

if __name__ == '__main__':
    #dim = {
            #'M': 1,
            #'N': 2,
            #'D': 2.0
            #}
#
    #spawner_thread = threading.Thread(target=lambda: SwarmSpawner(dim), daemon=True)
    #spawner_thread.start()
    #
    GCS = SwarmGCS()
    GCS.start_cli()

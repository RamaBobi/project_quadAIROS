import os
import time
import subprocess
import math
from pathlib import Path

class SwarmSpawner:
    def __init__(self, dim):
        super().__init__()
   
        self.PROJECT_ROOT = Path(__file__).resolve().parent.parent


        self.process = []
        self.spawn_swarm(dim)
        
    
    def __del__(self):
        for proc in self.process:
            proc.terminate()


    def generate_instance_sdf(self, instance_id, base_sdf, base_gimbal_sdf):
        # Gimbal SDF 
        target_port_in = 9002 + instance_id*10
        target_port_out = 9003 + instance_id*10
        new_sdf_path = f"/tmp/iris_swarm_{instance_id}.sdf"
        new_sdf_gimbal_path = f"/tmp/gimbal_swarm_{instance_id}.sdf"


        with open(base_gimbal_sdf, 'r') as f:
            content_gimbal = f.read()
        content_gimbal = content_gimbal.replace("{DRONE_ID}", f"drone_{instance_id}")
        
        with open(new_sdf_gimbal_path, 'w') as f:
            f.write(content_gimbal)

        with open(base_sdf, 'r') as f:
            content = f.read()
        
        # Replace the default JSON port with the instance-specific port
        # Assumes the ardupilot_gazebo plugin tag has <port>9002</port>
        content = content.replace("<fdm_port_in>9002</fdm_port_in>", f"<fdm_port_in>{target_port_in}</fdm_port_in>")
        content = content.replace("<fdm_port_out>9003</fdm_port_out>", f"<fdm_port_out>{target_port_out}</fdm_port_out>")

        content = content.replace("<uri>model://gimbal_small_3d</uri>", f"<uri>{new_sdf_gimbal_path}</uri>")

        with open(new_sdf_path, 'w') as f:
            f.write(content)
            
        return new_sdf_path

    def spawn_swarm(self, dim):
        M, N, D = dim['M'], dim['N'], dim['D']



        WORLD_NAME = "swarm_world" # Must match your running gz sim world name
        BASE_SDF_PATH = self.PROJECT_ROOT / "external" / "ardupilot_gazebo" /"models"/"iris_with_gimbal"/"model.sdf"
        GIMBAL_SDF_PATH = self.PROJECT_ROOT / "external" / "ardupilot_gazebo" / "models" / "gimbal_small_3d" / "model.sdf"

        total_drones = M * N
        print(f"Deploying {total_drones} drones in a {M}x{N} grid...")

        for i in range(total_drones):
            # 1. Calculate Grid Position (X, Y)
            row = i // N
            col = i % N
            
            # Center the swarm around 0,0
            x_offset = (col - (N - 1) / 2.0) * D
            y_offset = (row - (M - 1) / 2.0) * D
            
            print(f"Spawning Instance {i} at X:{x_offset}, Y:{y_offset}")

            sdf_file = self.generate_instance_sdf(i, BASE_SDF_PATH, GIMBAL_SDF_PATH)

            spawn_cmd = [
                "gz", "service", "-s", f"/world/{WORLD_NAME}/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "3000",
                "--req", f'sdf_filename: "{sdf_file}", name: "iris_{i}", pose: {{position: {{x: {x_offset}, y: {y_offset}, z: 0.195}}, orientation: {{z:0.707, w:0.707}}}}'
            ]
            subprocess.run(spawn_cmd)
            
        
        print("Spawning Success, Try to open the SITL")
        time.sleep(5.0)
        self.spawn_sitl(M*N)

    def spawn_sitl(self, total_drones):
        full_env = os.environ.copy()
        for i in range(total_drones):
            run_dir = self.PROJECT_ROOT / "run" / f"agent_{i}" 
            run_dir.mkdir(parents=True, exist_ok=True)

            sitl_cmd = [
                str(self.PROJECT_ROOT/"external/ardupilot/Tools/autotest/sim_vehicle.py"),
                "-v", "ArduCopter",
                "-f", "gazebo-iris",
                "-I", str(i),
                "--sysid", str(i + 1),
                "--model", "json",
                "--no-rebuild",
                "--console"
                ]
            
            proc = subprocess.Popen(
                sitl_cmd, 
                cwd=str(run_dir),
                env=full_env,
                stdout=None,
                stderr=None, 
                )
            
            self.process.append(proc)
            time.sleep(3.0)

if __name__ == '__main__':
    dim = {
            'M': 1,
            'N': 2,
            'D': 2.0
            }
    SwarmSpawner(dim)      



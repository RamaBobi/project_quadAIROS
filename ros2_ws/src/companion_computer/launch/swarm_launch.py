from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    num_drones = 3
    start_port = 14552

    for i in range(num_drones):
        drone_port = start_port + (i * 10)
        drone_name = f"drone_{i}"

        # Create a separate instance of the SAME binary for each drone
        drone_node = Node(
            package='companion_computer',
            executable='cc_telemetry',
            name='telemetry_node',
            namespace=drone_name,  # Separates topics like /drone_0/telemetry/odom
            parameters=[{
                'udp_port': drone_port
            }]
        )
        ld.add_action(drone_node)

    return ld


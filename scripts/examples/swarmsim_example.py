import numpy as np

from swarm_server import SimulationRunner
from swarm_server.swarmsim import create_objects_Point_yaw


def main():
    number_of_objects = 82

    script = "./scripts/mission_script.txt"
    output = "scripted_swarm_data.npz"
    simulation_duration = 120.0  # seconds
    # Create swarm objects externally

    objs = create_objects_Point_yaw(
        int(np.sqrt(number_of_objects)), [-50, 50], [-50, 50], 0.2
    )

    # Assign even/odd groups

    for idx, o in enumerate(objs):
        o.group = idx % 2

    # Initialize runner

    runner = SimulationRunner(
        num_drones=number_of_objects - 1,
        script_file=script,
        output_file=output,
        sim_time=simulation_duration,
        p_coeff=1.0,
        i_coeff=0.0,
        d_coeff=1.0,
    )

    # Override objects and start

    runner.objects = objs

    runner.run()


if __name__ == "__main__":
    main()

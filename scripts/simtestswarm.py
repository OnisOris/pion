import numpy as np

from swarm_server.swarmsim import SwarmSim, create_objects_Point_yaw

if __name__ == "__main__":
    number_of_objects = 60
    swarmsim = SwarmSim(
        create_objects_Point_yaw(
            int(np.sqrt(number_of_objects)), [-5, 5], [-5, 5]
        ),
        logger=True,
    )
    t_speed = np.ones((number_of_objects, 4))
    swarmsim.t_speed = t_speed
    swarmsim.start_simulation_while()

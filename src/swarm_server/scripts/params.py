import numpy as np

params = {
    "kp": np.ones((1, 6)),
    "ki": np.zeros((1, 6)),
    "kd": np.ones((1, 6)) * 0.1,
    "attraction_weight": 1.0,
    "cohesion_weight": 1.0,
    "alignment_weight": 1.0,
    "repulsion_weight": 1.0,
    "unstable_weight": 1.0,
    "current_velocity_weight": 0.0,
    "noise_weight": 1.0,
    "safety_radius": 1,
    "max_acceleration": 2,
    "max_speed": 0.4,
    "unstable_radius": 1.5,
}

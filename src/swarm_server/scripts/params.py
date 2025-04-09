import numpy as np

params = {
    "kp": np.ones((1, 6)),
    "ki": np.zeros((1, 6)),
    "kd": np.ones((1, 6)) * 0.1,
    "attraction_weight": 1.0,
    "cohesion_weight": 1.0,
    "alignment_weight": 1.0,
    "repulsion_weight": 4.0,
    "unstable_weight": 1.0,
    "noise_weight": 1.0,
    "safety_radius": 0.3,
    "max_acceleration": 1,
    "max_speed": 0.4,
    "unstable_radius": 1.5,
}

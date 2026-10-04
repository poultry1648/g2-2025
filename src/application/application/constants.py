import math

import numpy as np

# all in meters
WIDTH = 1
LENGTH = 1.5

RADIUS = math.hypot(WIDTH / 2, LENGTH / 2)

FRONT_LEFT_TANGENT_VECTOR = np.array([(LENGTH / 2) / RADIUS, (WIDTH / 2) / RADIUS])
FRONT_RIGHT_TANGENT_VECTOR = np.array([(LENGTH / 2) / RADIUS, -(WIDTH / 2) / RADIUS])
BACK_LEFT_TANGENT_VECTOR = np.array([-(LENGTH / 2) / RADIUS, (WIDTH / 2) / RADIUS])
BACK_RIGHT_TANGENT_VECTOR = np.array([-(LENGTH / 2) / RADIUS, -(WIDTH / 2) / RADIUS])

import numpy as np

noise = 0
n = 8
radius = 50
delt = 0.1

epsilon = np.pi/64
k=50

spread = np.pi/16

sensor_range = 30
"""
priority = False, Gaussian, Uniform
"""
priority_type="Gaussian"
pert=True

maxv = 3

"""
priority to import to the other files
"""
seed = 5


"""
b represents the spread of the repulsion field
"""
b=0.01

"""
strength of vector force fields
"""
attractive_gain=2
repulsive_gain=10


"""
distance within which if you get it is a collision
"""
collision_distance=1


"""
clipping power represents how strong clipping is
"""
clipping_power=1
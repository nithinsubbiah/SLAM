import numpy as np

def measurement_noise(laser_reading, true_reading):
        "Models the inherent measurement noise in the sensor as Gaussian"
        #TODO: Somehow get max_laser_reading
        z_max = 8000
        normalizer = 1
        p_hit = normalizer*np.random.normal(true_reading, 1) if 0<=laser_reading<=z_max else 0

        return p_hit

def v(occ,state,l):
    return occ[0]*state[0]*l

# print(v(np.array([1,2,3]), np.array([10,10,0]), 10))
f = np.vectorize(measurement_noise)
# print(f(np.array([1,2,3]), np.array([10,10,0]), np.array([10,20,30])))
laser_reading = np.array([1,2,3,4,5,6,7,8,9])
true_reading = np.array([1,2,3,4,5,6,7,8,9])
print(f(laser_reading,true_reading).shape)
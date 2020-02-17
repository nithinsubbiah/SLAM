import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
from scipy.special import erf
import sys


from MapReader import MapReader
from RayCasting import cast_ray

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    def __init__(self, occupancy_map, logfileID):
        # max_laser_reading
        self.max_laser_reading = 8183 if logfileID is '1' else 8191
        # Parameter for measurement noise
        self.std_deviation_noise = 400
        # Parameter for unexpected object
        self.lambda_short = 0.01
        # Distribution weight parameters
        self.alpha_hit = 10
        self.alpha_short = 1
        self.alpha_max = 0.1
        self.alpha_rand = 5000
        self.distribution_parameters = np.array([self.alpha_hit, self.alpha_short, self.alpha_max, self.alpha_rand])

        # if(np.sum(self.distribution_parameters) != 1):
        #     sys.exit("Distribution paramters do not sum to 1")

        self.occupancy_map = occupancy_map
        self.map_resolution = 10

        # Vectorized functions
        self.v_measurement_noise = np.vectorize(self.measurement_noise)
        self.v_unexpected_object = np.vectorize(self.unexpected_object)
        self.v_failure_to_detect = np.vectorize(self.failure_to_detect)
        self.v_random_measurements = np.vectorize(self.random_measurements)

    def measurement_noise(self, laser_reading, true_reading):
        "Models the inherent measurement noise in the sensor as Gaussian"
        normalizer = 1/((math.sqrt(2*np.pi))*self.std_deviation_noise)
        # normalizer = 1
        exponent  = -0.5*((laser_reading - true_reading)/self.std_deviation_noise)**2
        p_hit = normalizer*np.exp(exponent) if 0<=laser_reading<=self.max_laser_reading else 0

        return p_hit

    def unexpected_object(self, laser_reading, true_reading):
        "Models the unexpected objects that obstruct the sensor reading using exponential distribution"
        # normalizer = 1/(1-math.exp(-self.lambda_short*true_reading))
        # normalizer = 1
        # p_short = normalizer * np.random.exponential(1/self.lambda_short) if 0<= laser_reading <= true_reading else 0
        p_short = self.lambda_short*np.exp(-self.lambda_short*laser_reading) if 0<= laser_reading <= true_reading else 0

        return p_short

    def failure_to_detect(self, laser_reading):
        "Models the case when sensor fails to detect obstacle using point-mass distribution"
        p_max = 1 if laser_reading == self.max_laser_reading else 0

        return p_max

    def random_measurements(self, laser_reading):
        "Models erroneous unexplainable measurements using uniform distribution"
        # uniform function calculates for range [low,high) but need [low,high]. Is that a problem?
        p_rand = np.random.uniform(low=0, high=self.max_laser_reading) if 0<=laser_reading<=self.max_laser_reading else 0

        return p_rand

    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """
        #TODO: Check if this module is right independently
        #NOTE: From Piazza - The laser sensor spans 180∘ starting from the right and going left.
        q = 1

        angle_stride = 5
        selected_idx = np.arange(int(180/angle_stride)) * angle_stride
        robot_angle = np.degrees(x_t1[-1])
        angle_measurements = np.radians(np.linspace(robot_angle-90, robot_angle+90, int(180/angle_stride), endpoint=False))
        # import pdb;pdb.set_trace()
        z_t1_arr_selected = np.take(z_t1_arr, selected_idx)
        
        true_readings = cast_ray(self.occupancy_map, x_t1, angle_measurements, self.map_resolution)
        p_hit = self.v_measurement_noise(z_t1_arr_selected, true_readings)
        p_short = self.v_unexpected_object(z_t1_arr_selected, true_readings)
        # print(p_short.shape)
        p_max = self.v_failure_to_detect(z_t1_arr_selected)
        # print(p_max.shape)
        p_rand = self.v_random_measurements(z_t1_arr_selected)
        # print(p_rand.shape)
        p = np.stack([p_hit, p_short, p_max, p_rand], axis=0)
        q = p*self.distribution_parameters.reshape(4,1)
        q = np.prod(q.sum(axis=0))
        return q

if __name__=='__main__':
    pass
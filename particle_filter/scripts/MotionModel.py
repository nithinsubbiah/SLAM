import sys
import numpy as np
import math
import matplotlib.pyplot as plt
pi = np.pi

class MotionModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 5]
    """

    def __init__(self):

        #alpha_1&4 affects rotation and alpha_2&3 translation
        self.alpha_1 = 1e-4
        self.alpha_2 = 1e-4
        self.alpha_3 = 1e-2       #1e-3
        self.alpha_4 = 1e-3

    def check_angle_wrap_around(self, angle):
        '''
        Input: Angle (in radians)
        Output: Angle constrained to [-pi,pi]
        '''

        angle = np.fmod(angle+np.pi, 2*np.pi)
        if(angle<0):
            angle += 2*np.pi
        return angle-np.pi

    def update(self, u_t0, u_t1, x_t0):
        """
        param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]
        param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """

        rot_1 = np.arctan2(u_t1[1]-u_t0[1], u_t1[0]-u_t0[0]) - u_t0[2]
        rot_1 = self.check_angle_wrap_around(rot_1)
        trans = np.linalg.norm((u_t0[0]-u_t1[0], u_t0[1]-u_t1[1]))
        rot_2 = u_t1[2] - u_t0[2] - rot_1
        rot_2 = self.check_angle_wrap_around(rot_2)

        rot_1_with_noise = rot_1 - np.random.normal(0, self.alpha_1*rot_1*rot_1 + self.alpha_2*trans*trans)
        rot_1_with_noise = self.check_angle_wrap_around(rot_1_with_noise)
        trans_with_noise = trans - np.random.normal(0, self.alpha_4*rot_1*rot_1 + self.alpha_3*trans*trans + self.alpha_4*rot_2*rot_2)
        rot_2_with_noise = rot_2 - np.random.normal(0, self.alpha_1*rot_2*rot_2 + self.alpha_2*trans*trans)
        rot_2_with_noise = self.check_angle_wrap_around(rot_2_with_noise)

        x = x_t0[0] + trans_with_noise * np.cos(x_t0[2] + rot_1_with_noise)    
        y = x_t0[1] + trans_with_noise * np.sin(x_t0[2] + rot_1_with_noise)    
        theta = x_t0[2] + rot_1_with_noise + rot_2_with_noise
        theta = self.check_angle_wrap_around(theta)

        x_t1 = np.array([x,y,theta])

        return x_t1

if __name__=="__main__":
    mm = MotionModel()
    u = np.array([[0,0,0], [1,1,pi/4], [2,2,pi/4], [3,3,pi/4], [4,4,pi/4]])
    x_init = np.array([0,0,0])
    x_vals = [x_init]
    # print(np.shape(u)[0])
    for i in range(np.shape(u)[0]-1):
        x_vals.append(mm.update(u[i],u[i+1],x_vals[-1]))
        # x_init = x_vals[-1]

    x_vals = np.asarray(x_vals)
    # print(u)
    plt.scatter(u[:,0], u[:,1])
    plt.scatter(x_vals[:,0],x_vals[:,1],s=2)
    plt.show()
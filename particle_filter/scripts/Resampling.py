import numpy as np
import pdb

class Resampling:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 4.3]
    """

    def __init__(self):
        """
        TODO : Initialize resampling process parameters here
        """

    def multinomial_sampler(self, X_bar):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """

        """
        TODO : Add your code here
        """

        return X_bar_resampled

    def low_variance_sampler(self, X_bar, num_particles):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """
        #TODO: Check if this module is right independently

        X_bar_resampled = []

        r = np.random.uniform(low=0,high=1/num_particles)
        weights = X_bar[:,-1]
        weights = weights/np.sum(weights)
        c = weights[0]
        idx = 0

        for i in range(num_particles):
            U = r + (i/num_particles)
            while U > c:
                idx += 1
                c += weights[idx]
            X_bar_resampled.append(X_bar[idx])

        X_bar_resampled = np.vstack(X_bar_resampled)

        return X_bar_resampled

if __name__ == "__main__":
    pass
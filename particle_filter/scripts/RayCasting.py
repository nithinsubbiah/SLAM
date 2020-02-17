import numpy as np
import cv2
from matplotlib import pyplot as plt
from matplotlib import figure as fig

"Approximate ray casting"
def cast_ray(occupancy_map, robot_pose, angle_measurements, map_resolution):
    #TODO: Test ray casting on original map
    #TODO: bring in obstacle_threshold
    # Assumption: Horizontal x-axis & vertical y-axis
    # Current coordinates of the ray
    # Map value = probability(free_cell)
    sensor_offset = 25
    true_laser_readings = []
    obstacle_threshold = 0.1

    x_robot, y_robot, theta_robot = robot_pose

    x_ray_start = x_robot + sensor_offset * np.cos(theta_robot)
    y_ray_start = y_robot + sensor_offset * np.sin(theta_robot)
    laser_ends = []

    for angle in angle_measurements:

        # theta = theta_robot + angle
        theta = angle

        # x_curr_coords & y_curr_coords in map frame
        x_curr_coords = x_ray_start//map_resolution
        y_curr_coords = y_ray_start//map_resolution
        
        step_size = 2
        # import pdb;pdb.set_trace()
        while(0 <= x_curr_coords < occupancy_map.shape[0] and 0 <= y_curr_coords < occupancy_map.shape[1]
            and 0<= occupancy_map[int(x_curr_coords), int(y_curr_coords)] < obstacle_threshold):
            # print("ray passing ===>")

            x_curr_coords = x_curr_coords + step_size * np.cos(theta)
            y_curr_coords = y_curr_coords + step_size * np.sin(theta)
        
        x_curr_coords = x_curr_coords - step_size * np.cos(theta)
        y_curr_coords = y_curr_coords - step_size * np.sin(theta)
        laser_ends.append(np.array([x_curr_coords, y_curr_coords]))

        true_laser_reading = np.linalg.norm((x_curr_coords-x_ray_start//map_resolution,y_curr_coords-y_ray_start//map_resolution))
        true_laser_readings.append(true_laser_reading)
        # print("laser_reading:", true_laser_reading)
        # print("theta:", angle)

        # ##
        # fig = plt.figure()
        # # plt.switch_backend('TkAgg')
        # mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
        # plt.ion(); plt.imshow(occupancy_map, cmap='Greys'); plt.axis([0, 800, 0, 800]);
        # plt.plot(int(y_curr_coords), int(x_curr_coords), 'o', color='red')
        # plt.pause(10)
        # plt.close()
        # ##
    true_laser_readings = np.vstack(true_laser_readings)
    # print(true_laser_readings)
    # ##
    # laser_ends = np.asarray(laser_ends)
    # fig = plt.figure()
    # x_vals = laser_ends[:,0].reshape(-1)
    # y_vals = laser_ends[:,1].reshape(-1)
    # print(x_vals.shape)
    # plt.switch_backend('TkAgg')
    # mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    # plt.ion(); plt.imshow(occupancy_map, cmap='Greys'); plt.axis([0, 800, 0, 800]);
    # plt.plot(y_vals,x_vals, 'o', color='cyan')
    # plt.plot(int(y_ray_start//map_resolution), int(x_ray_start//map_resolution), 'o', color='black')
    # plt.pause(20)
    # plt.close()
    ##

    return 10*true_laser_readings.reshape(-1)

def main():
    p = 0.35 # Probability of False values
    N = 40 # Size of occupancy gird
    occupancy_map = np.random.rand(N,N) > p
    occupancy_map = occupancy_map.astype(np.uint8)
    x = 20
    y = 20
    ray_start = np.array([x,y,450*np.pi/180])
    occupancy_map[x,y] = 0
    # occupancy_map[30,30] = 1
    print(occupancy_map[x,y])
    grid_size = 1
    image = occupancy_map * 255
    cv2.line(image, (0, 0), (35, 25), (255,0,255), 2)
    cv2.imshow("occupancy_map",image)
    cv2.waitKey(0)
    # cast_ray(occupancy_map, ray_start,np.array([0]), np.array([grid_size,grid_size]))

# if __name__ == "__main__":
#     main()


import numpy as np
import cv2
import sys
"Accurate ray casting"
"Adapted from: https://theshoemaker.de/2016/02/ray-casting-in-2d-grids/"

def cast_ray(occupancy_map, ray_start, grid_size):

    # Assumption: Horizontal x-axis & vertical y-axis
    # Current coordinates of the ray
    #TODO: Offset of the sensor from front of the robot
    x_ray_start, y_ray_start, theta = ray_start # 'theta' should be in radians

    x_curr_coords = x_ray_start
    y_curr_coords = y_ray_start

    image = occupancy_map*255
    # NOTE: Direction along which x and y increase!
    X_dir = np.sin(theta)
    Y_dir = np.cos(theta)
    X_dir = 0 if -1e-6 < X_dir <1e-6  else X_dir
    Y_dir = 0 if -1e-6 < Y_dir < 1e-6 else Y_dir

    # Coordiantes of grid intersection with a ray from current coordinates at angle 'theta'
    X_grid = x_curr_coords//grid_size + 1
    Y_grid = y_curr_coords//grid_size + 1

    X_sign = 1 if X_dir>0 else -1
    Y_sign = 1 if Y_dir>0 else -1

    X_grid_offset = (1 if X_dir>0 else 0) - 1
    Y_grid_offset = (1 if Y_dir>0 else 0) - 1

    ray_stride = 0

    while(0 <= x_curr_coords < 40 and 0 <= y_curr_coords < 40 and not occupancy_map[int(x_curr_coords), int(y_curr_coords)]):

        print("ray passing ===>")

        # Coordiantes that correspond to the ray intersection with grid
        x_grid_coords = (X_grid+X_grid_offset) * grid_size
        y_grid_coords = (Y_grid+Y_grid_offset) * grid_size

        # Ray length striking both axes
        ray_length_x_axis = (x_grid_coords-x_curr_coords)/X_dir
        ray_length_y_axis = (y_grid_coords-y_curr_coords)/Y_dir


        ray_length_x_axis = sys.maxsize if X_dir==0 else (x_grid_coords-x_curr_coords)/X_dir
        ray_length_y_axis = sys.maxsize if Y_dir==0 else (y_grid_coords-y_curr_coords)/Y_dir

        if(ray_length_x_axis < ray_length_y_axis):
            ray_stride = ray_stride + ray_length_x_axis
            X_grid = X_grid + X_sign
        else:
            ray_stride = ray_stride + ray_length_y_axis
            Y_grid = Y_grid + Y_sign

        x_curr_coords = x_ray_start + ray_stride * Y_dir
        y_curr_coords = y_ray_start + ray_stride * X_dir

    cv2.line(image, (int(x_ray_start), int(y_ray_start)), (int(x_curr_coords), int(y_curr_coords)), (255,0,0), 2)

    cv2.imshow("raycast",image)
    cv2.waitKey(0)


def main():
    p = 1 # Probability of False values
    N = 40 # Size of occupancy gird
    occupancy_map = np.random.rand(N,N) > p
    occupancy_map = occupancy_map.astype(np.uint8)
    x = 20
    y = 20
    ray_start = np.array([x,y,45*np.pi/180])
    occupancy_map[x,y] = 0
    # occupancy_map[30,30] = 1
    print(occupancy_map[x,y])
    grid_size = 1
    # cv2.imshow("occupancy_map",occupancy_map*255)
    # cv2.waitKey(0)
    cast_ray(occupancy_map, ray_start, grid_size, N)

if __name__ == "__main__":
    main()


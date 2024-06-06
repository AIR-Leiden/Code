import traceback

import cv2
import numpy as np
import rospy
import sys
import time
import random

from getfrontier import getfrontier
from astar import astar
from utils import *
import multiprocessing

MAPPING_CONSTANT = {-1: "?", 0: " ", 100: "#"}


def recovery_method_make_path(world, start_position):
    # type: (np.ndarray, (int, int)) -> None

    visited = []
    queue = [(start_position, [start_position])]

    while len(queue) > 0:
        current, path = queue[0]
        del queue[0]

        block = world[current]
        if block == 0:
            # goal found, clear the path to it
            for entry in path:
                print "clearing", entry
                if world[entry] != 0:
                    world[entry] = 0
            return

        if current in visited:
            continue

        visited.append(current)

        for delta in [
            (-1, 0), (1, 0), (0, -1), (0, 1)
        ]:
            neighbour = (current[0] + delta[0], current[1] + delta[1])
            if 0 <= neighbour[0] < world.shape[0] and \
                    0 <= neighbour[1] < world.shape[1]:
                queue.append((neighbour, path + [neighbour]))


def print_world(world):
    print "cropped world:"
    MARGIN = 0
    for y in range(MARGIN, world.shape[0]-MARGIN):
        for x in range(MARGIN, world.shape[1]-MARGIN):
            entry = world[y][x]
            out = " "
            if entry < 0:
                out = "?"
            if entry > 0:
                out = "#"
            sys.stdout.write(out)
        sys.stdout.write("\n")


def occupancy_to_numpy(occupancy_grid):
    return np.array(occupancy_grid.data).reshape((
        occupancy_grid.info.height,
        occupancy_grid.info.width,
    ))


circle_radius = 7
circle_mask = np.zeros((circle_radius * 2 + 1, circle_radius * 2 + 1))
center = (circle_radius, circle_radius)
cv2.circle(circle_mask, center, circle_radius, 99, -1)


def calculate_driving_random_spots(grid):

    random.seed(int(time.time()) % 30)

    result = []

    n = 15

    for i in range(n):
        y = random.randint(0, grid.shape[0]-1)
        x = random.randint(0, grid.shape[1]-1)

        if grid[y, x] == 0:
            result.append((x, y))
            break

    print("random spots: " + str(result))

    return result


def calculate_path(
        planning_request, # either None for exploration or a list of points
        occupancy_grid,
        position_pipe
):

    t = time.time()

    input_grid = occupancy_to_numpy(occupancy_grid)

    print "0:", t - time.time(); t = time.time()


    frontiers = getfrontier(occupancy_grid, input_grid)


    print "1:", t - time.time(); t = time.time()
    """
    # TODO: THIS IS TAKING THE AVERAGE OF ALL PIXELS IN A CIRCLE
    # THIS IS PROBLEMATIC, AS WALLS ARE USUALLY JUST 1 PIXEL WIDE
    # THIS MEANS IT IS NOT AS THE ORIGINAL IDEA OF 'DRAWING A CIRCLE OF circle_radius SIZE AROUND EACH WALL POINT'
    # WE SHOULD USE cv2.dilate BUT MAKE SURE TO ONLY DILATE WALLS AND NOT THE 'KNOWN' AREAS (WHICH ARE REPRESENTED BY 0)
    # (UNKOWN AREAS ARE REPRESENTED BY -1)
    grid = cv2.dilate(input_grid.astype(np.uint8), kernel).astype(np.int8)

    print "1.5:", t - time.time(); t = time.time()

    grid[input_grid == -1] = -1

    print "2:", t - time.time(); t = time.time()

    # Threshold to replace pixels where convolution value is greater than 0
    grid[grid > 0] = 100
    """

    def print_surroundings(x, y):
        size = 20
        y1, y2 = y - size, y + size + 1
        x1, x2 = x - size, x + size + 1
        np.set_printoptions(threshold=np.inf)
        print_world(grid[
                    y1:y2,
                    x1:x2
                    ])

    grid = input_grid
    # Create a binary mask for the circle

    # print_surroundings(robot_in_map[1], robot_in_map[0])

    # Loop through the image and apply the circular mask
    radius_robot_filter = 4

    radius_robot_filter_mask = np.zeros((radius_robot_filter * 2 + 1, radius_robot_filter * 2 + 1))
    center = (radius_robot_filter, radius_robot_filter)
    cv2.circle(radius_robot_filter_mask, center, radius_robot_filter, 1, -1)
    for y in range(radius_robot_filter, grid.shape[0] - radius_robot_filter):
        for x in range(radius_robot_filter, grid.shape[1] - radius_robot_filter):
            if grid[y, x] == 100:
                y1, y2 = y-radius_robot_filter, y + radius_robot_filter + 1
                x1, x2 = x-radius_robot_filter, x + radius_robot_filter + 1

                if np.sum((radius_robot_filter_mask * grid[y1:y2, x1:x2]) > 0) < 6:
                    grid[y, x] = 0

    # print_surroundings(robot_in_map[1], robot_in_map[0])


    # Loop through the image and apply the circular mask
    for y in range(circle_radius, grid.shape[0] - circle_radius):
        for x in range(circle_radius, grid.shape[1] - circle_radius):
            if grid[y, x] == 100:
                # Calculate the region of interest using the circle mask
                y1, y2 = y-circle_radius, y + circle_radius + 1
                x1, x2 = x-circle_radius, x + circle_radius + 1
                grid[
                    y1:y2,
                    x1:x2
                ] = np.maximum(
                    circle_mask,
                    grid[y1:y2, x1:x2]
                )



    print "3:", t - time.time(); t = time.time()



    robot_in_map = position_pipe.get()
    while not position_pipe.empty():
        robot_in_map = position_pipe.get() # realworld_to_grid(robot_position())


    print_surroundings(robot_in_map[1], robot_in_map[0])

    print "4:", t - time.time(); t = time.time()

    recovery_method_make_path(
        grid,
        (int(robot_in_map[1]), int(robot_in_map[0]))
    )


    print "5:", t - time.time(); t = time.time()

    # frontiers.sort(key=lambda frontier: -((robot_in_map[0] - frontier[0]) ** 2 + (robot_in_map[1] - frontier[1]) ** 2))

    path = None
    for p in ((planning_request or []) + frontiers + calculate_driving_random_spots(grid)):
        print "6:", t - time.time(); t = time.time()

        print "point:", p
        start = time.time()
        path = astar(grid, (int(robot_in_map[0]), int(robot_in_map[1])), p)
        if path is not None and p in frontiers:
            print "CUTTING PATH BECAUSE IT IS A FRONTIER"
            path = path[:-14]  # DO NOT WALK TO THE LAST COUPLE PLACES OF THE PATH, SINCE THIS IS HIGHLY LIKELY
            # UNEXPLORED SPACE AND UNNECESSARY
        end = time.time()
        print "astar took: ", str(end - start)
        if path is not None:
            break

    print "7:", t - time.time(); t = time.time()
    if path is None:
        return None

    print path
    return [point(p[0], p[1]) for p in path]

def planner(planning_request_queue, map_queue, position_queue, path_queue):
    # type: (multiprocessing.Queue, multiprocessing.Queue, multiprocessing.Queue, multiprocessing.Queue) -> None
    try:
        occupancy_grid = map_queue.get()
        while True:
            planning_request = planning_request_queue.get()
            print "GOT PLANNING REQUEST:", planning_request

            while not map_queue.empty():
                occupancy_grid = map_queue.get()

            path = calculate_path(
                planning_request,
                occupancy_grid,
                position_queue
            )
            path_queue.put_nowait(path)

    except:
        traceback.print_exc()
        print "planning stopped due to exception"
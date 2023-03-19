import cv2 as cv
import numpy as np
from numpy import tan, deg2rad

ROBOT_RADIUS = 3

SCALE_FACTOR = 2

BLUE = (255, 0, 0)
DARK_GREEN = (15, 168, 33)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
YELLOW = (9, 227, 212)
BLACK = (0, 0, 0)
GRAY = (199, 198, 195)

# for node breakdown
COST_TO_COME = 0
PARENT_NODE = 1
COORDINATES = 2

# for coordinates
X = 0
Y = 1

# map dimensions
X_MAX = 600
Y_MAX = 250


def draw_map():
    # Background
    background_color = BLACK
    map = np.zeros((250*SCALE_FACTOR, 600*SCALE_FACTOR, 3), np.uint8)
    map[:] = background_color

    # Map boarder
    map[0:ROBOT_RADIUS*SCALE_FACTOR, :] = YELLOW                                    # north edge
    map[(Y_MAX - ROBOT_RADIUS) * SCALE_FACTOR: Y_MAX * SCALE_FACTOR, :] = YELLOW    # south edge
    map[:, 0:ROBOT_RADIUS * SCALE_FACTOR] = YELLOW                                                 # east edge
    map[:, (X_MAX - ROBOT_RADIUS) * SCALE_FACTOR: X_MAX * SCALE_FACTOR] = YELLOW      # west edge

    # box 1 boundary
    pts = np.array([[(100 - ROBOT_RADIUS) * SCALE_FACTOR, 0 * SCALE_FACTOR],
                    [(150 + ROBOT_RADIUS) * SCALE_FACTOR, 0 * SCALE_FACTOR],
                    [(150 + ROBOT_RADIUS) * SCALE_FACTOR, (100 + ROBOT_RADIUS) * SCALE_FACTOR],
                    [(100 - ROBOT_RADIUS) * SCALE_FACTOR, (100 + ROBOT_RADIUS) * SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], YELLOW)

    # box 1
    pts = np.array([[100 * SCALE_FACTOR, 0 * SCALE_FACTOR],
                    [150 * SCALE_FACTOR, 0 * SCALE_FACTOR],
                    [150 * SCALE_FACTOR, 100 * SCALE_FACTOR],
                    [100 * SCALE_FACTOR, 100 * SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], BLUE)

    # box 2  boundary
    pts = np.array([[(100 - ROBOT_RADIUS) * SCALE_FACTOR, (150 - ROBOT_RADIUS) * SCALE_FACTOR],
                    [(150 + ROBOT_RADIUS) * SCALE_FACTOR, (150 - ROBOT_RADIUS) * SCALE_FACTOR],
                    [(150 + ROBOT_RADIUS) * SCALE_FACTOR, 250 * SCALE_FACTOR],
                    [(100 - ROBOT_RADIUS) * SCALE_FACTOR, 250 * SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], YELLOW)

    # box 2
    pts = np.array([[100*SCALE_FACTOR, 150*SCALE_FACTOR],
                    [150*SCALE_FACTOR, 150*SCALE_FACTOR],
                    [150*SCALE_FACTOR, 250*SCALE_FACTOR],
                    [100*SCALE_FACTOR, 250*SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], BLUE)

    # hexagon boundry
    pts = np.array([[300*SCALE_FACTOR, (50 - ROBOT_RADIUS) * SCALE_FACTOR],  # 1
                    [(365+ROBOT_RADIUS)*SCALE_FACTOR, (87 - ROBOT_RADIUS*tan(deg2rad(30)))*SCALE_FACTOR],  # 2
                    [(365+ROBOT_RADIUS)*SCALE_FACTOR, (161 + ROBOT_RADIUS*tan(deg2rad(30)))*SCALE_FACTOR],
                    [300*SCALE_FACTOR, (200 + ROBOT_RADIUS) * SCALE_FACTOR],
                    [(235-ROBOT_RADIUS)*SCALE_FACTOR, (161 + ROBOT_RADIUS*tan(deg2rad(30)))*SCALE_FACTOR],
                    [(235-ROBOT_RADIUS)*SCALE_FACTOR, (87 - ROBOT_RADIUS*tan(deg2rad(30)))*SCALE_FACTOR]],  # 6
                   np.int32)
    cv.fillPoly(map, [pts], YELLOW)

    # hexagon
    pts = np.array([[300*SCALE_FACTOR, 50*SCALE_FACTOR],
                    [365*SCALE_FACTOR, 87*SCALE_FACTOR],
                    [365*SCALE_FACTOR, 161*SCALE_FACTOR],
                    [300*SCALE_FACTOR, 200*SCALE_FACTOR],
                    [235*SCALE_FACTOR, 161*SCALE_FACTOR],
                    [235*SCALE_FACTOR, 87*SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], BLUE)

    # triangle boundry
    pts = np.array([[(460-ROBOT_RADIUS)*SCALE_FACTOR, (25-ROBOT_RADIUS)*SCALE_FACTOR],
                    [(460+ROBOT_RADIUS)*SCALE_FACTOR, (25-ROBOT_RADIUS)*SCALE_FACTOR],
                    [(510+ROBOT_RADIUS)*SCALE_FACTOR, 125*SCALE_FACTOR],
                    [(460+ROBOT_RADIUS)*SCALE_FACTOR, (225+ROBOT_RADIUS)*SCALE_FACTOR],
                    [(460-ROBOT_RADIUS)*SCALE_FACTOR, (225+ROBOT_RADIUS)*SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], YELLOW)

    # triangle
    pts = np.array([[460*SCALE_FACTOR, 25*SCALE_FACTOR],
                    [510*SCALE_FACTOR, 125*SCALE_FACTOR],
                    [460*SCALE_FACTOR, 225*SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], BLUE)

    return map


def get_valid_point_map(color_map):
    valid_point_map = np.ones((250 * SCALE_FACTOR, 600 * SCALE_FACTOR), np.uint8)
    for x in range(0, 600 * SCALE_FACTOR):
        for y in range(0, 250 * SCALE_FACTOR):
            pixel_color = tuple(color_map[y, x])
            if pixel_color == YELLOW or pixel_color == BLUE:
                valid_point_map[y, x] = 0
    return valid_point_map


def determine_valid_point(valid_point_map, coordinates):
    if not __point_is_inside_map(coordinates[X], coordinates[Y]):
        return False
    if valid_point_map[coordinates[Y], coordinates[X]] == 1:
        return True
    else:
        return False


def __point_is_inside_map(x, y):
    if (x > 600) or (x < 0):
        return False
    elif (y > 250) or (y < 0):
        return False
    else:
        return True


def __add_point(x, y, map, color):
    map[y, x] = color
    return map


def __draw_line(p1, p2, map, color):
    pts = np.array([[p1[0], p1[1]], [p2[0], p2[1]]],
                   np.int32)
    cv.fillPoly(map, [pts], color)


def draw_node(child_coordinates, parent_coordinates, map, color):

    child_coordinates = tuple(int(SCALE_FACTOR * x) for x in child_coordinates)
    cv.circle(map, child_coordinates, radius=3, color=color, thickness=-1)

    if parent_coordinates is not None:
        parent_coordinates = tuple(SCALE_FACTOR * x for x in parent_coordinates)
        cv.circle(map, parent_coordinates, radius=3, color=color, thickness=-1)
        __draw_line(child_coordinates, parent_coordinates, map, color)


'''
run_simulation()

Runs an example program of the functions in the Project3_visual.py file
'''
def run_simulation():
    print("Drawing map...")
    map = draw_map()
    print("Determining valid points...")
    valid_point_map = get_valid_point_map(map)

    draw_node((0, 0), None, map, GRAY)
    draw_node((50, 50), (0, 0), map, GRAY)
    draw_node((125, 125), (50, 50), map, GRAY)
    draw_node((145.5, 125.5), (125, 125), map, GRAY)
    cv.imshow('Djikstra\'s Algorith', map)
    cv.waitKey(0)
    cv.destroyAllWindows()


if __name__ == '__main__':
    run_simulation()



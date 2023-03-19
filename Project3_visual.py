import cv2 as cv
import numpy as np

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


def draw_map():
    # Background
    background_color = BLACK
    map = np.zeros((250*SCALE_FACTOR, 600*SCALE_FACTOR, 3), np.uint8)
    map[:] = background_color

    # box 1 boundry
    pts = np.array([[95*SCALE_FACTOR, 0*SCALE_FACTOR], [155*SCALE_FACTOR, 0*SCALE_FACTOR],
                    [155*SCALE_FACTOR, 105*SCALE_FACTOR], [95*SCALE_FACTOR, 105*SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], YELLOW)

    # box 1
    pts = np.array([[100*SCALE_FACTOR, 0*SCALE_FACTOR], [150*SCALE_FACTOR, 0*SCALE_FACTOR],
                    [150*SCALE_FACTOR, 100*SCALE_FACTOR], [100*SCALE_FACTOR, 100*SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], BLUE)

    # box 2  boundry
    pts = np.array([[95*SCALE_FACTOR, 145*SCALE_FACTOR], [155*SCALE_FACTOR, 145*SCALE_FACTOR],
                    [155*SCALE_FACTOR, 250*SCALE_FACTOR], [95*SCALE_FACTOR, 250*SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], YELLOW)

    # box 2
    pts = np.array([[100*SCALE_FACTOR, 150*SCALE_FACTOR], [150*SCALE_FACTOR, 150*SCALE_FACTOR],
                    [150*SCALE_FACTOR, 250*SCALE_FACTOR], [100*SCALE_FACTOR, 250*SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], BLUE)

    # hexagon boundry
    pts = np.array([[300*SCALE_FACTOR, 45*SCALE_FACTOR], [369*SCALE_FACTOR, 85*SCALE_FACTOR], [369*SCALE_FACTOR, 163*SCALE_FACTOR],
                    [300*SCALE_FACTOR, 205*SCALE_FACTOR], [231*SCALE_FACTOR, 163*SCALE_FACTOR], [231*SCALE_FACTOR, 85*SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], YELLOW)

    # hexagon
    pts = np.array([[300*SCALE_FACTOR, 50*SCALE_FACTOR], [365*SCALE_FACTOR, 87*SCALE_FACTOR], [365*SCALE_FACTOR, 161*SCALE_FACTOR],
                    [300*SCALE_FACTOR, 200*SCALE_FACTOR], [235*SCALE_FACTOR, 161*SCALE_FACTOR], [235*SCALE_FACTOR, 87*SCALE_FACTOR]],
                   np.int32)
    cv.fillPoly(map, [pts], BLUE)

    # triangle boundry
    pts = np.array([[456*SCALE_FACTOR, 20*SCALE_FACTOR], [462*SCALE_FACTOR, 20*SCALE_FACTOR],
                    [515*SCALE_FACTOR, 125*SCALE_FACTOR], [462*SCALE_FACTOR, 230*SCALE_FACTOR],
                    [456*SCALE_FACTOR, 230*SCALE_FACTOR]], np.int32)
    cv.fillPoly(map, [pts], YELLOW)

    # triangle
    pts = np.array([[460*SCALE_FACTOR, 25*SCALE_FACTOR], [510*SCALE_FACTOR, 125*SCALE_FACTOR],
                    [460*SCALE_FACTOR, 225*SCALE_FACTOR]], np.int32)
    cv.fillPoly(map, [pts], BLUE)

    return map


def get_valid_point_map():
    valid_point_map = np.ones((250, 600), np.uint8)
    for x in range(0, 600):
        for y in range(0, 250):
            if not determine_valid_point((x, y)):
                print(x, "  ", y)
                valid_point_map[y][x] = 0
    return valid_point_map


def determine_valid_point(coordinates):
    if not point_is_inside_map(coordinates[X], coordinates[Y]):
        return False
    if point_is_inside_box1(coordinates[X], coordinates[Y]):
        return False
    if point_is_inside_box2(coordinates[X], coordinates[Y]):
        return False
    if point_is_inside_hexagon(coordinates[X], coordinates[Y]):
        return False
    if point_is_inside_triangle(coordinates[X], coordinates[Y]):
        return False
    else:
        return True


def point_is_inside_map(x, y):
    if (x > 600) or (x < 0):
        return False
    elif (y > 250) or (y < 0):
        return False
    else:
        return True


def point_is_inside_box1(x, y):
    if (x >= 95) and (x <= 155) and (y >= 0) and (y <= 105):
        return True


def point_is_inside_box2(x, y):
    if (x >= 95) and (x <= 155) and (y >= 145) and (y <= 250):
        return True


def above_line_1(x, y):
    # p1 = (300, 44)  p2 = (370, 85)
    m = (85-44)/(370-300)
    y_line = m * (x - 300) + 44
    if y > y_line:
        return True
    else:
        return False


def below_line_2(x, y):
    # p1=(300, 206)  p2=(370, 164)
    m = (164-206)/(370-300)
    y_line = m * x - m * 300 + 206
    if y < y_line:
        return True
    else:
        return False


def below_line_3(x, y):
    # [300, 206], [230, 163]
    m = (206-163)/(300-230)
    y_line = m * (x - 300) + 206
    if y < y_line:
        return True
    else:
        return False


def above_line_4(x, y):

    # [230, 84], [300, 44]
    m = (44-84)/(300-230)
    y_line = m * (x - 230) + 84
    if y > y_line:
        return True
    else:
        return False


def point_is_inside_hexagon(x, y):
    if above_line_1(x, y) \
    and (x < 370) \
    and below_line_2(x, y) \
    and below_line_3(x, y) \
    and (x > 230) \
    and above_line_4(x, y):
        return True
    else:
        return False


def above_line_5(x, y):
    # [462, 20], [515, 125]
    m = (125 - 20)/(516 - 462)
    y_line = m * (x - 462) + 20
    if y > y_line:
        return True
    else:
        return False


def below_line_6(x, y):
    # [463, 231],  [516, 125]
    m = (125 - 231)/(516 - 463)
    y_line = m * (x - 463) + 231
    if y < y_line:
        return True
    else:
        return False


def point_is_inside_triangle(x, y):
    if (x>455) \
    and (y>19) \
    and above_line_5(x, y) \
    and below_line_6(x, y) \
    and (y<231):
        return True
    else:
        return False


def add_point(x, y, map, color):
    map[y, x] = color
    return map


def draw_line(p1, p2, map, color):
    pts = np.array([[p1[0], p1[1]], [p2[0], p2[1]]],
                   np.int32)
    cv.fillPoly(map, [pts], color)


def draw_node(child_coordinates, parent_coordinates, map, color):

    child_coordinates = tuple(SCALE_FACTOR * x for x in child_coordinates)
    cv.circle(map, child_coordinates, radius=3, color=color, thickness=-1)

    if parent_coordinates is not None:
        parent_coordinates = tuple(SCALE_FACTOR * x for x in parent_coordinates)
        cv.circle(map, parent_coordinates, radius=3, color=color, thickness=-1)
        draw_line(child_coordinates, parent_coordinates, map, color)

'''
run_simulation()

Runs an example program of the functions in the Project3_visual.py file
'''
def run_simulation():
    map = draw_map()
    valid_point_map = get_valid_point_map()

    draw_node((0, 0), None, map, GRAY)
    draw_node((50, 50), (0, 0), map, GRAY)
    draw_node((125, 125), (50, 50), map, GRAY)
    draw_node((145.5, 125.5), (125, 125), map, GRAY)
    cv.imshow('Djikstra\'s Algorith', map)
    cv.waitKey(0)
    cv.destroyAllWindows()


if __name__ == '__main__':
    run_simulation()
    valid_point_map = get_valid_point_map()
    print("done")


import heapq as hq
import copy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.patches import Polygon
import math
from matplotlib import pyplot as plt

side_length1 = 75
side_length2 = 80
center = (300, 125)
angle = 30
HexVert = []
HexVertClea = []
for i in range(6):
    x = center[0] + side_length1 * math.cos(math.pi / 3 * i + math.radians(angle))
    y = center[1] + side_length1 * math.sin(math.pi / 3 * i + math.radians(angle))
    HexVert.append((x, y))

for i in range(6):
    x = center[0] + side_length2 * math.cos(math.pi / 3 * i + math.radians(angle))
    y = center[1] + side_length2 * math.sin(math.pi / 3 * i + math.radians(angle))
    HexVertClea.append((x, y))
fig, ax = plt.subplots()
ax.set_xlim([0, 600])
ax.set_ylim([0, 250])
poly_1 = Rectangle((100, 0), 50, 100, linewidth=1, edgecolor='b', facecolor='b')
clea_1 = Rectangle((95, 0), 60, 105, linewidth=1, edgecolor='r', facecolor='none')
ax.add_patch(poly_1)
ax.add_patch(clea_1)
poly_2 = Rectangle((100,150), 50, 100, linewidth=1, edgecolor='b', facecolor='b')
clea_2 = Rectangle((95, 145), 60, 105, linewidth=1, edgecolor='r', facecolor='none')
ax.add_patch(poly_2)
ax.add_patch(clea_2)
poly_3 = Polygon([(460, 25), (460, 225), (510, 125)], linewidth=1, edgecolor='b', facecolor='b')
clea_3 = Polygon([(465, 20), (455, 20), (455, 230), (465, 230), (515, 125)], linewidth=1, edgecolor='r', facecolor='none')
ax.add_patch(poly_3)
ax.add_patch(clea_3)
poly_4 = Polygon(HexVert, linewidth=1, edgecolor='b', facecolor='b')
clea_4 = Polygon(HexVertClea, linewidth=1, edgecolor='r', facecolor='None')
ax.add_patch(poly_4)
ax.add_patch(clea_4)

Open_List = []
Closed_List = []
Closed_Coor = set()
threshold_coor = set()
node_index = 0
obstacle_points = set()
map_points = set()
x_range = np.arange(0, 600.5, 0.5)
y_range = np.arange(0, 250.5, 0.5)

def boundries (x,y):
    if x >= 5 and x <= 595 and y >= 5 and y <= 245:
        return True
    else: 
        return False

#check if coordinates are within obs_1
def obs_1 (x, y):
    if y-105 <= 0 and x-95 >= 0 and x-155 <= 0: 
        return True
    else:
        return False
    
#check if coordinates are within obs_2    
def obs_2 (x, y):
    if y-145 >= 0 and x-95 >= 0 and x-155 <= 0: 
        return True
    else:
        return False

#check if coordinates are within obs_3    
def obs_3 (x, y): 
    if y-20 >= 0 and y-230 <= 0 and x-455 >= 0 and math.ceil(y - 2.1*(x) + 956.5) >= 0 and math.ceil(y + 2.1*(x) - 1206.5) <= 0:
        return True
    else: 
        return False

#check if coordinates are within obs_4
def obs_4 (x, y): 
    if math.ceil(y - (4/7)*(x) + (885/7)) >= 0 and math.ceil(y + (4/7)*(x) - (1515/7)) >= 0 and math.ceil(y - (4/7)*(x) - (235/7)) <= 0 and math.ceil(y + (4/7)*(x) - (2635/7)) <= 0 and x - 230 >= 0 and x - 370 <= 0:

        return True
    else: 
        return False

def point_in_goal(x, y):
    distance = math.sqrt((x-goal_position[0])**2 + (y-goal_position[1])**2)
    if distance <= 5:
        return True
    else:
        return False

#creat 2 sets, 1 containing all the possible points within map, 1 containg all possible points within the obstacle spaces
#this will be used later to check the created points to see if they can be used
for x in x_range:
    for y in y_range:
        if boundries(x, y):
            map_points.add((x, y))
        if obs_1(x, y) or obs_2(x, y) or obs_3(x, y) or obs_4(x, y):
            obstacle_points.add((x, y))

start_position = (10, 10, 30)
goal_position = (595, 125, 30)
step_size = 10
clearance = 5
robot_size = 5
thresh = 0.5

if (start_position[0], start_position[1]) in obstacle_points:
    print("start point selected is in obstacle space, try again")
    exit()
if (start_position[0], start_position[1]) not in map_points:
    print("start point selected is  outside the map, try again")
    exit()
if (goal_position[0], goal_position[1]) in obstacle_points:
    print("start point selected is in obstacle space, try again")
    exit()
if (goal_position[0], goal_position[1]) not in map_points:
    print("start point selected is  outside the map, try again")
    exit()

def C2G_func (n_position, g_position): 
    C2G = round(((g_position[0]-n_position[0])**2 + (g_position[1]-n_position[1])**2)**0.5, 1)
    return C2G

C2G1 = C2G_func(start_position, goal_position)
C2C1 = 0
TC1 = C2G1 + C2C1
#(C2C, C2G, TC, node_Index, parent_coord(X,Y,Theta), point_coordinates(X,Y,Theta))
start_node = (C2G1, C2C1, TC1, node_index, None, start_position)
hq.heappush(Open_List, start_node)

def explore_pos30 (n):
    angle = 30
    rad = math.radians(angle)
    new_coor_x = round((n[5][0] + math.cos(rad)*step_size)*2)/2
    new_coor_y = round((n[5][1] + math.sin(rad)*step_size)*2)/2
    new_coor_theta = (n[5][2] + 30) % 360
    temp_xy_coor = (new_coor_x, new_coor_y)
    temp_C2C = n[0]+step_size
    temp_C2G = C2G_func(temp_xy_coor, goal_position)
    temp_TC = round(temp_C2C + temp_C2G, 1)
    new = (temp_C2G, temp_C2C, temp_TC, node_index, n[5], (new_coor_x, new_coor_y, new_coor_theta))
    return new

def explore_pos60 (n):
    angle = 60
    rad = math.radians(angle)
    new_coor_x = round((n[5][0] + math.cos(rad)*step_size)*2)/2
    new_coor_y = round((n[5][1] + math.sin(rad)*step_size)*2)/2
    new_coor_theta = (n[5][2] + 60) % 360
    temp_xy_coor = (new_coor_x, new_coor_y)
    temp_C2C = n[0]+step_size
    temp_C2G = C2G_func(temp_xy_coor, goal_position)
    temp_TC = round(temp_C2C + temp_C2G, 1)
    new = (temp_C2G, temp_C2C, temp_TC, node_index, n[5], (new_coor_x, new_coor_y, new_coor_theta))
    return new

def explore_neg30 (n):
    angle = -30
    rad = math.radians(angle)
    new_coor_x = round((n[5][0] + math.cos(rad)*step_size)*2)/2
    new_coor_y = round((n[5][1] + math.sin(rad)*step_size)*2)/2
    new_coor_theta = (n[5][2] - 30) % 360
    temp_xy_coor = (new_coor_x, new_coor_y)
    temp_C2C = n[0]+step_size
    temp_C2G = C2G_func(temp_xy_coor, goal_position)
    temp_TC = round(temp_C2C + temp_C2G, 1)
    new = (temp_C2G, temp_C2C, temp_TC, node_index, n[5], (new_coor_x, new_coor_y, new_coor_theta))
    return new

def explore_neg60 (n):
    angle = -60
    new_angle = n[5][2] + angle
    rad = math.radians(new_angle)
    new_coor_x = round((n[5][0] + math.cos(rad)*step_size)*2)/2
    new_coor_y = round((n[5][1] + math.sin(rad)*step_size)*2)/2
    new_coor_theta = (n[5][2] - 60) % 360 
    temp_xy_coor = (new_coor_x, new_coor_y)
    temp_C2C = n[0]+step_size
    temp_C2G = C2G_func(temp_xy_coor, goal_position)
    temp_TC = round(temp_C2C + temp_C2G, 1)
    new = (temp_C2G, temp_C2C, temp_TC, node_index, n[5], (new_coor_x, new_coor_y, new_coor_theta))
    return new

def explore_straight (n):
    angle = 0
    new_angle = n[5][2] + angle
    rad = math.radians(new_angle)
    new_coor_x = round((n[5][0] + math.cos(rad)*step_size)*2)/2
    new_coor_y = round((n[5][1] + math.sin(rad)*step_size)*2)/2
    new_coor_theta = (n[5][2] + 0) % 360 
    temp_xy_coor = (new_coor_x, new_coor_y)
    temp_C2C = n[1]+step_size
    temp_C2G = C2G_func(temp_xy_coor, goal_position)
    temp_TC = round(temp_C2C + temp_C2G, 1)
    new = (temp_C2G, temp_C2C, temp_TC, node_index, n[5], (new_coor_x, new_coor_y, new_coor_theta))
    return new

def exploreNodes(): 
    global goal_found
    hq.heapify(Open_List)
    while Open_List:
        if goal_found:
            break
        popped_node = hq.heappop(Open_List)
        Closed_Coor.add((popped_node[5][0], popped_node[5][1]))
        check_popped_status(popped_node)
        popped_node_dic = {"C2G": popped_node[0], "C2C": popped_node[1], "TC": popped_node[2], "node_index": popped_node[3], "parent_coor": popped_node[4], "node_coor": popped_node[5]}
        Closed_List.append(popped_node_dic)

        new_node = explore_pos30(copy.deepcopy(popped_node))
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                        checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore_pos60(copy.deepcopy(popped_node))
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                        checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore_neg30(copy.deepcopy(popped_node))
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                        checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore_pos60(copy.deepcopy(popped_node))
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                        checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore_straight(copy.deepcopy(popped_node))
        if ((new_node[5][0], new_node[5][1])) in map_points: 
            if ((new_node[5][0], new_node[5][1])) not in obstacle_points:
                if ((new_node[5][0], new_node[5][1])) not in Closed_Coor:
                    if threshhold(new_node[5][0], new_node[5][1], new_node[5][2]):
                        checkC2C(copy.deepcopy(popped_node), new_node)
        return Open_List, Closed_Coor, Closed_List

def threshhold(nx, ny, nt):
    if (nx, ny, nt) in threshold_coor: 
        return True
    else: 
        threshold_coor.add((nx+0.5, ny, nt))
        threshold_coor.add((nx+0.5, ny+0.5, nt))
        threshold_coor.add((nx, ny+0.5, nt))
        threshold_coor.add((nx-0.5, ny+0.5, nt))
        threshold_coor.add((nx-0.5, ny, nt))
        threshold_coor.add((nx-0.5, ny-0.5, nt))
        threshold_coor.add((nx, ny-0.5, nt))
        threshold_coor.add((nx, ny-0.5, nt))
        return threshold_coor, False

#check if newly explored point has been explored previously, if so compare C2C and update if the new C2C is lower than the one originally stored
def checkC2C (on, n):
    global node_index
    # for i, nodes in enumerate(Open_List):
    #     if nodes[5] == n[5]:
    #         if n[2] < nodes[2]:
    #             new_node = (n[0], n[1], n[2], nodes[3], n[5], nodes[5])
    #             Open_List[i] = new_node
    #             hq.heapify(Open_List)
    #         return Open_List
    # else:
    node_index += 1
    new = (n[0], n[1], n[2], node_index, on[5], n[5])
    hq.heappush(Open_List, new)
    hq.heapify(Open_List)
    return Open_List

def check_popped_status (n):
    global goal_found
    if point_in_goal(n[5][0], n[5][1]) and n[5][2] == goal_position[2]:
        goal_node = {"C2G": n[0], "C2C": n[1], "TC": n[2], "node_index": n[3], "parent_coor": n[4], "node_coor": n[5]}
        Closed_List.append(goal_node)
        print("goal node position:", n[5])
        print("target position:", goal_position)
        print("Goal found")
        print("destination info:", n)
        goal_found = True
        start_backtrack ()
    else: 
        return(n)


def plot_function(path):
    closed_nodes = [node['node_coor'] for node in Closed_List]
    # plot_closed_nodes()
    # path_nodes = [node['node_coor'] for node in path]
    x_coords, y_coords = zip(*path)
    plt.scatter(*zip(*closed_nodes), marker='o', color='green', alpha=.1)
    plt.scatter(x_coords, y_coords, marker='o', color='black', alpha=1)
    # plt.plot(*zip(*path_nodes), color='black')


def start_backtrack (): 
    path_nodes = []
    path_coor = []
    current_node = Closed_List[-1]
    path_nodes.append(current_node)
    path_coor.append((current_node['node_coor'][0], current_node['node_coor'][1]))
    print("First node used:", current_node)
    
# (C2C, point_index, (x,y)parent_coordinates, (x,y)coordinates)
    while current_node["parent_coor"] is not None:
        search_value = current_node["parent_coor"]
        for node in Closed_List:
            if node["node_coor"] == search_value:
                # If a matching value is found, assign the entire dictionary as the new current_node
                current_node = node
                break
        path_nodes.append(current_node)
        path_coor.append((current_node['node_coor'][0], current_node['node_coor'][1]))
    print(path_coor)
    plot_function(path_coor)
    plt.show()
    print("length of closed list:", len(Closed_List))    
    print("length of closed path:", len(path_nodes)) 
    print("length of closed path coor:", len(path_coor))

    

# print("start node:", start_node)
# y = explore_straight(start_node)
# print(y)

goal_found = False
print("start node:", start_node)
while not goal_found:
    exploreNodes()
    


# y = exploreNodes()
# print("Open_List:", Open_List)
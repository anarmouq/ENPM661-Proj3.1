import heapq as hq
import copy
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
node_index = 0
obstacle_points = set()
map_points = set()
x_range = range(0, 650)
y_range = range(0, 250)
#each point is saved as follows: (C2C, point_index, (x,y)parent_coordinates, (x,y)coordinates)
#user input needed to specify goal position coordinates
# start_x_position = int(input("enter start X position(0-600) "))
# start_y_position = int(input("enter start y position(0-250) "))
# start_position = (0, node_index, None, (start_x_position,start_y_position))
# if start_position[3] in obstacle_points:
#     print("start point selected is in obstacle space, try again")
#     print(start_position)
#     exit()
# if start_position[3] in map_points:
#     print("start point selected is  outside the map, try again")
#     print(start_position)
#     exit()

#user input needed to specify goal position coordinates
# goal_x_position = int(input("enter start X position (0-600) "))
# goal_y_position = int(input("enter start X position (0-250) "))
# goal_position = (goal_x_position,goal_y_position)
# if goal_position in obstacle_points or goal_position in map_points:
#     print("goal point selected is either outside the map or in obstacle space, try again")
#     exit()
# if goal_position in map_points:
#     print("start point selected is  outside the map, try again")
#     print(start_position)
#     exit()
# hq.heappush(Open_List, start_position)

#check if coordinates are within the map
def boundries (x,y):
    if x >= 0 and x <= 600 and y >= 0 and y <= 250:
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


#creat 2 sets, 1 containing all the possible points within map, 1 containg all possible points within the obstacle spaces
#this will be used later to check the created points to see if they can be used
for x in x_range:
    for y in y_range:
        if boundries(x, y):
            map_points.add((x, y))
        if obs_1(x, y) or obs_2(x, y) or obs_3(x, y) or obs_4(x, y):
            obstacle_points.add((x, y))

start_x_position = int(input("enter start X position(0-600) "))
start_y_position = int(input("enter start y position(0-250) "))
start_coor = (start_x_position, start_y_position)
start_position = (0, node_index, None, (start_x_position,start_y_position))
if start_coor in obstacle_points:
    print("start point selected is in obstacle space, try again")
    print(start_position)
    exit()
if start_coor not in map_points:
    print("start point selected is  outside the map, try again")
    print(start_position)
    exit()

goal_x_position = int(input("enter start X position (0-600) "))
goal_y_position = int(input("enter start X position (0-250) "))
goal_position = (goal_x_position,goal_y_position)
if goal_position in obstacle_points:
    print("goal point selected is in obstacle space, try again")
    exit()
if goal_position not in map_points:
    print("start point selected is  outside the map, try again")
    print(goal_position)
    exit()
hq.heappush(Open_List, start_position)

#explore point to the right and create a new node
def explore_right(n): 
    new = (round((n[0] + 1), 1) , node_index, n[1], (n[3][0]+1, n[3][1]))
    return new

#explore point to the left and create a new node
def explore_left(n): 
    new = (round((n[0] + 1), 1) , node_index, n[1], (n[3][0]-1, n[3][1]))
    return new

#explore point upwards and create a new node
def explore_up(n): 
    new = (round((n[0] + 1), 1) , node_index, n[1], (n[3][0], n[3][1]+1))
    return new

#explore point downwards and create a new node
def explore_down(n): 
    new = (round((n[0] + 1), 1) , node_index, n[1], (n[3][0], n[3][1]-1))
    return new

#explore upper-right point and create a new node
def explore_upRight(n): 
    new = (round((n[0] + 1.4), 1) , node_index, n[1], (n[3][0]+1, n[3][1]+1))
    return new

#explore upper-left point and create a new node
def explore_upLeft(n): 
    new = (round((n[0] + 1.4), 1) , node_index, n[1], (n[3][0]-1, n[3][1]+1))
    return new

#explore down-right point and create a new node
def explore_downRight(n): 
    new = (round((n[0] + 1.4), 1) , node_index, n[1], (n[3][0]+1, n[3][1]-1))
    return new

#explore down-left point and create a new node
def explore_downLeft(n): 
    new = (round((n[0] + 1.4), 1) , node_index, n[1], (n[3][0]-1, n[3][1]-1))
    return new

#Main function that takes in the start position and explores all possible points until the goal point has been located
def exploreNodes(): 
    global goal_found
    hq.heapify(Open_List)
    while Open_List:
        if goal_found:
            break
        popped_node = hq.heappop(Open_List)
        Closed_Coor.add((popped_node[3][0], popped_node[3][1]))
        check_popped_status(popped_node)
        popped_node_dic = {"C2C": popped_node[0], "node_index": popped_node[1], "parent_node": popped_node[2], "node_coor": popped_node[3]}
        Closed_List.append(popped_node_dic)

        new_node = explore_right(copy.deepcopy(popped_node))
        if ((new_node[3][0], new_node[3][1])) in map_points: 
            if ((new_node[3][0], new_node[3][1])) not in obstacle_points:
                if ((new_node[3][0], new_node[3][1])) not in Closed_Coor:
                    checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore_left(copy.deepcopy(popped_node))
        if ((new_node[3][0], new_node[3][1])) in map_points: 
            if ((new_node[3][0], new_node[3][1])) not in obstacle_points:
                if ((new_node[3][0], new_node[3][1])) not in Closed_Coor:
                    checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore_up(copy.deepcopy(popped_node))
        if ((new_node[3][0], new_node[3][1])) in map_points: 
            if ((new_node[3][0], new_node[3][1])) not in obstacle_points:
                if ((new_node[3][0], new_node[3][1])) not in Closed_Coor:
                    checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore_down(copy.deepcopy(popped_node))
        if ((new_node[3][0], new_node[3][1])) in map_points: 
            if ((new_node[3][0], new_node[3][1])) not in obstacle_points:
                if ((new_node[3][0], new_node[3][1])) not in Closed_Coor:
                    checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore_upRight(copy.deepcopy(popped_node))
        if ((new_node[3][0], new_node[3][1])) in map_points: 
            if ((new_node[3][0], new_node[3][1])) not in obstacle_points:
                if ((new_node[3][0], new_node[3][1])) not in Closed_Coor:
                    checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore_upLeft(copy.deepcopy(popped_node))
        if ((new_node[3][0], new_node[3][1])) in map_points: 
            if ((new_node[3][0], new_node[3][1])) not in obstacle_points:
                if ((new_node[3][0], new_node[3][1])) not in Closed_Coor:
                    checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore_downRight(copy.deepcopy(popped_node))
        if ((new_node[3][0], new_node[3][1])) in map_points: 
            if ((new_node[3][0], new_node[3][1])) not in obstacle_points:
                if ((new_node[3][0], new_node[3][1])) not in Closed_Coor:
                    checkC2C(copy.deepcopy(popped_node), new_node)
        new_node = explore_downLeft(copy.deepcopy(popped_node))
        if ((new_node[3][0], new_node[3][1])) in map_points: 
            if ((new_node[3][0], new_node[3][1])) not in obstacle_points:
                if ((new_node[3][0], new_node[3][1])) not in Closed_Coor:
                    checkC2C(copy.deepcopy(popped_node), new_node)
        return Open_List, Closed_Coor, Closed_List

#check if newly explored point has been explored previously, if so compare C2C and update if the new C2C is lower than the one originally stored
def checkC2C (on, n):
    global node_index
    for i, nodes in enumerate(Open_List):
        if nodes[3] == n[3]:
            if n[0] < nodes[0]:
                new_node = (n[0], nodes[1], n[3], nodes[3])
                Open_List[i] = new_node
                hq.heapify(Open_List)
            return Open_List
    else:
        node_index += 1
        new = (n[0], node_index, on[3], n[3])
        hq.heappush(Open_List, new)
    return Open_List

#check if the point in discussion is the goal node, if so start the backtracking function. if not, store the new point in the open_list and move on
def check_popped_status (n):
    global goal_found
    if n[3] == goal_position:
        goal_node = {"C2C": n[0], "node_index": n[1], "parent_node": n[2], "node_coor": n[3]}
        Closed_List.append(goal_node)
        print("Goal found")
        print("destination info:", n)
        goal_found = True
        start_backtrack ()
    else: 
        return(n)

#function to plot the explored/closed notes that were looked at on the way to the goal node
def plot_closed_nodes():
    n = 5000 # insert a pause every 8000 nodes
    for i, node in enumerate(Closed_List):
        x, y = node['node_coor']
        plt.scatter(x, y, marker='o', color='green', alpha=0.1)
        if i % n == 0:
            plt.pause(0.00000001)


#plot function that plots the closed nodes on the way to the goal and then plots the path from the start node to the goal node in black
def plot_function(path):
    closed_nodes = [node['node_coor'] for node in Closed_List]
    # plot_closed_nodes()
    path_nodes = [node['node_coor'] for node in path]
    plt.scatter(*zip(*closed_nodes), marker='o', color='green', alpha=0.1)
    plt.scatter(*zip(*path_nodes), marker='o', color='black', alpha=1, s=50)

    
    
#when the goal point has been located, trace back through the closed nodes using the parent_coordinates to generate a list containing 
#the path from the start node to the solution
def start_backtrack (): 
    path = []
    current_node = Closed_List[-1]
    path.append(current_node)
    print("First node used:", current_node)
# (C2C, point_index, (x,y)parent_coordinates, (x,y)coordinates)
    while current_node["parent_node"] is not None:
        search_value = current_node["parent_node"]
        for node in Closed_List:
            if node["node_coor"] == search_value:
                # If a matching value is found, assign the entire dictionary as the new current_node
                current_node = node
                break
        path.append(current_node)
    plot_function(path)
    plt.show()
    

#main line that runs the code, while the goal is not met, continue looking while exploring nodes
goal_found = False
while not goal_found:
    exploreNodes()







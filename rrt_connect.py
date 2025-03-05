import numpy as np
import heapq
import math
import cv2
import random
import time
import matplotlib.pyplot as plt
start_time = time.time()
#------------------------------------------------------------
#----------------------DEFINING THE MAP----------------------
#------------------------------------------------------------
# empty map
map_image = np.zeros((200, 600, 3), dtype=np.uint8)
# radius = int(input("enter the radius of the robot: ")) #radius of the robot
# clearance = int(input("enter the obstacle clearance: ")) #clearance of the obstacles

radius = 2.2
clearance = 10

no_go_points = set()
#obsatcles initialization 
for i in range(0,601): #6000 width
    for j in range(0,201): #2000 height

            #the fist rectangle
            if(150-clearance-radius <= i <= 175+clearance+radius and 0 <= j <= 100+clearance+radius):
                cv2.circle(map_image, (int(i), int(j)), 2, (0, 255, 0), -1)
                no_go_points.add((i, j))

            #second rectangle
            elif(250-clearance-radius <= i <= 275+clearance+radius and 100-clearance-radius <= j <= 200):
                cv2.circle(map_image, (int(i), int(j)), 2, (0, 255, 0), -1)
                no_go_points.add((i, j))
            
            #circle obstacle
            elif((i - 420) ** 2 + (j -80) ** 2 <= (60 + radius + clearance)**2):
                cv2.circle(map_image, (int(i), int(j)), 2, (0, 255, 0), -1)
                no_go_points.add((i, j))


            #boundries
            elif(j<= radius+clearance or j>=200-radius-clearance):
                cv2.circle(map_image, (int(i), int(j)), 2, (0, 255, 0), -1)
                no_go_points.add((i, j))
            elif(i<= radius+clearance or i>= 600-radius-clearance):
                cv2.circle(map_image, (int(i), int(j)), 2, (0, 255, 0), -1)
                no_go_points.add((i, j))
            
            #everything else is white
            else:
                cv2.circle(map_image, (int(i), int(j)), 2, (255, 255, 255), -1)

#displaying the map after alloting the relevant space for radius and clearance
# img_gray = cv2.cvtColor(map_image,cv2.COLOR_BGR2GRAY)
cv2.imshow('map',map_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

start_time = time.time()
height, width, _ = map_image.shape 

def is_free(x, y):
    return (x, y) not in no_go_points


def generate_random_points(width = 600, height = 200):
    x = random.randint(0, width - 1)
    y = random.randint(0, height - 1)
    
    return x,y

def heuristic(node, rand_pt):
    return math.sqrt((node[0] - rand_pt[0]) ** 2 + (node[1] - rand_pt[1]) ** 2)

def move_towards(closest_node, rand_pt, step_size):
    # Calculate the vector from closest_node to rand_pt
    vector_x = rand_pt[0] - closest_node[0]
    vector_y = rand_pt[1] - closest_node[1]
    
    #calculate the distance between the nodes
    distance = math.sqrt(vector_x**2 + vector_y**2)
    if distance == 0:
        return closest_node
    #normalize the vector
    unit_vector_x = vector_x / distance
    unit_vector_y = vector_y / distance
    
    #move by step_size along the direction to rand_pt
    next_node_x = closest_node[0] + unit_vector_x * step_size
    next_node_y = closest_node[1] + unit_vector_y * step_size
    
    #Ensure the next node is within bounds and rounds to integer coordinates
    next_node = (int(next_node_x), int(next_node_y))
    
    return next_node


def is_goal_reached(current_node, goal_node, distance_tolerance=5):
    distance = heuristic(current_node, goal_node)
    # checking if the current node is with in distance of 1.5 from the goal node
    return distance <= distance_tolerance

def backtrack(parents, node):
    path = []
    while node is not None:
        path.append(node)
        node = parents.get(node)
    path.reverse()
    return path

start,goal = (100,50), (500, 50)

step_size = 10
# def RRTalgorithim(start,rand_pt):
visited_start = set()
visited_goal = set()

visited_start.add(start)
visited_goal.add(goal)
node_parent_map_start = {start: None}
node_parent_map_goal = {goal: None}
free_space = np.all(map_image == [255, 255, 255], axis=2)

while visited_start or visited_goal:
    rand_pt_start = generate_random_points()
    rand_pt_goal = generate_random_points()
    distance = 0.0
    dis = 0.0


    costs_start = {start: 0}
    for i in visited_start:
        distance = heuristic(i,rand_pt_start)
        costs_start[i] = distance
    # sorted_costs = sorted(costs.items(), key=lambda item: item[1])
    closest_node_start = min(costs_start, key=costs_start.get)
    # print(closest_node)
    next_node_start = move_towards(closest_node_start, rand_pt_start, step_size)


    costs_goal = {goal: 0}
    for j in visited_goal:
        dis = heuristic(j,rand_pt_goal)
        costs_goal[j] = dis
    closest_node_goal = min(costs_goal, key=costs_goal.get)
    # closest_node = min(closest_node_goal,closest_node_start)
    # print(closest_node)
    next_node_goal = move_towards(closest_node_goal, rand_pt_goal, step_size)

    if is_free(next_node_start[0], next_node_start[1]) and next_node_start not in visited_start :
        visited_start.add(next_node_start)
        node_parent_map_start[next_node_start] = closest_node_start  # Store the parent of this new node
        print(f"Moving from(start) {closest_node_start} to {next_node_start} towards {rand_pt_start}")

    if is_free(next_node_goal[0], next_node_goal[1]) and next_node_goal not in visited_goal :
        visited_goal.add(next_node_goal)
        node_parent_map_goal[next_node_goal] = closest_node_goal  # Store the parent of this new node
        print(f"Moving from(goal) {closest_node_goal} to {next_node_goal} towards {rand_pt_goal}")

    if next_node_start in visited_goal or any(heuristic(next_node_start, other) <= step_size for other in visited_goal):
        connection = next_node_start
        connection_node_goal = min((node for node in visited_goal if heuristic(node, connection) <= step_size), key=lambda n: heuristic(n, connection))
        break
    if next_node_goal in visited_start or any(heuristic(next_node_goal, other) <= step_size for other in visited_start):
        connection = next_node_goal
        connection_node_start = min((node for node in visited_start if heuristic(node, connection) <= step_size), key=lambda n: heuristic(n, connection))
        break
    
def combine_paths(start_path, goal_path):
    # Reverse the goal path to start from the connection node to the goal node
    goal_path = goal_path[::-1]
    # Combine the two paths
    full_path = start_path + goal_path
    return full_path

if connection:
    path_from_start = backtrack(node_parent_map_start, connection_node_start if 'connection_node_start' in locals() else connection)
    path_from_goal = backtrack(node_parent_map_goal, connection_node_goal if 'connection_node_goal' in locals() else connection)
    full_path = combine_paths(path_from_start, path_from_goal)

#Remove duplicate connection point and combine paths
full_path = path_from_start + path_from_goal[::-1][1:]

print(full_path)


def visualize(img, path,node_parent_map_start,node_parent_map_goal):
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter('output_harshavarthan_hariharasudan_RRT_Connect.mp4', fourcc, 20.0, (width, height))
    cv2.circle(img, start, 2, (0, 255, 0), -1)
    # drawing the goal in blue
    cv2.circle(img, goal, 2, (255, 0, 0), -1)

    for node, parent in node_parent_map_start.items():
        if parent is not None:
            cv2.line(img, parent, node, (255, 0, 0), 1)  # Blue lines for edges
            cv2.circle(img, node, 2, (0, 0, 255), -1)    # Red points for nodes
            video.write(img)
    for node, parent in node_parent_map_goal.items():
        if parent is not None:
            cv2.line(img, parent, node, (255, 0, 0), 1)  # Blue lines for edges
            cv2.circle(img, node, 2, (0, 0, 255), -1)    # Red points for nodes
            video.write(img)
    for path_plot in path:
        cv2.circle(img,path_plot, 2, (0, 255, 255), -1)
        video.write(img)
    video.release()

visualize(map_image,full_path,node_parent_map_start,node_parent_map_goal)
print("Total Time Taken : ",time.time() - start_time, "seconds")



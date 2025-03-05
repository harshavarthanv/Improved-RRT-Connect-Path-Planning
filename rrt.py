import numpy as np
import heapq
import math
import cv2
import random
import time
import matplotlib.pyplot as plt

#------------------------------------------------------------
#----------------------DEFINING THE MAP----------------------
#------------------------------------------------------------
#empty map
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

height, width, _ = map_image.shape 
start_time = time.time()

def is_free(x, y):
    return (x, y) not in no_go_points


def generate_random_points(width = 600, height = 200):
    x = random.randint(0, width - 1)
    y = random.randint(0, height - 1)
    
    return x,y

def heuristic(node, rand_pt):
    return math.sqrt((node[0] - rand_pt[0]) ** 2 + (node[1] - rand_pt[1]) ** 2)

def move_towards(closest_node, rand_pt, step_size):
    #calculate the vector from closest_node to rand_pt
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
    
    #ensure the next node is within bounds and rounds to integer coordinates
    next_node = (int(next_node_x), int(next_node_y))
    
    return next_node


def is_goal_reached(current_node, goal_node, distance_tolerance=5):
    distance = heuristic(current_node, goal_node)
    # checking if the current node is with in distance of 1.5 from the goal node
    return distance <= distance_tolerance


def backtrack(parents, start, goal):
    path = [goal]
    while path[-1] != start:
        path.append(parents[path[-1]])
    path.reverse()
    return path

# start = 100,100
# goal = 300,50
start,goal = (100,50), (500, 50)
step_size = 10
#def RRT
visited = set()
visited.add(start)
node_parent_map = {start: None}
free_space = np.all(map_image == [255, 255, 255], axis=2)
while visited:
    rand_pt = generate_random_points()
    distance = 0.0
    
    costs = {start: 0}
    for i in visited:
        distance = heuristic(i,rand_pt)
        costs[i] = distance
    # sorted_costs = sorted(costs.items(), key=lambda item: item[1])
    closest_node = min(costs, key=costs.get)
    # print(closest_node)
    next_node = move_towards(closest_node, rand_pt, step_size)

    if is_free(next_node[0], next_node[1]) and next_node not in visited:
        visited.add(next_node)
        node_parent_map[next_node] = closest_node  # Store the parent of this new node
        print(f"Moving from {closest_node} to {next_node} towards {rand_pt}")

    if is_goal_reached(next_node, goal):
        print('goal reached')
        node_parent_map[goal] = next_node
        path = backtrack(node_parent_map,start,goal)
        break

print(path)


def visualize(img, path,node_parent_map):
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter('output_harshavarthan_hariharasudan_RRT.mp4', fourcc, 20.0, (width, height))
    cv2.circle(img, start, 2, (0, 255, 0), -1)
    # drawing the goal in blue
    cv2.circle(img, goal, 2, (255, 0, 0), -1)

    for node, parent in node_parent_map.items():
        if parent is not None:
            cv2.line(img, parent, node, (255, 0, 0), 1)  # Blue lines for edges
            cv2.circle(img, node, 2, (0, 0, 255), -1)    # Red points for nodes
            video.write(img)

    for path_plot in path:
        cv2.circle(img,path_plot, 2, (0, 255, 255), -1)
        video.write(img)
    video.release()

visualize(map_image,path,node_parent_map)
print("Total Time Taken : ",time.time() - start_time, "seconds")
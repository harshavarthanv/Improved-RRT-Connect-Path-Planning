import numpy as np
import math
import cv2
import random
import time


#initialize the map
radius = 2.2
clearance = 10
map_image = np.zeros((200, 600, 3), dtype=np.uint8)

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
cv2.imshow('map',map_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
start_time = time.time()
height, width, _ = map_image.shape

def generate_random_points(x_min, x_max, height=200):
    """Generate random points within specified x-coordinate boundaries."""
    x = random.randint(x_min, x_max)
    y = random.randint(0, height - 1)
    return x, y


#heuristic function to compute distance
def heuristic(node, target):
    return math.sqrt((node[0] - target[0]) ** 2 + (node[1] - target[1]) ** 2)

#function to move towards a target point
def move_towards(closest_node, target, step_size):
    dx, dy = target[0] - closest_node[0], target[1] - closest_node[1]
    distance = math.hypot(dx, dy)
    if distance == 0:
        return closest_node
    unit_dx, unit_dy = dx / distance, dy / distance
    next_node_x = closest_node[0] + unit_dx * step_size
    next_node_y = closest_node[1] + unit_dy * step_size
    return int(next_node_x), int(next_node_y)

#backtrack function to create a path from parents mapping
def backtrack(parents, node):
    path = []
    while node is not None:
        path.append(node)
        node = parents.get(node)
    path.reverse()
    return path

def is_line_of_sight_free(start, end, no_go_points):
    """Check if the line between start and end is free of obstacles using Bresenham's line algorithm."""
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy

    while True:
        if (x0, y0) in no_go_points:
            return False
        if (x0, y0) == (x1, y1):
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return True


def find_rrt_path(start, goal, step_size, no_go_points, x_min, x_max):
    visited_start = {start}
    visited_goal = {goal}
    node_parent_map_start = {start: None}
    node_parent_map_goal = {goal: None}

    while True:
        # rand_pt_start = generate_random_points(start,goal)
        # rand_pt_goal = generate_random_points(start,goal)
        rand_pt_start = generate_random_points(x_min,x_max)
        rand_pt_goal = generate_random_points(x_min,x_max)

        closest_node_start = min(visited_start, key=lambda n: heuristic(n, rand_pt_start))
        closest_node_goal = min(visited_goal, key=lambda n: heuristic(n, rand_pt_goal))

        next_node_start = move_towards(closest_node_start, rand_pt_start, step_size)
        next_node_goal = move_towards(closest_node_goal, rand_pt_goal, step_size)

        if next_node_start not in visited_start and next_node_start not in no_go_points:
            visited_start.add(next_node_start)
            node_parent_map_start[next_node_start] = closest_node_start
            #check for direct connection to any node in the goal tree
            for goal_node in visited_goal:
                if is_line_of_sight_free(next_node_start, goal_node, no_go_points):
                    node_parent_map_start[goal_node] = next_node_start
                    return backtrack(node_parent_map_start, goal_node) + backtrack(node_parent_map_goal, goal_node)[::-1],node_parent_map_start, node_parent_map_goal

        if next_node_goal not in visited_goal and next_node_goal not in no_go_points:
            visited_goal.add(next_node_goal)
            node_parent_map_goal[next_node_goal] = closest_node_goal
            # Check for direct connection to any node in the start tree
            for start_node in visited_start:
                if is_line_of_sight_free(next_node_goal, start_node, no_go_points):
                    node_parent_map_goal[start_node] = next_node_goal
                    return backtrack(node_parent_map_start, start_node) + backtrack(node_parent_map_goal, start_node)[::-1],node_parent_map_start, node_parent_map_goal
# Example usage of the function
start,goal = (100,50), (500, 50)



step_size = 10
#from the start to mid tree

def find_mid(start, goal, no_go_points):
    x_1, y_1 = start[0],start[1]
    x_2, y_2 = goal[0],goal[1]
    mid_x = (x_1 + x_2)/2
    mid_y = (y_1 + y_2)/2
    mid = (int(mid_x),int(mid_y))
    return mid

mid = find_mid(start,goal,no_go_points)

x_min_start = min(start[0], mid[0])  # Depending on your coordinate setup
x_max_mid = max(start[0], mid[0])

#from the mid to goal tree
x_min_goal = min(goal[0], mid[0])
x_max_end = max(goal[0], mid[0])
# Find the path
full_path, node_parent_map_start, node_parent_map_goal = find_rrt_path(start, mid, step_size, no_go_points, x_min_start, x_max_mid)
full_path_1, node_parent_map_start_1, node_parent_map_goal_1 = find_rrt_path(goal,mid, step_size, no_go_points, x_min_goal, x_max_end)
#visualization function for debugging
def visualize(img, path, node_parent_map_start, node_parent_map_goal, node_parent_map_goal_1,node_parent_map_start_1):
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter('output_rrt.mp4', fourcc, 20.0, (img.shape[1], img.shape[0]))
    cv2.circle(img, start, 2, (0, 255, 0), -1)
    cv2.circle(img, goal, 2, (255, 0, 0), -1)
    cv2.circle(img, mid, 2, (255, 0, 0), -1)

    for node, parent in node_parent_map_start.items():
        if parent:
            cv2.line(img, parent, node, (255, 0, 0), 1)
            cv2.circle(img, node, 2, (0, 0, 255), -1)
            video.write(img)
    for node, parent in node_parent_map_goal.items():
        if parent:
            cv2.line(img, parent, node, (255, 0, 0), 1)
            cv2.circle(img, node, 2, (0, 0, 255), -1)
            video.write(img)
    for node, parent in node_parent_map_goal_1.items():
        if parent:
            cv2.line(img, parent, node, (255, 0, 0), 1)
            cv2.circle(img, node, 2, (0, 0, 255), -1)
            video.write(img)
    for node, parent in node_parent_map_start_1.items():
        if parent:
            cv2.line(img, parent, node, (255, 0, 0), 1)
            cv2.circle(img, node, 2, (0, 0, 255), -1)
            video.write(img)
    for path_plot in path:
        cv2.circle(img, path_plot, 2, (0, 255, 255), -1)
        video.write(img)
    video.release()



full_path = full_path + full_path_1[::-1][1:]
#visualize the output
visualize(map_image, full_path, node_parent_map_start, node_parent_map_goal, node_parent_map_goal_1,node_parent_map_start_1)
print("Final Path:", full_path)
print("Total Time Taken:", time.time() - start_time, "seconds")
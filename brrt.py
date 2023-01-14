import matplotlib.pyplot as plt
import random
import math
import numpy as np
# PARAMETERS AND INITIAL SETTINGS

START = (1, 1)
GOAL = (20, 20)
MAX_DIST = 2
MAX_ITER = 400
CIRCLE_OBSTACLES = [(4.5, 3, 2), (3, 12, 2), (15, 15, 3)]


def line_intersects_circle(x1, y1, x2, y2, a, b, r):
    # subsitution of circle eq in line euqation and solve for x
    # two unkowns, two equations

    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1
    A = m**2 + 1
    B = 2 * (m * b - m * a - a)
    C = a**2 + b**2 - 2 * b * a + r**2 - r**2
    discriminant = B**2 - 4 * A * C
    if discriminant < 0:
        # No intersection
        return False    

    # line intersects, but we care only about  intserction of LINE SEGMENT for checking the intersection with obstacles
 
   
    x_intersect = (-B + math.sqrt(discriminant)) / (2 * A)
    y_intersect = m * x_intersect + b
    
    # Check if the intersection point is within the bounds of the line segment
    if (x_intersect >= min(x1, x2) and x_intersect <= max(x1, x2)) and (y_intersect >= min(y1, y2) and y_intersect <= max(y1, y2)):
        return True
    else:
        return False
    

def check_in_circle(point, circles):
    for i in circles:
        if math.sqrt((point[0] - i[0]) ** 2 + (point[1] - i[1]) ** 2) <= i[2]:
            return True
    return False


def check_connectable(p1, p2, circle_obstacles):
    
    for c in circle_obstacles:
        if (line_intersects_circle(*p1,*p2,*c)):
            return False

    return True   

def interpolate_second_if_big(p1,p2,max_dist):
    p1 = np.array(p1)
    p2 = np.array(p2)
    #return (p2[0],p2[1])
    dir_vec = p2-p1
    if (math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2) > max_dist):
        p2 = p1 +  (dir_vec/(np.linalg.norm(dir_vec))) * 1
    return (p2[0],p2[1])
def build_rrt(start, goal, iterations, max_dist, circle_obstacles):
    node_list1 = [start]
    node_list2 = [goal]
    node_dict1 = {start:()}
    node_dict2 = {goal:()}

    
    for i in range(iterations):
        while True:
            candidate1 = (random.uniform(0, 30), random.uniform(0, 30))
            if not check_in_circle(candidate1,circle_obstacles):
                sorted_list = sorted(node_dict1.keys(),key = lambda x:math.sqrt((candidate1[0]-x[0])**2 + (candidate1[1]-x[1])**2))

                flag_no_connect = True
                for i in sorted_list:
                    if(check_connectable(i,candidate1,circle_obstacles)):
                        candidate1 = interpolate_second_if_big(i,candidate1,max_dist)
                        if check_in_circle(candidate1,circle_obstacles):
                            break;
                        node_dict1[candidate1]=[(i)]
                        node_list1+=[(candidate1)]

                        flag_no_connect = False
                        break
                    else:
                        print("candidate not connectable")
                
                if not flag_no_connect:
                    break;


        while True:
            candidate2 = (random.uniform(0, 30), random.uniform(0, 30))
            if not check_in_circle(candidate2,circle_obstacles):
                sorted_list = sorted(node_dict2.keys(),key = lambda x:math.sqrt((candidate2[0]-x[0])**2 + (candidate2[1]-x[1])**2))

                flag_no_connect = True
                for i in sorted_list:
                    if(check_connectable(i,candidate2,circle_obstacles)):
                        candidate2 = interpolate_second_if_big(i,candidate2,max_dist)
                        if check_in_circle(candidate2,circle_obstacles):
                            break;

                        node_dict2[candidate2]=[(i)]
                        node_list2+=[(candidate2)]

                        flag_no_connect = False
                        break
                    else:
                        print("candidate not connectable")
                
                if not flag_no_connect:
                    break;
 



        for i in node_list1:
            sorted_node_list2 = sorted(node_list2 ,key = lambda a:math.sqrt((i[0]-a[0])**2 + (i[1]-a[1])**2))
            
            for k in sorted_node_list2:
                if check_connectable(i,k,circle_obstacles) and math.sqrt((k[0]-i[0])**2 + (k[1]-i[1])**2) < max_dist:
                    return node_list1,node_list2                    

    return node_list1,node_list2


# initialize the plot
plt.ion()  # turn on interactive mode
fig, ax = plt.subplots()




plt_circles = [plt.Circle((i[0],i[1]),i[2], color='g') for i in CIRCLE_OBSTACLES]

for p in plt_circles:
    ax.add_patch(p)

# build the RRT



tree1,tree2 = build_rrt(START, GOAL, MAX_ITER, MAX_DIST, CIRCLE_OBSTACLES)

# plot the points on the RRT incrementally
for p1,p2 in zip(tree1,tree2):
    ax.plot(p1[0], p1[1], "bo")
    ax.plot(p2[0], p2[1], "ro")

    plt.pause(0.1)  # pause for 0.1 seconds to allow the plot to be updated

# show the plot
plt.show()

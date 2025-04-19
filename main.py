import pygame
import math
import heapq
import time


# Defining Graph Constants
height = 500
width = 1500



class Node:
    """
    Node class : This class is built to store the node information.
    A node is simply a location on a map. For each node, its adjacent_node, parents & distance to reach that node is stored.
    """

    def __init__(self, i, j, endI, endJ, theta):
        """
        Description: Defining all properties for each node - adjacent_node, Parents, Distance.
        """
        self.i = i
        self.j = j
        self.theta = theta
        self.cost_to_come = 0.0
        self.cost_to_go = 2.5*(math.sqrt((i - endI) ** 2 + (j - endJ) ** 2))
        self.cost = None
        self.adjacent_node = {}
        self.ok_actions = {}
        self.parent = None

    def __lt__(self, other):
        return self.cost < other.cost

class map_exploration:
    """
    Graph class : This class defines all methods to generate a graph and perform AStar Algorithm.
    """

    def __init__(self, start, end, RPM1, RPM2, radius, clearance):
        self.visited = {}
        self.endI = end.i
        self.endJ = end.j
        self.RPM1 = RPM1
        self.RPM2 = RPM2
        self.radius = radius
        self.clearance = clearance + self.radius

    def get_adjacent_node(self, current_node):
        """
        Description: Returns adjacent_node for the current_node.
        """
        i, j, theta = current_node.i, current_node.j, current_node.theta
        adjacent_node = {}
        ok_actions = {}
        actions = [[0, self.RPM1], [self.RPM1, 0], [self.RPM1, self.RPM1], [0, self.RPM2], [self.RPM2, 0], [self.RPM2, self.RPM2], [self.RPM1, self.RPM2], [self.RPM2, self.RPM1]]
        for UL, UR in actions:
            x, y, new_theta, distance = self.get_new_coordinates (i, j, theta, UL, UR)
            if (not self.is_within_map(x, y)) and (not self.obs_space(x, y)):
                newNode = Node(x, y, self.endI, self.endJ, new_theta)
                adjacent_node[newNode] = distance
                ok_actions[newNode] = [UL, UR]
        return adjacent_node, ok_actions

    def get_new_coordinates (self, i, j, theta, UL, UR):
        t = 0
        r = 0.033
        L = 0.16
        dt = 0.01

        UL = 3.14*UL/30
        UR = 3.14*UR/30

        newI = i
        newJ = j
        new_theta = 3.14 * theta/180
        D = 0

        while t < 1:
            t = t + dt
            Delta_Xn = 0.5 * r * (UL + UR) * math.cos(new_theta) * dt
            Delta_Yn = 0.5 * r * (UL + UR) * math.sin(new_theta) * dt
            newI += Delta_Xn
            newJ += Delta_Yn
            new_theta += (r / L) * (UR - UL) * dt
            D = D + math.sqrt(math.pow(Delta_Xn, 2) + math.pow(Delta_Yn, 2))
        new_theta = 180*new_theta/3.14

        if new_theta > 0:
            new_theta = new_theta % 360
        elif new_theta < 0:
            new_theta = (new_theta + 360) % 360

        newI = self.round_off(newI)
        newJ = self.round_off(newJ)

        return newI, newJ, new_theta, D

    def draw_action(self, i, j, theta, UL, UR, color):
        t = 0
        r = 0.033
        L = 0.16
        dt = 0.01

        newI = i
        newJ = j
        new_theta = 3.14*theta/180
        UL = 3.14*UL/30
        UR = 3.14*UR/30

        while t < 1:
            t = t + dt
            oldI = newI
            oldJ = newJ
            newI += 0.5 * r * (UL + UR) * math.cos(new_theta) * dt
            newJ += 0.5 * r * (UL + UR) * math.sin(new_theta) * dt
            pygame.draw.line(map_canvas, color, [int(100*oldI), int(height - 100*oldJ)], [int(100*newI), int(height - 100*newJ)], 2)
            new_theta += (r / L) * (UR - UL) * dt
        pygame.display.update()
        time.sleep(0.1)

    def round_off(self, i):

        i = 100*i
        i = int(i)
        i = i/100
        return i
    
    def gen_non_holo_graph(self, ):
        """
        Description: Checks if a point is in the Ellipse.
        Input: Point with co-ordinates (x,y)
        Output: True or False
        """

        # Make background (0,0,0)
        map_canvas.fill((0,0,0))
            #screen.fill((0,0,0))  # Fill the screen with (0,0,0)

    # Draw rectangles
        pygame.draw.circle(map_canvas, (255,0,0), (1050, 200), 150)
        pygame.draw.rect(map_canvas, (255,0,0), (375, 0, 62.5, 250))  # Rectangle 1
        pygame.draw.rect(map_canvas, (255,0,0), (625, 250, 62.5, 500))  # Rectangle 2

    # Draw a circle
        pygame.draw.circle(map_canvas, (255,0,0), (1050, 200), 150)  # Circle

    # Update the display
        #pygame.display.flip()

        # Circles
        # pygame.draw.circle(map_canvas, (255,0,0), [100, int(height - 100)], 50)

        # # Rectangles
        # pygame.draw.polygon(map_canvas, (255,0,0), [(int(50*0.25), int(height - 50*5.75)), (int(50*1.75), int(height - 50*5.75)), (int(50*1.75), int(height - 50*4.25)), (50*0.25, height - 50*4.25)])
        # pygame.draw.polygon(map_canvas, (255,0,0), [(int(50*3.75), int(height - 50*5.75)), (int(50*6.25), int(height - 50*5.75)), (int(50*6.25), int(height) - int(50*4.25)), (int(50*3.75), int(height - 50*4.25))])

    def a_star_search(self, start, end):
        # Checking is start and end are in obstancle.
        if (self.obs_space(start.i, start.j) and self.obs_space(end.i, end.j)):
            print("Start node and end node are in the obstacle space. ")
            return

        if self.obs_space(start.i, start.j):
            print("starting node is in obstacle space.")
            return
        if not self.obs_space(end.i, end.j):
            print("Goal node is in obstacle space.")
            return

        if not self.is_within_map(start.i, start.j):
            print("Start node is out of bounds")
            return

        if not self.is_within_map(end.i, end.j):
            print("Goal node is out of bounds")
            return

        # print("Finding path...")
        list_open = []
        list_closed = {}
        heapq.heappush(list_open, (start.cost, start))
        while len(list_open):
            current_node = heapq.heappop(list_open)
            current_node = current_node[1]
            if self.is_near_goal(current_node.i, current_node.j):
                print("Path Found.")
                return True

            if tuple([current_node.i, current_node.j]) in list_closed:
                continue
            list_closed[tuple([current_node.i, current_node.j])] = True

            current_distance = current_node.cost_to_come
            adjacent_node, ok_actions = self.get_adjacent_node(current_node)
            current_node.adjacent_node = adjacent_node
            current_node.ok_actions = ok_actions
            for adjacent_node, new_distance in adjacent_node.items():
                adjacent_node.cost_to_come = current_distance + new_distance
                adjacent_node.cost = adjacent_node.cost_to_come + adjacent_node.cost_to_go
                adjacent_node.parent = current_node
                heapq.heappush(list_open, (adjacent_node.cost, adjacent_node))
                print((adjacent_node.i, adjacent_node.j))
        print("Cannot find a path :(")
        return False

    def visualizeAStar(self, start, end):
       

        list_closed = {}
        list_open = []
        heapq.heappush(list_open, (start.cost, start))
        pygame.draw.circle(map_canvas, (255, 255, 255), [int(100*start.i), int(height - 100*start.j)], 5)
        pygame.draw.circle(map_canvas, (255, 255, 255), [int(100*end.i), int(height - 100*end.j)], 5)
        pygame.display.update()
        while len(list_open):

            current_node = heapq.heappop(list_open)
            current_node = current_node[1]

            if self.is_near_goal(current_node.i, current_node.j):
                self.create_path(current_node)
                print("Distance from start node to goal node is:", current_node.cost_to_come)
                return

            if tuple([current_node.i, current_node.j]) in list_closed:
                continue
            list_closed[tuple([current_node.i, current_node.j])] = True

            for adjacent_node, action in current_node.ok_actions.items():
                self.draw_action(current_node.i,current_node.j,current_node.theta,action[0],action[1],(0, 255, 255))

            for adjacent_node, new_distance in current_node.adjacent_node.items():
                heapq.heappush(list_open, (adjacent_node.cost, adjacent_node))

    def create_path(self, child):
       
        while child != None:
            path.append(child)
            print(child.i, child.j, "Path")
            child = child.parent
        return True

    

    def obs_space(self, x, y):
       
        if((x>= 375-self.clearance and x<=437.5 + self.clearance and y>=250 - self.clearance and y<=500) 
           or (x>=625 - self.clearance and x<=687.5 + self.clearance
                and y>=0 and y<=250 + self.clearance)or((math.sqrt((x - 1050) ** 2 + (y - 300) ** 2))<=150+ self.clearance)):
            return True
        else:
            return False

    def is_near_goal(self, i, j):
       
        if (i - self.endI) ** 2 + (j - self.endJ) ** 2 - 0.01 <= 0:
            return True
        else:
            return False
    def is_within_map(self, x, y):
        

        return True if x < self.clearance or y < self.clearance or x > 1500 - self.clearance or y > 500 - self.clearance else False

x1 = float(input("Enter the x coordinate of the start node: "))
y1 = float(input("Enter the y coordinate of the start node: "))
thetaStart = int(input("Enter the start theta: "))


x2 = float(input("Enter the x coordinate of the goal node: "))
y2 = float(input("Enter the y coordiante of the goal node: "))

RPM1 = float(input("Enter RPM1: "))
RPM2 = float(input("Enter RPM2: "))

# RPM1 = 100.0
# RPM2 = 70.0
radius = float(input("Enter the radius of the robot:  "))
clearance = float(input("Enter the clearance:  "))

# Algorithm Driver
end = Node(x2, y2, x2, y2, 0)
start = Node(x1, y1, x2, y2, thetaStart)
start.cost_to_come = 0
robot = map_exploration(start, end, RPM1, RPM2, radius, clearance)
path = []

# Check if path can be found
if robot.a_star_search(start, end):
    #pass
    pygame.init()  # Setup Pygame
    map_canvas = pygame.display.set_mode((width, height))
    pygame.display.set_caption("A* Algorithm - Rigid Robot")
    exiting = False
    clock = pygame.time.Clock()
    grid = [[0 for j in range(height)] for i in range(width)]
    canvas = map_exploration(start, end, RPM1, RPM2, radius, clearance)  # Create Canvas
    canvas.gen_non_holo_graph()
    robot.visualizeAStar(start, end)
    path.reverse()
else:
    # No Path Found
    exiting = True


# Running the simulation in loop

while not exiting:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            exiting = True

            # Visualizing the final path
    for index in range(len(path)-1):
        node = path[index]
        action = node.ok_actions[path[index+1]]
        robot.draw_action(node.i, node.j, node.theta, action[0], action[1], (255,0,0)) 
        pygame.display.flip()
    #exiting = True
    clock.tick(5000)
pygame.quit()

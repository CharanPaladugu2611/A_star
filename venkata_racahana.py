import pygame
import math
import heapq
import time
import functools

# Defining non_holo_gph Constants
height = 500
width = 1500



class Node:
    

    def __init__(self, i, j, endI, endJ, theta):
        
        self.i = i
        self.j = j
        self.theta = theta
        self.costToCome = 0.0
        self.costToGo = 2.5*(math.sqrt((i - endI) ** 2 + (j - endJ) ** 2))
        self.cost = None
        self.adjacent_nodes = {}
        self.ok_actions = {}
        self.parent = None

    def __lt__(self, other):
        return self.cost < other.cost

class non_holo_gph:
    

    def __init__(self, start, end, RPM1, RPM2, radius, clearance):
        self.visited = {}
        self.endI = end.i
        self.endJ = end.j
        self.RPM1 = RPM1
        self.RPM2 = RPM2
        self.radius = radius
        self.clearance = clearance + self.radius

    def get_adjacent_nodes(self, currentNode):
        
        i, j, theta = currentNode.i, currentNode.j, currentNode.theta
        adjacent_nodes = {}
        ok_actions = {}
        actions = [[0, self.RPM1], [self.RPM1, 0], [self.RPM1, self.RPM1], [0, self.RPM2], [self.RPM2, 0], [self.RPM2, self.RPM2], [self.RPM1, self.RPM2], [self.RPM2, self.RPM1]]
        for UL, UR in actions:
            x, y, newTheta, distance = self.new_node(i, j, theta, UL, UR)
            if (not self.is_within_map(x, y)) and (not self.obs_space(x, y)):
                newNode = Node(x, y, self.endI, self.endJ, newTheta)
                adjacent_nodes[newNode] = distance
                ok_actions[newNode] = [UL, UR]
        return adjacent_nodes, ok_actions

    def new_node(self, i, j, theta, UL, UR):
        t = 0
        r = 0.033
        L = 0.16
        dt = 0.01

        UL = 3.14*UL/30
        UR = 3.14*UR/30

        newI = i
        newJ = j
        newTheta = 3.14 * theta/180
        D = 0

        while t < 1:
            t = t + dt
            Delta_Xn = 0.5 * r * (UL + UR) * math.cos(newTheta) * dt
            Delta_Yn = 0.5 * r * (UL + UR) * math.sin(newTheta) * dt
            newI += Delta_Xn
            newJ += Delta_Yn
            newTheta += (r / L) * (UR - UL) * dt
            D = D + math.sqrt(math.pow(Delta_Xn, 2) + math.pow(Delta_Yn, 2))
        newTheta = 180*newTheta/3.14

        if newTheta > 0:
            newTheta = newTheta % 360
        elif newTheta < 0:
            newTheta = (newTheta + 360) % 360

        newI = self.getRoundNumber(newI)
        newJ = self.getRoundNumber(newJ)

        return newI, newJ, newTheta, D

    def draw_action(self, i, j, theta, UL, UR, color):
        t = 0
        r = 0.033
        L = 0.16
        dt = 0.01

        newI = i
        newJ = j
        newTheta = 3.14*theta/180
        UL = 3.14*UL/30
        UR = 3.14*UR/30

        while t < 1:
            t = t + dt
            oldI = newI
            oldJ = newJ
            newI += 0.5 * r * (UL + UR) * math.cos(newTheta) * dt
            newJ += 0.5 * r * (UL + UR) * math.sin(newTheta) * dt
            pygame.draw.line(map_canvas, color, [int(100*oldI), int(height - 100*oldJ)], [int(100*newI), int(height - 100*newJ)], 2)
            newTheta += (r / L) * (UR - UL) * dt
        pygame.display.update()
        time.sleep(0.1)

    def getRoundNumber(self, i):

        i = 100*i
        i = int(i)
        i = i/100
        return i
    
    def gen_non_holo_gph(self, ):
        

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

        
    def a_search(self, start, end):
        

        # Checking is start and end are in obstancle.
        if self.obs_space(start.i, start.j) and self.obs_space(end.i, end.j):
            print("Starting and ending point are inside the obstacle!")
            return

        if self.obs_space(start.i, start.j):
            print("Starting point is inside the obstacle!")
            return
        if self.obs_space(end.i, end.j):
            print("Ending point is inside the obstacle!")
            return

        if self.is_within_map(start.i, start.j):
            print("Starting point is outside the arena!")
            return

        if self.is_within_map(end.i, end.j):
            print("Ending point is outside the arena!")
            return

        print("Finding path...")
        list_open = []
        list_close = {}
        heapq.heappush(list_open, (start.cost, start))
        while len(list_open):
            currentNode = heapq.heappop(list_open)
            currentNode = currentNode[1]
            if self.near_goal(currentNode.i, currentNode.j):
                print("Found a path!")
                return True

            if tuple([currentNode.i, currentNode.j]) in list_close:
                continue
            list_close[tuple([currentNode.i, currentNode.j])] = True

            currentDistance = currentNode.costToCome
            adjacent_nodes, ok_actions = self.get_adjacent_nodes(currentNode)
            currentNode.adjacent_nodes = adjacent_nodes
            currentNode.ok_actions = ok_actions
            for neighbourNode, newDistance in adjacent_nodes.items():
                neighbourNode.costToCome = currentDistance + newDistance
                neighbourNode.cost = neighbourNode.costToCome + neighbourNode.costToGo
                neighbourNode.parent = currentNode
                heapq.heappush(list_open, (neighbourNode.cost, neighbourNode))
                print((neighbourNode.i, neighbourNode.j))
        print("Cannot find a path :(")
        return False

    def a_star_visualisation(self, start, end):
        
        list_close = {}
        list_open = []
        heapq.heappush(list_open, (start.cost, start))
        pygame.draw.circle(map_canvas, (255, 255, 255), [int(100*start.i), int(height - 100*start.j)], 5)
        pygame.draw.circle(map_canvas, (255, 255, 255), [int(100*end.i), int(height - 100*end.j)], 5)
        pygame.display.update()
        while len(list_open):

            currentNode = heapq.heappop(list_open)
            currentNode = currentNode[1]

            if self.near_goal(currentNode.i, currentNode.j):
                self.create_path(currentNode)
                print("Distance Required to reach from start to end is:", currentNode.costToCome)
                return

            if tuple([currentNode.i, currentNode.j]) in list_close:
                continue
            list_close[tuple([currentNode.i, currentNode.j])] = True

            for neighbourNode, action in currentNode.ok_actions.items():
                self.draw_action(currentNode.i,currentNode.j,currentNode.theta,action[0],action[1],(0, 255, 255))

            for neighbourNode, newDistance in currentNode.adjacent_nodes.items():
                heapq.heappush(list_open, (neighbourNode.cost, neighbourNode))

    def create_path(self, child):
       
        while child != None:
            path.append(child)
            print(child.i, child.j, "Path")
            child = child.parent
        return True

    

    def obs_space(self, x, y):
        
        if((x>=37.5 - self.clearance and x<=43.75 + self.clearance and y>=250 - self.clearance and y<=50) 
           or (x>=62.5 - self.clearance and x<=68.75 + self.clearance
                and y>=0 and y<=250 + self.clearance)or((math.sqrt((x - 105.0) ** 2 + (y - 30.0) ** 2))<=150+ self.clearance)):
            return True
        else:
            return False

        #return self.isInCircle1(x, y) or self.isInCircle2(x, y) or self.isInRectangle1(x, y) or self.isInRectangle2(x, y) or self.isInRectangle3(x, y)
    def near_goal(self, i, j):
       
        if (i - self.endI) ** 2 + (j - self.endJ) ** 2 - 0.01 <= 0:
            return True
        else:
            return False
    def is_within_map(self, x, y):
       

        return True if x < self.clearance or y < self.clearance or x > 15 - self.clearance or y > 5 - self.clearance else False

x1 = float(input("Enter the x-coordiante of the start node: "))
y1 = float(input("Enter the y-coordiante of the start node: "))
orientation_start = int(input("Enter the orientation of the start node: "))


x2 = float(input("Enter the x-coordiante of the goal node: "))
y2 = float(input("Enter the y-coordiante of the goal node: "))
orientation_end = int(input("Enter the orientation of the goal node: "))    


RPM1 = float(input("Enter left-RPM: "))
RPM2 = float(input("Enter right-RPM: "))

# RPM1 = 100.0
# RPM2 = 70.0
radius = 55.0
clearance = float(input("Enter the clearance:  "))



end = Node(x2, y2, x2, y2, 0)
start = Node(x1, y1, x2, y2, orientation_start)
start.costToCome = 0
robot = non_holo_gph(start, end, RPM1, RPM2, radius, clearance)
path = []

# Check if path can be found
if robot.a_search(start, end):
    #pass
    pygame.init()  # Setup Pygame
    map_canvas = pygame.display.set_mode((width, height))
    pygame.display.set_caption("A* Algorithm Visualization")
    exiting = False
    clock = pygame.time.Clock()
    grid = [[0 for j in range(height)] for i in range(width)]
    canvas = non_holo_gph(start, end, RPM1, RPM2, radius, clearance)  # Create Canvas
    canvas.gen_non_holo_gph()
    robot.a_star_visualisation(start, end)
    path.reverse()
else:
    # No Path Found
    exiting = True


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

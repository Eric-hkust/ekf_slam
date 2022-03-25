# A* Version 2, for Room 3126 lab environment
# Add smoothing of the path
# Support change of points and obstacles
# Part of the code modified from https://www.redblobgames.com/pathfinding/a-star/implementation.html

import heapq
import numpy as np

# ---------- Classes ----------
# Continuous Graph
class RealGraph:
    def __init__(self, width, height, grid):
        self.width = width
        self.height = height
        self.grid = grid
        self.rectangles = []
        self.circles = []
        self.start_point = []
        self.end_point = []

    def add_rect(self, x1, y1, x2, y2):
        if x1 < x2 and y1 < y2:
            self.rectangles.append(((x1, y1), (x2, y2)))

    def add_cir(self, x, y, r):
        if x-r < x+r and y-r < y+r:
            self.circles.append(((x, y), r))
    
# Discrete graph
class SquareGrid:
    def __init__(self, width, height, grid):
        self.width = width
        self.height = height
        self.grid = grid
        self.walls = []
        self.start_point = []
        self.end_point = []
    
    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id):
        return id not in self.walls
    
    def neighbors(self, id):
        (x, y) = id
        neighbors = [(x+1, y), (x-1, y), (x, y-1), (x, y+1)]
        if (x + y) % 2 == 0: neighbors.reverse()
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return results

class GridWithWeights(SquareGrid):
    def __init__(self, width, height, grid):
        SquareGrid.__init__(self, width, height, grid)
        self.weights = {}
    
    def cost(self, from_node, to_node):
        pre_cost = self.weights.get(to_node, 1)
        nudge = 0
        (x1, y1) = from_node
        (x2, y2) = to_node
        if (x1 + y1) % 2 == 0 and x2 != x1: nudge = 1
        if (x1 + y1) % 2 == 1 and y2 != y1: nudge = 1
        return pre_cost + 0.001 * nudge

    def add_rect(self, x1, y1, x2, y2):
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(self.width, x2+1)
        y2 = min(self.height, y2+1)

        if x1 < x2 and y1 < y2:
            for i in range(x1, x2):
                for h in range(y1, y2):
                    self.walls.append((i, h))

    def add_cir(self, x, y, r):
        left = int((x-r) // self.grid)
        right = int((x+r) // self.grid)
        bottom = int((y-r) // self.grid)
        up = int((y+r) // self.grid) 
        left = max(0, left)
        bottom = max(0, bottom)
        right = min(self.width, right+1)
        up = min(self.height, up+1)

        for i in range(left, right):
            for h in range(bottom, up):
                if ((x - self.grid*(i+0.5))**2 + (y - self.grid*(h+0.5))**2) < r**2:
                    self.walls.append((i, h))
# Priority Queue
class PQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return not self.elements

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

# A* Main 
class AStar:
    def __init__(self, parameter = []):
        self.__width = float(parameter[0])
        self.__height = float(parameter[1])
        self.__grid = float(parameter[2])
        self.__continuous_graph = RealGraph(self.__width, self.__height, self.__grid)
        self.__discrete_graph = GridWithWeights(round(self.__width/self.__grid), round(self.__height/self.__grid), self.__grid)
        self.__robot_radius = 0.12 # turtlebot3 size
        self.__residual_path = []

    # ---------- Functions ----------
    # A* search for discrete path
    def find_path(self):
        start = self.__discrete_graph.start_point
        goal = self.__discrete_graph.end_point

        if goal in self.__discrete_graph.walls:
            return [start]

        fringe = PQueue()
        fringe.put(start, 0)
        prev = {}
        prev[start] = None
        distance = {}
        distance[start] = 0
    
        while not fringe.empty():
            current = fringe.get()
        
            if current == goal:
                break
        
            for next in self.__discrete_graph.neighbors(current):
                new_cost = distance[current] + self.__discrete_graph.cost(current, next)
                if next not in distance or new_cost < distance[next]:
                    distance[next] = new_cost
                    priority = new_cost + self.heuristic(next, goal)
                    fringe.put(next, priority)
                    prev[next] = current
    
        current = goal
        path = []
        while current != start:
            if current not in prev:
                return [start]
            path.append(current)
            current = prev[current]
        path.append(start)
        path.reverse()
        return path

    # Output continuous path
    def find_new_path(self):
        d_path = self.find_path()
        grid = self.__continuous_graph.grid
        c_path = []
        for point in d_path:
            x, y = point
            c_path.append((float(x)*grid + 0.5*grid, float(y)*grid + 0.5*grid))
        self.__residual_path =  c_path
        print('new path')

    def heuristic(self, a, b):
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)

    def line_position(self, l1, l2, p):
        x1, y1 = l1
        x2, y2 = l2
        x3, y3 = p
        result = (y2-y1)*(x3-x1) - (y3-y1)*(x2-x1)
        return np.sign(result)

    # Check whether the line between two points intersects with the obstacles
    def rect_intersect(self, start, end, s1, s2):
        position1 = self.line_position(start, end, s1)
        position2 = self.line_position(start, end, s2)
        position3 = self.line_position(s1, s2, start)
        position4 = self.line_position(s1, s2, end)
        return position1 * position2 < 0 and position3 * position4 < 0

    def cir_intersect(self, start, end, circle):
        v1 = np.asarray(start) - np.asarray(circle[0])
        v2 = np.asarray(end) - np.asarray(circle[0])
        v3 = np.asarray(start) - np.asarray(end)
        v4 = -v3
        if np.dot(v1, v3) < 0 or np.dot(v2, v4) < 0:
            dist1 = np.linalg.norm(v1)
            dist2 = np.linalg.norm(v2)
            return min(dist1, dist2) < circle[1]
        else:
            dist = np.linalg.norm(np.cross(v3, v1))/np.linalg.norm(v3)
            return dist < circle[1]

    def check_intersect(self, u, v):
        rect_intersection = False
        cir_intersection = False
        for rect in self.__continuous_graph.rectangles:
            ul, br = rect
            ur = (br[0], ul[1])
            bl = (ul[0], br[1])
            rect_intersection = self.rect_intersect(u, v, ul, br) or self.rect_intersect(u, v, bl, ur)
            if rect_intersection:
                break
        for cir in self.__continuous_graph.circles:
            cir_intersection = self.cir_intersect(u, v, cir)
            if cir_intersection:
                break
        return rect_intersection or cir_intersection

    # Smooth the path
    def smooth_path(self):
        c_path = self.__residual_path
        c_path1 = []
        c_path1.append(c_path[0])
        safe = 0
        end = 1
        while end < len(c_path) - 1:
            u = c_path[safe]
            v = c_path[end + 1]
            intersection = self.check_intersect(u, v)
            # cir_intersection = False
            if intersection:
                safe = end
                c_path1.append(c_path[safe])
                end = safe + 1
            else:
                end += 1
        if c_path[-1] not in c_path1:
            c_path1.append(c_path[-1])
        self.__residual_path = c_path1

    # Given a point s, return the index of the next closest point in the AStar path
    def find_next_closest(self, s):
        if len(self.__residual_path) <= 1:
            return 0
        dists = []
        safe = 0
        end = 1
        length = len(self.__residual_path)
        while end < length:
            t = self.__residual_path[safe]
            u = self.__residual_path[end]
            v1 = np.asarray(t) - np.asarray(s)
            v2 = np.asarray(u) - np.asarray(s)
            v3 = np.asarray(t) - np.asarray(u)
            v4 = -v3
            if np.dot(v1, v3) < 0 or np.dot(v2, v4) < 0:
                dist1 = np.linalg.norm(v1)
                dist2 = np.linalg.norm(v2)
                current_dist = min(dist1, dist2)
            else:
                current_dist = np.linalg.norm(np.cross(v3, v1))/np.linalg.norm(v3)
                
            dists.append(current_dist)
            safe = end
            end = safe + 1
        return (dists.index(min(dists)) + 1)

    # Find the residual path given the current position of the robot
    def find_residual_path(self):
        s = self.__continuous_graph.start_point
        index = self.find_next_closest(s)
        new_path = []
        new_path.append(s)
        for i in range(index, len(self.__residual_path)):
            new_path.append(self.__residual_path[i])
        new_path.append(self.__continuous_graph.end_point)
        self.__residual_path = new_path

    def residual_path_is_safe(self):
        intersection = False
        length = len(self.__residual_path)
        safe = 0
        end = 1
        while end < length:
            u = self.__residual_path[safe]
            v = self.__residual_path[end]
            intersection = self.check_intersect(u, v)
            # cir_intersection = False
            if intersection:
                break
            else:
                safe = end
                end = safe + 1
        return not intersection

    # Functions that provide users access to the class attributes 
    def set_start_point(self, parameter = []):
        x1 = float(parameter[0])
        y1 = float(parameter[1])
        x2 = int(x1//self.__grid)
        y2 = int(y1//self.__grid)
        self.__continuous_graph.start_point = (x1, y1)
        self.__discrete_graph.start_point = (x2, y2)

    def set_end_point(self, parameter = []):
        x1 = float(parameter[0])
        y1 = float(parameter[1])
        x2 = int(x1//self.__grid)
        y2 = int(y1//self.__grid)
        self.__continuous_graph.end_point = (x1, y1)
        self.__discrete_graph.end_point = (x2, y2)

    def add_rect(self, parameter = []):
        grid = self.__grid
        x1 = float(parameter[0]) - self.__robot_radius
        y1 = float(parameter[1]) - self.__robot_radius
        x2 = float(parameter[2]) + self.__robot_radius
        y2 = float(parameter[3]) + self.__robot_radius
        self.__continuous_graph.add_rect(x1, y1, x2, y2)
        self.__discrete_graph.add_rect(int((x1)//grid), int((y1)//grid), int((x2)//grid), int((y2)//grid))

    def add_cir(self, parameter = []):
        grid = self.__grid
        x = float(parameter[0])
        y = float(parameter[1])
        r = float(parameter[2]) + self.__robot_radius
        self.__continuous_graph.add_cir(x, y, r)
        self.__discrete_graph.add_cir(x, y, r)

    def get_residual_path(self):
        return self.__residual_path

    def clear(self):
        self.__continuous_graph.circles = []
        self.__continuous_graph.rectangles = []
        self.__discrete_graph.walls = []

    # Update
    def update(self):
        if not self.__residual_path:
            self.find_new_path()
        else:
            self.find_residual_path()
            if not self.residual_path_is_safe():
                self.find_new_path()
        self.find_residual_path()
        self.smooth_path()
        if len(self.__residual_path) == 1:
            print('BUMPED')

    # For plot
    def get_rectangle(self):
        return self.__continuous_graph.rectangles

    def get_circle(self):
        return self.__continuous_graph.circles

    def get_robot_radius(self):
        return self.__robot_radius
            



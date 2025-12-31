import pyamaze as maze
import heapq
import time
import csv
from dataclasses import dataclass, field
from typing import Any


ROWS = 50
COLS = 50
m = maze.maze(ROWS, COLS)


# Method1 = Heuristic + Cost  (f = g + h)
@dataclass(order=True)
class PrioritizedNode:
    priority: float
    node: Any = field(compare=False)

# Method2 = Heuristic >>> Cost
@dataclass(order=True)
class PrioritizedNodeHeuristic:
    priorityHeuristic: float
    priorityCost: float
    node: Any = field(compare=False)
    
# Method3 = Cost >>> Heuristic
@dataclass(order=True)
class PrioritizedNodeCost:
    priorityCost: float
    priorityHeuristic: float
    node: Any = field(compare=False)



def saveMazeToCSV(m, filename='maze.csv'):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Row', 'Column', 'Conexions'])

        for cell in m.maze_map:
            connections = [d for d, open_ in m.maze_map[cell].items() if open_ == 1]
            writer.writerow([cell[0], cell[1], ','.join(connections)])

def distance(cell1, cell2):
    x1, y1 = cell1
    x2, y2 = cell2
    return abs(x1 - x2) + abs(y1 - y2)


# Method1 = Heuristic + Cost  (f = g + h)
def aStar_method1(m):
    end = (1, 1)
    start = (m.rows, m.cols)

    g_score = {cell: float('inf') for cell in m.maze_map.keys()}
    g_score[start] = 0
    came_from = {}

    pq = []
    heapq.heappush(pq, PrioritizedNode(priority=0, node=start))

    while pq:
        current_point = heapq.heappop(pq).node

        if current_point == end:
            break

        for dir in ["E", "W", "S", "N"]:
            if m.maze_map[current_point][dir] == 1:
                match dir:
                    case "N":
                        next_point = (current_point[0] - 1, current_point[1])
                    case "S":
                        next_point = (current_point[0] + 1, current_point[1])
                    case "E":
                        next_point = (current_point[0], current_point[1] + 1)
                    case "W":
                        next_point = (current_point[0], current_point[1] - 1)

                temp_g = g_score[current_point] + 1

                if temp_g < g_score[next_point]:
                    g_score[next_point] = temp_g
                    came_from[next_point] = current_point

                    f = temp_g + distance(next_point, end)
                    heapq.heappush(pq, PrioritizedNode(priority=f, node=next_point))

    forwardPath = {}
    cell = end
    while cell != start:
        forwardPath[came_from[cell]] = cell
        cell = came_from[cell]
    return forwardPath

# Method2 = Heuristic >>> Cost
def aStar_method2(m):
    end = (1, 1)
    start = (m.rows, m.cols)

    g_score = {cell: float('inf') for cell in m.maze_map.keys()}
    g_score[start] = 0
    came_from = {}

    pq = []
    heapq.heappush(pq, PrioritizedNodeHeuristic(priorityHeuristic=0, priorityCost=0, node=start))

    while pq:
        current_point = heapq.heappop(pq).node

        if current_point == end:
            break

        for dir in ["E", "W", "S", "N"]:
            if m.maze_map[current_point][dir] == 1:
                match dir:
                    case "N":
                        next_point = (current_point[0] - 1, current_point[1])
                    case "S":
                        next_point = (current_point[0] + 1, current_point[1])
                    case "E":
                        next_point = (current_point[0], current_point[1] + 1)
                    case "W":
                        next_point = (current_point[0], current_point[1] - 1)

                temp_g = g_score[current_point] + 1
                h = distance(next_point, end)

                if temp_g < g_score[next_point]:
                    g_score[next_point] = temp_g
                    came_from[next_point] = current_point

                    heapq.heappush(pq, PrioritizedNodeHeuristic(priorityHeuristic=h, priorityCost=temp_g, node=next_point))

    forwardPath = {}
    cell = end
    while cell != start:
        forwardPath[came_from[cell]] = cell
        cell = came_from[cell]
    return forwardPath

# Method3 = Cost >>> Heuristic
def aStar_method3(m):
    end = (1, 1)
    start = (m.rows, m.cols)

    g_score = {cell: float('inf') for cell in m.maze_map.keys()}
    g_score[start] = 0
    came_from = {}

    pq = []
    heapq.heappush(pq, PrioritizedNodeCost(priorityCost=0, priorityHeuristic=0, node=start))

    while pq:
        current_point = heapq.heappop(pq).node

        if current_point == end:
            break

        for dir in ["E", "W", "S", "N"]:
            if m.maze_map[current_point][dir] == 1:
                match dir:
                    case "N":
                        next_point = (current_point[0] - 1, current_point[1])
                    case "S":
                        next_point = (current_point[0] + 1, current_point[1])
                    case "E":
                        next_point = (current_point[0], current_point[1] + 1)
                    case "W":
                        next_point = (current_point[0], current_point[1] - 1)

                temp_g = g_score[current_point] + 1
                h = distance(next_point, end)

                if temp_g < g_score[next_point]:
                    g_score[next_point] = temp_g
                    came_from[next_point] = current_point

                    heapq.heappush(pq, PrioritizedNodeCost(priorityCost=temp_g, priorityHeuristic=h, node=next_point))
                    
    forwardPath = {}
    cell = end
    while cell != start:
        forwardPath[came_from[cell]] = cell
        cell = came_from[cell]
    return forwardPath



m.CreateMaze(loadMaze='map200x200.csv')
saveMazeToCSV(m)

pre_Astar = time.time()
path = aStar_method1(m)   #method1
#path = aStar_method2(m)   #method2
#path = aStar_method3(m)   #method3
post_Astar = time.time()

print("Execution time:", post_Astar - pre_Astar)

a = maze.agent(m, footprints=True)
m.tracePath({a: path}, delay=1)
m.run()

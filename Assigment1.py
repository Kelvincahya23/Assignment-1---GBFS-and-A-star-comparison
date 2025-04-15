import time
from heapq import heappush, heappop

# Grid
grid_str = [
    "S.#......",
    ".#..#.###",
    ".#......#",
    ".####..#.",
    ".....#.G.",
    "####..##.",
    "..##.#...",
    ".#..#.###",
    ".#.......",
    "....####."
]

ROWS, COLS = len(grid_str), len(grid_str[0])
grid = [list(row) for row in grid_str]

def find_points(grid):
    start = goal = None
    for r in range(ROWS):
        for c in range(COLS):
            if grid[r][c] == 'S':
                start = (r, c)
            elif grid[r][c] == 'G':
                goal = (r, c)
    return start, goal

start, goal = find_points(grid)

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def neighbors(pos):
    r, c = pos
    for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < ROWS and 0 <= nc < COLS and grid[nr][nc] != '#':
            yield (nr, nc)

def greedy_best_first_search(start, goal):
    heap = [(manhattan(start, goal), start)]
    came_from = {start: None}
    visited = set()

    while heap:
        _, current = heappop(heap)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            break
        for neighbor in neighbors(current):
            if neighbor not in came_from:
                came_from[neighbor] = current
                heappush(heap, (manhattan(neighbor, goal), neighbor))

    return reconstruct_path(came_from, start, goal), visited

def a_star(start, goal):
    heap = [(manhattan(start, goal), 0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}
    visited = set()

    while heap:
        _, cost, current = heappop(heap)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            break
        for neighbor in neighbors(current):
            new_cost = cost_so_far[current] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + manhattan(neighbor, goal)
                heappush(heap, (priority, new_cost, neighbor))
                came_from[neighbor] = current

    return reconstruct_path(came_from, start, goal), visited

def reconstruct_path(came_from, start, goal):
    path = []
    current = goal
    while current and current != start:
        path.append(current)
        current = came_from.get(current)
    if current == start:
        path.append(start)
        path.reverse()
        return path
    return []

def print_path(grid, path):
    grid_copy = [row[:] for row in grid]
    for r, c in path:
        if grid_copy[r][c] not in ('S', 'G'):
            grid_copy[r][c] = '*'
    for row in grid_copy:
        print(''.join(row))

# Run GBFS
start_time = time.time()
path_gbfs, visited_gbfs = greedy_best_first_search(start, goal)
gbfs_time = time.time() - start_time

print("Greedy Best-First Search:")
print_path(grid, path_gbfs)
print(f"Path length: {len(path_gbfs)}")
print(f"Nodes explored: {len(visited_gbfs)}")
print(f"Time: {gbfs_time:.6f} seconds\n")

# Run A*
start_time = time.time()
path_astar, visited_astar = a_star(start, goal)
astar_time = time.time() - start_time

print("A* Search:")
print_path(grid, path_astar)
print(f"Path length: {len(path_astar)}")
print(f"Nodes explored: {len(visited_astar)}")
print(f"Time: {astar_time:.6f} seconds")

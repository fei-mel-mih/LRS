from collections import deque

'''
0 = free path
1 = obstacle
2 = goal
'''

def flood_fill(start_point, grid):
    start = (start_point.x, start_point.y, start_point.z)
    
    queue = deque([start])
    visited = set([start])  # Contains nodes that have been processed
    deltas = [-1, 0, 1]

    while queue:
        x, y, z = queue.popleft()
        current_value = grid[x][y][z]

        for dx in deltas:
            for dy in deltas:
                for dz in deltas:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    
                    nx, ny, nz = x + dx, y + dy, z + dz
                    neighbor = (nx, ny, nz)
                    
                    if neighbor in visited:
                        continue

                    if is_point_valid(neighbor, grid):
                        if grid[nx][ny][nz] == 2:
                            print("Path found to a goal at:", neighbor)
                        elif grid[nx][ny][nz] == 0:
                            queue.append(neighbor)
                            grid[nx][ny][nz] = current_value + 1

                    visited.add(neighbor)
    return grid

def reverse_flood_fill2(start_point, goal, grid):
    if (grid[start_point[0]][start_point[1]][start_point[2]]) == 1:
        print("The start is barrier!")
        return
    # goal = (goal_point.x, goal_point.y, goal_point.z)
    grid[goal[0]][goal[1]][goal[2]] = 2
    queue = deque([goal])
    visited = set([goal])  # Contains nodes that have been processed
    deltas = [-1, 0, 1]

    while queue:
        x, y, z = queue.popleft()
        current_value = grid[x][y][z]

        for dx in deltas:
            for dy in deltas:
                for dz in deltas:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    
                    nx, ny, nz = x + dx, y + dy, z + dz
                    neighbor = (nx, ny, nz)
                    
                    if neighbor in visited:
                        continue

                    if is_point_valid(neighbor, grid) and grid[nx][ny][nz] == 0:
                        grid[nx][ny][nz] = current_value + 1
                        queue.append(neighbor)

                    visited.add(neighbor)

    if grid[start_point[0]][start_point[1]][start_point[2]] > 2:
        print(f"Path found to the start from the goal with a length of { grid[start_point[0]][start_point[1]][start_point[2]] - 2}")
    else:
        print(f"Path not found! start_value: { grid[start_point[0]][start_point[1]][start_point[2]]}" )
    return grid

def is_point_valid(point, grid) -> bool:
    x, y, z = point
    if 0 <= x < len(grid):
        if 0 <= y < len(grid[x]):
            if 0 <= z < len(grid[x][y]):
                cell_value = grid[x][y][z]
                return cell_value == 0  # Only visit if cell value is 0 (unvisited)
    return False

def is_point_valid_path(point, grid) -> bool:
    x, y, z = point
    if 0 <= x < len(grid):
        if 0 <= y < len(grid[x]):
            if 0 <= z < len(grid[x][y]):
                cell_value = grid[x][y][z]
                return cell_value != 1  # Only visit if cell value is not 1 (barrier)
    return False

def find_path(starting_point, goal_point, grid):
    path = []
    current_position = starting_point  # this should be a tuple (x, y, z)
    deltas = [-1, 0, 1]

    while current_position != goal_point:
        x, y, z = current_position
        path.append(current_position)

        min_neighbor = None
        min_neighbor_value = float('inf')
        
        for dx in deltas:
            for dy in deltas:
                for dz in deltas:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue

                    nx, ny, nz = x + dx, y + dy, z + dz
                    neighbor = (nx, ny, nz)
                    # we choose the lowest value from neighbors
                    if is_point_valid_path(neighbor, grid) and grid[nx][ny][nz] < min_neighbor_value:
                        min_neighbor_value = grid[nx][ny][nz]
                        min_neighbor = neighbor

        # If no valid neighbor was found, break out of the loop.
        # This might happen if there's no valid path to the goal
        if min_neighbor is None:
            break

        current_position = min_neighbor

    return path

def reverse_flood_fill(start_point, goal, grid):
    if grid[start_point[0]][start_point[1]][start_point[2]] == 1:
        print("The start is a barrier!")
        return grid
    
    x_len, y_len, z_len = len(grid), len(grid[0]), len(grid[0][0])
    grid[goal[0]][goal[1]][goal[2]] = 2
    queue = deque([goal])
    
    # Use deltas without the (0, 0, 0) combination.
    deltas = [-1, 0, 1]
    combinations = [(dx, dy, dz) for dx in deltas for dy in deltas for dz in deltas if dx or dy or dz]

    while queue:
        x, y, z = queue.popleft()
        current_value = grid[x][y][z]

        for dx, dy, dz in combinations:
            nx, ny, nz = x + dx, y + dy, z + dz

            # Use inlined boundary checks.
            if 0 <= nx < x_len and 0 <= ny < y_len and 0 <= nz < z_len and grid[nx][ny][nz] == 0:
                grid[nx][ny][nz] = current_value + 1
                queue.append((nx, ny, nz))
    
    start_value = grid[start_point[0]][start_point[1]][start_point[2]]
    if start_value > 2:
        print(f"Path found to the start from the goal with a length of {start_value - 2}")
    else:
        print(f"Path not found! start_value: {start_value}")
    
    return grid


def reverse_flood_fill_4(start_point, goal, grid):
    if grid[start_point[0]][start_point[1]] == 1:
        print("The start is a barrier!")
        return grid
    
    x_len, y_len, z_len = len(grid), len(grid[0]), len(grid[0][0])
    grid[goal[0]][goal[1]][goal[2]] = 2
    queue = deque([goal])
    
    # Define the 4 cardinal direction offsets.
    directions = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]

    while queue:
        x, y, z = queue.popleft()
        current_value = grid[x][y][z]

        for dx, dy, dz in directions:
            nx, ny, nz = x + dx, y + dy, z + dz

          # Use inlined boundary checks.
            if 0 <= nx < x_len and 0 <= ny < y_len and 0 <= nz < z_len and grid[nx][ny][nz] == 0:
                grid[nx][ny][nz] = current_value + 1
                queue.append((nx, ny, nz))
    
    start_value = grid[start_point[0]][start_point[1]][start_point[2]]
    if start_value > 2:
        print(f"Path found to the start from the goal with a length of {start_value - 2}")
    else:
        print(f"Path not found! start_value: {start_value}")
    
    return grid

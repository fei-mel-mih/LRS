#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# from flood_fill import reverse_flood_fill

class FloodFill(Node):
    def __init__(self):
        super().__init__("floodfill")
        self.get_logger().info("Hello from flood fill node")
        # reverse_flood_fill()

    # def reverse_flood_fill(start_point, goal, grid):
    # if grid[start_point[0]][start_point[1]][start_point[2]] == 1:
    #     print("The start is a barrier!")
    #     return grid
    
    # x_len, y_len, z_len = len(grid), len(grid[0]), len(grid[0][0])
    # grid[goal[0]][goal[1]][goal[2]] = 2
    # queue = deque([goal])
    
    # # Use deltas without the (0, 0, 0) combination.
    # deltas = [-1, 0, 1]
    # combinations = [(dx, dy, dz) for dx in deltas for dy in deltas for dz in deltas if dx or dy or dz]

    # while queue:
    #     x, y, z = queue.popleft()
    #     current_value = grid[x][y][z]

    #     for dx, dy, dz in combinations:
    #         nx, ny, nz = x + dx, y + dy, z + dz

    #         # Use inlined boundary checks.
    #         if 0 <= nx < x_len and 0 <= ny < y_len and 0 <= nz < z_len and grid[nx][ny][nz] == 0:
    #             grid[nx][ny][nz] = current_value + 1
    #             queue.append((nx, ny, nz))
    
    # start_value = grid[start_point[0]][start_point[1]][start_point[2]]
    # if start_value > 2:
    #     print(f"Path found to the start from the goal with a length of {start_value - 2}")
    # else:
    #     print(f"Path not found! start_value: {start_value}")
    
    # return grid


def main(args=None):
    rclpy.init(args=args)
    node = FloodFill()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
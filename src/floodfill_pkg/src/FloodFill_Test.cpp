#include <queue>
#include "MapReader.h"
#include <limits>
#include <math.h>

#define GRID_IN_CM 5.0

class FloodFillNode
{
public:
    struct Point
    {
        int x, y, z;

        bool operator==(const Point &other) const
        {
            return (x == other.x) && (y == other.y) && (z == other.z);
        }

        bool operator!=(const Point &other) const
        {
            return (x != other.x) || (y != other.y) || (z != other.z);
        }

        std::string toString() const
        {
            return "Point(x,y,z)=[" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "]";
        }
    };


    bool has_line_of_sight(const Point& start, const Point& end, const std::vector<std::vector<std::vector<int>>>& map) 
    {
        int x1 = start.x;
        int y1 = start.y;
        int z1 = start.z;
        int x2 = end.x;
        int y2 = end.y;
        int z2 = end.z;

        int dx = abs(x2 - x1);
        int dy = abs(y2 - y1);
        int dz = abs(z2 - z1);

        int xs = (x1 < x2) ? 1 : -1;
        int ys = (y1 < y2) ? 1 : -1;
        int zs = (z1 < z2) ? 1 : -1;

        // Driving axis is X-axis
        if (dx >= dy && dx >= dz) 
        {
            int p1 = 2 * dy - dx;
            int p2 = 2 * dz - dx;
            while (x1 != x2) 
            {
                x1 += xs;
                if (p1 >= 0) 
                {
                    y1 += ys;
                    p1 -= 2 * dx;
                }
                if (p2 >= 0) 
                {
                    z1 += zs;
                    p2 -= 2 * dx;
                }
                p1 += 2 * dy;
                p2 += 2 * dz;
                if (map[x1][y1][z1] == 1) 
                {
                    return false;
                }
            }
        }
        // Driving axis is Y-axis
        else if (dy >= dx && dy >= dz) 
        {
            int p1 = 2 * dx - dy;
            int p2 = 2 * dz - dy;
            while (y1 != y2) 
            {
                y1 += ys;
                if (p1 >= 0) 
                {
                    x1 += xs;
                    p1 -= 2 * dy;
                }
                if (p2 >= 0) 
                {
                    z1 += zs;
                    p2 -= 2 * dy;
                }
                p1 += 2 * dx;
                p2 += 2 * dz;
                if (map[x1][y1][z1] == 1) 
                {
                    return false;
                }
            }
        }
        // Driving axis is Z-axis
        else 
        {
            int p1 = 2 * dy - dz;
            int p2 = 2 * dx - dz;
            while (z1 != z2) 
            {
                z1 += zs;
                if (p1 >= 0) 
                {
                    y1 += ys;
                    p1 -= 2 * dz;
                }
                if (p2 >= 0) 
                {
                    x1 += xs;
                    p2 -= 2 * dz;
                }
                p1 += 2 * dy;
                p2 += 2 * dx;
                if (map[x1][y1][z1] == 1) 
                {
                    return false;
                }
            }
        }
        return true;
    }

    std::vector<Point> simplify_path(const std::vector<Point>& path, const std::vector<std::vector<std::vector<int>>>& map) 
    {
        std::vector<Point> simplified_path;
        Point A = path[0];
        simplified_path.push_back(A);

        for (int i = 1; i < path.size(); i++) 
        {
            if (!has_line_of_sight(A, path[i], map)) 
            {
                simplified_path.push_back(path[i-1]);
                A = path[i-1];
            }
        }

        simplified_path.push_back(path.back());  // add the last point
        return simplified_path;
    }

    int map_to_height_index(float value, const std::vector<int>& heights) {
        int foundIndex = -1;
        for (size_t i = 0; i < heights.size(); ++i) 
        {
            if (value <= heights[i])
            {
                foundIndex = i;
                break;
            }
        }
        if (foundIndex == -1)
        {
            std::cout << "Value of " << value << " is not possible!" << std::endl;
        }
        return foundIndex;
    }

    int real_to_index(float value)
    {
        value /= GRID_IN_CM;
        value = std::ceil(value);
        return value;
    }

    std::vector<Point> path_to_real(std::vector<Point>& path, std::vector<int>& heights)
    {
        std::vector<Point> copy = path;
        for (int i = 0; i <  copy.size(); i++) 
        {
            copy[i].x = heights[copy[i].x];
            copy[i].y *= GRID_IN_CM;
            copy[i].z *= GRID_IN_CM;
        }
        return copy;
    }


    void flood_fill()
    {
        std::cout << "Starting flood_fill..." << std::endl;

        
        MapReader map_reader;

        Point start; Point goal; std::vector<std::vector<std::vector<int>>> map = map_reader.getMap();

        // We need to recalculate real position to indices
        std::vector<int> heights = map_reader.getHeights();
        std::cout << real_to_index(324) << std::endl;
        // [START POINT] - TRANSFORM REAL COORDINATES TO INDICES OF MAP 3D VECTOR
        start = {25, 160, 115};
        std::cout << "[START] Real coords: [" << start.x << ", " << start.y << ", " << start.z << "]\n";
        start.x = map_to_height_index(start.x, heights);
        start.y = real_to_index(start.y);
        start.z = real_to_index(start.z);
        std::cout << "[START] Indices: [" << start.x << ", " << start.y << ", " << start.z << "]\n";
        // [GOAL POINT] - TRANSFORM REAL COORDINATES TO INDICES OF MAP 3D VECTOR
        goal = {156, 1234, 1000};
        std::cout << "[GOAL] Real coords: [" << goal.x << ", " << goal.y << ", " << goal.z << "]\n";
        goal.x = map_to_height_index(goal.x, heights);
        goal.y = real_to_index(goal.y);
        goal.z = real_to_index(goal.z);
        std::cout << "[GOAL] Indices: [" << goal.x << ", " << goal.y << ", " << goal.z << "]\n";


        std::cout << map[start.x][start.y][start.z] << std::endl;
        // goal = {2, 1234, 1000};
        std::cout << map[goal.x][goal.y][goal.z] << std::endl;

        // Define the 6 face neighbor offsets.
        int directions[6][3] = {
            {1, 0, 0}, {-1, 0, 0},
            {0, 1, 0}, {0, -1, 0},
            {0, 0, 1}, {0, 0, -1}
        };

        int x_len = map.size();
        int y_len = map[0].size();
        int z_len = map[0][0].size();
        
        int deltas[3] = {-1, 0, 1};

        std::queue<Point> q;
        q.push(goal);

        if (map[start.x][start.y][start.z] == 1 || map[goal.x][goal.y][goal.z] == 1)
        {
            std::cout << "Path finding is not possible!\nEither start or goal point is in the barrier!" << std::endl;
            std::cout << "Start value: " << map[start.x][start.y][start.z] << std::endl;
            std::cout << "Goal value: " << map[goal.x][goal.y][goal.z] << std::endl;
            return;
        }

        map[goal.x][goal.y][goal.z] = 2;  // goal is set to 2

        while (!q.empty())
        {   
            Point current_point = q.front();
            q.pop();

            int current_value = map[current_point.x][current_point.y][current_point.z];
            // 26 susednost
            for (int dx : deltas)
            {
                for (int dy : deltas)
                {
                    for (int dz : deltas)
                    {
                        if (dx == 0 && dy == 0 && dz == 0)
                        {
                            continue;
                        } 
                        Point neighbor{current_point.x + dx, current_point.y + dy, current_point.z + dz};

                        if (0 <= neighbor.x && neighbor.x < x_len && 0 <= neighbor.y && neighbor.y < y_len && 0 <= neighbor.z && neighbor.z < z_len && map[neighbor.x][neighbor.y][neighbor.z] == 0) {
                            map[neighbor.x][neighbor.y][neighbor.z] = current_value + 1;
                            q.push(neighbor);
                        }
                    }
                }
            }
            // 6 susednost
            // for (int i = 0; i < 6; ++i) {
            //     Point neighbor{current_point.x + directions[i][0], current_point.y + directions[i][1], current_point.z + directions[i][2]};

            //     // Use inlined boundary checks.
            //     if (0 <= neighbor.x && neighbor.x < x_len && 0 <= neighbor.y && neighbor.y < y_len && 0 <= neighbor.z && neighbor.z < z_len && map[neighbor.x][neighbor.y][neighbor.z] == 0) {
            //         map[neighbor.x][neighbor.y][neighbor.z] = current_value + 1;
            //         q.push(neighbor);
            //     }
            // }
        }

        int start_value = map[start.x][start.y][start.z];
        if (start_value > 2) 
        {
            std::cout << "After flood_fill..." << std::endl;
            std::cout << "Path found to the start from the goal with a length of " << start_value - 2 << std::endl;
        } else 
        {
            std::cout << "Path not found! start_value: " << start_value << std::endl;
        }
        std::cout << "Finding path" << std::endl;

        // Find path
        std::vector<Point> path;
        Point current_position = start;
        Point min_neighbour;
        int min_neighbour_value = std::numeric_limits<int>::max();
        bool found;

        std::cout << "Goal: " << goal.toString() << std::endl;
        while (current_position != goal)
        {
            // std::cout << "point " << current_position.toString() << ":\n";
            path.push_back(current_position);
            int current_value = map[current_position.x][current_position.y][current_position.z];
            // std::cout << "Current value: " << current_value << std::endl;
            int min_neighbour_value = current_value; // Set it to the current value so we can find a lesser value
            found = false; // reset
            for (int dx : deltas)
            {
                for (int dy : deltas)
                {
                    for (int dz : deltas)
                    {
                        if (dx == 0 && dy == 0 && dz == 0)
                        {
                            continue;
                        } 
                        Point neighbor{current_position.x + dx, current_position.y + dy, current_position.z + dz};
                        int neighbor_value = map[neighbor.x][neighbor.y][neighbor.z];
                        if (0 <= neighbor.x && neighbor.x < x_len && 0 <= neighbor.y && neighbor.y < y_len && 0 <= neighbor.z && neighbor.z < z_len &&  neighbor_value < min_neighbour_value && neighbor_value != 1) {
                            // std::cout << "chosen neighbor " << neighbor.toString() << ":\n";
                            min_neighbour = neighbor;
                            min_neighbour_value = map[neighbor.x][neighbor.y][neighbor.z];
                            found = true;
                        }
                    }
                }
            }
            if (!found)
            {
                std::cout << "Path was not found!" << std::endl;
                break;
            }
            current_position = min_neighbour;
            // std::cout << "Current position: " << current_position.toString() << std::endl;

        }
        // If path was found
        if (found)
        {
            // Add goal to path 
            path.push_back(goal);

            // print path
            // std::cout << "Original found path: " << std::endl;
            // for (int i = 0; i <  path.size(); i++) 
            // {
            //     std::cout << "point " << path[i].toString() << ":\n";
            // }

            std::cout << "Starting simplification of path" << std::endl;
            std::vector<Point> simplified = simplify_path(path, map);

            // print path
            for (int i = 0; i <  simplified.size(); i++) 
            {
                std::cout << "point " << simplified[i].toString() << ":\n";
            }

            std::cout << "Original path length: " << path.size() << std::endl;
            std::cout << "Simplified path length: " << simplified.size() << std::endl;

            // now lets reverse the indices to real coordinates
            std::vector<Point> real_simplified = path_to_real(simplified, heights);

            // print path
            for (int i = 0; i <  real_simplified.size(); i++) 
            {
                std::cout << "point " << real_simplified[i].toString() << ":\n";
            }
        }

    }
};

int main(int argc, char **argv)
{
    std::cout << "Entering main..." << std::endl;
    FloodFillNode fn = FloodFillNode();
    std::cout << "Before flood_fill..." << std::endl;
    fn.flood_fill();
    return 0;
}

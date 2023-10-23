#include <queue>
#include "MapReader.h"

class FloodFillNode
{
public:
    struct Point {
        int x, y, z;
    };

    void flood_fill()
    {
        std::cout << "Starting flood_fill..." << std::endl;
        MapReader map_reader;

        Point start; Point goal; std::vector<std::vector<std::vector<int>>> map = map_reader.getMap();
        
        start = {0, 0, 0};
        goal = {3, 3, 3};

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
        if (start_value > 2) {
            std::cout << ("Path found to the start from the goal with a length of %d", start_value - 2) << std::endl;
        } else {
            std::cout << ("Path not found! start_value: %d", start_value) << std::endl;
        }
    }
};

int main(int argc, char **argv)
{
    std::cout << "Entering main..." << std::endl;
    FloodFillNode fn = FloodFillNode();
    std::cout << "Before flood_fill..." << std::endl;
    fn.flood_fill();
    std::cout << "After flood_fill..." << std::endl;
    return 0;
}

#include <queue>

#include <rclcpp/rclcpp.hpp>

#include "lrs_interfaces/msg/point_list.hpp"
#include "lrs_interfaces/msg/point.hpp"
#include "lrs_interfaces/srv/flood_fill.hpp"

// Your Point struct here...
struct Point {
    int x, y, z;
};

void flood_fill(Point start, Point goal, std::vector<std::vector<std::vector<int>>> map)
{
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
        std::cout << "Path found to the start from the goal with a length of " << start_value - 2 << std::endl;
    } else {
        std::cout << "Path not found! start_value: " << start_value << std::endl;
    }
}

class FloodFillNode : public rclcpp::Node
{
public:
    FloodFillNode() : Node("floodfill_node")
    {
        service_ = this->create_service<lrs_interfaces::srv::FloodFill>("floodfill_points", &flood_fill);
    }

private:
    void callback(const lrs_interfaces::msg::Point::SharedPtr msg)
    {
        Point goal_point = {msg->x, msg->y, msg->z};

        // Create a map, set goal point value, call the reverse_flood_fill function
        std::vector<std::vector<std::vector<int>>> map(10, std::vector<std::vector<int>>(288, std::vector<int>(366, 0)));

        // Define the start and goal points.
        Point start_point = {0, 0, 0};
        Point goal_point = {9, 9, 9};

        map[goal_point.x][goal_point.y][goal_point.z] = 2;
        
        // For this example, after the flood fill, gather the results
        lrs_interfaces::msg::PointList result;
    }

    rclcpp::Service<lrs_interfaces::srv::FloodFill>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloodFillNode>());
    rclcpp::shutdown();
    return 0;
}

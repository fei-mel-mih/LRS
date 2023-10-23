#include <rclcpp/rclcpp.hpp>

#include <queue>

#include "MapReader.h"
#include "lrs_interfaces/srv/flood_fill.hpp"

class FloodFillNode : public rclcpp::Node
{
public:
    FloodFillNode() : Node("floodfill_node")
    {
        RCLCPP_INFO(this->get_logger(), "Creating FloodFillNode...");

        RCLCPP_INFO(this->get_logger(), "Creating flood fill service");
        this->service_ = this->create_service<lrs_interfaces::srv::FloodFill>
        (
            "floodfill_service",
            std::bind(&FloodFillNode::flood_fill, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "Flood fill service created");

        RCLCPP_INFO(this->get_logger(), "FloodFillNode created...");
    }

    struct Point {
        int x, y, z;
    };

    void flood_fill(const lrs_interfaces::srv::FloodFill::Request::SharedPtr request,
                    const lrs_interfaces::srv::FloodFill::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Starting flood_fill...");
        MapReader map_reader;

        Point start;
        Point goal;
        std::vector<std::vector<std::vector<int>>> map = map_reader.getMap();

        // TODO: handle map size boundaries
        
        start = {request->start_point.z, request->start_point.y, request->start_point.x};
        std::cout << map[start.x][start.y][start.z] << std::endl;
        
        goal = {request->goal_point.z, request->goal_point.y, request->goal_point.x};
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

        map[goal.x][goal.y][goal.z] = 2;  // example starting value

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
            RCLCPP_INFO(this->get_logger(), "After flood_fill...");
            RCLCPP_INFO(this->get_logger(), "Path found to the start from the goal with a length of %d", start_value - 2);
        } else 
        {
            RCLCPP_INFO(this->get_logger(), "Path not found! start_value: %d", start_value);
        }

        // TODO: path extraction

        response->points = lrs_interfaces::msg::PointList();
    }

    std::vector<Point> getFloodFillPath(std::vector<std::vector<std::vector<int>>> map)
    {
        // TODO: path finder
    }

private:
    rclcpp::Service<lrs_interfaces::srv::FloodFill>::SharedPtr service_;
    std::string mapPath = "/home/lrs-ubuntu/Documents/lrs-git/LRS/src/floodfill_pkg/src/maps/";
    std::vector<std::string> filenames = {"map_025.pgm", "map_075.pgm", "map_080.pgm", "map_100.pgm", "map_125.pgm", "map_150.pgm", "map_175.pgm", "map_180.pgm", "map_200.pgm", "map_225.pgm"};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloodFillNode>());
    rclcpp::shutdown();
    return 0;
}

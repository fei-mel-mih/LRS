#include <rclcpp/rclcpp.hpp>

#include <queue>
#include <math.h>

#include "MapReader.h"
#include "lrs_interfaces/srv/flood_fill.hpp"

#define GRID_IN_CM 5.0
#define DRONE_GLOBAL_START_X 14.f
#define DRONE_GLOBAL_START_Y 1.51f
#define MAP_HEIGHT_PIXEL_OFFSET 25
#define MAP_WIDTH_PIXEL_OFFSET 8

class FloodFillNode : public rclcpp::Node
{
public:
    FloodFillNode() : Node("floodfill_node")
    {
        RCLCPP_INFO(this->get_logger(), "Creating FloodFillNode...");

        RCLCPP_INFO(this->get_logger(), "Creating flood fill service");
        this->service_ = this->create_service<lrs_interfaces::srv::FloodFill>(
            "floodfill_service",
            std::bind(&FloodFillNode::flood_fill, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Flood fill service created");

        RCLCPP_INFO(this->get_logger(), "FloodFillNode created...");
    }

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

    bool has_line_of_sight(const Point &start, const Point &end, const std::vector<std::vector<std::vector<int>>> &map)
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

    std::vector<Point> simplify_path(const std::vector<Point> &path, const std::vector<std::vector<std::vector<int>>> &map)
    {
        std::vector<Point> simplified_path;
        Point A = path[0];
        simplified_path.push_back(A);

        for (int i = 1; i < path.size(); i++)
        {
            if (!has_line_of_sight(A, path[i], map))
            {
                simplified_path.push_back(path[i - 1]);
                A = path[i - 1];
            }
        }

        simplified_path.push_back(path.back()); // add the last point
        return simplified_path;
    }

    int map_to_height_index(float value, const std::vector<int> &heights)
    {
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
            RCLCPP_ERROR(this->get_logger(), "Height value of %d is not possible!", value);
        }
        return foundIndex;
    }

    int real_to_index(float value)
    {
        value /= GRID_IN_CM;
        value = std::ceil(value);
        return (int)value;
    }

    std::vector<Point> path_to_real(std::vector<Point> &path, std::vector<int> &heights)
    {
        std::vector<Point> copy = path;
        for (int i = 0; i < copy.size(); i++)
        {
            copy[i].x = heights[copy[i].x];
            copy[i].y *= GRID_IN_CM;
            copy[i].z *= GRID_IN_CM;
        }
        return copy;
    }

    void flood_fill(const lrs_interfaces::srv::FloodFill::Request::SharedPtr request,
                    const lrs_interfaces::srv::FloodFill::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "Starting flood_fill...");
        RCLCPP_INFO(this->get_logger(), "Request: start=[%d,%d,%d] goal=[%d,%d,%d]",
                    request->start_point.x, request->start_point.y, request->start_point.z,
                    request->goal_point.x, request->goal_point.y, request->goal_point.z);

        MapReader map_reader;

        Point start;
        Point goal;
        std::vector<std::vector<std::vector<int>>> map = map_reader.getMap();
        std::vector<int> heights = map_reader.getHeights();

        int sz_map_x = map.size();
        int sz_map_y = map[0].size();
        int sz_map_z = map[0][0].size();
        RCLCPP_INFO(this->get_logger(), "Map dimensions: %dx%dx%d", sz_map_x, sz_map_y, sz_map_z);

        // We need to recalculate real position to indices
        // [START POINT] - TRANSFORM REAL COORDINATES TO INDICES OF MAP 3D VECTOR
        start = {request->start_point.z, request->start_point.y, request->start_point.x};
        start.x = map_to_height_index((float)start.x, heights); // z
        start.y = real_to_index((float)start.y); // y
        start.z = real_to_index((float)start.z); // x

        // Conversion of local drone coordinations
        {
            Point converted_start = {};
            converted_start.z = start.z + 8;
            converted_start.y = 263 - start.y;
            converted_start.x = start.x;

            start = converted_start;

            RCLCPP_INFO(get_logger(), "Drone to map = [%d,%d,%d]",
                        converted_start.x, converted_start.y, converted_start.z);

            RCLCPP_INFO(get_logger(), "Drone to map in cm = [%d,%d,%d]",
                        converted_start.x * 5, converted_start.y * 5, converted_start.z * 5);
        }

        goal = {request->goal_point.z, request->goal_point.y, request->goal_point.x};
        goal.x = map_to_height_index((float)goal.x, heights);
        goal.y = real_to_index((float)goal.y);
        goal.z = real_to_index((float)goal.z);

        // Conversion of global coordinations
        {
            Point converted_goal = {};
            // From global CS to map CS
            // converted_goal.z = goal.z + MAP_WIDTH_PIXEL_OFFSET;
            // converted_goal.y = (sz_map_y - MAP_HEIGHT_PIXEL_OFFSET) - goal.y;
            // converted_goal.x = goal.x;

            converted_goal.z = goal.z + 8;
            converted_goal.y = 263 - goal.y;
            converted_goal.x = goal.x;

            goal = converted_goal;

            RCLCPP_INFO(get_logger(), "Global to map in cm = [%d,%d,%d]",
                        converted_goal.x * 5, converted_goal.y * 5, converted_goal.z * 5);
        }

        // Handle map boundaries
        RCLCPP_INFO(this->get_logger(), "Converted start to indexes: [%d,%d,%d]", start.x, start.y, start.z);
        RCLCPP_INFO(this->get_logger(), "Converted goal to indexes: [%d,%d,%d]", goal.x, goal.y, goal.z);

        // Check start point
        if ((start.x >= sz_map_x || start.y >= sz_map_y || start.z >= sz_map_z) ||
            (start.x < 0 || start.y < 0 || start.z < 0))
        {
            RCLCPP_ERROR(this->get_logger(), "Start position out of map [%d,%d,%d]", start.x, start.y, start.z);
            return;
        }
        // Check goal point
        if ((goal.x >= sz_map_x || goal.y >= sz_map_y || goal.z >= sz_map_z) ||
            (goal.x < 0 || goal.y < 0 || goal.z < 0))
        {
            RCLCPP_ERROR(this->get_logger(), "Goal position out of map [%d,%d,%d]", goal.x, goal.y, goal.z);
            return;
        }

        RCLCPP_INFO(get_logger(), "Map start value=%d", map[start.x][start.y][start.z]);
        RCLCPP_INFO(get_logger(), "Map goal value=%d", map[goal.x][goal.y][goal.z]);

        // Define the 6 face neighbor offsets.
        int directions[6][3] = {
            {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

        int x_len = map.size();
        int y_len = map[0].size();
        int z_len = map[0][0].size();

        int deltas[3] = {-1, 0, 1};

        std::queue<Point> q;
        q.push(goal);

        map[goal.x][goal.y][goal.z] = 2; // example starting value

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

                        if (0 <= neighbor.x && neighbor.x < x_len && 0 <= neighbor.y && neighbor.y < y_len && 0 <= neighbor.z && neighbor.z < z_len && map[neighbor.x][neighbor.y][neighbor.z] == 0)
                        {
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
            //     }f
            // }
        }

        RCLCPP_INFO(this->get_logger(), "Start and Goal are equal: %s", std::to_string(start == goal).c_str());

        int start_value = map[start.x][start.y][start.z];
        if (start_value > 2)
        {
            RCLCPP_INFO(this->get_logger(), "After flood_fill...");
            RCLCPP_INFO(this->get_logger(), "Path found to the start from the goal with a length of %d", start_value - 2);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Path not found! start_value: %d", start_value);
            return;
        }

        std::vector<Point> points = get_flood_fill_path(map, start, goal, heights);

        std::string log_message = "{";
        for (const Point p : points)
        {
            log_message += "[";
            log_message += std::to_string(p.z);
            log_message += ",";
            log_message += std::to_string(p.y);
            log_message += ",";
            log_message += std::to_string(p.x);
            log_message += "]";
        }
        log_message += "}";
        RCLCPP_INFO(this->get_logger(), "Generated path is: %s", log_message.c_str());

        lrs_interfaces::msg::PointList pList = converToPointList(points);

        std::string cv_points = "";
        for (const auto &p : pList.points)
        {
            cv_points += "[";
            cv_points += std::to_string(p.x) + ", ";
            cv_points += std::to_string(p.y) + ", ";
            cv_points += std::to_string(p.z) + ", ";
            cv_points += "]\n";
        }
        RCLCPP_INFO(get_logger(), "Final converted points: %s", cv_points.c_str());

        // Tu sa ta posrata vec z nejakeho dovodu nenaplniiii
        // response->points = pList;
        // response->points.points = pList.points;
        response->points.points.clear();  // Clear the existing points

        for (const auto &p : pList.points)
        {
            response->points.points.push_back(p);
        }
        return;
    }

    std::vector<Point> get_flood_fill_path(std::vector<std::vector<std::vector<int>>> map, const Point &start, const Point &goal, std::vector<int> &heights)
    {
        RCLCPP_INFO(this->get_logger(), "get_flood_fill_path started");
        // Find path
        std::vector<Point> path;
        Point current_position = start;
        Point min_neighbour;
        int min_neighbour_value = std::numeric_limits<int>::max();
        bool found;

        int x_len = map.size();
        int y_len = map[0].size();
        int z_len = map[0][0].size();

        int deltas[3] = {-1, 0, 1};

        while (current_position != goal)
        {
            // std::cout << "point " << current_position.toString() << ":\n";
            path.push_back(current_position);
            int current_value = map[current_position.x][current_position.y][current_position.z];
            // Set it to the current value so we can find a lesser value
            int min_neighbour_value = current_value;
            found = false; // Reset
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
                        if (0 <= current_position.x + dx &&
                            current_position.x + dx < x_len &&
                            0 <= current_position.y + dy &&
                            current_position.y + dy < y_len &&
                            0 <= current_position.z + dz &&
                            current_position.z + dz < z_len)
                        {
                            Point neighbor{current_position.x + dx, current_position.y + dy, current_position.z + dz};
                            int neighbor_value = map[neighbor.x][neighbor.y][neighbor.z];

                            if (neighbor_value < min_neighbour_value && neighbor_value != 1)
                            {
                                // std::cout << "chosen neighbor " << neighbor.toString() << ":\n";
                                min_neighbour = neighbor;
                                min_neighbour_value = map[neighbor.x][neighbor.y][neighbor.z];

                                found = true;
                            }
                        }
                    }
                }
            }
            if (!found)
            {
                RCLCPP_ERROR(this->get_logger(), "Path was not found!");
                break;
            }
            current_position = min_neighbour;
        }

        // If path was found
        if (found)
        {
            // Add goal to path
            path.push_back(goal);
            RCLCPP_INFO(this->get_logger(), "Starting simplification of path!");
            std::vector<Point> simplified = simplify_path(path, map);

            RCLCPP_INFO(this->get_logger(), "Original path length: %d", path.size());
            RCLCPP_INFO(this->get_logger(), "Simplified path length: %d", simplified.size());

            // Convert path to global system
            RCLCPP_INFO(this->get_logger(), "Converting points to global coordination system");
            for (auto &point : simplified)
            {
                point.z -= 8;
                point.y = 263 - point.y;
            }
            RCLCPP_INFO(get_logger(), "Conversion ended");
            RCLCPP_INFO(get_logger(), "Points after conversion");
            std::string log_message = "{";
            for (const auto &p : simplified)
            {
                log_message += "[";
                log_message += std::to_string(p.z);
                log_message += ",";
                log_message += std::to_string(p.y);
                log_message += ",";
                log_message += std::to_string(p.x);
                log_message += "]";
            }
            log_message += "}";
            RCLCPP_INFO(this->get_logger(), "%s", log_message.c_str());

            // now lets reverse the indices to real coordinates
            std::vector<Point> real_simplified = path_to_real(simplified, heights);
            RCLCPP_INFO(get_logger(), "Floodfill ended");
            return real_simplified;
        }
        else
        {
            return {};
        }
    }

private:
    rclcpp::Service<lrs_interfaces::srv::FloodFill>::SharedPtr service_;
    std::string mapPath = "/home/lrs-ubuntu/Documents/lrs-git/LRS/src/floodfill_pkg/src/maps/";
    std::vector<std::string> filenames = {"map_025.pgm", "map_075.pgm", "map_080.pgm", "map_100.pgm", "map_125.pgm", "map_150.pgm", "map_175.pgm", "map_180.pgm", "map_200.pgm", "map_225.pgm"};

    const int MAP_HEIGHT_OFFSETS[10] = {25, 75, 80, 100, 125, 150, 175, 180, 200, 225};
    const int X_GRID_SIZE = 5;
    const int Y_GRID_SIZE = 5;

    lrs_interfaces::msg::PointList converToPointList(const std::vector<Point> points)
    {
        lrs_interfaces::msg::PointList point_list = lrs_interfaces::msg::PointList();
        std::vector<lrs_interfaces::msg::Point> sub_points;

        for (const Point point : points)
        {
            lrs_interfaces::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;

            sub_points.push_back(p);
        }

        RCLCPP_INFO(get_logger(), "Converted to PointList");

        std::string cv_points = "";
        for (const auto &p : sub_points)
        {
            cv_points += "[";
            cv_points += std::to_string(p.x) + ", ";
            cv_points += std::to_string(p.y) + ", ";
            cv_points += std::to_string(p.z) + ", ";
            cv_points += "]\n";
        }
        RCLCPP_INFO(get_logger(), "Converted points: %s", cv_points.c_str());

        point_list.points = sub_points;
        return point_list;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloodFillNode>());
    rclcpp::shutdown();
    return 0;
}

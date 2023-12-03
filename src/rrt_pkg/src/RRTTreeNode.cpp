#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <string>
#include <vector>
#include <memory>
#include <math.h>
#include <queue>
#include <iostream>
#include <random>
#include <fstream>

#include "Point.h"
#include "MapReader.h"
#include "lrs_interfaces/srv/rrt.hpp"

#define GRID_IN_CM 5.0
#define GLOBAL_IN_MAP_START_X 8
#define GLOBAL_IN_MAP_START_Y 263

struct RRTNode
{
    Point point;
    std::shared_ptr<RRTNode> parent;
    std::vector<std::shared_ptr<RRTNode>> children;

    RRTNode(Point p) : point(p), parent(nullptr) {}
    RRTNode(Point p, std::shared_ptr<RRTNode> parent_node) : point(p), parent(parent_node) {}
};

class RRTTreeNode : public rclcpp::Node
{
public:
    RRTTreeNode() : Node("rrt_node")
    {
        RCLCPP_INFO(get_logger(), "Creating RRT node...");

        RCLCPP_INFO(get_logger(), "Creating RRT service...");
        service_ = this->create_service<lrs_interfaces::srv::Rrt>(
            "rrt_service",
            std::bind(&RRTTreeNode::handleRrtService, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(get_logger(), "RRT service created");

        RCLCPP_INFO(get_logger(), "Loading map from MapReader...");
        map_reader_ = std::make_shared<MapReader>();
        this->map = map_reader_->getMap();
        RCLCPP_INFO(get_logger(), "Inflated map loaded...");
    }

    std::shared_ptr<RRTNode> addNode(Point point, std::shared_ptr<RRTNode> parent = nullptr)
    {
        auto new_node = std::make_shared<RRTNode>(point, parent);
        nodes.push_back(new_node);
        return new_node;
    }

    void addLink(std::shared_ptr<RRTNode> parent, std::shared_ptr<RRTNode> child)
    {
        child->parent = parent; // Assuming child's parent is set to nullptr initially.
        if (parent)
        {                                      // Check if the parent is not null
            parent->children.push_back(child); // Add the child to the parent's list of children
        }
    }

    std::vector<std::shared_ptr<RRTNode>> &getNodes()
    {
        return nodes;
    }

    std::shared_ptr<RRTNode> findNearest(Point point)
    {
        if (nodes.empty())
        {
            return nullptr; // No nodes in the tree, return null
        }

        std::shared_ptr<RRTNode> nearest_node = nodes[0];               // Start with the first RRTNode as the nearest
        float min_distance = chebyshevDistance(point, nodes[0]->point); // Calculate the distance to the first node

        for (int i = 0; i < (int)nodes.size(); i++)
        {
            float current_distance = chebyshevDistance(point, nodes[i]->point);
            if (current_distance < min_distance) // Found a closer node
            {
                min_distance = current_distance;
                nearest_node = nodes[i];
            }
        }
        return nearest_node;
    }

    Point steer(Point start, Point end, int step_size)
    {
        // Create a vector to store the direction in each dimension
        Point direction = {end.x - start.x, end.y - start.y, end.z - start.z};

        // Normalize the direction vector to a unit step in discrete space
        if (direction.x != 0)
            direction.x = (direction.x > 0) ? 1 : -1;
        if (direction.y != 0)
            direction.y = (direction.y > 0) ? 1 : -1;
        if (direction.z != 0)
            direction.z = (direction.z > 0) ? 1 : -1;

        // Determine the Chebyshev distance
        int chebyshev_dist = chebyshevDistance(start, end);

        // Create the new point using the direction vector and step size
        Point new_point = start;
        if (chebyshev_dist > 0)
        {
            // Use min to prevent overshooting the end point
            new_point.x += direction.x * std::min(step_size, std::abs(end.x - start.x));
            new_point.y += direction.y * std::min(step_size, std::abs(end.y - start.y));
            new_point.z += direction.z * std::min(step_size, std::abs(end.z - start.z));
        }

        return new_point;
    }

    bool pathIsFree(Point start, Point end, float step_size)
    {
        // Calculate the vector from start to end
        Point direction = {end.x - start.x, end.y - start.y, end.z - start.z};

        // Calculate the distance from start to end
        float dist = chebyshevDistance(start, end);

        // Normalize the direction vector
        direction.x /= dist;
        direction.y /= dist;
        direction.z /= dist;

        // How many checks we need to do along the path
        int num_checks = std::ceil(dist / step_size);

        Point current_point = start;
        for (int i = 0; i < num_checks; ++i)
        {
            // If the current point is in an obstacle, the path is not free
            if (isPointInObstacle(current_point, map))
            {
                std::cout << "\033[1;33mPATH IS NOT FREE\033[0m\n";
                return false;
            }

            // Move the current point along the direction vector
            current_point.x += direction.x * step_size;
            current_point.y += direction.y * step_size;
            current_point.z += direction.z * step_size;

            // If we've reached or passed the end point, we stop checking
            if (chebyshevDistance(current_point, start) >= dist)
            {
                break;
            }
        }

        // Finally, check the end point
        if (isPointInObstacle(end, map))
        {
            std::cout << "\033[1;33mPATH IS NOT FREE\033[0m\n";
            return false;
        }

        return true;
    }

    std::vector<Point> pathFromStartToGoal(std::shared_ptr<RRTNode> goal_node, Point goal)
    {
        std::vector<Point> path;
        std::shared_ptr<RRTNode> current_node = goal_node;

        // Follow the parent pointers back to the start
        while (current_node != nullptr)
        {
            // Prepend the current node's point to the path
            path.insert(path.begin(), current_node->point);
            std::cout << "Current point: " << current_node->point.toString() << std::endl;
            current_node = current_node->parent;
        }
        path.push_back(goal);
        return path; // The path is from start to goal
    }

    std::vector<Point> rrtAlgorithm(Point start, Point goal, int max_iterations)
    {
        std::vector<int> heights = map_reader_->getHeights();

        addNode(start);
        std::shared_ptr<RRTNode> new_node;
        RCLCPP_INFO(get_logger(), "Adding start node to the tree");

        float step_size = 25;

        for (int i = 0; i < max_iterations; i++)
        {
            std::cout << "Starting iteration number: " << i + 1 << std::endl;
            Point random_point = createRandomPoint(map.size(), map[0].size(), map[0][0].size());
            std::cout << "\033[1;31mRandom point has been created!: " << random_point.toString() << std::endl
                      << "Checking the position of the random point!\033[0m" << std::endl;

            if (!isPointInObstacle(random_point, map))
            {
                auto nearest_node = findNearest(random_point);
                Point new_point = steer(nearest_node->point, random_point, step_size);
                if (pathIsFree(nearest_node->point, new_point, step_size))
                {
                    new_node = addNode(new_point);
                    addLink(nearest_node, new_node);
                    std::cout << "\033[1;33mPoint has been added!\033[0m\n";
                    float dist = chebyshevDistance(new_node->point, goal);
                    std::cout << "Distance for points: " << new_node->point.toString() << " and " << goal.toString() << " the distance is: " << dist << std::endl;
                    if (dist < step_size)
                    {
                        // std::cout << "\032[1;31mTerminal distance reached\032[0m\n";
                        RCLCPP_INFO(get_logger(), "Terminal distance reached");
                        return pathFromStartToGoal(new_node, goal);
                    }
                }
            }
            else
            {
                std::cout << "\033[1;33mRANDOM POINT STARTING IN THE OBSTACLE\033[0m\n";
            }
        }

        std::cout << "\033[1;31mNO PATH WAS FOUND\033[0m\n";
        return pathFromStartToGoal(new_node, goal);
        // If no path found
        return std::vector<Point>();
    }

    // Variables
private:
    std::vector<std::shared_ptr<RRTNode>> nodes;
    std::vector<std::vector<std::vector<int>>> map;

    std::shared_ptr<MapReader> map_reader_;

    // Services
    rclcpp::Service<lrs_interfaces::srv::Rrt>::SharedPtr service_;

    // Methods
private:
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
            RCLCPP_ERROR(this->get_logger(), "Height value of %f is not possible!", value);
            foundIndex = heights.size() - 1;
        }
        return foundIndex;
    }

    int real_to_index(float value)
    {
        value /= GRID_IN_CM;
        value = std::ceil(value);
        return (int)value;
    }

    Point globalToMap(const Point &point)
    {
        Point map_point;
        map_point.x = point.x;
        map_point.y = GLOBAL_IN_MAP_START_Y - point.y;
        map_point.z = GLOBAL_IN_MAP_START_X + point.z;

        return map_point;
    }

    Point mapToGlobal(const Point &point)
    {
        Point global_point;
        global_point.x = point.x;
        global_point.y = GLOBAL_IN_MAP_START_Y - point.y;
        global_point.z = point.z - GLOBAL_IN_MAP_START_X;

        return global_point;
    }

    Point pointToReal(const Point point)
    {
        Point copy = point;

        copy = mapToGlobal(copy);

        copy.x = map_reader_->getHeights()[copy.x];
        copy.y *= GRID_IN_CM;
        copy.z *= GRID_IN_CM;

        return copy;
    }

    void handleRrtService(const lrs_interfaces::srv::Rrt::Request::SharedPtr request,
                          const lrs_interfaces::srv::Rrt::Response::SharedPtr response)
    {
        response->success = false;
        // Empty existing nodes
        while (!nodes.empty())
            nodes.pop_back();

        // Convert points
        Point start = {request->start_point.z, request->start_point.y, request->start_point.x};
        Point goal = {request->goal_point.z, request->goal_point.y, request->goal_point.x};

        // Convert points to indices
        start.x = map_to_height_index((float)start.x, map_reader_->getHeights());
        start.y = real_to_index((float)start.x);
        start.z = real_to_index((float)start.z);

        goal.x = map_to_height_index((float)goal.x, map_reader_->getHeights());
        goal.y = real_to_index((float)goal.y);
        goal.z = real_to_index((float)goal.z);

        // Convert points from global coordination system to local map coordination system
        start = globalToMap(start);
        goal = globalToMap(goal);

        RCLCPP_INFO(get_logger(), "Requested start: [%d, %d, %d]", start.x, start.y, start.z);
        RCLCPP_INFO(get_logger(), "Requested goal: [%d, %d, %d]", goal.x, goal.y, goal.z);

        int sz_map_x = map.size();
        int sz_map_y = map[0].size();
        int sz_map_z = map[0][0].size();

        // Check map boundaries
        if ((start.x >= sz_map_x || start.y >= sz_map_y || start.z >= sz_map_z) ||
            (start.x < 0 || start.y < 0 || start.z < 0))
        {
            RCLCPP_WARN(get_logger(), "Map dimensions [%d, %d, %d]", sz_map_x, sz_map_y, sz_map_z);
            RCLCPP_ERROR(this->get_logger(), "Start position out of map [%d,%d,%d]", start.x, start.y, start.z);
            response->success = false;
            return;
        }
        // Check goal point
        if ((goal.x >= sz_map_x || goal.y >= sz_map_y || goal.z >= sz_map_z) ||
            (goal.x < 0 || goal.y < 0 || goal.z < 0))
        {
            RCLCPP_WARN(get_logger(), "Map dimensions [%d, %d, %d]", sz_map_x, sz_map_y, sz_map_z);
            RCLCPP_ERROR(this->get_logger(), "Goal position out of map [%d,%d,%d]", goal.x, goal.y, goal.z);
            response->success = false;
            return;
        }

        // Call RRT algorithm
        auto path_points = rrtAlgorithm(start, goal, request->max_iterations);

        for (const auto point : path_points)
        {
            RCLCPP_WARN(get_logger(), "Before conversion: %d, %d, %d", point.x, point.y, point.z);
            auto globalPoint = pointToReal(point);
            RCLCPP_WARN(get_logger(), "After conversion: %d, %d, %d", globalPoint.x, globalPoint.y, globalPoint.z);

            auto gPoint = lrs_interfaces::msg::Point();
            gPoint.x = globalPoint.x;
            gPoint.y = globalPoint.y;
            gPoint.z = globalPoint.z;

            response->points.push_back(gPoint);
            RCLCPP_INFO(get_logger(), "Pushed point: [%d, %d, %d]", gPoint.x, gPoint.y, gPoint.z);
        }

        response->success = true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRTTreeNode>());
    rclcpp::shutdown();
    return 0;
}
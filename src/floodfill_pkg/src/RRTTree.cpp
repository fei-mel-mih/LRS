#include "Point.h"
#include "MapReader.h"
#include <string>
#include <vector>
#include <memory>
#include <math.h>
#include <queue>
#include <iostream>
#include <random>
#include <fstream>

struct Node
{
    Point point;
    std::shared_ptr<Node> parent;
    std::vector<std::shared_ptr<Node>> children; // Add this to keep track of children

    Node(Point p) : point(p), parent(nullptr) {}
    Node(Point p, std::shared_ptr<Node> parent_node) : point(p), parent(parent_node) {}

};

class RRTTree
{
    private:
        std::vector<std::shared_ptr<Node>> nodes;
        std::vector<std::vector<std::vector<int>>> map;
    public:
        RRTTree(std::vector<std::vector<std::vector<int>>> map) 
        {
            this->map = map;
        }

        std::shared_ptr<Node> addNode(Point point, std::shared_ptr<Node> parent = nullptr)
        {
            auto new_node = std::make_shared<Node>(point, parent);
            nodes.push_back(new_node);
            return new_node;
        }

        void addLink(std::shared_ptr<Node> parent, std::shared_ptr<Node> child)
        {
            child->parent = parent; // Assuming child's parent is set to nullptr initially.
            if (parent)
            { // Check if the parent is not null
                parent->children.push_back(child); // Add the child to the parent's list of children
            }
        }

        std::vector<std::shared_ptr<Node>>& getNodes() 
        {
            return nodes;
        }

        std::shared_ptr<Node> findNearest(Point point)
        {
            if (nodes.empty())
            {
                return nullptr; // No nodes in the tree, return null
            }

            std::shared_ptr<Node> nearest_node = nodes[0]; // Start with the first node as the nearest
            float min_distance = chebyshevDistance(point, nodes[0]->point); // Calculate the distance to the first node


            for(int i = 0; i < nodes.size(); i++)
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

        Point steer(Point start, Point end, int step_size) {
            // Create a vector to store the direction in each dimension
            Point direction = {end.x - start.x, end.y - start.y, end.z - start.z};

            // Normalize the direction vector to a unit step in discrete space
            if (direction.x != 0) direction.x = (direction.x > 0) ? 1 : -1;
            if (direction.y != 0) direction.y = (direction.y > 0) ? 1 : -1;
            if (direction.z != 0) direction.z = (direction.z > 0) ? 1 : -1;

            // Determine the Chebyshev distance
            int chebyshev_dist = chebyshevDistance(start, end);

            // Create the new point using the direction vector and step size
            Point new_point = start;
            if (chebyshev_dist > 0) {
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
                    std:: cout << "\033[1;33mPATH IS NOT FREE\033[0m\n";
                    return false;
                }
                
                // Move the current point along the direction vector
                current_point.x += direction.x * step_size;
                current_point.y += direction.y * step_size;
                current_point.z += direction.z * step_size;
                
                // If we've reached or passed the end point, we stop checking
                if (chebyshevDistance(current_point, start) >= dist) {
                    break;
                }
            }
                
            // Finally, check the end point
            if (isPointInObstacle(end, map)) {
                std:: cout << "\033[1;33mPATH IS NOT FREE\033[0m\n";
                return false;
            }
            
            return true;
        }
        
};

void writePathToCSV(const std::vector<Point>& path, const std::string& filename) {
    std::ofstream file(filename);

    // Check if the file stream is open
    if (file.is_open()) {
        // Write the header
        file << "x,y,z\n";

        // Write the data
        for (const auto& point : path) {
            file << point.x << "," << point.y << "," << point.z << "\n";
        }

        // Close the file stream
        file.close();
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
}

void printMap(std::vector<std::vector<std::vector<int>>> map) 
    {
        for (int k = 0; k <  map.size(); ++k) {
            std::cout << "Layer " << k << ":\n";
            for (int i = 0; i < map[0].size(); ++i) {
                for (int j = 0; j < map[0][0].size(); ++j) {
                    std::cout << map[k][i][j] << " ";
                }
                std::cout << std::endl;
            }
            std::cout << "\n";
        }
    }

std::vector<Point> pathFromStartToGoal(std::shared_ptr<Node> goal_node, Point goal)
{
    std::vector<Point> path;
    std::shared_ptr<Node> current_node = goal_node;

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
    // std::vector<std::vector<std::vector<int>>> map = {
    //     {
    //         {0, 1, 0},
    //         {0, 0, 0},
    //         {1, 1, 0}
    //     },
    //     {
    //         {0, 1, 0},
    //         {0, 0, 0},
    //         {1, 1, 0}
    //     },
    //     {
    //         {0, 1, 0},
    //         {0, 0, 0},
    //         {1, 1, 0}
    //     }
    // };
    
    MapReader map_reader;
    std::vector<std::vector<std::vector<int>>> map = map_reader.getMap();

    // We need to recalculate real position to indices
    std::vector<int> heights = map_reader.getHeights();

    RRTTree rrt_tree = RRTTree(map);
    rrt_tree.addNode(start);
    std::shared_ptr<Node> new_node;
    std::cout<<"Adding start node to the tree" << std::endl;

    float step_size = 25;

    for (int i = 0; i < max_iterations; i++)
    {
        std::cout << "Starting iteration number: " << i + 1 << std::endl;
        Point random_point = createRandomPoint(map.size(), map[0].size(), map[0][0].size());
        std::cout << "\033[1;31mRandom point has been created!: " << random_point.toString() << std::endl
        << "Checking the position of the random point!\033[0m" << std::endl;

        if (!isPointInObstacle(random_point, map))
        {
            auto nearest_node = rrt_tree.findNearest(random_point);
            Point new_point = rrt_tree.steer(nearest_node->point, random_point, step_size);
            if (rrt_tree.pathIsFree(nearest_node->point, new_point, step_size))
            {
                new_node = rrt_tree.addNode(new_point);
                rrt_tree.addLink(nearest_node, new_node);
                std:: cout << "\033[1;33mPoint has been added!\033[0m\n";
                float dist = chebyshevDistance(new_node->point, goal);
                std::cout << "Distance for points: " << new_node->point.toString() << " and " << goal.toString() << " the distance is: " << dist << std::endl;
                if (dist < step_size)
                {
                    std:: cout << "\032[1;31mTerminal distance reached\032[0m\n";
                    return pathFromStartToGoal(new_node, goal);
                }
            }
        } else 
        {
            std:: cout << "\033[1;33mRANDOM POINT STARTING IN THE OBSTACLE\033[0m\n";
        }
    }

    std:: cout << "\033[1;31mNO PATH WAS FOUND\033[0m\n";
    return pathFromStartToGoal(new_node, goal);
    // If no path found
    return std::vector<Point>();
}


int main(int argc, char **argv)
{
    std::cout << "Entering main..." << std::endl;
    
    std::cout << "Before RRT..." << std::endl;
    Point start = {0, 163, 46};
    Point goal = {8, 233, 280};
    std::vector<Point> path = rrtAlgorithm(start, goal, 1000);

    writePathToCSV(path, "rrt_path.csv");
    return 0;
}



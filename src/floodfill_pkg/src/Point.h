#ifndef POINT_H
#define POINT_H

#include <random>
#include <iostream>
#include <algorithm>

std::random_device rd;  // Obtain a random number from hardware
std::mt19937 gen(rd()); // Seed the generator

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
Point createRandomPoint(int x_max, int y_max, int z_max)
{
    std::uniform_int_distribution<> distr_x(0, x_max-1);
    std::uniform_int_distribution<> distr_y(0, y_max-1);
    std::uniform_int_distribution<> distr_z(0, z_max-1);

    Point p;
    p.x = distr_x(gen);
    p.y = distr_y(gen);
    p.z = distr_z(gen);

    return p;
}

bool isPointInObstacle(Point point, std::vector<std::vector<std::vector<int>>> map)
{
    // std::cout << "Checking point validation" << std::endl;
    return map[point.x][point.y][point.z] == 1;
}

// Chebyshev distance (L∞ norm) allows diagonal movement
int chebyshevDistance(const Point& a, const Point& b) 
{
    return std::max({std::abs(a.x - b.x), std::abs(a.y - b.y), std::abs(a.z - b.z)});
}

float distance(const Point& a, const Point& b)
{
    // Implement the Euclidean distance formula
    return std::sqrt((a.x - b.x) * (a.x - b.x) +
                    (a.y - b.y) * (a.y - b.y) +
                    (a.z - b.z) * (a.z - b.z));
}
#endif
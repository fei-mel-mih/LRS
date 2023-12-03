#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "lrs_interfaces/msg/command.hpp"

#include <math.h>
#include <iostream>

namespace lrs_utils
{
    float quaternionToYaw(const geometry_msgs::msg::Quaternion &quaternion)
    {
        // Convert quaternion to yaw angle
        float sinYaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
        float cosYaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);

        return atan2(sinYaw, cosYaw);
    }

    geometry_msgs::msg::Quaternion yawToQuaternion(float yaw)
    {
        geometry_msgs::msg::Quaternion quaternion;

        float halfYaw = yaw * 0.5;
        float cosYaw = cos(halfYaw);
        float sinYaw = sin(halfYaw);

        float qx = 0.0;
        float qy = 0.0;
        float qz = sinYaw;
        float qw = cosYaw;

        quaternion.x = qx;
        quaternion.y = qy;
        quaternion.z = qz;
        quaternion.w = qw;

        return quaternion;
    }

    bool isLocationInsideRegion(const geometry_msgs::msg::Point &current, const geometry_msgs::msg::Point &goal, float offset)
    {
        float distance = std::sqrt(
            std::pow(current.x - goal.x, 2) +
            std::pow(current.y - goal.y, 2) +
            std::pow(current.z - goal.z, 2));

        // Check if the distance is less than or equal to the offset
        return distance <= offset;
    }
    
    // Enums
    enum PRECISION_ENUM
    {
        SOFT,
        HARD,
    };

    enum TASK_ENUM
    {
        TAKEOFF,
        LAND,
        LANDTAKEOFF,
        YAW,
        CIRCLE,
        NONE
    };

    // Structures
    struct ConvertedCommand
    {
        float x;
        float y;
        float z;
        PRECISION_ENUM precision;
        TASK_ENUM task = TASK_ENUM::NONE;
        int yaw_value = -1;

        bool operator!=(const ConvertedCommand &other)
        {
            return (x != other.x) || (y != other.y) || (z != other.z) ||
                   (precision != other.precision) || (task != other.task) ||
                   (yaw_value != other.yaw_value);
        }

        bool operator==(const ConvertedCommand &other)
        {
            return (x == other.x) && (y == other.y) && (z == other.z) &&
                   (precision == other.precision) && (task == other.task) &&
                   (yaw_value == other.yaw_value);
        }
    };

}
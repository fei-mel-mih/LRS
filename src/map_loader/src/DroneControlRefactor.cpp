#include <regex>
#include <queue>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

#include "lrs_data_structures.hpp"
#include "lrs_utils.hpp"

#include "lrs_interfaces/msg/command.hpp"
#include "lrs_interfaces/msg/point.hpp"
#include "lrs_interfaces/msg/point_list.hpp"

#include "lrs_interfaces/srv/mission_command.hpp"
#include "lrs_interfaces/srv/flood_fill.hpp"

#define WHILE_CHECK_TIMEOUT 250ms
#define WHILE_CHECK_POSITION_TIMEOUT 250ms
#define WAIT_FOR_SERVICE_TIMEOUT 1s

#define TO_CM 100
#define TO_M (1.0 / 100.0)

#define DRONE_IN_GLOBAL_START_X 13.60f
#define DRONE_IN_GLOBAL_START_Y 1.50f
#define DRONE_IN_GLOBAL_START_Z 0.00f
#define DRONE_START_YAW M_PI_2
#define MAP_MAX_WIDTH 18.20f
#define MAP_MAX_HEIGHT 13.50f

#define HARD_PRECISION 0.05f
#define SOFT_PRECISION 0.15f
#define FLOODFILL_PRECISION 0.20f

using namespace std::chrono_literals;

class DroneControlNode : public rclcpp::Node
{
public:
    DroneControlNode() : Node("drone_control_node")
    {
        RCLCPP_INFO(get_logger(), "Starting initialization...");
        init();
        RCLCPP_INFO(get_logger(), "Initialization ended");

        RCLCPP_INFO(get_logger(), "Waiting for mavros sitl...");
        // MAVROS SITL connection
        while (rclcpp::ok() && !current_state_.connected)
        {
            rclcpp::spin_some(get_node_base_interface());
            std::this_thread::sleep_for(WHILE_CHECK_POSITION_TIMEOUT);
        }
        RCLCPP_INFO(get_logger(), "Mavros sitl connected");

        RCLCPP_INFO(get_logger(), "Aquiring mission plan...");
        acquireMission();

        RCLCPP_INFO(get_logger(), "Changing mode to guided...");
        handleModeChange("GUIDED");

        RCLCPP_INFO(get_logger(), "Arming started...");
        handleArming();

        RCLCPP_INFO(get_logger(), "Starting control loop...");
        controlLoop();
    }

    // Init function
    void init()
    {
        rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
        custom_qos.depth = 1;
        custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(custom_qos.history, 1), custom_qos);

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state",
            10,
            std::bind(&DroneControlNode::stateCallback, this, std::placeholders::_1));

        local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose",
            qos,
            std::bind(&DroneControlNode::localPositionCallback, this, std::placeholders::_1));

        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        floodfill_cleint_ = this->create_client<lrs_interfaces::srv::FloodFill>("floodfill_service");
        mission_client_ = this->create_client<lrs_interfaces::srv::MissionCommand>("mission_loader_service");
        land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/land");
    }

    // Control loop function
    void controlLoop()
    {
        while (rclcpp::ok() && !commands_.empty())
        {
            rclcpp::spin_some(get_node_base_interface());

            float command_precision;
            switch (current_command_.precision)
            {
            case lrs_utils::PRECISION_ENUM::HARD:
                command_precision = HARD_PRECISION;
                break;

            case lrs_utils::PRECISION_ENUM::SOFT:
                command_precision = SOFT_PRECISION;
                break;
            }

            // Check if mission command was reached
            geometry_msgs::msg::Point command_goal;
            command_goal.x = current_command_.x;
            command_goal.y = current_command_.y;
            command_goal.z = current_command_.z;

            if (lrs_utils::isLocationInsideRegion(current_position_.position, command_goal, command_precision) ||
                current_command_.task == lrs_utils::TASK_ENUM::TAKEOFF)
            {
                RCLCPP_INFO(get_logger(), "Mission command reached. Performing task...");
                // Complete action
                switch (current_command_.task)
                {
                case lrs_utils::TASK_ENUM::TAKEOFF:
                    handleTakeOff(current_command_.z, command_precision);
                    break;
                case lrs_utils::TASK_ENUM::YAW:
                    handleYaw(current_command_.yaw_value, current_command_.precision == lrs_utils::PRECISION_ENUM::HARD ? 10.0f : 20.0f);
                    break;
                case lrs_utils::TASK_ENUM::LANDTAKEOFF:
                    handleLandTakeOff(current_command_.z, command_precision);
                    break;
                case lrs_utils::TASK_ENUM::LAND:
                    handleLanding();
                    break;
                case lrs_utils::TASK_ENUM::NONE:
                    break;
                }
                RCLCPP_INFO(get_logger(), "Mission task completed. Moving to next one...");

                // Move to next command
                commands_.erase(commands_.begin());
                current_command_ = commandConverter(commands_.front());

                RCLCPP_INFO(get_logger(), "Current mission command is: %.2f %.2f %.2f %d %d yaw=%d",
                            current_command_.x, current_command_.y, current_command_.z, current_command_.precision, current_command_.task, current_command_.yaw_value);

                // Generate new floodfill path
                acquirePath();
            }
            else
            {
                if (floodfill_points_.empty())
                {
                    RCLCPP_ERROR(get_logger(), "Missing floodfill points! Landing...");
                    handleLanding();
                    rclcpp::shutdown();
                    return;
                }
                
                // Check if floodfill point was reached
                if (lrs_utils::isLocationInsideRegion(current_position_.position, floodfill_points_.front(), command_precision))
                {
                    RCLCPP_INFO(get_logger(), "Path checkpoint reached. Moving to next one...");
                    floodfill_points_.pop();

                    RCLCPP_INFO(get_logger(), "Current checkpoint: %.2f, %.2f, %.2f",
                                floodfill_points_.front().x, floodfill_points_.front().y, floodfill_points_.front().z);
                }

                // Generate requested position message
                auto header = std_msgs::msg::Header();
                header.frame_id = "control loop - position control";
                header.stamp = get_clock()->now();

                auto pose = geometry_msgs::msg::Pose();
                pose.position = globalToLocal(floodfill_points_.front());
                RCLCPP_INFO(get_logger(), "Requested position: %.2f, %.2f, %.2f", pose.position.x, pose.position.y, pose.position.z);

                auto message = geometry_msgs::msg::PoseStamped();
                message.header = header;
                message.pose = pose;

                local_pos_pub_->publish(message);
            }

            std::this_thread::sleep_for(WHILE_CHECK_POSITION_TIMEOUT);
        }

        RCLCPP_INFO(get_logger(), "MISSION ENDED... Shuting down");
        rclcpp::shutdown();
        return;
    }

    // Aquire functions
    void acquireMission()
    {
        waitForService(mission_client_, "mission", WAIT_FOR_SERVICE_TIMEOUT);

        lrs_interfaces::srv::MissionCommand::Request::SharedPtr request = std::make_shared<lrs_interfaces::srv::MissionCommand::Request>();
        auto future = mission_client_->async_send_request(request);
        RCLCPP_INFO(get_logger(), "Loading mission plan");

        while (rclcpp::ok())
        {
            rclcpp::spin_some(get_node_base_interface());

            auto status = future.wait_for(100ms);
            if (status == std::future_status::ready)
            {
                auto response = future.get();
                if (response->success)
                {
                    commands_ = response->commands;

                    if (commands_.empty())
                    {
                        RCLCPP_ERROR(get_logger(), "Received empty command list");
                        rclcpp::shutdown();
                        return;
                    }

                    std::string command_message = "\nx\ty\tz\tprecision\ttask\n";
                    for (const auto &command : commands_)
                    {
                        command_message += std::to_string(command.x) + "\t";
                        command_message += std::to_string(command.y) + "\t";
                        command_message += std::to_string(command.z) + "\t";
                        command_message += command.precision + "\t";
                        command_message += command.task + "\n";
                    }
                    RCLCPP_INFO(this->get_logger(), "Mission commands loaded");
                    RCLCPP_INFO(this->get_logger(), command_message.c_str());

                    RCLCPP_INFO(get_logger(), "Setting current mission command...");
                    current_command_ = commandConverter(commands_.front());
                    RCLCPP_INFO(get_logger(), "Current mission command is: %.2f %.2f %.2f %d %d %.2f",
                                current_command_.x, current_command_.y, current_command_.z, current_command_.precision, current_command_.task, current_command_.yaw_value);

                    break;
                }
                else
                {
                    RCLCPP_ERROR(get_logger(), "Mission loading was not succesfull");
                    rclcpp::shutdown();
                    return;
                }
                RCLCPP_INFO(get_logger(), "Waiting for mission plan");
                std::this_thread::sleep_for(WHILE_CHECK_TIMEOUT);
            }
        }
        RCLCPP_INFO(get_logger(), "Mission plan loaded succesfully");
    }

    void acquirePath()
    {
        waitForService(floodfill_cleint_, "flood_fill", WAIT_FOR_SERVICE_TIMEOUT);

        lrs_interfaces::msg::Point start;
        start.x = (int) (current_position_.position.x * TO_CM);
        start.y = (int) (current_position_.position.y * TO_CM);
        start.z = (int) (current_position_.position.z * TO_CM);

        lrs_interfaces::msg::Point goal;
        goal.x = (int) (current_command_.x * TO_CM);
        goal.y = (int) (current_command_.y * TO_CM);
        goal.z = (int) (current_command_.z * TO_CM);

        lrs_interfaces::srv::FloodFill::Request::SharedPtr request = std::make_shared<lrs_interfaces::srv::FloodFill::Request>();
        request->start_point = start;
        request->goal_point = goal;

        auto future = floodfill_cleint_->async_send_request(request);

        RCLCPP_INFO(get_logger(), "Requesting path from start[%d, %d, %d] to goal[%d, %d, %d]",
                    request->start_point.x, request->start_point.y, request->start_point.z,
                    request->goal_point.x, request->goal_point.y, request->goal_point.z);

        while (rclcpp::ok())
        {
            rclcpp::spin_some(get_node_base_interface());

            auto status = future.wait_for(100ms);
            if (status == std::future_status::ready)
            {
                auto response = future.get();

                if (response->success)
                {
                    // Clear old checkpoints if necessary
                    while (!floodfill_points_.empty())
                        floodfill_points_.pop();

                    // Set new checkpoints
                    std::string _log_message = "\n--- Flood fill points ---\nX\t\tY\t\tZ\n";
                    for (const auto &point : response->points)
                    {
                        geometry_msgs::msg::Point conv_point;
                        conv_point.x = point.z * TO_M;
                        conv_point.y = point.y * TO_M;
                        conv_point.z = point.x * TO_M;

                        floodfill_points_.push(conv_point);
                        _log_message += std::to_string(conv_point.x) + "\t\t";
                        _log_message += std::to_string(conv_point.y) + "\t\t";
                        _log_message += std::to_string(conv_point.z) + "\t\t";
                        _log_message += "\n";
                    }
                    RCLCPP_INFO(get_logger(), _log_message.c_str());
                    break;
                }
                else
                {
                    RCLCPP_ERROR(get_logger(), "Cannot get new floodfill path. Drone is going to land...");
                    handleLanding();
                    rclcpp::shutdown();
                    return;
                }
            }
            RCLCPP_INFO(get_logger(), "Waiting for new floodfill points");
            std::this_thread::sleep_for(WHILE_CHECK_TIMEOUT);
        }
        RCLCPP_INFO(get_logger(), "Floodfill point aquired");
    }

    // Handle functions
    void handleModeChange(std::string mode)
    {
        waitForService(set_mode_client_, "set_mode", WAIT_FOR_SERVICE_TIMEOUT);

        mavros_msgs::srv::SetMode::Request::SharedPtr request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;

        set_mode_client_->async_send_request(request);

        while (rclcpp::ok())
        {
            rclcpp::spin_some(get_node_base_interface());
            if (current_state_.mode == mode)
            {
                RCLCPP_INFO(get_logger(), "Mode changed to %s", current_state_.mode.c_str());
                break;
            }

            RCLCPP_INFO(get_logger(), "Waiting for mode to change");
            std::this_thread::sleep_for(WHILE_CHECK_TIMEOUT);
        }
    }

    void handleArming()
    {
        waitForService(arming_client_, "arming", WAIT_FOR_SERVICE_TIMEOUT);

        mavros_msgs::srv::CommandBool::Request::SharedPtr request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;
        arming_client_->async_send_request(request);

        while (rclcpp::ok())
        {
            rclcpp::spin_some(get_node_base_interface());

            if (current_state_.armed)
            {
                RCLCPP_INFO(get_logger(), "Drone armed");
                break;
            }

            RCLCPP_INFO(get_logger(), "Drone arming");
            std::this_thread::sleep_for(WHILE_CHECK_TIMEOUT);
        }
    }

    void handleTakeOff(float altitude, float precision)
    {
        waitForService(takeoff_client_, "take-off", WAIT_FOR_SERVICE_TIMEOUT);

        mavros_msgs::srv::CommandTOL::Request::SharedPtr request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->altitude = altitude;
        // FIXME: required yaw ??

        takeoff_client_->async_send_request(request);

        while (rclcpp::ok())
        {
            rclcpp::spin_some(get_node_base_interface());

            auto z = current_position_.position.z;

            bool b_tookoff = (z >= (request->altitude - precision) && (z <= (request->altitude + precision)));

            if (b_tookoff)
            {
                RCLCPP_INFO(get_logger(), "Drone took off");
                break;
            }

            RCLCPP_INFO(get_logger(), "Drone taking off");
            std::this_thread::sleep_for(WHILE_CHECK_POSITION_TIMEOUT);
        }
    }

    void handleLanding()
    {
        waitForService(land_client_, "land", WAIT_FOR_SERVICE_TIMEOUT);

        mavros_msgs::srv::CommandTOL::Request::SharedPtr request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

        land_client_->async_send_request(request);

        while (rclcpp::ok())
        {
            rclcpp::spin_some(get_node_base_interface());

            if (!current_state_.armed)
            {
                RCLCPP_INFO(get_logger(), "Drone landed");
                break;
            }

            RCLCPP_INFO(get_logger(), "Drone landing");
            std::this_thread::sleep_for(WHILE_CHECK_TIMEOUT);
        }
    }

    void handleLandTakeOff(float altitude, float precision)
    {
        handleLanding();
        handleModeChange("GUIDED");
        handleArming();
        handleTakeOff(altitude, precision);
    }

    void handleYaw(float yaw, float precision_degrees)
    {
        RCLCPP_INFO(get_logger(), "Requested yaw=%.3f deg", yaw);
        yaw = yaw * (M_PI / 180.0);
        RCLCPP_INFO(get_logger(), "Requested yaw=%.3f rad", yaw);

        RCLCPP_INFO(get_logger(), "Requested precision=%.3f deg", precision_degrees);
        float precision = precision_degrees * M_PI / 180.0;
        RCLCPP_INFO(get_logger(), "Requested precision=%.3f rad", precision);

        auto request = lrs_utils::yawToQuaternion(yaw);
        auto pose = geometry_msgs::msg::Pose();
        pose.orientation = request;
        pose.position = globalToLocal(floodfill_points_.front());

        auto header = std_msgs::msg::Header();
        header.frame_id = "control_loop-yaw";
        header.stamp = get_clock()->now();

        auto message = geometry_msgs::msg::PoseStamped();
        message.header = header;
        message.pose = pose;

        local_pos_pub_->publish(message);
        RCLCPP_INFO(get_logger(), "Requested orientation\nw=%f x=%ff y=%f z=%f",
                    request.w, request.x, request.y, request.z);

        while (rclcpp::ok())
        {
            rclcpp::spin_some(get_node_base_interface());

            float current_yaw = std::abs(lrs_utils::quaternionToYaw(current_position_.orientation));
            RCLCPP_INFO(get_logger(), "Current yaw=%.2f", current_yaw);

            float yaw_difference = yaw - current_yaw;
            RCLCPP_INFO(get_logger(), "Yaw difference=%.2f", yaw_difference);
            
            RCLCPP_INFO(get_logger(), "%.2f <= %.2f == %d", yaw_difference, precision, yaw_difference <= precision);
            if (yaw_difference <= precision)
            {
                RCLCPP_INFO(get_logger(), "Drone reached request yaw %.2f", yaw);
                break;
            }

            RCLCPP_INFO(get_logger(), "Drone rotating. Current yaw=%.2f  Yaw difference=%.2f  Requested yaw=%.2f", current_yaw, yaw_difference, yaw);
            std::this_thread::sleep_for(WHILE_CHECK_POSITION_TIMEOUT);
        }
    }

    // Callback functions
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    void localPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped current_pose = *msg;
        current_position_.position = localToGlobal(current_pose.pose.position);
        current_position_.orientation = current_pose.pose.orientation;
    }

    // Converters
    lrs_utils::ConvertedCommand commandConverter(const lrs_interfaces::msg::Command &command)
    {
        std::string task = command.task;
        lrs_utils::ConvertedCommand converted_command;

        // Parse command position
        converted_command.x = command.x;
        converted_command.y = command.y;
        converted_command.z = command.z;

        // Parse precision
        lrs_utils::PRECISION_ENUM precision;
        if (command.precision == "soft")
        {
            precision = lrs_utils::PRECISION_ENUM::SOFT;
        }
        if (command.precision == "hard")
        {
            precision = lrs_utils::PRECISION_ENUM::HARD;
        }
        converted_command.precision = precision;

        // Parse task
        if (task == "takeoff")
        {
            converted_command.task = lrs_utils::TASK_ENUM::TAKEOFF;
        }
        else if (task == "land")
        {
            converted_command.task = lrs_utils::TASK_ENUM::LAND;
        }
        else if (task == "landtakeoff")
        {
            converted_command.task = lrs_utils::TASK_ENUM::LANDTAKEOFF;
        }
        else if (task.find("yaw") == 0)
        {
            // Create a regular expression pattern to match the string.
            std::regex pattern(R"(yaw(\d+))");
            // Use the regex_search() function to search for a match in the string.
            std::smatch match;
            if (std::regex_search(task, match, pattern))
            {
                // Get the captured group containing the number value.
                std::string number_value = match[1];

                // Convert the captured group to an integer using the stoi() function.
                int number = std::stoi(number_value);

                // Print the number value.
                RCLCPP_INFO(this->get_logger(), "Parsed yaw value as %d", number);

                converted_command.task = lrs_utils::TASK_ENUM::YAW;
                converted_command.yaw_value = number;
            }
            else
            {
                converted_command.yaw_value = -666;
                RCLCPP_ERROR(this->get_logger(), "Cannot parse task as it doesn't match regular expresion");
            }
        }
        else
        {
            converted_command.task = lrs_utils::TASK_ENUM::NONE;
        }
        return converted_command;
    }

    geometry_msgs::msg::Point localToGlobal(const geometry_msgs::msg::Point &local_position)
    {
        geometry_msgs::msg::Point global_position;

        global_position.x = DRONE_IN_GLOBAL_START_X + local_position.y;
        global_position.y = DRONE_IN_GLOBAL_START_Y - local_position.x;
        global_position.z = DRONE_IN_GLOBAL_START_Z + local_position.z;

        return global_position;
    }

    geometry_msgs::msg::Point globalToLocal(const geometry_msgs::msg::Point &global_position)
    {
        geometry_msgs::msg::Point local_position;

        local_position.x = DRONE_IN_GLOBAL_START_Y - global_position.y;
        local_position.y = global_position.x - DRONE_IN_GLOBAL_START_X;
        local_position.z = global_position.z - DRONE_IN_GLOBAL_START_Z;

        return local_position;
    }

    // Template methods
    template <typename T>
    void waitForService(const std::shared_ptr<rclcpp::Client<T>> &client, std::string service_name, std::chrono::seconds timeout)
    {
        while (!client->wait_for_service(timeout))
        {
            rclcpp::spin_some(get_node_base_interface());
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(get_logger(), "Interupted while waiting for %s service");
                return;
            }
            RCLCPP_INFO(get_logger(), "Waiting for %s service", service_name.c_str());
            std::this_thread::sleep_for(WHILE_CHECK_TIMEOUT);
        }
    }

private:
    // Subscribers and publishers
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;

    // Custom clients
    rclcpp::Client<lrs_interfaces::srv::FloodFill>::SharedPtr floodfill_cleint_;
    rclcpp::Client<lrs_interfaces::srv::MissionCommand>::SharedPtr mission_client_;

    // Variables
    lrs_utils::ConvertedCommand current_command_;
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::Pose current_position_;
    std::vector<lrs_interfaces::msg::Command> commands_;
    std::queue<geometry_msgs::msg::Point> floodfill_points_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneControlNode>());
    rclcpp::shutdown();
    return 0;
}
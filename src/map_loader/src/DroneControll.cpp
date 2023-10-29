#pragma region includes
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

#include "lrs_interfaces/msg/command.hpp"
#include "lrs_interfaces/msg/point.hpp"
#include "lrs_interfaces/msg/point_list.hpp"

#include "lrs_interfaces/srv/mission_command.hpp"
#include "lrs_interfaces/srv/flood_fill.hpp"
#pragma endregion includes

#define TO_CM 100

using namespace std::chrono_literals;

class DroneControll : public rclcpp::Node
{
public:
    DroneControll() : Node("drone_controll_node")
    {
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state",
            10,
            std::bind(&DroneControll::state_cb, this, std::placeholders::_1));

        rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
        custom_qos.depth = 1;
        custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(custom_qos.history, 1), custom_qos);

        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos, std::bind(&DroneControll::handleLocalPosition, this, std::placeholders::_1));

        floodfill_cleint_ = this->create_client<lrs_interfaces::srv::FloodFill>("floodfill_service");
        mission_client_ = this->create_client<lrs_interfaces::srv::MissionCommand>("mission_loader_service");
        land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/land");

        // Wait for MAVROS SITL connection
        while (rclcpp::ok() && !current_state_.connected)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }

        // Set mode
        mavros_msgs::srv::SetMode::Request guided_set_mode_req;
        guided_set_mode_req.custom_mode = "GUIDED";
        while (!set_mode_client_->wait_for_service(1s))
        {
            rclcpp::spin_some(this->get_node_base_interface());
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the set_mode service. Exiting.");
                return;
            }
        }
        auto result = set_mode_client_->async_send_request(std::make_shared<mavros_msgs::srv::SetMode::Request>(guided_set_mode_req));

        // Test if drone state really changed to GUIDED
        while (rclcpp::ok() && !(current_state_.mode == "GUIDED"))
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }

        // Arm
        while (!arming_client_->wait_for_service(1s))
        {
            rclcpp::spin_some(this->get_node_base_interface());
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the arming service. Exiting.");
                return;
            }
        }
        mavros_msgs::srv::CommandBool::Request arming_request;
        arming_request.value = true;
        auto aiming_result = arming_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandBool::Request>(arming_request));

        // Take-off
        while (rclcpp::ok() && !current_state_.armed)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }
        while (!takeoff_client_->wait_for_service(1s))
        {
            rclcpp::spin_some(this->get_node_base_interface());
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the take-off service. Exiting.");
                return;
            }
        }
        mavros_msgs::srv::CommandTOL::Request takeoff_request;
        takeoff_request.altitude = TAKEOFF_ALTITUDE;
        auto takeoff_future = takeoff_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandTOL::Request>(takeoff_request));

        // Load mission plan
        while (!mission_client_->wait_for_service(1s))
        {
            rclcpp::spin_some(this->get_node_base_interface());
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the mission_client service. Exiting...");
                std::cerr << "Interrupted while waiting for the mission_client service. Exiting..." << std::endl;
                return;
            }
        }
        auto mission_future = mission_client_->async_send_request(
            std::make_shared<lrs_interfaces::srv::MissionCommand::Request>(),
            std::bind(&DroneControll::handleMissionPlan, this, std::placeholders::_1));

        // Check if mission was loaded
        while (this->commands.size() <= 0)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            RCLCPP_INFO(this->get_logger(), "Waiting for mission commands...");
            std::this_thread::sleep_for(100ms);
        }

        // Convert commands
        for (const auto &command : this->commands)
        {
            this->converted_commands.push(commandConverter(command));
        }

        this->current_command = converted_commands.front();
        converted_commands.pop();

        // Wait for initial position
        while (!this->b_initial_position_aquired)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(250ms);
        }
        RCLCPP_INFO(this->get_logger(), "Initial positon aquired: positon=[%f,%f,%f]", current_position.position.x, current_position.position.y, current_position.position.z);

        // Get initial floodfill points
        while (!floodfill_cleint_->wait_for_service(1s))
        {
            rclcpp::spin_some(this->get_node_base_interface());
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interupted while waiting for foodfill client. Exiting...");
                return;
            }
        }
        lrs_interfaces::srv::FloodFill::Request initial_floodfill_request;

        lrs_interfaces::msg::Point start_point;
        start_point.x = current_position.position.x * TO_CM;
        start_point.y = current_position.position.y * TO_CM;
        start_point.z = current_position.position.z * TO_CM;

        lrs_interfaces::msg::Point goal_point;
        goal_point.x = current_command.x * TO_CM;
        goal_point.y = current_command.y * TO_CM;
        goal_point.z = current_command.z * TO_CM;

        initial_floodfill_request.start_point = start_point;
        initial_floodfill_request.goal_point = goal_point;

        RCLCPP_INFO(
            this->get_logger(),
            "Retreiveing point for path start=[%d,%d,%d] : goal=[%d,%d,%d]",
            start_point.x, start_point.y, start_point.z,
            goal_point.x, goal_point.y, goal_point.z);

        auto floofill_future = this->floodfill_cleint_->async_send_request(std::make_shared<lrs_interfaces::srv::FloodFill::Request>(initial_floodfill_request));
        retrieveNewFloodFillPoints(floofill_future);

        // Wait for take off
        while (rclcpp::ok())
        {
            rclcpp::spin_some(this->get_node_base_interface());

            auto current_z = current_position.position.z;

            bool b_tookoff = (current_z >= (TAKEOFF_ALTITUDE - SOFT_PRECISION)) && (current_z <= (TAKEOFF_ALTITUDE + SOFT_PRECISION));

            if (b_tookoff)
            {
                RCLCPP_INFO(this->get_logger(), "Drone took off...");
                break;
            }

            RCLCPP_INFO(this->get_logger(), "Drone taking off...");
            std::this_thread::sleep_for(250ms);
        }

        if (this->floodfill_points.size() <= 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Path was not found for requested goals");
            return;
        }
        RCLCPP_INFO(get_logger(), "Length of floodfill path is %d", this->floodfill_points.size());

        RCLCPP_INFO(this->get_logger(), "----------------------- Controlling started -----------------------");
    }

private:
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
        NONE
    };

    // Structs
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

    // Methods

    ConvertedCommand commandConverter(const lrs_interfaces::msg::Command command)
    {
        std::string task = command.task;
        ConvertedCommand converted_command;

        // Parse command position
        converted_command.x = command.x;
        converted_command.y = command.y;
        converted_command.z = command.z;

        // Parse precision
        PRECISION_ENUM precision;
        if (command.precision == "soft")
        {
            precision = PRECISION_ENUM::SOFT;
        }
        if (command.precision == "hard")
        {
            precision = PRECISION_ENUM::HARD;
        }
        converted_command.precision = precision;

        // Parse task
        if (task == "takeoff")
        {
            converted_command.task = TASK_ENUM::TAKEOFF;
        }
        else if (task == "land")
        {
            converted_command.task = TASK_ENUM::LAND;
        }
        else if (task == "landtakeoff")
        {
            converted_command.task = TASK_ENUM::LANDTAKEOFF;
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

                converted_command.task = TASK_ENUM::YAW;
                converted_command.yaw_value = number;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Cannot parse task as it doesn't match regular expresion");
            }
        }
        else
        {
            converted_command.task = TASK_ENUM::NONE;
        }
        return converted_command;
    }

    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
        // RCLCPP_INFO(this->get_logger(), "Current State: %s", current_state_.mode.c_str());
    }

    void handleLanding(rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future)
    {
        RCLCPP_INFO(this->get_logger(), "handleLanding started");
        try
        {
            auto landing_response = future.get();
            RCLCPP_INFO(this->get_logger(), "Landing request sended: %s", std::to_string(landing_response->success));
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in handleLanding: %s", e.what());
            return;
        }
    }

    void handleMissionPlan(rclcpp::Client<lrs_interfaces::srv::MissionCommand>::SharedFuture future)
    {
        RCLCPP_INFO(get_logger(), "handleMissionPlan started");
        try
        {
            auto mission_response = future.get();
            this->commands = mission_response->commands;

            std::string command_message = "\nx\ty\tz\tprecision\ttask\n";
            for (const auto command : this->commands)
            {
                command_message += std::to_string(command.x) + "\t";
                command_message += std::to_string(command.y) + "\t";
                command_message += std::to_string(command.z) + "\t";
                command_message += command.precision + "\t";
                command_message += command.task + "\n";
            }
            RCLCPP_INFO(this->get_logger(), "Mission commands loaded");
            RCLCPP_INFO(this->get_logger(), command_message.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
            std::cerr << e.what() << std::endl;
            return;
        }
        RCLCPP_INFO(get_logger(), "handleMissionPlan ended");
        return;
    }

    void handlePositionControll()
    {
        // Check if current goal is reached
        bool b_checkpoint_reached;

        geometry_msgs::msg::Pose goal_pose;
        goal_pose.position.x = current_command.x;
        goal_pose.position.y = current_command.y;
        goal_pose.position.z = current_command.z;

        switch (current_command.precision)
        {
        case PRECISION_ENUM::HARD:
            b_checkpoint_reached = isLocationInsideRegion(current_position, goal_pose, HARD_PRECISION);
            break;
        case PRECISION_ENUM::SOFT:
            b_checkpoint_reached = isLocationInsideRegion(current_position, goal_pose, SOFT_PRECISION);
            break;
        }

        // New checkpoint for ardu-pilot
        auto final_pose = geometry_msgs::msg::Pose();

        if (b_checkpoint_reached)
        {
            // TAKE-OFF
            if (current_command.task == TASK_ENUM::TAKEOFF)
            {
                // Handle takeoff
                while (!takeoff_client_->wait_for_service(1s))
                {
                    rclcpp::spin_some(this->get_node_base_interface());
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the take-off service. Exiting.");
                        return;
                    }
                }
                mavros_msgs::srv::CommandTOL::Request request;
                request.altitude = current_command.z;
                takeoff_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandTOL::Request>(request));
                while (rclcpp::ok())
                {
                    rclcpp::spin_some(this->get_node_base_interface());

                    auto current_z = current_position.position.z;

                    float offset;
                    switch (current_command.precision)
                    {
                    case PRECISION_ENUM::HARD:
                        offset = HARD_PRECISION;
                        break;
                    case PRECISION_ENUM::SOFT:
                        offset = SOFT_PRECISION;
                        break;
                    }

                    bool b_tookoff = (current_z >= (request.altitude - offset)) && (current_z <= (request.altitude + offset));

                    if (b_tookoff)
                    {
                        RCLCPP_INFO(this->get_logger(), "Drone took off...");
                        break;
                    }

                    RCLCPP_INFO(this->get_logger(), "Drone taking off...");
                    std::this_thread::sleep_for(250ms);
                }
            }
            // YAW
            else if (current_command.task == TASK_ENUM::YAW)
            {
            }
            // LAND AND TAKEOFF
            else if (current_command.task == TASK_ENUM::LANDTAKEOFF)
            {
                // LAND
                while (!this->takeoff_client_->wait_for_service(1s))
                {
                    rclcpp::spin_some(get_node_base_interface());
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(get_logger(), "Interupted while wainting for takeoff_client.");
                        return;
                    }
                }
                mavros_msgs::srv::CommandTOL::Request land_request;
                takeoff_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandTOL::Request>(land_request));

                while (rclcpp::ok())
                {
                    rclcpp::spin_some(this->get_node_base_interface());
                    if (!current_state_.armed)
                    {
                        RCLCPP_INFO(get_logger(), "Drone landed");
                        break;
                    }
                    RCLCPP_INFO(get_logger(), "Drone landing");
                    std::this_thread::sleep_for(250ms);
                }

                // ARM
                while (!arming_client_->wait_for_service(1s))
                {
                    rclcpp::spin_some(this->get_node_base_interface());
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the arming service. Exiting.");
                        return;
                    }
                }
                mavros_msgs::srv::CommandBool::Request arming_request;
                arming_request.value = true;
                arming_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandBool::Request>(arming_request));

                while (rclcpp::ok())
                {
                    rclcpp::spin_some(get_node_base_interface());
                    if (current_state_.armed)
                    {
                        RCLCPP_INFO(get_logger(), "Drone armed");
                        break;
                    }
                    RCLCPP_INFO(get_logger(), "Drone arming");
                    std::this_thread::sleep_for(250ms);
                }

                // TAKE-OFF
                while (!takeoff_client_->wait_for_service(1s))
                {
                    rclcpp::spin_some(this->get_node_base_interface());
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the take-off service. Exiting.");
                        return;
                    }
                }
                mavros_msgs::srv::CommandTOL::Request takeoff_request;
                takeoff_request.altitude = TAKEOFF_ALTITUDE;
                takeoff_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandTOL::Request>(takeoff_request));

                // Eval precision
                float current_precision;
                switch (current_command.precision)
                {
                case PRECISION_ENUM::HARD:
                    current_precision = HARD_PRECISION;
                    break;
                case PRECISION_ENUM::SOFT:
                    current_precision = SOFT_PRECISION;
                    break;
                default:
                    current_precision = SOFT_PRECISION;
                    break;
                }

                while (rclcpp::ok())
                {
                    rclcpp::spin_some(get_node_base_interface());
                    bool b_z_height = (current_position.position.z >= current_command.z - current_precision) &&
                                      (current_position.position.z <= current_command.z + current_precision);

                    if (b_z_height)
                    {
                        RCLCPP_INFO(get_logger(), "Drone took off");
                        break;
                    }

                    RCLCPP_INFO(get_logger(), "Drone taking off");
                    std::this_thread::sleep_for(250ms);
                }
            }
            // LAND
            else if (current_command.task == TASK_ENUM::LAND)
            {
                // LAND
                while (!this->takeoff_client_->wait_for_service(1s))
                {
                    rclcpp::spin_some(get_node_base_interface());
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(get_logger(), "Interupted while wainting for takeoff_client.");
                        return;
                    }
                }
                mavros_msgs::srv::CommandTOL::Request land_request;
                takeoff_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandTOL::Request>(land_request));

                while (rclcpp::ok())
                {
                    rclcpp::spin_some(this->get_node_base_interface());
                    if (!current_state_.armed)
                    {
                        RCLCPP_INFO(get_logger(), "Drone landed");
                        break;
                    }
                    RCLCPP_INFO(get_logger(), "Drone landing");
                    std::this_thread::sleep_for(250ms);
                }
            }
            // DO NOTHING
            else if (current_command.task == TASK_ENUM::NONE)
            {
                // NOOP
            }

            // Pop ended command
            current_command = this->converted_commands.front();
            this->converted_commands.pop();
            RCLCPP_INFO(get_logger(), "Command ended. Command poped form queue");

            // Generate new floodfill points
            while (!this->floodfill_cleint_->wait_for_service(1s))
            {
                rclcpp::spin_some(this->get_node_base_interface());
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interupted while waiting for floodFill_service");
                    return;
                }
            }
            lrs_interfaces::srv::FloodFill::Request new_ff_request;

            lrs_interfaces::msg::Point start_point;
            start_point.x = current_position.position.x;
            start_point.y = current_position.position.y;
            start_point.z = current_position.position.z;

            lrs_interfaces::msg::Point goal_point;
            goal_point.x = current_command.x;
            goal_point.y = current_command.y;
            goal_point.z = current_command.z;

            new_ff_request.start_point = start_point;
            new_ff_request.goal_point = goal_point;

            RCLCPP_INFO(
                this->get_logger(),
                "Retreiveing point for path start=[%d,%d,%d] : goal=[%d,%d,%d]",
                start_point.x, start_point.y, start_point.z,
                goal_point.x, goal_point.y, goal_point.z);

            auto new_ff_future = floodfill_cleint_->async_send_request(std::make_shared<lrs_interfaces::srv::FloodFill::Request>(new_ff_request));
            retrieveNewFloodFillPoints(new_ff_future);
        }
        else
        {
            geometry_msgs::msg::Pose ff_goal_pose;
            ff_goal_pose.position.x = this->floodfill_points.front().x;
            ff_goal_pose.position.y = this->floodfill_points.front().y;
            ff_goal_pose.position.z = this->floodfill_points.front().z;

            bool b_ff_reached = isLocationInsideRegion(current_position, ff_goal_pose, HARD_PRECISION);

            if (b_ff_reached)
            {
                this->floodfill_points.pop();
                RCLCPP_INFO(this->get_logger(), "Flood fill point poped");
            }

            final_pose.position.x = this->floodfill_points.front().x;
            final_pose.position.y = this->floodfill_points.front().y;
            final_pose.position.z = this->floodfill_points.front().z;
        }

        // TODO: calculate offsets
        auto message = geometry_msgs::msg::PoseStamped();
        auto header = std_msgs::msg::Header();
        header.frame_id = "position_control";
        header.stamp = this->get_clock()->now();

        message.header = header;
        message.pose = final_pose;

        local_pos_pub_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Requested position published: x=%.3f y=%.3f z=%.3f", final_pose.position.x, final_pose.position.y, final_pose.position.z);
    }

    void handleLocalPosition(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped current_local_pos_ = *msg;
        this->current_position = current_local_pos_.pose;

        this->b_initial_position_aquired = true;

        // TODO: prepocty medzi suradnicovymi systemami

        // RCLCPP_INFO(this->get_logger(), "Current Local Position: %f, %f, %f", current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, current_local_pos_.pose.position.z);
    }

    bool isLocationInsideRegion(const geometry_msgs::msg::Pose &current, const geometry_msgs::msg::Pose &goal, float offset)
    {
        float distance = std::sqrt(
            std::pow(current.position.x - goal.position.x, 2) +
            std::pow(current.position.y - goal.position.y, 2) +
            std::pow(current.position.z - goal.position.z, 2));

        // Check if the distance is less than or equal to the offset
        return distance <= offset;
    }

    void retrieveNewFloodFillPoints(rclcpp::Client<lrs_interfaces::srv::FloodFill>::SharedFuture future)
    {
        while (rclcpp::ok())
        {
            rclcpp::spin_some(this->get_node_base_interface());

            auto status = future.wait_for(100ms);
            if (status == std::future_status::ready)
            {
                auto response = future.get();

                if (response != nullptr)
                {
                    // Clear old checkpoints if necessary
                    while (!this->floodfill_points.empty())
                        this->floodfill_points.pop();

                    // Set new checkpoints
                    std::string _log_message = "--- Flood fill points ---\nX\t\tY\t\tZ\n";
                    for (const auto &point : response->points.points)
                    {
                        this->floodfill_points.push(point);
                        _log_message += std::to_string(floodfill_points.front().x) + "\t\t";
                        _log_message += std::to_string(floodfill_points.front().y) + "\t\t";
                        _log_message += std::to_string(floodfill_points.front().z) + "\t\t";
                        _log_message += "\n";
                    }
                    RCLCPP_INFO(this->get_logger(), _log_message.c_str());

                    // Break future await loop
                    break;
                }
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for new floodfill points");
        }
        RCLCPP_INFO(this->get_logger(), "New flood fill points achieved");
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
    mavros_msgs::msg::State current_state_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_position_control;

    // Custom clients
    rclcpp::Client<lrs_interfaces::srv::FloodFill>::SharedPtr floodfill_cleint_;
    rclcpp::Client<lrs_interfaces::srv::MissionCommand>::SharedPtr mission_client_;

    // Variables
    std::vector<lrs_interfaces::msg::Command> commands;
    geometry_msgs::msg::Pose current_position;
    ConvertedCommand current_command;
    std::queue<ConvertedCommand> converted_commands;
    std::queue<lrs_interfaces::msg::Point> floodfill_points;

    bool b_initial_position_aquired = false;

    // Precisions
    const float SOFT_PRECISION = 0.05f;  // 5 cm
    const float HARD_PRECISION = 0.2f;   // 20 cm
    const float TAKEOFF_ALTITUDE = 0.3f; // 25 cm
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneControll>());
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

#include "lrs_interfaces/msg/command.hpp"
#include "lrs_interfaces/msg/point.hpp"
#include "lrs_interfaces/msg/point_list.hpp"

#include "lrs_interfaces/srv/mission_command.hpp"
#include "lrs_interfaces/srv/flood_fill.hpp"

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

        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        floodfill_cleint_ = this->create_client<lrs_interfaces::srv::FloodFill>("floodfill_service");
        mission_client_ = this->create_client<lrs_interfaces::srv::MissionCommand>("mission_loader_service");

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
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the set_mode service. Exiting.");
                std::cerr << "Interrupted while waiting for the set_mode service. Exiting." << std::endl;
                return;
            }
        }
        auto result = set_mode_client_->async_send_request(
            std::make_shared<mavros_msgs::srv::SetMode::Request>(guided_set_mode_req) /*,
             std::bind(&DroneControll::handleModeChange, this, std::placeholders::_1)*/
        );

        // TODO: Test if drone state really changed to GUIDED

        // Arm and Take-off
        while (!arming_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the arming service. Exiting.");
                std::cout << "Interrupted while waiting for the arming service. Exiting." << std::endl;
                return;
            }
        }
        mavros_msgs::srv::CommandBool::Request arming_request;
        arming_request.value = true;
        auto aiming_result = arming_client_->async_send_request(
            std::make_shared<mavros_msgs::srv::CommandBool::Request>(arming_request),
            std::bind(&DroneControll::handleDroneArm, this, std::placeholders::_1));

        // Load mission plan
        while (!mission_client_->wait_for_service(1s))
        {
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

        // TODO: Implement position controller and mission commands here
        timer_position_control = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&DroneControll::handlePositionControll, this));

        // Check current drone position

        // Check if drone is in finish

        // Calculate offsets

        // Set new desired position
    }

private:
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Current State: %s", current_state_.mode.c_str());
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

    void handleModeChange(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
    {
        RCLCPP_INFO(get_logger(), "handleModeChange started");
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Sent mode: %s", response->mode_sent);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
            std::cerr << e.what() << std::endl;
            return;
        }
        RCLCPP_INFO(get_logger(), "handeModeChange ended");
    }

    void handleDroneArm(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
    {
        RCLCPP_INFO(get_logger(), "handleDroneArm started");
        try
        {
            auto response = future.get();

            RCLCPP_INFO(this->get_logger(), "Armed: %s", std::to_string(response->success).c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
            std::cerr << e.what() << std::endl;
            return;
        }
        RCLCPP_INFO(get_logger(), "handleDroneArm ended");
    }

    void handleTakeOff(rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future)
    {
        RCLCPP_INFO(get_logger(), "handleTakeOff started");
        try
        {
            auto response = future.get();

            RCLCPP_INFO(this->get_logger(), "Take-off succesfull: %s", std::to_string(response->success).c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
            std::cerr << e.what() << std::endl;
            return;
        }
        RCLCPP_INFO(get_logger(), "handleTakeOff ended");
    }

    void handlePositionControll()
    {
        auto header = std_msgs::msg::Header();
        header.frame_id = "position_control";
        header.stamp = this->get_clock()->now();

        auto message = geometry_msgs::msg::PoseStamped();
        message.header = header;
        message.pose.position.x = 2.5;
        message.pose.position.y = 2.5;
        message.pose.position.z = 1.5;

        local_pos_pub_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Requested position published: x=%.3f y=%.3f z=%.3f", 2.5, 2.5, 1.5);
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    mavros_msgs::msg::State current_state_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_position_control;

    // Custom clients
    rclcpp::Client<lrs_interfaces::srv::FloodFill>::SharedPtr floodfill_cleint_;
    rclcpp::Client<lrs_interfaces::srv::MissionCommand>::SharedPtr mission_client_;

    // Variables
    std::vector<lrs_interfaces::msg::Command> commands;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneControll>());
    rclcpp::shutdown();
    return 0;
}
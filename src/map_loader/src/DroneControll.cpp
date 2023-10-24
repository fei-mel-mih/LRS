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
        thread_pool_.push_back(std::thread(std::bind(&DroneControll::handleModeChange, this, "GUIDED")));

        // TODO: Test if drone state really changed to GUIDED
        

        // Arm
        thread_pool_.push_back(std::thread(std::bind(&DroneControll::handleDroneArm, this)));
        RCLCPP_INFO(this->get_logger(), "Drone armed");

        // Take-off
        thread_pool_.push_back(std::thread(std::bind(&DroneControll::handleTakeOff, this, 2.f, 0.f, 90.f)));
        RCLCPP_INFO(this->get_logger(), "Take-off completed");

        // Load mission plan
        thread_pool_.push_back(std::thread(std::bind(&DroneControll::handleMissionPlan, this)));
        RCLCPP_INFO(this->get_logger(), "Mission plan loaded");

        // TODO: Implement position controller and mission commands here
        timer_position_control = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&DroneControll::handlePositionControll, this));
        RCLCPP_INFO(this->get_logger(), "Position publisher created");

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

    void handleMissionPlan()
    {
        while (!mission_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the mission_client service. Exiting...");
                std::cerr << "Interrupted while waiting for the mission_client service. Exiting..." << std::endl;
                return;
            }
        }

        auto mission_future = mission_client_->async_send_request(std::make_shared<lrs_interfaces::srv::MissionCommand::Request>());
        try
        {
            auto mission_response = mission_future.get();
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
            std::cout << "Mission commands loaded:" << std::endl;
            std::cout << command_message << std::endl;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
            std::cerr << e.what() << std::endl;
            return;
        }
        return;
    }

    void handleModeChange(std::string mode)
    {
        mavros_msgs::srv::SetMode::Request guided_set_mode_req;
        guided_set_mode_req.custom_mode = mode;
        while (!set_mode_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the set_mode service. Exiting.");
                std::cerr << "Interrupted while waiting for the set_mode service. Exiting." << std::endl;
                return;
            }
        }
        auto result = set_mode_client_->async_send_request(std::make_shared<mavros_msgs::srv::SetMode::Request>(guided_set_mode_req));

        try
        {
            auto response = result.get();
            RCLCPP_INFO(this->get_logger(), "Sent mode: %s", response->mode_sent);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
            std::cerr << e.what() << std::endl;
            return;
        }
    }

    void handleDroneArm()
    {
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
        auto aiming_result = arming_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandBool::Request>(arming_request));

        try
        {
            auto response = aiming_result.get();
            RCLCPP_INFO(this->get_logger(), "Armed: %s", std::to_string(response->success).c_str());
            RCLCPP_INFO(this->get_logger(), "Drone armed");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
            std::cerr << e.what() << std::endl;
            return;
        }
    }

    void handleTakeOff(float altitude, float min_pitch, float yaw)
    {
        while (!takeoff_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the take_off service. Exiting.");
                return;
            }
        }
        mavros_msgs::srv::CommandTOL::Request takeoff_request;
        takeoff_request.altitude = altitude;
        takeoff_request.min_pitch = min_pitch;
        takeoff_request.yaw = yaw;
        auto takeoff_result = takeoff_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandTOL::Request>(takeoff_request));

        try
        {
            auto response = takeoff_result.get();
            RCLCPP_INFO(this->get_logger(), "Take-off succesfull: %s", std::to_string(response->success).c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
            std::cerr << e.what() << std::endl;
            return;
        }
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

    // Threads
    std::thread mission_thread;
    std::vector<std::thread> thread_pool_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneControll>());
    rclcpp::shutdown();
    return 0;
}
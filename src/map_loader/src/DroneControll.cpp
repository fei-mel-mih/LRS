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

        // TODO: Test if drone state really changed to GUIDED
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
        takeoff_request.altitude = 2.5;
        takeoff_request.min_pitch = 1.0;
        takeoff_request.yaw = 90.0;
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

        // TODO: Implement position controller and mission commands here

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

    void handleLocalPosition(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped current_local_pos_ = *msg;

        // To obtain the position of the drone use this data fields withing the message, please note, that this is the local position of the drone in the NED frame so it is different to the map frame
        // current_local_pos_.pose.position.x
        // current_local_pos_.pose.position.y
        // current_local_pos_.pose.position.z
        // you can do the same for orientation, but you will not need it for this seminar 

        RCLCPP_INFO(this->get_logger(), "Current Local Position: %f, %f, %f", current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, current_local_pos_.pose.position.z);
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
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
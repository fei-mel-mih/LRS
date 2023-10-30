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

#define TO_CM 100

#define DRONE_START_X 14.035203f
#define DRONE_START_Y 1.514892f
#define DRONE_START_Z 0.003074f
#define DRONE_START_YAW M_PI_2
#define MAP_MAX_WIDTH 18.2f
#define MAP_MAX_HEIGHT 13.5f

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

		while (rclcpp::ok() && !this->commands.empty())
		{
			rclcpp::spin_some(get_node_base_interface());

			RCLCPP_INFO(get_logger(), "Handling position control");
			handlePositionControll();
			RCLCPP_INFO(get_logger(), "Position controll handled");

			std::this_thread::sleep_for(250ms);
		}
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
		RCLCPP_INFO(get_logger(), "Current drone position x=%f y=%f z=%f",
					current_position.position.x, current_position.position.y, current_position.position.z);

		// Check if current goal is reached
		bool b_checkpoint_reached;

		geometry_msgs::msg::Pose goal_pose;
		goal_pose.position.x = current_command.x;
		goal_pose.position.y = current_command.y;
		goal_pose.position.z = current_command.z;

		RCLCPP_INFO(get_logger(), "Comparing command positions: command[%f,%f,%f] == drone[%f,%f,%f]",
					goal_pose.position.x, goal_pose.position.y, goal_pose.position.z,
					current_position.position.x, current_position.position.y, current_position.position.z);

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
				float yaw = current_command.yaw_value * (M_PI / 180.0);
				auto req_pose = yawToQuaternion(yaw);

				auto message = geometry_msgs::msg::PoseStamped();
				auto header = std_msgs::msg::Header();
				header.frame_id = "position_control";
				header.stamp = this->get_clock()->now();

				message.header = header;
				message.pose = req_pose;

				local_pos_pub_->publish(message);
				RCLCPP_INFO(this->get_logger(), "Requested orientation published: w=%f x=%f y=%f z=%f",
							req_pose.orientation.w, req_pose.orientation.x, req_pose.orientation.y, req_pose.orientation.z);

				while (rclcpp::ok())
				{
					rclcpp::spin_some(get_node_base_interface());

					float yaw_offset = std::abs(yaw - quaternionToYaw(current_position));
					bool b_yaw_reached = false;

					if (current_command.precision == PRECISION_ENUM::HARD)
					{
						b_yaw_reached = yaw_offset <= (15.0 * (M_PI / 180.0));
					}
					else
					{
						b_yaw_reached = yaw_offset <= (5.0 * (M_PI / 180.0));
					}

					if (b_yaw_reached)
						break;

					RCLCPP_INFO(get_logger(), "Drone rotating");

					std::this_thread::sleep_for(250ms);
				}
			}
			// LAND AND TAKEOFF
			else if (current_command.task == TASK_ENUM::LANDTAKEOFF)
			{
				// LAND
				while (!this->land_client_->wait_for_service(1s))
				{
					rclcpp::spin_some(get_node_base_interface());
					if (!rclcpp::ok())
					{
						RCLCPP_ERROR(get_logger(), "Interupted while wainting for takeoff_client.");
						return;
					}
				}
				mavros_msgs::srv::CommandTOL::Request land_request;
				land_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandTOL::Request>(land_request));

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
				while (!this->land_client_->wait_for_service(1s))
				{
					rclcpp::spin_some(get_node_base_interface());
					if (!rclcpp::ok())
					{
						RCLCPP_ERROR(get_logger(), "Interupted while wainting for takeoff_client.");
						return;
					}
				}
				mavros_msgs::srv::CommandTOL::Request land_request;
				land_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandTOL::Request>(land_request));

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
			start_point.x = current_position.position.x * TO_CM;
			start_point.y = current_position.position.y * TO_CM;
			start_point.z = current_position.position.z * TO_CM;

			lrs_interfaces::msg::Point goal_point;
			goal_point.x = current_command.x * TO_CM;
			goal_point.y = current_command.y * TO_CM;
			goal_point.z = current_command.z * TO_CM;

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
			ff_goal_pose.position.x = this->floodfill_points.front().x / 100.0;
			ff_goal_pose.position.y = this->floodfill_points.front().y / 100.0;
			ff_goal_pose.position.z = this->floodfill_points.front().z / 100.0;

			RCLCPP_INFO(get_logger(), "Comparing positions: goal[%f,%f,%f] == drone[%f,%f,%f]",
						ff_goal_pose.position.x, ff_goal_pose.position.y, ff_goal_pose.position.z,
						current_position.position.x, current_position.position.y, current_position.position.z);

			bool b_ff_reached = isLocationInsideRegion(current_position, ff_goal_pose, HARD_PRECISION);

			if (b_ff_reached)
			{
				this->floodfill_points.pop();
				RCLCPP_INFO(this->get_logger(), "Flood fill point poped");
			}
		}

		if (!this->floodfill_points.empty())
		{
			// Calculate requested position (global coordination system)
			// final_pose.position.x = (this->floodfill_points.front().z / 100.0) - DRONE_START_X;
			// final_pose.position.y = (this->floodfill_points.front().y / 100.0) - DRONE_START_Y;
			// final_pose.position.z = (this->floodfill_points.front().x / 100.0) - DRONE_START_Z;

			// Calculate the relative position of the goal in the global coordinate system
			float relative_global_x = (floodfill_points.front().x / 100.0) - DRONE_START_X;
			float relative_global_y = (floodfill_points.front().y / 100.0) - DRONE_START_Y;
			float relative_global_z = (floodfill_points.front().z / 100.0) - DRONE_START_Z;

			// Rotate the relative position to the local coordinate system
			float relative_local_x = cos(DRONE_START_YAW) * relative_global_x + sin(DRONE_START_YAW) * relative_global_y;
			float relative_local_y = -sin(DRONE_START_YAW) * relative_global_x + cos(DRONE_START_YAW) * relative_global_y;
			float relative_local_z = relative_global_z;

			// Calculate the distances to move in the local coordinate system
			final_pose.position.x = relative_local_x;
			final_pose.position.y = relative_local_y;
			final_pose.position.z = relative_local_z;

			RCLCPP_INFO(get_logger(), "Flood fill points: x=%f y=%f z=%f",
						(this->floodfill_points.front().x / 100.0), (this->floodfill_points.front().y / 100.0), (this->floodfill_points.front().z / 100.0));

			auto message = geometry_msgs::msg::PoseStamped();
			auto header = std_msgs::msg::Header();
			header.frame_id = "position_control";
			header.stamp = this->get_clock()->now();

			message.header = header;
			message.pose = final_pose;

			local_pos_pub_->publish(message);
			RCLCPP_INFO(this->get_logger(), "Requested position published: x=%.3f y=%.3f z=%.3f", final_pose.position.x, final_pose.position.y, final_pose.position.z);
		}
	}

	void handleLocalPosition(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		geometry_msgs::msg::PoseStamped current_local_pos_ = *msg;

		// this->current_position.position.x = DRONE_START_X + current_local_pos_.pose.position.y;
		// this->current_position.position.y = DRONE_START_Y + current_local_pos_.pose.position.x;
		// this->current_position.position.z = DRONE_START_Z + current_local_pos_.pose.position.z;

		this->current_position.position.x = DRONE_START_X + cos(DRONE_START_YAW) * current_local_pos_.pose.position.x - sin(DRONE_START_YAW) * current_local_pos_.pose.position.y;
		this->current_position.position.y = DRONE_START_Y + sin(DRONE_START_YAW) * current_local_pos_.pose.position.x + cos(DRONE_START_YAW) * current_local_pos_.pose.position.y;
		this->current_position.position.z = DRONE_START_Z + current_local_pos_.pose.position.z;

		this->current_position.orientation = current_local_pos_.pose.orientation;

		if (!this->b_initial_position_aquired)
			this->b_initial_position_aquired = true;

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
					std::string _log_message = "\n--- Flood fill points ---\nX\t\tY\t\tZ\n";
					for (const auto &point : response->points.points)
					{
						lrs_interfaces::msg::Point converted_point = point;
						converted_point.x = point.z;
						converted_point.z = point.x;

						this->floodfill_points.push(converted_point);
						_log_message += std::to_string(converted_point.z) + "\t\t";
						_log_message += std::to_string(converted_point.y) + "\t\t";
						_log_message += std::to_string(converted_point.x) + "\t\t";
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

	geometry_msgs::msg::Pose yawToQuaternion(float yaw)
	{
		geometry_msgs::msg::Pose pose;

		float halfYaw = yaw * 0.5;
		float cosYaw = cos(halfYaw);
		float sinYaw = sin(halfYaw);

		float qx = 0.0;
		float qy = 0.0;
		float qz = sinYaw;
		float qw = cosYaw;

		pose.orientation.x = qx;
		pose.orientation.y = qy;
		pose.orientation.z = qz;
		pose.orientation.w = qw;

		return pose;
	}

	float quaternionToYaw(const geometry_msgs::msg::Pose &pose)
	{
		auto orientation = pose.orientation;

		// Convert quaternion to yaw angle
		float sinYaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
		float cosYaw = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);

		float yaw = atan2(sinYaw, cosYaw) * 2.0;

		return yaw;
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
	const float SOFT_PRECISION = 0.05f;	 // 5 cm
	const float HARD_PRECISION = 0.2f;	 // 20 cm
	const float TAKEOFF_ALTITUDE = 0.3f; // 25 cm
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DroneControll>());
	rclcpp::shutdown();
	return 0;
}
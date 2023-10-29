#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include "lrs_data_structures.hpp"
#include "lrs_interfaces/srv/mission_command.hpp"
#include "lrs_interfaces/msg/command.hpp"

class MissionLoader : public rclcpp::Node
{
public:
    MissionLoader() : Node("mission_loader_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MissionLoader");

        this->service_ = this->create_service<lrs_interfaces::srv::MissionCommand>(
            "mission_loader_service",
            std::bind(&MissionLoader::handleMissionLoaderService, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "MissionLoader initialized");
    }

private:
    std::string mission_file_path = "/home/lrs-ubuntu/Desktop/missions/mission.csv";
    std::vector<std::vector<std::string>> mission_data;
    rclcpp::Service<lrs_interfaces::srv::MissionCommand>::SharedPtr service_;

    void handleMissionLoaderService(const lrs_interfaces::srv::MissionCommand::Request::SharedPtr request,
                                    const lrs_interfaces::srv::MissionCommand::Response::SharedPtr response)
    {
        parseConfiguration();
        printMissionData();
        auto commands = parseToServiceResponse();

        response->commands = commands;
        return;
    }

    std::vector<lrs_interfaces::msg::Command> parseToServiceResponse()
    {
        std::vector<lrs_interfaces::msg::Command> commands;
        RCLCPP_INFO(this->get_logger(), "Mission data length: %d", mission_data.size());
        RCLCPP_INFO(this->get_logger(), "Mission data row lenght: %d", mission_data.at(0).size());
        for (const auto& row : this->mission_data)
        {
            lrs_interfaces::msg::Command command;
            command.x = std::stof(row.at(0));
            command.y = std::stof(row.at(1));
            command.z = std::stof(row.at(2));
            command.precision = row.at(3);
            command.task = row.at(4);

            commands.push_back(command);
        }
        return commands;
    }

    void parseConfiguration()
    {
        std::ifstream config_file(this->mission_file_path);
        if (!config_file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not load configuration file %s", mission_file_path.c_str());
            return;
        }

        std::vector<std::vector<std::string>> data;

        std::string line;
        while (std::getline(config_file, line))
        {
            std::vector<std::string> row;
            std::istringstream line_stream(line);
            std::string cell;
            while (std::getline(line_stream, cell, ','))
            {
                row.push_back(cell);
            }
            data.push_back(row);
        }

        mission_data = data;
    }

    void printMissionData()
    {
        std::string log_message = "Mission data:\n";
        for (const auto &row : mission_data)
        {
            for (const auto &data : row)
            {
                log_message += data + "\t";
            }
            log_message += "\n";
        }
        RCLCPP_INFO(this->get_logger(), log_message);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionLoader>());
    rclcpp::shutdown();
    return 0;
}
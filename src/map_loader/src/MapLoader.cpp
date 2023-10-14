#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <vector>

#include "lrs_data_structures.hpp"

// namespace MelMih 
// {
//     struct Map {
//         int width;
//         int height;
//         int max_val;
//         std::vector<std::vector<int>> pixels;
//     };

//     Map readPGM(const std::string &filename) {
//         Map img;
//         std::ifstream file(filename, std::ios::in);

//         if (!file.is_open()) {
//             std::cerr << "Error opening file!" << std::endl;
//             exit(1);
//         }

//         std::string header;
//         file >> header;
//         if (header != "P2") {
//             std::cerr << "Unsupported PGM format!" << std::endl;
//             exit(1);
//         }

//         file >> img.width >> img.height >> img.max_val;

//         img.pixels.resize(img.height, std::vector<int>(img.width));

//         for (int i = 0; i < img.height; i++) {
//             for (int j = 0; j < img.width; j++) {
//                 file >> img.pixels[i][j];
//             }
//         }

//         file.close();
//         return img;
//     }   

//     class MapLoader: public rclcpp::Node
//     {
//         public:
//             MapLoader() : Node("map_loader")
//             {
//                 this->declare_parameter<std::string>("pgm_file", "");
//                 std::string filename;
//                 this->get_parameter("Maps_2d/map_025", filename);

//                 if (filename.empty()) {
//                     RCLCPP_ERROR(this->get_logger(), "PGM filename is not provided.");
//                     return;
//                 }

//                 Map image = readPGM(filename);
//                 RCLCPP_INFO(this->get_logger(), "Loaded PGM with dimensions: %dx%d", image.width, image.height);
//             }
//     };

//     int main(int argc, char **argv) {
//         rclcpp::init(argc, argv);
//         auto node = std::make_shared<MapLoader>();
//         rclcpp::spin(node);
//         rclcpp::shutdown();
//         return 0;
//     }
// }


class MapLoader : public rclcpp::Node 
{
    public:
        MapLoader(): Node("map_loader")
        {   
            RCLCPP_INFO(this->get_logger(), "Loading map!");
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapLoader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
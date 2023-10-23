#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream> 
#include <algorithm>

class MapReader 
{
    public:
        MapReader(const std::vector<std::string>& filenames) : depth(filenames.size()) 
        {
            for (const auto& filename : filenames) {
                loadFile(filename);
            }
        }

        void printData() const 
        {
            for (int k = 0; k < depth; ++k) {
                std::cout << "Layer " << k << ":\n";
                for (int i = 0; i < height; ++i) {
                    for (int j = 0; j < width; ++j) {
                        std::cout << data[k][i][j] << " ";
                    }
                    std::cout << std::endl;
                }
                std::cout << "\n";
            }
        }

        void printHeights() const 
        {
            for (int i = 0; i < 10; i++)
            {
                std::cout << heights[i] << std::endl;
            }
        }

        const std::vector<std::vector<std::vector<int>>>& getData() const 
        {
            return data;
        }

        const std::vector<int>& getHeights() const 
        {
            return heights;
        }

    private:
        int width, height, depth, maxGrayValue;
        std::vector<std::vector<std::vector<int>>> data;
        std::vector<int> heights;

        void loadFile(const std::string& filename) 
        {
            std::ifstream file(filename);
            if (!file) 
            {
                std::cerr << "Cannot open the file: " << filename << std::endl;
                return;
            }

            // Store heights
            size_t startPos = filename.find("map_") + 4;
            size_t endPos = filename.find(".pgm");

            // start after "map_" and end before ".pgm"
            int fileHeight = std::stoi(filename.substr(startPos, endPos - startPos));
            heights.push_back(fileHeight);

            // Read the header
            std::string lineMagicNumber;
            std::getline(file, lineMagicNumber); // Read magic number

            if (lineMagicNumber != "P2") 
            {
                std::cerr << "Unsupported PGM format!" << std::endl;
                return;
            }

            std::string line;

            // Skip comments
            while (std::getline(file, line) && line[0] == '#');

            // Read width, height
            std::stringstream ss(line);
            ss >> width >> height;
            std::cout << ss.str() << std::endl;

            // Read max grayscale value
            std::getline(file, line);
            ss.clear();
            ss.str(line);;
            ss >> maxGrayValue;
            std::cout << ss.str() << std::endl;

            std::vector<std::vector<int>> currentMap(height, std::vector<int>(width));

            if (lineMagicNumber == "P2") 
            {
                for (int i = 0; i < height; ++i) {
                    for (int j = 0; j < width; ++j) {
                        file >> currentMap[i][j];
                    }
                }
            }

            // Push the 2D vector to 3D vector
            data.push_back(currentMap);
        }
};

int main() 
{
    std::vector<std::string> filenames = {"maps/map_025.pgm", "maps/map_075.pgm", "maps/map_080.pgm", "maps/map_100.pgm", "maps/map_125.pgm", "maps/map_150.pgm", "maps/map_175.pgm", "maps/map_180.pgm","maps/map_200.pgm", "maps/map_225.pgm"};
    MapReader reader(filenames);
    // reader.printData();
    reader.printHeights();

    return 0;
}
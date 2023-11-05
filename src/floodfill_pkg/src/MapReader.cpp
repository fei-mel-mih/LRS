#include "MapReader.h"

#define ISTEST 0

MapReader::MapReader()
{
    // Parse filenames -> append absolute path

    for (auto &filename : filenames)
    {
        filename = mapAbsolutePath + filename;
    }

    for (const auto &filename : filenames)
    {
        loadFile(filename);
    }

    this->inflated_map = inflateMap(this->map);
}

void MapReader::printMap() const
{
    for (int k = 0; k < (int)map.size(); ++k)
    {
        std::cout << "Layer " << k << ":\n";
        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                std::cout << map[k][i][j] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "\n";
    }
}

void MapReader::printHeights() const
{
    for (int i = 0; i < 10; i++)
    {
        std::cout << heights[i] << std::endl;
    }
}

const std::vector<std::vector<std::vector<int>>> &MapReader::getMap() const
{
    return map;
}

const std::vector<std::vector<std::vector<int>>> &MapReader::getInflatedMap() const
{
    return inflated_map;
}

const std::vector<int> &MapReader::getHeights() const
{
    return heights;
}

void MapReader::loadFile(const std::string &filename)
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
    while (std::getline(file, line) && line[0] == '#')
        ;

    // Read width, height
    std::stringstream ss(line);
    ss >> width >> height;
    // std::cout << ss.str() << std::endl;

    // Read max grayscale value
    std::getline(file, line);
    ss.clear();
    ss.str(line);
    ;
    ss >> maxGrayValue;
    // std::cout << ss.str() << std::endl;

    std::vector<std::vector<int>> currentMap(height, std::vector<int>(width));

    if (lineMagicNumber == "P2")
    {
        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                file >> currentMap[i][j];

                if (currentMap[i][j] == 255)
                {
                    currentMap[i][j] = 0;
                }
                else if (currentMap[i][j] == 0)
                {
                    currentMap[i][j] = 1;
                }
            }
        }
    }

    // Push the 2D vector to 3D vector
    map.push_back(currentMap);
}

std::vector<std::vector<std::vector<int>>> MapReader::inflateMap(const std::vector<std::vector<std::vector<int>>> &original_map)
{
    std::vector<std::vector<std::vector<int>>> temp_map = original_map;

    for (int i = 0; i < (int)original_map.size(); ++i)
    {
        for (int j = 0; j < (int)original_map[i].size(); ++j)
        {
            for (int k = 0; k < (int)original_map[i][j].size(); ++k)
            {
                if (original_map[i][j][k] == 1)
                {
                    // Set the neighbors of temp_map[i][j][k] to 1 along the Y and Z axes
                    for (int y = -8; y <= 8; ++y)
                    {
                        for (int z = -8; z <= 8; ++z)
                        {
                            // Check for out of bounds
                            if (j + y >= 0 && j + y < (int)original_map[i].size() &&
                                k + z >= 0 && k + z < (int)original_map[i][j].size())
                            {
                                temp_map[i][j + y][k + z] = 1;
                            }
                        }
                    }
                }
            }
        }
    }
    return temp_map;
}

#if ISTEST
int main()
{

    MapReader reader;
    // reader.printHeights();
    // reader.getMap();

    std::vector<std::vector<std::vector<int>>> original_map = {
        {{1, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0}},
        {{0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0}},
        {{0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0}},
        {{0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0}},
        {{0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0}},
        {{0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0}},
        {{0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 0, 0}}};

    auto map = reader.inflateMap(original_map);

    // Print the inflated map
    for (const auto &plane : map)
    {
        for (const auto &row : plane)
        {
            for (int val : row)
            {
                std::cout << val << " ";
            }
            std::cout << "\n";
        }
        std::cout << "---\n";
    }

    return 0;
}
#endif

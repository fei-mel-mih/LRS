#ifndef MAP_READER_H
#define MAP_READER_H

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream> 
#include <algorithm>

class MapReader 
{
    public:
        MapReader();
        
        void printMap() const;
        void printHeights() const;

        const std::vector<std::vector<std::vector<int>>>& getMap() const;
        const std::vector<int>& getHeights() const;
        std::vector<std::vector<std::vector<int>>> inflateMap(const std::vector<std::vector<std::vector<int>>>& original_map);

    private:
        int width, height, maxGrayValue;
        std::vector<std::vector<std::vector<int>>> map;
        std::vector<int> heights;
        std::vector<std::string> filenames = {"map_025.pgm", "map_075.pgm", "map_080.pgm", "map_100.pgm", "map_125.pgm", "map_150.pgm", "map_175.pgm", "map_180.pgm", "map_200.pgm", "map_225.pgm"};
        std::string mapAbsolutePath = "/home/lrs-ubuntu/Documents/lrs-git/LRS/src/floodfill_pkg/src/maps/";

        // load file and create map
        void loadFile(const std::string& filename);
};

#endif // MAP_READER_H

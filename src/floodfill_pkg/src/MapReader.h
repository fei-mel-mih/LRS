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

    private:
        int width, height, depth, maxGrayValue;
        std::vector<std::vector<std::vector<int>>> map;
        std::vector<int> heights;
        std::vector<std::string> filenames = {"maps/map_025.pgm", "maps/map_075.pgm", "maps/map_080.pgm", "maps/map_100.pgm", "maps/map_125.pgm", "maps/map_150.pgm", "maps/map_175.pgm", "maps/map_180.pgm","maps/map_200.pgm", "maps/map_225.pgm"};

        // load file and create map
        void loadFile(const std::string& filename);
};

#endif // MAP_READER_H

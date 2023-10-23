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
        MapReader(const std::vector<std::string>& filenames);
        
        void printData() const;
        void printHeights() const;

        const std::vector<std::vector<std::vector<int>>>& getData() const;
        const std::vector<int>& getHeights() const;

    private:
        int width, height, depth, maxGrayValue;
        std::vector<std::vector<std::vector<int>>> data;
        std::vector<int> heights;

        void loadFile(const std::string& filename);
};

#endif // MAP_READER_H

#ifndef MAZE_PARSER
#define MAZE_PARSER

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <limits>

std::vector<std::vector<char>> parse_maze(std::string filename) {

    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
    }

    int width, height;
    file >> width >> height;
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    std::vector<std::vector<char>> area(height, std::vector<char>(width));
    std::string line;
    int row = 0;
    while (std::getline(file, line) && row < height) {
        for (int col = 0; col < width; ++col) {
            if(line[col] == '1') area[row][col] = 'X';
            else area[row][col] = '.';
        }
        row++;
    }

    file.close();

    // std::cout << "Area:" << std::endl;
    // for (const auto& row : area) {
    //     for (char cell : row) {
    //         std::cout << cell;
    //     }
    //     std::cout << std::endl;
    // }

    return area;
}


#endif
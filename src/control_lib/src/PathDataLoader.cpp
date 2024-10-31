#include "../include/control_lib/PathDataLoader.hpp"
#include <iostream>
#include <fstream>

using json = nlohmann::json;

PathDataLoader::PathDataLoader(const std::string& fileName) {
    // Open and parse the JSON file
    std::ifstream file(fileName);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << fileName << std::endl;
        return;
    }

    json jsonData;
    file >> jsonData;

    // Extract metadata
    units = jsonData["meta_data"]["units"];
    name = jsonData["meta_data"]["name"];

    // Extract x and y coordinates
    x_coords = jsonData["x_coords"].get<std::vector<double>>();
    y_coords = jsonData["y_coords"].get<std::vector<double>>();

    // Extract probing_waypoints_indices
    probing_indices = jsonData["probing_waypoints_indicies"].get<std::vector<int>>();

    // Print metadata upon loading
    std::cout << "Metadata loaded:" << std::endl;
    std::cout << "Units: " << units << std::endl;
    std::cout << "Name: " << name << std::endl;
}

std::vector<double> PathDataLoader::getXCoords() const {
    return x_coords;
}

std::vector<double> PathDataLoader::getYCoords() const {
    return y_coords;
}

std::vector<int> PathDataLoader::getProbingIndices() const {
    return probing_indices;
}

std::string PathDataLoader::getName() const {
    return name;
}

std::string PathDataLoader::getUnits() const {
    return units;
}

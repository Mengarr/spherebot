#include <iostream>
#include <vector>
#include <nlohmann/json.hpp>  // Include the nlohmann json library
#include <fstream>            // For file handling

using json = nlohmann::json;

class PathDataLoader {
private:
    std::vector<double> x_coords;
    std::vector<double> y_coords;
    std::vector<int> probing_indices;
    std::string units;
    std::string name;

public:
    // Constructor to load data from the JSON file
    PathDataLoader(const std::string& fileName) {
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

    // Method to return x coordinates
    std::vector<double> getXCoords() const {
        return x_coords;
    }

    // Method to return y coordinates
    std::vector<double> getYCoords() const {
        return y_coords;
    }

    // Method to return probing indices
    std::vector<int> getProbingIndices() const {
        return probing_indices;
    }

    // Optional: Method to return the metadata (name)
    std::string getName() const {
        return name;
    }

    // Optional: Method to return the metadata (units)
    std::string getUnits() const {
        return units;
    }
};

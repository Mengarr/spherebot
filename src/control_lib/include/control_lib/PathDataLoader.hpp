#ifndef PATHDATALOADER_HPP
#define PATHDATALOADER_HPP

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

class PathDataLoader {
private:
    std::vector<double> x_coords;
    std::vector<double> y_coords;
    std::vector<int> probing_indices;
    std::string units;
    std::string name;

public:
    // Constructor to load data from the JSON file
    explicit PathDataLoader(const std::string& fileName);

    // Method to return x coordinates
    std::vector<double> getXCoords() const;

    // Method to return y coordinates
    std::vector<double> getYCoords() const;

    // Method to return probing indices
    std::vector<int> getProbingIndices() const;

    // Optional: Method to return the metadata (name)
    std::string getName() const;

    // Optional: Method to return the metadata (units)
    std::string getUnits() const;
};

#endif // PATHDATALOADER_HPP

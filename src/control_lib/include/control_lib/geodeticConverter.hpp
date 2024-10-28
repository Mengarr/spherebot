// GeodeticConverter.hpp

#ifndef GEODETIC_CONVERTER_HPP
#define GEODETIC_CONVERTER_HPP

#include <tuple>
#include <cmath>

// Constants for WGS84 Ellipsoid
constexpr double WGS84_A = 6378137.0;               // Semi-major axis in meters
constexpr double WGS84_F = 1.0 / 298.257223563;     // Flattening
constexpr double WGS84_E_SQ = WGS84_F * (2 - WGS84_F); // Square of eccentricity

// Struct to hold ENU coordinates
struct ENU {
    double east;
    double north;
    double up;
};

class GeodeticConverter {
public:
    /**
     * @brief Constructor to set the reference origin.
     * 
     * @param refLat Reference latitude in decimal degrees.
     * @param refLon Reference longitude in decimal degrees.
     * @param refAlt Reference altitude in meters.
     */
    GeodeticConverter(double refLat, double refLon, double refAlt);

    // Default constructor
    GeodeticConverter();
    
    /**
     * @brief Converts geodetic coordinates to ENU coordinates relative to the reference origin.
     * 
     * @param lat Target latitude in decimal degrees.
     * @param lon Target longitude in decimal degrees.
     * @param alt Target altitude in meters.
     * @return ENU Structure containing east, north, and up coordinates in meters.
     */
    ENU geodeticToENU(double lat, double lon, double alt) const;

    // Getter methods for reference origin
    double getRefLat() const;
    double getRefLon() const;
    double getRefAlt() const;

    /**
     * @brief Sets a new reference origin and updates internal variables.
     * 
     * @param refLat New reference latitude in decimal degrees.
     * @param refLon New reference longitude in decimal degrees.
     * @param refAlt New reference altitude in meters.
     */
    void setReferenceOrigin(double refLat, double refLon, double refAlt);

private:
    // Reference origin in geodetic coordinates
    double refLat_; // degrees
    double refLon_; // degrees
    double refAlt_; // meters

    // Reference origin in ECEF coordinates
    double refX_;
    double refY_;
    double refZ_;

    // Reference origin in radians
    double refLatRad_;
    double refLonRad_;

    // Sine and cosine of reference latitude and longitude
    double sinRefLat_;
    double cosRefLat_;
    double sinRefLon_;
    double cosRefLon_;

    // Helper function to convert degrees to radians
    static double deg2rad(double degrees);

    // Helper function to convert geodetic to ECEF coordinates
    static std::tuple<double, double, double> geodeticToECEF(double lat, double lon, double alt);
};

#endif // GEODETIC_CONVERTER_HPP

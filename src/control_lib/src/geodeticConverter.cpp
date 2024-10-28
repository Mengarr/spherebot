// GeodeticConverter.cpp

#include "../include/control_lib/geodeticConverter.hpp"

// Implementation of GeodeticConverter methods

// Helper function to convert degrees to radians
double GeodeticConverter::deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

// Helper function to convert geodetic to ECEF coordinates
std::tuple<double, double, double> GeodeticConverter::geodeticToECEF(double lat, double lon, double alt) {
    double latRad = deg2rad(lat);
    double lonRad = deg2rad(lon);

    double sinLat = std::sin(latRad);
    double cosLat = std::cos(latRad);
    double sinLon = std::sin(lonRad);
    double cosLon = std::cos(lonRad);

    // Radius of curvature in the prime vertical
    double N = WGS84_A / std::sqrt(1 - WGS84_E_SQ * sinLat * sinLat);

    double X = (N + alt) * cosLat * cosLon;
    double Y = (N + alt) * cosLat * sinLon;
    double Z = (N * (1 - WGS84_E_SQ) + alt) * sinLat;

    return std::make_tuple(X, Y, Z);
}

// Constructor to set the reference origin
GeodeticConverter::GeodeticConverter(double refLat, double refLon, double refAlt)
    : refLat_(refLat), refLon_(refLon), refAlt_(refAlt) {
    // Convert reference origin to ECEF
    std::tie(refX_, refY_, refZ_) = geodeticToECEF(refLat_, refLon_, refAlt_);

    // Precompute sine and cosine of reference latitude and longitude
    refLatRad_ = deg2rad(refLat_);
    refLonRad_ = deg2rad(refLon_);
    sinRefLat_ = std::sin(refLatRad_);
    cosRefLat_ = std::cos(refLatRad_);
    sinRefLon_ = std::sin(refLonRad_);
    cosRefLon_ = std::cos(refLonRad_);
}

// Default constructor
GeodeticConverter::GeodeticConverter() {

}

// Method to convert geodetic coordinates to ENU
ENU GeodeticConverter::geodeticToENU(double lat, double lon, double alt) const {
    // Convert target point to ECEF
    double X, Y, Z;
    std::tie(X, Y, Z) = geodeticToECEF(lat, lon, alt);

    // Compute the difference vector between target and reference ECEF
    double dX = X - refX_;
    double dY = Y - refY_;
    double dZ = Z - refZ_;

    // Convert ECEF difference to ENU using precomputed sine and cosine
    double east  = -sinRefLon_ * dX + cosRefLon_ * dY;
    double north = -sinRefLat_ * cosRefLon_ * dX - sinRefLat_ * sinRefLon_ * dY + cosRefLat_ * dZ;
    double up    =  cosRefLat_ * cosRefLon_ * dX + cosRefLat_ * sinRefLon_ * dY + sinRefLat_ * dZ;

    return ENU{east, north, up};
}

// Getter methods for reference origin
double GeodeticConverter::getRefLat() const {
    return refLat_;
}

double GeodeticConverter::getRefLon() const {
    return refLon_;
}

double GeodeticConverter::getRefAlt() const {
    return refAlt_;
}

// Setter methods for reference origin (recomputes reference ECEF and trigonometric values)
void GeodeticConverter::setReferenceOrigin(double refLat, double refLon, double refAlt) {
    refLat_ = refLat;
    refLon_ = refLon;
    refAlt_ = refAlt;

    // Convert new reference origin to ECEF
    std::tie(refX_, refY_, refZ_) = geodeticToECEF(refLat_, refLon_, refAlt_);

    // Recompute trigonometric values
    refLatRad_ = deg2rad(refLat_);
    refLonRad_ = deg2rad(refLon_);
    sinRefLat_ = std::sin(refLatRad_);
    cosRefLat_ = std::cos(refLatRad_);
    sinRefLon_ = std::sin(refLonRad_);
    cosRefLon_ = std::cos(refLonRad_);
}

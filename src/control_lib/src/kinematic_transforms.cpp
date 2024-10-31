#include "../include/control_lib/kinematic_transforms.hpp"

std::pair<float, float> computeJointVariables(float alpha, float u) {
    // Initialize the result variables for Psi_L and Psi_R
    float Psi_L, Psi_R;

    // Calculate each term of the Jacobian matrix
    float term1 = 2.0f * M_PI;  // 2 * pi
    float term2 = BALL_SCREW_PITCH * GEAR_REDUCTION;

    // Using the Jacobian matrix to compute joint variables
    Psi_L = (1.0f / BALL_SCREW_PITCH) * (term1 * u + term2 * alpha);
    Psi_R = (1.0f / BALL_SCREW_PITCH) * (-term1 * u + term2 * alpha);

    // Return the joint variable velocities as a pair
    return std::make_pair(Psi_L, Psi_R);
}

std::pair<float, float> computeJointVariablesInverse(float Psi_L, float Psi_R) {
    // Initialize variables for u and alpha
    float u, alpha;

    // Constants for the Jacobian inverse
    float term1 = 2.0f * M_PI; // 2 * pi
    float term2 = BALL_SCREW_PITCH * GEAR_REDUCTION;
    float denom = 1.0f / (4.0f * M_PI * M_PI + term2 * term2);

    // Using the inverse Jacobian matrix to compute u and alpha
    u = denom * (term2 * (Psi_L - Psi_R));
    alpha = denom * (term1 * (Psi_L + Psi_R));

    // Return the state variables as a pair (u, alpha)
    return std::make_pair(u, alpha);
}

float calculate_u_ref(float alpha, float theta, float dtheta, float rc, const std::map<std::string, float>& params) {
    // Retrieve parameters from the map
    float g = params.at("g");
    float r = params.at("r");
    float R = params.at("R");
    float Is = params.at("Is");
    float mp = params.at("mp");
    float ms = params.at("ms");

    // Calculate r_prime
    float r_prime = r * std::cos(alpha - theta);

    // Calculate numerator and denominator
    float numerator = R * R * R * (ms + mp) * dtheta * dtheta 
                    + Is * R * dtheta * dtheta 
                    - mp * r_prime * R * R * dtheta * dtheta;
    float denominator = mp * g * rc;

    // Avoid div by 0 error at small foward velocities
    const float epsilon = 1e-8;
    denominator += epsilon;

    // Return the result
    return numerator / denominator;
}

float u_phi_eq(float u, float r) {
    return -std::atan(u / r);
}
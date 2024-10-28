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
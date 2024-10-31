#include <cmath> // For abs()
#include <utility> // For pair

#define BALL_SCREW_PITCH   70.0f // p
#define GEAR_REDUCTION     2.4070175f // G  
#define CPR                12.0f
#define MOTOR_GEAR_RATIO   98.7779f

#define U_LIM              50 // +-50 mm

// Functions
std::pair<float, float> computeJointVariables(float alpha, float u); // Computes Phi_L and Phi_R

std::pair<float, float> computeJointVariablesInverse(float Psi_L, float Psi_R); // Computes u and alpha in (m, rad)

float calculate_u_ref(float alpha, float theta, float dtheta, float rc, const std::map<std::string, float>& params); // Computes reference u based on rc

float u_phi_eq(float u, float r); // computes equilibrium phi

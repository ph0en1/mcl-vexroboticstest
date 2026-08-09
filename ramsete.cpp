#include <cmath>

// Tuning parameters (These are standard starting values for b and zeta)
const double b = 2.0; 
const double zeta = 0.7; 

// Structure to hold our velocity commands
struct ChassisSpeeds {
    double linearVelocity;  // inches per second
    double angularVelocity; // radians per second
};

// Ramsete Calculation Function
ChassisSpeeds calculateRamsete(
    double currentX, double currentY, double currentTheta,
    double targetX, double targetY, double targetTheta,
    double targetVel, double targetAngularVel) 
{
    // 1. Calculate global errors
    double dx = targetX - currentX;
    double dy = targetY - currentY;
    
    // Normalize heading error to be between -PI and PI
    double dTheta = targetTheta - currentTheta;
    while (dTheta > M_PI) dTheta -= 2 * M_PI;
    while (dTheta < -M_PI) dTheta += 2 * M_PI;

    // 2. Rotate global error into robot's local frame
    double eX = (dx * std::cos(currentTheta)) + (dy * std::sin(currentTheta));
    double eY = (dy * std::cos(currentTheta)) - (dx * std::sin(currentTheta));

    // 3. Calculate the k gain
    double k = 2.0 * zeta * std::sqrt(std::pow(targetAngularVel, 2) + b * std::pow(targetVel, 2));

    // 4. Handle the sin(x)/x limit as x approaches 0 to avoid dividing by zero
    double sinThetaOverTheta;
    if (std::abs(dTheta) < 1e-6) {
        sinThetaOverTheta = 1.0; // Limit as x -> 0
    } else {
        sinThetaOverTheta = std::sin(dTheta) / dTheta;
    }

    // 5. Calculate commanded velocities
    ChassisSpeeds output;
    output.linearVelocity = (targetVel * std::cos(dTheta)) + (k * eX);
    output.angularVelocity = targetAngularVel + (b * targetVel * sinThetaOverTheta * eY) + (k * dTheta);

    return output;
}

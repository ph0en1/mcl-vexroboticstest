#include <cmath>

// tuning
const double b = 2.0; 
const double zeta = 0.7; 


struct ChassisSpeeds {
    double linearVelocity;  // inches per second
    double angularVelocity; // radians per second
};

ChassisSpeeds calculateRamsete(
    double currentX, double currentY, double currentTheta,
    double targetX, double targetY, double targetTheta,
    double targetVel, double targetAngularVel) 
{

    double dx = targetX - currentX;
    double dy = targetY - currentY;
    
    // normalize heading
    double dTheta = targetTheta - currentTheta;
    while (dTheta > M_PI) dTheta -= 2 * M_PI;
    while (dTheta < -M_PI) dTheta += 2 * M_PI;


    double eX = (dx * std::cos(currentTheta)) + (dy * std::sin(currentTheta));
    double eY = (dy * std::cos(currentTheta)) - (dx * std::sin(currentTheta));


    double k = 2.0 * zeta * std::sqrt(std::pow(targetAngularVel, 2) + b * std::pow(targetVel, 2));


    double sinThetaOverTheta;
    if (std::abs(dTheta) < 1e-6) {
        sinThetaOverTheta = 1.0; // Limit as x -> 0
    } else {
        sinThetaOverTheta = std::sin(dTheta) / dTheta;
    }


    ChassisSpeeds output;
    output.linearVelocity = (targetVel * std::cos(dTheta)) + (k * eX);
    output.angularVelocity = targetAngularVel + (b * targetVel * sinThetaOverTheta * eY) + (k * dTheta);

    return output;
}

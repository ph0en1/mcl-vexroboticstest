#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>

// config
const double FIELD_MIN = -72.0; // al in inches
const double FIELD_MAX = 72.0;  
const int NUM_PARTICLES = 150;
const double SENSOR_NOISE = 2.5; // tune
const double ROBOT_WIDTH = 14.0; 
const double ROBOT_LENGTH = 14.0; 


std::default_random_engine gen;

struct Particle {
    double x, y, theta, weight;
};

std::vector<Particle> particles;


struct { double get() { return 24.0; } } distanceF, distanceB, distanceL, distanceR, inertial;
struct { double getX() { return 0.0; } double getY() { return 0.0; } } odometry;



// Calculate expected distance wall at start
double getExpectedDistance(double x, double y, double angle, double sensorOffset) {
    // Distance to vertical walls (X-axis)
    double dx = (std::cos(angle) > 0) ? (FIELD_MAX - x) : (FIELD_MIN - x);
    double dist_x = std::abs(dx / std::cos(angle));

    // Distance to horizontal walls (Y-axis)
    double dy = (std::sin(angle) > 0) ? (FIELD_MAX - y) : (FIELD_MIN - y);
    double dist_y = std::abs(dy / std::sin(angle));

    return std::min(dist_x, dist_y) - sensorOffset;
}

// Calculate likelihood of a reading based on Gaussian distribution
double calculateLikelihood(double expected, double actual) {
    return std::exp(-(std::pow(actual - expected, 2)) / (2 * std::pow(SENSOR_NOISE, 2)));
}

// --- Core MCL Functions ---

void initMCL(double startX, double startY, double startTheta) {
    particles.clear();
    for (int i = 0; i < NUM_PARTICLES; i++) {
        particles.push_back({startX, startY, startTheta, 1.0 / NUM_PARTICLES});
    }
}

void motionUpdate(double deltaX, double deltaY, double currentTheta) {
    // add noise
    std::normal_distribution<double> dist_pos(0.0, 1.0); // 1 inch standard deviation noise
    
    for (auto& p : particles) {
        // updating pos
        p.x += deltaX + dist_pos(gen);
        p.y += deltaY + dist_pos(gen);
        
        p.theta = currentTheta; 
        
        // particles must be in field
        p.x = std::clamp(p.x, FIELD_MIN, FIELD_MAX);
        p.y = std::clamp(p.y, FIELD_MIN, FIELD_MAX);
    }
}

void sensorUpdate() {
    double actualF = distanceF.get();
    double actualB = distanceB.get();
    double actualL = distanceL.get();
    double actualR = distanceR.get();
    double weightSum = 0.0;

    for (auto& p : particles) {

        double expF = getExpectedDistance(p.x, p.y, p.theta, ROBOT_LENGTH / 2.0);
        double expB = getExpectedDistance(p.x, p.y, p.theta + M_PI, ROBOT_LENGTH / 2.0);
        double expL = getExpectedDistance(p.x, p.y, p.theta + M_PI_2, ROBOT_WIDTH / 2.0);
        double expR = getExpectedDistance(p.x, p.y, p.theta - M_PI_2, ROBOT_WIDTH / 2.0);

        // Calculate probability of particle
        double weightF = calculateLikelihood(expF, actualF);
        double weightB = calculateLikelihood(expB, actualB);
        double weightL = calculateLikelihood(expL, actualL);
        double weightR = calculateLikelihood(expR, actualR);

        // total weight
        p.weight = weightF * weightB * weightL * weightR;
        weightSum += p.weight;
    }

    // Normalizing here
    if (weightSum > 0) {
        for (auto& p : particles) {
            p.weight /= weightSum;
        }
    } else {
        for (auto& p : particles) {
            p.weight = 1.0 / NUM_PARTICLES;
        }
    }
}

void resample() {
    std::vector<Particle> newParticles;
    std::uniform_real_distribution<double> dist(0.0, 1.0 / NUM_PARTICLES);
    
    // Stochastic Universal Sampling
    double r = dist(gen);
    double c = particles[0].weight;
    int i = 0;

    for (int m = 0; m < NUM_PARTICLES; m++) {
        double U = r + (double)m / NUM_PARTICLES;
        while (U > c && i < NUM_PARTICLES - 1) {
            i++;
            c += particles[i].weight;
        }
        newParticles.push_back(particles[i]);
    }
    particles = newParticles;
}

// returns pos estimate
Particle getEstimatedPose() {
    Particle est = {0, 0, inertial.get(), 0};
    for (const auto& p : particles) {
        est.x += p.x;
        est.y += p.y;
    }
    est.x /= NUM_PARTICLES;
    est.y /= NUM_PARTICLES;
    return est;
}

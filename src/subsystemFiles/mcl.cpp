#include "main.h"
#include <random>
#include <cmath>
#include "lemlib/api.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "subsystemHeaders/globals.hpp"

std::vector<Particle> particles;
const int NUM_PARTICLES = 100;
std::default_random_engine generator;

void initializeParticles() {
    std::uniform_real_distribution<double> xDist(-72, 72); // Assuming field dimensions
    std::uniform_real_distribution<double> yDist(-72, 72);
    std::uniform_real_distribution<double> thetaDist(0, 360);

    particles.clear();
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        particles.push_back({xDist(generator), yDist(generator), thetaDist(generator), 1.0 / NUM_PARTICLES});
    }
}

void motionUpdate(double dx, double dy) {
    double theta = chassis.odom_theta_get() * M_PI / 180.0; // Get odometry theta in radians
    for (auto &particle : particles) {
        particle.x += dx * cos(theta) - dy * sin(theta);
        particle.y += dx * sin(theta) + dy * cos(theta);
        particle.theta = chassis.odom_theta_get(); // Use odometry theta directly
    }
}

void sensorUpdate(double sensorReading, double expectedReading) {
    double totalWeight = 0.0;
    for (auto &particle : particles) {
        double error = sensorReading - expectedReading;
        particle.weight = exp(-error * error / 2.0);
        totalWeight += particle.weight;
    }

    for (auto &particle : particles) {
        particle.weight /= totalWeight;
    }
}

void resampleParticles() {
    std::vector<Particle> newParticles;
    std::uniform_real_distribution<double> dist(0, 1);

    for (int i = 0; i < NUM_PARTICLES; ++i) {
        double r = dist(generator);
        double cumulativeWeight = 0.0;
        for (const auto &particle : particles) {
            cumulativeWeight += particle.weight;
            if (r <= cumulativeWeight) {
                newParticles.push_back(particle);
                break;
            }
        }
    }

    particles = newParticles;
}

Particle estimatePosition() {
    double x = 0.0, y = 0.0, theta = 0.0;
    for (const auto &particle : particles) {
        x += particle.x * particle.weight;
        y += particle.y * particle.weight;
        theta += particle.theta * particle.weight;
    }
    return {x, y, theta, 1.0};
}

void mclTask() {
    initializeParticles();

    while (true) {
        // Example motion update (replace with actual odometry data)
        double dx = 1.0; // Change in x
        double dy = 0.5; // Change in y
        double dtheta = 5.0; // Change in theta
        motionUpdate(dx, dy);

        // Example sensor update (replace with actual sensor data)
        double sensorReading = rightAlignmentSensor.get_distance() * 0.0393701; // Convert to inches
        double expectedReading = 50.0; // Replace with expected sensor reading based on map
        sensorUpdate(sensorReading, expectedReading);

        resampleParticles();

        Particle estimate = estimatePosition();

        // Display estimated position on the controller
        master.set_text(0, 0, "X: " + std::to_string(estimate.x) + " Y: " + std::to_string(estimate.y));
        master.set_text(1, 0, "Theta: " + std::to_string(estimate.theta));

        pros::delay(100); // Update every 100ms
    }
}
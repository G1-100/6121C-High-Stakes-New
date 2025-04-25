#ifndef MCL_HPP
#define MCL_HPP

#include <vector>
#include "main.h"

struct Particle {
    double x, y, theta, weight;
};

void initializeParticles();
void motionUpdate(double dx, double dy, double dtheta);
void sensorUpdate(double sensorReading, double expectedReading);
void resampleParticles();
Particle estimatePosition();
void mclTask();

#endif // MCL_HPP
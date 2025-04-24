#include "subsystemHeaders/mcl.hpp"
#include "subsystemHeaders/globals.hpp"

// Global instance
MCL mcl;

MCL::MCL(int numParticles, double resampleThreshold, 
         double motionNoiseTranslation, double motionNoiseRotation, 
         double sensorNoise, double fieldWidth, double fieldLength)
    : mNumParticles(numParticles),
      mResampleThreshold(resampleThreshold),
      mMotionNoiseTranslation(motionNoiseTranslation),
      mMotionNoiseRotation(motionNoiseRotation),
      mSensorNoise(sensorNoise),
      mFieldWidth(fieldWidth),
      mFieldLength(fieldLength),
      mGen(mRd()),
      mTask(nullptr),
      mRunning(false),
      mLastPose{0, 0, 0} {
    
    // Initialize particles
    resetParticles();
}

MCL::~MCL() {
    stop();
}

void MCL::start() {
    if (!mRunning) {
        mRunning = true;
        mTask = new pros::Task(taskFunctionWrapper, this, "MCL Task");
    }
}

void MCL::stop() {
    if (mRunning && mTask != nullptr) {
        mRunning = false;
        // Give task time to exit
        pros::delay(50);
        delete mTask;
        mTask = nullptr;
    }
}

void MCL::resetParticles() {
    mParticleMutex.take();
    
    mParticles.clear();
    mParticles.reserve(mNumParticles);
    
    // Uniform distribution over field
    std::uniform_real_distribution<double> distX(0, mFieldWidth);
    std::uniform_real_distribution<double> distY(0, mFieldLength);
    std::uniform_real_distribution<double> distTheta(0, 360);
    
    for (int i = 0; i < mNumParticles; ++i) {
        mParticles.emplace_back(
            distX(mGen),     // x
            distY(mGen),     // y
            distTheta(mGen), // theta
            1.0 / mNumParticles  // uniform weight
        );
    }
    
    mParticleMutex.give();
}

void MCL::resetParticles(double x, double y, double theta, 
                        double spreadX, double spreadY, double spreadTheta) {
    mParticleMutex.take();
    
    mParticles.clear();
    mParticles.reserve(mNumParticles);
    
    // Normal distribution around the given pose
    std::normal_distribution<double> distX(x, spreadX);
    std::normal_distribution<double> distY(y, spreadY);
    std::normal_distribution<double> distTheta(theta, spreadTheta);
    
    for (int i = 0; i < mNumParticles; ++i) {
        double newTheta = distTheta(mGen);
        // Wrap angle to [0, 360)
        while (newTheta < 0) newTheta += 360;
        while (newTheta >= 360) newTheta -= 360;
        
        mParticles.emplace_back(
            distX(mGen),  // x
            distY(mGen),  // y
            newTheta,     // theta
            1.0 / mNumParticles  // uniform weight
        );
    }
    
    mParticleMutex.give();
}

lemlib::Pose MCL::getPose() {
    mParticleMutex.take();
    
    lemlib::Pose pose{0, 0, 0};
    double totalWeight = 0;
    
    // Calculate weighted average of particles
    for (const auto& p : mParticles) {
        pose.x += p.x * p.weight;
        pose.y += p.y * p.weight;
        totalWeight += p.weight;
        
        // Special handling for angular quantities to avoid issues at 0/360 boundary
        double sinSum = pose.theta * sin(pose.theta * M_PI / 180.0) + p.theta * sin(p.theta * M_PI / 180.0) * p.weight;
        double cosSum = pose.theta * cos(pose.theta * M_PI / 180.0) + p.theta * cos(p.theta * M_PI / 180.0) * p.weight;
        pose.theta = atan2(sinSum, cosSum) * 180.0 / M_PI;
    }
    
    if (totalWeight > 0) {
        pose.x /= totalWeight;
        pose.y /= totalWeight;
    } else if (!mParticles.empty()) {
        // If weights are invalid, use the first particle
        pose.x = mParticles[0].x;
        pose.y = mParticles[0].y;
        pose.theta = mParticles[0].theta;
    }
    
    // Ensure theta is in [0, 360)
    while (pose.theta < 0) pose.theta += 360;
    while (pose.theta >= 360) pose.theta -= 360;
    
    mParticleMutex.give();
    
    return pose;
}

MCL::Particle MCL::getBestParticle() {
    mParticleMutex.take();
    
    Particle best(0, 0, 0, 0);
    
    if (!mParticles.empty()) {
        best = *std::max_element(mParticles.begin(), mParticles.end(), 
            [](const Particle& a, const Particle& b) {
                return a.weight < b.weight;
            });
    }
    
    mParticleMutex.give();
    
    return best;
}

double MCL::getVariance() {
    mParticleMutex.take();
    
    lemlib::Pose mean = getPose();
    double varX = 0, varY = 0, varTheta = 0;
    double totalWeight = 0;
    
    for (const auto& p : mParticles) {
        double dx = p.x - mean.x;
        double dy = p.y - mean.y;
        
        // Handle angular differences properly
        double dTheta = p.theta - mean.theta;
        while (dTheta > 180) dTheta -= 360;
        while (dTheta <= -180) dTheta += 360;
        
        varX += dx * dx * p.weight;
        varY += dy * dy * p.weight;
        varTheta += dTheta * dTheta * p.weight;
        
        totalWeight += p.weight;
    }
    
    if (totalWeight > 0) {
        varX /= totalWeight;
        varY /= totalWeight;
        varTheta /= totalWeight;
    }
    
    mParticleMutex.give();
    
    // Return overall variance (can be weighted differently if needed)
    return varX + varY + varTheta * 0.01; // Scale theta variance to be comparable to position
}

std::vector<MCL::Particle> MCL::getParticles() {
    mParticleMutex.take();
    auto particles = mParticles; // Make a copy
    mParticleMutex.give();
    return particles;
}

void MCL::setParameters(double motionNoiseTranslation, double motionNoiseRotation, double sensorNoise) {
    mParticleMutex.take();
    mMotionNoiseTranslation = motionNoiseTranslation;
    mMotionNoiseRotation = motionNoiseRotation;
    mSensorNoise = sensorNoise;
    mParticleMutex.give();
}

void* MCL::taskFunctionWrapper(void* param) {
    static_cast<MCL*>(param)->taskFunction();
    return nullptr;
}

void MCL::taskFunction() {
    // Get initial pose
    lemlib::Pose currentPose = chassis.odom_pose_get();
    mLastPose = currentPose;
    
    while (mRunning) {
        // Get current pose from odometry
        currentPose = chassis.odom_pose_get();
        
        // Calculate motion delta
        double deltaX = currentPose.x - mLastPose.x;
        double deltaY = currentPose.y - mLastPose.y;
        double deltaTheta = currentPose.theta - mLastPose.theta;
        
        // Wrap angle difference to [-180, 180)
        while (deltaTheta > 180) deltaTheta -= 360;
        while (deltaTheta <= -180) deltaTheta += 360;
        
        // Only update if there was significant movement
        const double MIN_MOVEMENT = 0.05; // inches
        const double MIN_ROTATION = 0.5;  // degrees
        
        if (std::abs(deltaX) > MIN_MOVEMENT || 
            std::abs(deltaY) > MIN_MOVEMENT || 
            std::abs(deltaTheta) > MIN_ROTATION) {
            
            // Update motion model
            updateMotion(deltaX, deltaY, deltaTheta);
            
            // Update sensor model (if available sensors)
            updateSensor();
            
            // Check if we need to resample
            if (calculateEffectiveParticleCount() < mNumParticles * mResampleThreshold) {
                resampleParticles();
            }
            
            // Save current pose for next iteration
            mLastPose = currentPose;
        }
        
        // Sleep to avoid hogging CPU
        pros::delay(10);
    }
}

void MCL::updateMotion(double deltaX, double deltaY, double deltaTheta) {
    mParticleMutex.take();
    
    // Create noise distributions
    std::normal_distribution<double> noiseX(0, mMotionNoiseTranslation * std::abs(deltaX) + 0.01);
    std::normal_distribution<double> noiseY(0, mMotionNoiseTranslation * std::abs(deltaY) + 0.01);
    std::normal_distribution<double> noiseTheta(0, mMotionNoiseRotation * std::abs(deltaTheta) + 0.1);
    
    // Update each particle based on motion model with noise
    for (auto& p : mParticles) {
        // Convert from global to local coordinate frame
        double theta_rad = p.theta * M_PI / 180.0;
        double cos_theta = std::cos(theta_rad);
        double sin_theta = std::sin(theta_rad);
        
        // Apply motion in local frame
        p.x += (deltaX * cos_theta - deltaY * sin_theta) + noiseX(mGen);
        p.y += (deltaX * sin_theta + deltaY * cos_theta) + noiseY(mGen);
        p.theta += deltaTheta + noiseTheta(mGen);
        
        // Keep theta in [0, 360)
        while (p.theta < 0) p.theta += 360;
        while (p.theta >= 360) p.theta -= 360;
        
        // Keep particles inside the field (optional)
        p.x = std::max(0.0, std::min(p.x, mFieldWidth));
        p.y = std::max(0.0, std::min(p.y, mFieldLength));
    }
    
    mParticleMutex.give();
}

void MCL::updateSensor() {
    mParticleMutex.take();
    
    // Here we would incorporate sensor readings to update particle weights
    // For now, we'll use a simplified model comparing to distance readings
    
    // Check if we have distance sensors
    if (rightAlignmentSensor.get_status() == pros::E_DEVICE_STATUS_ONLINE) {
        // Get actual distance reading
        double wallDist = (rightAlignmentSensor.get_distance() * 0.0393701); // Convert to inches
        
        // Create sensor noise model
        std::normal_distribution<double> sensorNoiseModel(0, mSensorNoise);
        
        for (auto& p : mParticles) {
            // Calculate expected distance to wall based on particle position
            // This is a simplified example - in a real implementation we would 
            // have a proper map of the field and calculate expected sensor readings
            
            // For this example, assume the wall is at x=0
            double expectedDist = p.x;
            
            // Difference between measured and expected
            double diff = wallDist - expectedDist;
            
            // Update weight based on how well this particle's predicted measurement
            // matches the actual measurement
            double prob = std::exp(-(diff * diff) / (2 * mSensorNoise * mSensorNoise));
            p.weight *= prob;
        }
        
        // Normalize weights
        normalizeWeights();
    }
    
    mParticleMutex.give();
}

void MCL::normalizeWeights() {
    double sumWeights = 0;
    
    // Sum all weights
    for (const auto& p : mParticles) {
        sumWeights += p.weight;
    }
    
    if (sumWeights > 0) {
        // Normalize weights
        for (auto& p : mParticles) {
            p.weight /= sumWeights;
        }
    } else {
        // If all weights are zero (degenerate case), reset to uniform
        double uniformWeight = 1.0 / mParticles.size();
        for (auto& p : mParticles) {
            p.weight = uniformWeight;
        }
    }
}

double MCL::calculateEffectiveParticleCount() {
    double sumSquaredWeights = 0;
    
    for (const auto& p : mParticles) {
        sumSquaredWeights += p.weight * p.weight;
    }
    
    return sumSquaredWeights > 0 ? 1.0 / sumSquaredWeights : 0;
}

void MCL::resampleParticles() {
    std::vector<Particle> newParticles;
    newParticles.reserve(mNumParticles);
    
    // Create cumulative distribution for resampling
    std::vector<double> cumSum(mParticles.size());
    double sum = 0;
    
    for (size_t i = 0; i < mParticles.size(); ++i) {
        sum += mParticles[i].weight;
        cumSum[i] = sum;
    }
    
    // Resample using low-variance sampling
    std::uniform_real_distribution<double> dist(0, 1.0 / mNumParticles);
    double r = dist(mGen);
    
    size_t i = 0;
    for (int m = 0; m < mNumParticles; ++m) {
        double u = r + m * (1.0 / mNumParticles);
        while (u > cumSum[i] && i < mParticles.size() - 1) {
            i++;
        }
        
        // Add particle with some additional randomness
        std::normal_distribution<double> noiseX(0, mMotionNoiseTranslation);
        std::normal_distribution<double> noiseY(0, mMotionNoiseTranslation);
        std::normal_distribution<double> noiseTheta(0, mMotionNoiseRotation);
        
        newParticles.emplace_back(
            mParticles[i].x + noiseX(mGen),
            mParticles[i].y + noiseY(mGen),
            mParticles[i].theta + noiseTheta(mGen),
            1.0 / mNumParticles // Reset weights to be uniform
        );
        
        // Keep theta in [0, 360)
        while (newParticles.back().theta < 0) newParticles.back().theta += 360;
        while (newParticles.back().theta >= 360) newParticles.back().theta -= 360;
    }
    
    // Replace old particles with new ones
    mParticles = std::move(newParticles);
}
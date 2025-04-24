#pragma once
#include "main.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>

/**
 * @brief Monte Carlo Localization (Particle Filter) implementation for robot localization
 * 
 * This class provides an asynchronous implementation of the Monte Carlo Localization
 * algorithm allowing the robot to estimate its position on the field by maintaining
 * a set of particles (possible positions) that are updated based on sensor readings
 * and motion commands.
 */
class MCL {
public:
    /**
     * @brief Struct representing a single particle in the filter
     */
    struct Particle {
        double x;          // x coordinate
        double y;          // y coordinate
        double theta;      // orientation in degrees
        double weight;     // particle weight/probability

        Particle(double _x, double _y, double _theta, double _weight = 1.0)
            : x(_x), y(_y), theta(_theta), weight(_weight) {}
    };

    /**
     * @brief Constructs a new MCL object with the given parameters
     * 
     * @param numParticles Number of particles to use (default: 100)
     * @param resampleThreshold Threshold for resampling (default: 0.5)
     * @param motionNoiseTranslation Standard deviation for translation noise (default: 0.05)
     * @param motionNoiseRotation Standard deviation for rotation noise in degrees (default: 2.0)
     * @param sensorNoise Standard deviation for sensor noise (default: 0.1)
     * @param fieldWidth Width of the field in inches (default: 144)
     * @param fieldLength Length of the field in inches (default: 144)
     */
    MCL(int numParticles = 100, 
        double resampleThreshold = 0.5,
        double motionNoiseTranslation = 0.05, 
        double motionNoiseRotation = 2.0,
        double sensorNoise = 0.1,
        double fieldWidth = 144,
        double fieldLength = 144);

    /**
     * @brief Destroys the MCL object and stops the task
     */
    ~MCL();

    /**
     * @brief Start the MCL task to run asynchronously
     */
    void start();

    /**
     * @brief Stop the MCL task
     */
    void stop();

    /**
     * @brief Reset all particles with a uniform distribution across the field
     */
    void resetParticles();

    /**
     * @brief Reset particles around a specific pose
     * 
     * @param x X coordinate to center particles around
     * @param y Y coordinate to center particles around
     * @param theta Orientation to center particles around (in degrees)
     * @param spreadX Standard deviation for x coordinates (default: 5.0)
     * @param spreadY Standard deviation for y coordinates (default: 5.0)
     * @param spreadTheta Standard deviation for orientation in degrees (default: 10.0)
     */
    void resetParticles(double x, double y, double theta, 
                        double spreadX = 5.0, double spreadY = 5.0, double spreadTheta = 10.0);

    /**
     * @brief Get the current estimated pose
     * 
     * @return lemlib::Pose Current estimated pose
     */
    lemlib::Pose getPose();

    /**
     * @brief Get the particle with the highest weight
     * 
     * @return Particle Best particle
     */
    Particle getBestParticle();

    /**
     * @brief Get the variance of the particle distribution
     * 
     * @return double Variance of the particles (higher means more uncertain)
     */
    double getVariance();

    /**
     * @brief Get all particles for visualization or debugging
     * 
     * @return std::vector<Particle> Vector of all particles
     */
    std::vector<Particle> getParticles();

    /**
     * @brief Set parameters for the MCL algorithm
     * 
     * @param motionNoiseTranslation Standard deviation for translation noise
     * @param motionNoiseRotation Standard deviation for rotation noise in degrees
     * @param sensorNoise Standard deviation for sensor noise
     */
    void setParameters(double motionNoiseTranslation, double motionNoiseRotation, double sensorNoise);

private:
    // Parameters
    int mNumParticles;
    double mResampleThreshold;
    double mMotionNoiseTranslation;
    double mMotionNoiseRotation;
    double mSensorNoise;
    double mFieldWidth;
    double mFieldLength;

    // Particles
    std::vector<Particle> mParticles;

    // Random number generator
    std::random_device mRd;
    std::mt19937 mGen;

    // Task control
    pros::Task* mTask;
    bool mRunning;
    pros::Mutex mParticleMutex;

    // Last known position
    lemlib::Pose mLastPose;
    
    /**
     * @brief Main task function that runs the MCL algorithm
     */
    void taskFunction();

    /**
     * @brief Static wrapper for the task function
     * 
     * @param param Pointer to the MCL instance
     * @return void* Always returns nullptr
     */
    static void* taskFunctionWrapper(void* param);

    /**
     * @brief Resample particles based on their weights
     */
    void resampleParticles();

    /**
     * @brief Update particles based on odometry motion
     * 
     * @param deltaX Change in x position
     * @param deltaY Change in y position
     * @param deltaTheta Change in orientation (in degrees)
     */
    void updateMotion(double deltaX, double deltaY, double deltaTheta);

    /**
     * @brief Update particle weights based on sensor measurements
     */
    void updateSensor();

    /**
     * @brief Normalize particle weights so they sum to 1
     */
    void normalizeWeights();

    /**
     * @brief Calculate the effective number of particles (a measure of particle diversity)
     * 
     * @return double Effective number of particles
     */
    double calculateEffectiveParticleCount();
};

// Global instance for easy access from other files
extern MCL mcl;
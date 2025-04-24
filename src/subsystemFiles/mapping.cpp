#include "subsystemHeaders/mapping.hpp"
#include "subsystemHeaders/globals.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>

// Global mapper instance
Mapper mapper;

Mapper::Mapper(double sampleInterval, double angleThreshold, const std::string& filename)
    : mSampleInterval(sampleInterval),
      mAngleThreshold(angleThreshold),
      mFilename(filename),
      mIsMapping(false),
      mMappingTask(nullptr),
      mLastSamplePose{0, 0, 0} {
}

Mapper::~Mapper() {
    stopMapping();
}

bool Mapper::startMapping() {
    if (mIsMapping) {
        return false;
    }
    
    mIsMapping = true;
    mLastSamplePose = chassis.odom_pose_get();
    mMappingTask = new pros::Task(mappingTaskWrapper, this, "Mapping Task");
    
    return true;
}

bool Mapper::stopMapping() {
    if (!mIsMapping) {
        return false;
    }
    
    mIsMapping = false;
    
    // Wait for task to exit
    pros::delay(50);
    
    if (mMappingTask != nullptr) {
        delete mMappingTask;
        mMappingTask = nullptr;
    }
    
    return true;
}

bool Mapper::isMapping() const {
    return mIsMapping;
}

bool Mapper::saveMap() {
    std::ofstream file("/usd/" + mFilename);
    
    if (!file) {
        return false;
    }
    
    // Write header
    file << "# Field Map Data\n";
    file << "# Format: x y theta distance sensorId\n";
    file << "# Units: inches, degrees\n";
    
    // Write data with fixed precision
    file << std::fixed << std::setprecision(3);
    for (const auto& point : mMapPoints) {
        file << point.x << " " 
             << point.y << " " 
             << point.theta << " " 
             << point.distance << " " 
             << point.sensorId << "\n";
    }
    
    return true;
}

bool Mapper::loadMap() {
    std::ifstream file("/usd/" + mFilename);
    
    if (!file) {
        return false;
    }
    
    clearMap();
    
    std::string line;
    double x, y, theta, distance;
    int sensorId;
    
    // Skip header lines that start with #
    while (std::getline(file, line)) {
        if (!line.empty() && line[0] == '#') {
            continue;
        }
        
        std::stringstream ss(line);
        if (ss >> x >> y >> theta >> distance >> sensorId) {
            mMapPoints.emplace_back(x, y, theta, distance, sensorId);
        }
    }
    
    return true;
}

void Mapper::clearMap() {
    mMapPoints.clear();
}

size_t Mapper::getPointCount() const {
    return mMapPoints.size();
}

const std::vector<MapPoint>& Mapper::getMapPoints() const {
    return mMapPoints;
}

void Mapper::takeSample() {
    lemlib::Pose currentPose = chassis.odom_pose_get();
    
    // Check right distance sensor
    if (rightAlignmentSensor.get_status() == pros::E_DEVICE_STATUS_ONLINE) {
        double distance = rightAlignmentSensor.get_distance() * 0.0393701; // Convert to inches
        mMapPoints.emplace_back(currentPose.x, currentPose.y, currentPose.theta, distance, 0);
    }
    
    // Check left distance sensor
    if (leftAlignmentSensor.get_status() == pros::E_DEVICE_STATUS_ONLINE) {
        double distance = leftAlignmentSensor.get_distance() * 0.0393701; // Convert to inches
        mMapPoints.emplace_back(currentPose.x, currentPose.y, currentPose.theta, distance, 1);
    }
    
    mLastSamplePose = currentPose;
    
    // Print some feedback
    std::cout << "Map sample taken at: (" << currentPose.x << ", " << currentPose.y << ", " << currentPose.theta << ")" << std::endl;
    
    // Blink controller LED to indicate sampling
    master.rumble(".");
}

void* Mapper::mappingTaskWrapper(void* param) {
    static_cast<Mapper*>(param)->mappingTask();
    return nullptr;
}

void Mapper::mappingTask() {
    while (mIsMapping) {
        lemlib::Pose currentPose = chassis.odom_pose_get();
        
        // Calculate distance since last sample
        double dx = currentPose.x - mLastSamplePose.x;
        double dy = currentPose.y - mLastSamplePose.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // Calculate angle change since last sample
        double dTheta = std::abs(currentPose.theta - mLastSamplePose.theta);
        while (dTheta > 180) dTheta = 360 - dTheta; // Smallest angle difference
        
        // Take a sample if we've moved far enough or turned enough
        if (distance >= mSampleInterval || dTheta >= mAngleThreshold) {
            takeSample();
        }
        
        pros::delay(20); // 50Hz sampling rate
    }
}

void Mapper::runMappingRoutine() {
    // Clear any existing map
    clearMap();
    
    // Start mapping
    startMapping();
    
    // Print instructions to the screen
    std::cout << "=== MAPPING MODE ===" << std::endl;
    std::cout << "Running field perimeter mapping routine." << std::endl;
    std::cout << "The robot will drive around the field's perimeter." << std::endl;
    
    // Set holonomic drive mode if available
    // chassis.set_drive_mode(ez::Drive::HOLONOMIC);
    
    // Save current position to return to at the end
    lemlib::Pose startingPose = chassis.odom_pose_get();
    
    // Run a simple square path around the field
    // Assuming field is roughly 144" x 144" with the robot starting near (0,0)
    
    // Side 1: Drive forward along X axis
    chassis.pid_drive_set(132, 100); // Drive 132 inches forward at 100% speed
    chassis.pid_wait();
    pros::delay(500); // Pause at corner to get stable readings
    
    // Side 2: Turn and drive along Y axis
    chassis.pid_turn_set(90, 100);
    chassis.pid_wait();
    chassis.pid_drive_set(132, 100);
    chassis.pid_wait();
    pros::delay(500);
    
    // Side 3: Turn and drive back along X axis
    chassis.pid_turn_set(180, 100);
    chassis.pid_wait();
    chassis.pid_drive_set(132, 100);
    chassis.pid_wait();
    pros::delay(500);
    
    // Side 4: Turn and drive back to starting position
    chassis.pid_turn_set(270, 100);
    chassis.pid_wait();
    chassis.pid_drive_set(132, 100);
    chassis.pid_wait();
    pros::delay(500);
    
    // Return to original orientation
    chassis.pid_turn_set(startingPose.theta, 100);
    chassis.pid_wait();
    
    // Stop mapping
    stopMapping();
    
    // Save the map
    if (saveMap()) {
        std::cout << "Map saved successfully! " << getPointCount() << " points collected." << std::endl;
        master.set_text(1, 0, "Map saved: " + std::to_string(getPointCount()) + " pts");
    } else {
        std::cout << "Failed to save map!" << std::endl;
        master.set_text(1, 0, "Map save FAILED!");
    }
}
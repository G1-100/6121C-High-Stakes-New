#pragma once
#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "mcl.hpp"
#include <vector>
#include <string>
#include <fstream>

/**
 * @brief Structure to store map point data
 */
struct MapPoint {
    double x;          // x position in inches
    double y;          // y position in inches
    double theta;      // orientation in degrees
    double distance;   // distance reading from sensor in inches
    int sensorId;      // ID of the sensor used (0=right, 1=left)
    
    MapPoint(double _x, double _y, double _theta, double _distance, int _sensorId)
        : x(_x), y(_y), theta(_theta), distance(_distance), sensorId(_sensorId) {}
};

/**
 * @brief Class for mapping the field and storing sensor data
 */
class Mapper {
public:
    /**
     * @brief Construct a new Mapper object
     * 
     * @param sampleInterval Minimum distance traveled between samples in inches (default: 3.0)
     * @param angleThreshold Minimum angle change between samples in degrees (default: 10.0)
     * @param filename File name to save map data to (default: "field_map.txt")
     */
    Mapper(double sampleInterval = 3.0, double angleThreshold = 10.0, 
           const std::string& filename = "field_map.txt");

    /**
     * @brief Destructor
     */
    ~Mapper();

    /**
     * @brief Start the mapping process
     * 
     * @return true if mapping was started
     * @return false if mapping was already running
     */
    bool startMapping();

    /**
     * @brief Stop the mapping process
     * 
     * @return true if mapping was stopped
     * @return false if mapping wasn't running
     */
    bool stopMapping();

    /**
     * @brief Check if mapping is currently running
     * 
     * @return true if mapping is active
     * @return false if mapping is not active
     */
    bool isMapping() const;

    /**
     * @brief Save the current map to file
     * 
     * @return true if save was successful
     * @return false if save failed
     */
    bool saveMap();

    /**
     * @brief Load a map from file
     * 
     * @return true if load was successful
     * @return false if load failed
     */
    bool loadMap();

    /**
     * @brief Clear all map points
     */
    void clearMap();

    /**
     * @brief Get the total number of points in the map
     * 
     * @return size_t number of points
     */
    size_t getPointCount() const;

    /**
     * @brief Get all the map points
     * 
     * @return const std::vector<MapPoint>& reference to map points
     */
    const std::vector<MapPoint>& getMapPoints() const;

    /**
     * @brief Run a predefined mapping routine around the field perimeter
     */
    void runMappingRoutine();

private:
    double mSampleInterval;       // Minimum distance between samples
    double mAngleThreshold;       // Minimum angle change for samples
    std::string mFilename;        // Filename to save map data to
    
    std::vector<MapPoint> mMapPoints;  // Stored map points
    bool mIsMapping;                   // Flag for active mapping
    
    lemlib::Pose mLastSamplePose;      // Last sampled pose
    pros::Task* mMappingTask;          // Mapping task

    /**
     * @brief Take a sample at the current position and add it to the map
     */
    void takeSample();

    /**
     * @brief Main mapping task function
     */
    void mappingTask();

    /**
     * @brief Static wrapper for mapping task
     * 
     * @param param Pointer to Mapper instance
     * @return void* Always nullptr
     */
    static void* mappingTaskWrapper(void* param);
};

// Global mapper instance
extern Mapper mapper;
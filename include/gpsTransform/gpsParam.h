#ifndef GPS_PARAM_H
#define GPS_PARAM_H

#include <iostream>
#include <mutex>
#include <ros/ros.h>

namespace RoboSLAM3
{
namespace GpsTrans
{
class GpsParam {
private:
    GpsParam();

public:
    GpsParam( const GpsParam & )            = delete;
    GpsParam &operator=( const GpsParam & ) = delete;

    static GpsParam *GetInstance();

public:
public:
    // Save pcd
    bool        savePCD;
    std::string savePCDDirectory;
    std::string loadPCDDirectory;

    float       gpsCovThreshold;
    std::string gpsTopic;
    bool        useGpsElevation;

    int processSignal;

private:
    ros::NodeHandle nh;

    static GpsParam  *instance;
    static std::mutex mtx;
};

} // namespace GpsTrans
} // namespace RoboSLAM3

#endif
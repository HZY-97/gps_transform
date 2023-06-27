#ifndef GPS_PARAM_HPP
#define GPS_PARAM_HPP

#include <iostream>
#include <mutex>
#include <ros/ros.h>

namespace RoboSLAM3
{
namespace GpsTrans
{
class GpsParam {
private:
    GpsParam() {
        nh.param<bool>( "lio_sam/savePCD", savePCD, false );
        nh.param<std::string>( "lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/" );
        nh.param<std::string>( "lio_sam/loadPCDDirectory", loadPCDDirectory, "/Downloads/LOAM/" );

        nh.param<float>( "lio_sam/gpsCovThreshold", gpsCovThreshold, 2.0 );
        nh.param<std::string>( "lio_sam/gpsTopic", gpsTopic, "odometry/gps" );

        nh.param<bool>( "lio_sam/useGpsElevation", useGpsElevation, false );
        nh.param<int>( "lio_sam/processSignal", processSignal, 1 );

        std::cout << "processSignal: " << processSignal << std::endl;
    }

public:
    GpsParam( const GpsParam & )            = delete;
    GpsParam &operator=( const GpsParam & ) = delete;

    static GpsParam *GetInstance() {
        if ( instance == nullptr )
        {
            std::lock_guard<std::mutex> lock( mtx );
            if ( instance == nullptr )
            {
                instance = new GpsParam();
            }
        }
        return instance;
    }

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

GpsParam  *GpsParam::instance = nullptr;
std::mutex GpsParam::mtx;
} // namespace GpsTrans
} // namespace RoboSLAM3

#endif
#include "gpsTransform/gpsParam.h"
namespace RoboSLAM3
{
namespace GpsTrans
{
GpsParam  *GpsParam::instance = nullptr;
std::mutex GpsParam::mtx;
GpsParam::GpsParam() {
    nh.param<bool>( "lio_sam/savePCD", savePCD, false );
    nh.param<std::string>( "lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/" );
    nh.param<std::string>( "lio_sam/loadPCDDirectory", loadPCDDirectory, "/Downloads/LOAM/" );

    nh.param<float>( "lio_sam/gpsCovThreshold", gpsCovThreshold, 2.0 );
    nh.param<std::string>( "lio_sam/gpsTopic", gpsTopic, "odometry/gps" );

    nh.param<bool>( "lio_sam/useGpsElevation", useGpsElevation, false );
    nh.param<int>( "lio_sam/processSignal", processSignal, 1 );

    std::cout << "processSignal: " << processSignal << std::endl;
}

GpsParam *GpsParam::GetInstance() {
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
} // namespace GpsTrans
} // namespace RoboSLAM3
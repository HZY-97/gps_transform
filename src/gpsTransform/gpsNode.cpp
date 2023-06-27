#include <ros/ros.h>

#include "gpsTransform/gpsTransform.h"
#include "gpsTransform/gpsParam.hpp"

using namespace RoboSLAM3::GpsTrans;

ros::Publisher path_pub;
ros::Publisher odom_pub;
GpsTransform  *gpsTransform;

void gpsCallback( const sensor_msgs::NavSatFix::ConstPtr &msg ) {
    // 处理GPS数据                         排名
    // 0-未定位或无效解                      5
    // 1-单点定位                          4
    // 4-定位 RTK 固定解                    1
    // 5-定位 RTK 浮点解                    2
    // 6-INS 定位解或 GNSS/INS 组合定位解     3

    printf( "GPS_STATE: %d\n", msg->status.status );

    // if(4 == msg->status.status){
    gpsTransform->ReceiveGpsMsg( msg );
    path_pub.publish( gpsTransform->GetEnuPath() );
    odom_pub.publish( gpsTransform->GetEnuOdom() );
    // }
}

int main( int argc, char **argv ) {
    ros::init( argc, argv, "gps_transform_node" );
    ros::NodeHandle nh;
    gpsTransform = nullptr;

    int slamMethord = GpsParam::GetInstance()->processSignal;

    switch ( slamMethord )
    {
    case 0: // Map
        gpsTransform = new GpsTransform();
        break;
    case 1: // Locat
        // TODO load gps origin
        GpsParam::GetInstance()->loadPCDDirectory;
        gpsTransform = new GpsTransform();
        break;
    case 2: // SLAM
        ROS_ERROR( "Unknow SlamMethord %d", slamMethord );
        break;
    default:
        ROS_ERROR( "Unknow SlamMethord %d", slamMethord );
        break;
    }

    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>( "/hc_driver/gps_data", 10, gpsCallback );

    path_pub = nh.advertise<nav_msgs::Path>( "/gps_path_enu", 10 );
    odom_pub = nh.advertise<nav_msgs::Odometry>( "/gps_odom_enu", 10 );

    ROS_INFO( "\033[1;32m----> GPS_Trans Start.\033[0m" );

    ros::spin();

    return 0;
}

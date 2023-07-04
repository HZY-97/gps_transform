#include <ros/ros.h>
#include <csignal>

#include <iostream>
#include <fstream>
#include <map>
#include <string>

#include "gpsTransform/gpsTransform.h"
#include "gpsTransform/gpsParam.h"
#include "nlohmann/single_include/json.hpp"

using namespace RoboSLAM3::GpsTrans;

ros::Publisher               path_pub;
ros::Publisher               odom_pub;
GpsTransform                *gpsTransform;
std::map<std::string, float> configMap;

void sigintHandler( int sig );
void gpsCallback( const sensor_msgs::NavSatFix::ConstPtr &msg );

int main( int argc, char **argv ) {
    ros::init( argc, argv, "gps_transform_node" );
    ros::NodeHandle nh;

    std::signal( SIGINT, sigintHandler );

    gpsTransform = nullptr;

    int slamMethord = GpsParam::GetInstance()->processSignal;

    if ( 0 == slamMethord ) // MAP
    {
        gpsTransform = new GpsTransform();
    }
    else if ( 1 == slamMethord ) // LOCAT
    {
        std::ifstream inputFile( GpsParam::GetInstance()->loadPCDDirectory + "gpsConfig.json" );
        if ( inputFile.is_open() )
        {
            nlohmann::json json      = nlohmann::json::parse( inputFile );
            double         m_yaw     = json [ "FirstWithNorth" ];
            double         latitude  = json [ "Latitude" ];
            double         longitude = json [ "Longitude" ];
            double         altitude  = json [ "Altitude" ];

            gpsTransform = new GpsTransform( latitude, longitude, altitude, m_yaw );
        }
        else
        {
            std::cout << "无法打开文件。" << std::endl;
        }
    }
    else if ( 2 == slamMethord ) // SLAM
    {
        ROS_ERROR( "Unknow SlamMethord 2" );
    }
    else // UNKNOW
    {
        ROS_ERROR( "Unknow SlamMethord %d", slamMethord );
    }

    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>( "/hc_driver/gps_data", 10, gpsCallback );

    path_pub = nh.advertise<nav_msgs::Path>( "/gps_path_enu", 10 );
    odom_pub = nh.advertise<nav_msgs::Odometry>( "/gps_odom_enu", 10 );

    ROS_INFO( "\033[1;32m----> GPS_Trans Start.\033[0m" );

    ros::spin();

    return 0;
}

void sigintHandler( int sig ) {
    if ( 0 == GpsParam::GetInstance()->processSignal )
    {
        std::cout << "Saving gpsPath ..." << std::endl;
        pcl::io::savePCDFileBinary( GpsParam::GetInstance()->savePCDDirectory + "gpsPath.pcd",
                                    *gpsTransform->GetGpsPath() );

        std::cout << "Saving gpsPath Finished" << std::endl;
    }

    ros::shutdown(); // 关闭ROS节点
}

void gpsCallback( const sensor_msgs::NavSatFix::ConstPtr &msg ) {
    if ( gpsTransform == nullptr )
    {
        std::cout << "ERROR!!! No gpsTransform Obj" << std::endl;
        return;
    }
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

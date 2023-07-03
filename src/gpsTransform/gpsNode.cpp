#include <ros/ros.h>
#include <csignal>

#include <iostream>
#include <fstream>
#include <map>
#include <string>

#include "gpsTransform/gpsTransform.h"
#include "gpsTransform/gpsParam.h"

using namespace RoboSLAM3::GpsTrans;

ros::Publisher               path_pub;
ros::Publisher               odom_pub;
GpsTransform                *gpsTransform;
std::map<std::string, float> configMap;

void sigintHandler( int sig );
void parseConfigFile( const std::string &fileName );
void gpsCallback( const sensor_msgs::NavSatFix::ConstPtr &msg );
void updateConfigFile( const std::string &fileName );

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
        parseConfigFile( GpsParam::GetInstance()->loadPCDDirectory + "gpsConfig.txt" );
        auto it_yaw = configMap.find( "FirstWithNorth" );
        auto it_lat = configMap.find( "Latitude" );
        auto it_lon = configMap.find( "Longitude" );
        auto it_alt = configMap.find( "Altitude" );
        if ( it_lat != configMap.end() && it_lon != configMap.end() && it_alt != configMap.end() &&
             it_yaw != configMap.end() )
        {
            gpsTransform = new GpsTransform( it_lat->second, it_lon->second, it_alt->second, it_yaw->second );
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

        // updateConfigFile( GpsParam::GetInstance()->savePCDDirectory + "gpsConfig.txt" );
        std::cout << "Saving gpsPath Finished" << std::endl;
    }

    ros::shutdown(); // 关闭ROS节点
}

void parseConfigFile( const std::string &fileName ) {
    std::ifstream inputFile( fileName );

    if ( inputFile.is_open() )
    {
        std::string line;
        while ( std::getline( inputFile, line ) )
        {
            size_t delimiterPos = line.find( ":" );
            if ( delimiterPos != std::string::npos )
            {
                std::string key         = line.substr( 0, delimiterPos );
                std::string valueString = line.substr( delimiterPos + 1 );
                try
                {
                    double value      = std::stod( valueString );
                    configMap [ key ] = value;
                } catch ( const std::exception &e )
                { std::cout << "Failed to convert value to double for key: " << key << std::endl; }
            }
        }

        inputFile.close();
    }
    else
    {
        std::cout << "parseConfigFile :: Failed to open the file." << std::endl;
    }
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

void updateConfigFile( const std::string &fileName ) {
    if ( configMap.empty() )
    {
        std::cout << "updateConfigFile::ERROR!!! configMap is empyt" << std::endl;
        return;
    }
    std::ofstream outputFile( fileName );

    if ( outputFile.is_open() )
    {
        for ( const auto &pair : configMap )
        {
            outputFile << pair.first << ":" << std::fixed << std::setprecision( 8 ) << pair.second << std::endl;
        }

        outputFile.close();
        std::cout << "File updated." << std::endl;
    }
    else
    {
        std::cout << "Failed to open the file for writing." << std::endl;
    }
}

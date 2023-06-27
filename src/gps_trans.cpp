// author：zeng shengyao
// time: 2023.04.14
// faction: 本文件的作用是将GPS轨迹转换到世界坐标系下去

#include "../include/gps_trans.hpp"

ros::Publisher path_pub;
ros::Publisher odom_pub;
GpsTransform   gps_transform;

void gpsCallback( const sensor_msgs::NavSatFix::ConstPtr &msg ) {
    // 处理GPS数据                         排名
    // 0-未定位或无效解                      5
    // 1-单点定位                          4
    // 4-定位 RTK 固定解                    1
    // 5-定位 RTK 浮点解                    2
    // 6-INS 定位解或 GNSS/INS 组合定位解     3

    printf( "GPS_STATE: %d\n", msg->status.status );

    // if(4 == msg->status.status){
    gps_transform.add_gps_msg( msg );
    path_pub.publish( gps_transform.path_enu() );
    odom_pub.publish( gps_transform.odom_enu() );
    // }
}

int main( int argc, char **argv ) {
    ros::init( argc, argv, "my_node" );
    ros::NodeHandle nh;

    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>( "/hc_driver/gps_data", 10, gpsCallback );
    path_pub                = nh.advertise<nav_msgs::Path>( "/gps_path_enu", 10 );
    odom_pub                = nh.advertise<nav_msgs::Odometry>( "/gps_odom_enu", 10 );
    ros::spin();

    return 0;
}

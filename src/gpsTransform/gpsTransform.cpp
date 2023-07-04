#include "gpsTransform/gpsTransform.h"

namespace RoboSLAM3
{
namespace GpsTrans
{

GpsTransform::GpsTransform() {
    path_enu_.header.frame_id = "map";
    odom_rtk_.header.frame_id = "map";

    isFirstGps = true;

    m_gpsPathPcl.reset( new pcl::PointCloud<PointType>() );
    m_yaw = 0.0;
    matrix_enu_2_body_.Identity();
}

//这个函数还没有测试过
GpsTransform::GpsTransform( double ref_lat, double ref_lon, double ref_alt, double yaw ) {
    path_enu_.header.frame_id = "map";
    odom_rtk_.header.frame_id = "map";

    //初始化站心坐标

    //计算参考点的ecef坐标
    auto ref_ecef = Lla2Ecef( ref_lat, ref_lon, ref_alt );

    world_orign_.lat = ref_lat;
    world_orign_.lon = ref_lon;
    world_orign_.alt = ref_alt;

    world_orign_.xyz_ecef = ref_ecef;

    // 计算ECEF到ENU坐标系的变换矩阵
    matrix_ecef_2_enu_ = Ecef2EnuMatrix( world_orign_.xyz_ecef, world_orign_.lat, world_orign_.lon );

    // 计算ENU到BODY坐标系的变换矩阵
    Enu2BodyMatrix( yaw );

    m_gpsPathPcl.reset( new pcl::PointCloud<PointType>() );
    isFirstGps = true;
    m_yaw      = 0.0;

    initialized_ = true;
}

void GpsTransform::ReceiveGpsMsg( const sensor_msgs::NavSatFix::ConstPtr &msg ) {
    m_currentGpsTime = msg->header.stamp;

    path_ecef_.emplace_back( Lla2Ecef( msg->latitude, msg->longitude, msg->altitude ) );
    if ( initialized_ )
    {
        static int  id         = 0;
        const auto &point_ecef = path_ecef_.back();
        auto        point_enu  = Ecef2Enu( point_ecef );

        //更新ros路径用于显示
        geometry_msgs::PoseStamped pose;
        pose.header          = msg->header;
        pose.pose.position.x = point_enu.x();
        pose.pose.position.y = point_enu.y();
        pose.pose.position.z = point_enu.z();

        Eigen::Vector3d afterPose = Enu2Body( point_enu.x(), point_enu.y(), point_enu.z() );
        pose.pose.position.x      = afterPose.x();
        pose.pose.position.y      = afterPose.y();
        pose.pose.position.z      = afterPose.z();

        std::cout << "========================" << std::endl;
        std::cout << "GPS----x = " << pose.pose.position.x << " y = " << pose.pose.position.y
                  << " z = " << pose.pose.position.z << std::endl;

        pcl::PointXYZI tmpPose;
        tmpPose.x         = pose.pose.position.x;
        tmpPose.y         = pose.pose.position.y;
        tmpPose.z         = pose.pose.position.z;
        tmpPose.intensity = id;
        id++;
        m_gpsPathPcl->push_back( tmpPose );

        path_enu_.poses.push_back( pose );

        //更新里程计信息
        odom_rtk_.header.stamp       = msg->header.stamp;
        odom_rtk_.pose.pose.position = pose.pose.position;
    }
    else
    {
        // 未初始化则使用第一个点作为站心坐标
        // 初始化参考点
        world_orign_.lat      = msg->latitude;
        world_orign_.lon      = msg->longitude;
        world_orign_.alt      = msg->altitude;
        world_orign_.xyz_ecef = path_ecef_.front();

        // 计算ECEF到ENU坐标系的变换矩阵
        matrix_ecef_2_enu_ = Ecef2EnuMatrix( world_orign_.xyz_ecef, world_orign_.lat, world_orign_.lon );

        if ( isFirstGps && !isnan( msg->position_covariance [ 0 ] ) )
        {
            Enu2BodyMatrix( msg->position_covariance [ 0 ] );
            isFirstGps = false;
        }

        m_yaw = msg->position_covariance [ 0 ];

        std::ofstream outfile( GpsParam::GetInstance()->savePCDDirectory + "gpsConfig.json", std::ios::trunc );
        if ( outfile.is_open() )
        {
            nlohmann::json json;
            json [ "FirstWithNorth" ] = m_yaw;
            json [ "Latitude" ]       = world_orign_.lat;
            json [ "Longitude" ]      = world_orign_.lon;
            json [ "Altitude" ]       = world_orign_.alt;

            outfile << json.dump( 4 );

            outfile.close();
            std::cout << "gpsConfig.json文件已成功创建并写入。" << std::endl;
        }
        else
        {
            std::cout << "无法创建文件！" << std::endl;
        }
        initialized_ = true;
    }
}

// 维经高转到地心地固坐标系下
Eigen::Vector3d GpsTransform::Lla2Ecef( double lat, double lon, double alt ) {
    lat = lat * M_PI / 180.0; // 转弧度表示
    lon = lon * M_PI / 180.0; // 转弧度表示

    double W = 1 - e2 * sin( lat ) * sin( lat );
    double N = EARTH_RADIUS / sqrt( W ); // 卯酉圈曲率半径

    double X = ( N + alt ) * cos( lat ) * cos( lon );
    double Y = ( N + alt ) * cos( lat ) * sin( lon );
    double Z = ( N * ( 1 - e2 ) + alt ) * sin( lat );

    return Eigen::Vector3d( X, Y, Z );
}

// 计算参考点处的ceef到enu坐标系的变换矩阵
Eigen::Matrix3d GpsTransform::Ecef2EnuMatrix( const Eigen::Vector3d &ref_xyz, double ref_lat, double ref_lon ) {
    ref_lat = ref_lat * M_PI / 180.0; // 转弧度表示
    ref_lon = ref_lon * M_PI / 180.0; // 转弧度表示
    // 计算参考点的大地坐标系参数
    double W       = 1 - e2 * sin( ref_lat ) * sin( ref_lat );
    double N       = EARTH_RADIUS / sqrt( W ); // 卯酉圈曲率半径
    double sin_lat = sin( ref_lat );
    double cos_lat = cos( ref_lat );
    double sin_lon = sin( ref_lon );
    double cos_lon = cos( ref_lon );

    // 计算ENU坐标系的旋转矩阵
    static Eigen::Matrix3d R;
    R << -sin_lon, cos_lon, 0, -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat, cos_lat * cos_lon, cos_lat * sin_lon,
        sin_lat;
    return R;
}

void GpsTransform::Enu2BodyMatrix( const double &current_deg_with_north ) {
    float tmpNorthDeg = current_deg_with_north - 90;

    float northRadians = tmpNorthDeg * PI / 180;

    float matrix_00 = cos( northRadians );
    float matrix_01 = -sin( northRadians );
    float matrix_02 = 0.0;

    float matrix_10 = sin( northRadians );
    float matrix_11 = cos( northRadians );
    float matrix_12 = 0.0;

    float matrix_20 = 0.0;
    float matrix_21 = 0.0;
    float matrix_22 = 1.0;

    matrix_enu_2_body_ << matrix_00, matrix_01, matrix_02, matrix_10, matrix_11, matrix_12, matrix_20, matrix_21,
        matrix_22;
}

Eigen::Vector3d GpsTransform::Enu2Body( const double &before_x, const double &before_y, const double &before_z ) {
    Eigen::Vector3d beforePose = Eigen::Vector3d::Identity();
    Eigen::Vector3d afterPose  = Eigen::Vector3d::Identity();

    beforePose << before_x, before_y, before_z;
    afterPose = matrix_enu_2_body_ * beforePose;
    // afterPose = beforePose;

    // std::cout<<"==========================="<<std::endl;
    // std::cout<<"x:"<<before_x<<"; y:"<<before_y<<"; z:"<<before_z<<std::endl;
    // std::cout<<"x:"<<afterPose.x()<<"; y:"<<afterPose.y()<<";
    // z:"<<afterPose.z()<<std::endl;

    return afterPose;
}

Eigen::Vector3d GpsTransform::Ecef2Enu( const Eigen::Vector3d &xyz ) {
    return matrix_ecef_2_enu_ * ( xyz - world_orign_.xyz_ecef );
}

} // namespace GpsTrans
} // namespace RoboSLAM3
#ifndef GPS_TRANS_H
#define GPS_TRANS_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>

#include <iostream>
#include <fstream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "gpsTransform/gpsParam.h"

namespace RoboSLAM3
{
namespace GpsTrans
{
typedef pcl::PointXYZI PointType;

class GpsTransform {
    /*******************Func*******************/
public:
    /**
     * @brief 用于建图时使用的构造，以第一帧为初始位置，构建坐标原点
     * 
     */
    GpsTransform();

    /**
     * @brief 用于定位时使用的构造，给定一个纬度经度海拔为地图原点
     * 
     * @param ref_lat 纬度
     * @param ref_lon 经度
     * @param ref_alt 海拔
     */
    GpsTransform( double ref_lat, double ref_lon, double ref_alt, double yaw ); //选定参考点的纬度经度高度

    void ReceiveGpsMsg( const sensor_msgs::NavSatFix::ConstPtr &msg );

    nav_msgs::Path     GetEnuPath() { return path_enu_; }
    nav_msgs::Odometry GetEnuOdom() { return odom_rtk_; }

    pcl::PointCloud<PointType>::Ptr GetGpsPath() { return m_gpsPathPcl; }

protected:
private:
    Eigen::Vector3d Lla2Ecef( double B, double L, double H );
    Eigen::Vector3d Ecef2Enu( const Eigen::Vector3d &xyz );
    Eigen::Matrix3d Ecef2EnuMatrix( const Eigen::Vector3d &ref_xyz, double ref_lat, double ref_lon );

    void            Enu2BodyMatrix( const double &current_deg_with_north );
    Eigen::Vector3d Enu2Body( const double &before_x, const double &before_y, const double &before_z );

    /*******************Data*******************/
public:
protected:
private:
    const double PI = 3.14159265358979323846;

    // WGS84 参数
    /*WGS84 GPS坐标系的长半轴半径*/
    const double EARTH_RADIUS = 6378137.0;
    /* WGS84 GPS坐标系的椭球第一偏心率*/
    const double e2 = 0.00669438002290;

    /*判断是否初始化了地图原点*/
    bool initialized_ = false;
    /*ecef到enu的变换矩阵*/
    Eigen::Matrix3d matrix_ecef_2_enu_;
    /*enu到body的变换矩阵*/
    Eigen::Matrix3d matrix_enu_2_body_;

    // For pub
    /*换到ENU坐标系下的轨迹*/
    nav_msgs::Path     path_enu_;
    nav_msgs::Odometry odom_rtk_;

    std::vector<Eigen::Vector3d> path_ecef_;

    // 世界坐标系信息
    struct {
        double          lon;      // 经度
        double          lat;      //  纬度
        double          alt;
        Eigen::Vector3d xyz_ecef; // 参考点的地心地固坐标
    } world_orign_;

    /*判断是否是第一次接收gps数据，只有第一次接收，才初始化enu到body的旋转矩阵*/
    bool isFirstGps;

    /*For save gps Path*/
    pcl::PointCloud<PointType>::Ptr m_gpsPathPcl;

    /*save first yaw with North*/
    double m_yaw;

    // save time
    ros::Time m_currentGpsTime;
};

} // namespace GpsTrans
} // namespace RoboSLAM3

#endif
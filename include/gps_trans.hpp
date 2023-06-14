#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <ros/ros.h>

#include "siasunLog.h"

class GpsTransform
{
private:
    const double PI = 3.14159265358979323846;
    // WGS84 参数
    const double EARTH_RADIUS = 6378137.0; // WGS84 GPS坐标系的长半轴半径
    const double e2 = 0.00669438002290;    // WGS84 GPS坐标系的椭球第一偏心率

    bool initialized_ = false;
    Eigen::Matrix3d matrix_ecef_2_enu_; // ecef到enu的变换矩阵
    Eigen::Matrix3d matrix_enu_2_body_; // enu到body的变换矩阵

    nav_msgs::Path path_enu_;                    // 换到ENU坐标系下的轨迹
    nav_msgs::Odometry odom_rtk_;

    std::vector<Eigen::Vector3d> path_ecef_;

    // 判断是否是第一次接收gps数据，只有第一次接收，才初始化enu到body的旋转矩阵
    bool isFirstGps;
    int usefulNums;
    float degVec[4];

    // 世界坐标系信息
    struct
    {
        double lon;               // 经度
        double lat;               //  纬度
        Eigen::Vector3d xyz_ecef; // 参考点的地心地固坐标
    } world_orign_;

private:
    Eigen::Vector3d Lla2Xyz(double B, double L, double H);
    Eigen::Vector3d Ecef2Enu(const Eigen::Vector3d &xyz);
    Eigen::Matrix3d Ecef2EnuMatrix(const Eigen::Vector3d &ref_xyz, double ref_lat, double ref_lon);

    void Enu2BodyMatrix(const double &current_deg_with_north);
    Eigen::Vector3d Enu2Body(const double &before_x, const double &before_y, const double &before_z);

public:
    GpsTransform();
    GpsTransform(double ref_lat, double ref_lon, double ref_alt);  //选定参考点的纬度经度高度
    void add_gps_msg(const sensor_msgs::NavSatFix::ConstPtr &msg);
    nav_msgs::Path path_enu() { return path_enu_; };
    nav_msgs::Odometry odom_enu() { return odom_rtk_; };
};

GpsTransform::GpsTransform()
{
    path_enu_.header.frame_id = "map";
    odom_rtk_.header.frame_id ="map";

    isFirstGps = true;
    usefulNums = 0;
}

//这个函数还没有测试过
GpsTransform::GpsTransform(double ref_lat, double ref_lon, double ref_alt)
{
    path_enu_.header.frame_id = "map";
    odom_rtk_.header.frame_id="map";

    //初始化站心坐标

    //计算参考点的ecef坐标
    auto ref_ecef=Lla2Xyz(ref_lat, ref_lon, ref_alt);

    world_orign_.lat = ref_lat;
    world_orign_.lon = ref_lon;
    world_orign_.xyz_ecef = ref_ecef;

    // 计算ECEF到ENU坐标系的变换矩阵
    matrix_ecef_2_enu_ = Ecef2EnuMatrix(world_orign_.xyz_ecef, world_orign_.lat, world_orign_.lon);

    initialized_=true;
}

void GpsTransform::add_gps_msg(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    if(isFirstGps && !isnan(msg->position_covariance[0])){
        if(usefulNums<5){
            degVec[usefulNums]=msg->position_covariance[0];
            Enu2BodyMatrix(degVec[usefulNums]);
            usefulNums++;
        }else{
            float degSum = 0;
            for(int i = 0; i < 5; i++){
                degSum+=degVec[i];
            }
            Enu2BodyMatrix(degSum/5);
            isFirstGps = false;
        }
    }

    path_ecef_.emplace_back(Lla2Xyz(msg->latitude, msg->longitude, msg->altitude));
    if (initialized_)
    {
        const auto &point_ecef=path_ecef_.back(); 
        auto point_enu = Ecef2Enu(point_ecef);

        //更新ros路径用于显示
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose.position.x = point_enu.x();
        pose.pose.position.y = point_enu.y();
        pose.pose.position.z = point_enu.z();

        std::cout<<"========================"<<std::endl;
        std::cout<<"x = "<<pose.pose.position.x<<" y = "<<pose.pose.position.y<<" z = "<<pose.pose.position.z<<std::endl;

        // Eigen::Vector3d afterPose = Enu2Body(point_enu.x(),point_enu.y(),point_enu.z());
        // pose.pose.position.x = afterPose.x();
        // pose.pose.position.y = afterPose.y();
        // pose.pose.position.z = afterPose.z();

        path_enu_.poses.push_back(pose);
        
        //更新里程计信息
        odom_rtk_.header.stamp=msg->header.stamp;
        odom_rtk_.pose.pose.position=pose.pose.position;

    }
    else
    {
        // 未初始化则使用第一个点作为站心坐标
        // 初始化参考点
        world_orign_.lat = msg->latitude;
        world_orign_.lon = msg->longitude;
        world_orign_.xyz_ecef = path_ecef_.front();

        // 计算ECEF到ENU坐标系的变换矩阵
        matrix_ecef_2_enu_ = Ecef2EnuMatrix(world_orign_.xyz_ecef, world_orign_.lat, world_orign_.lon);

        initialized_ = true;
    }
}

// 维经高转到地心地固坐标系下
Eigen::Vector3d GpsTransform::Lla2Xyz(double lat, double lon, double alt)
{
    lat = lat * M_PI / 180.0; // 转弧度表示
    lon = lon * M_PI / 180.0; // 转弧度表示

    double W = 1 - e2 * sin(lat) * sin(lat);
    double N = EARTH_RADIUS / sqrt(W); // 卯酉圈曲率半径

    double X = (N + alt) * cos(lat) * cos(lon);
    double Y = (N + alt) * cos(lat) * sin(lon);
    double Z = (N * (1 - e2) + alt) * sin(lat);

    return Eigen::Vector3d(X, Y, Z);
}

// 计算参考点处的ceef到enu坐标系的变换矩阵
Eigen::Matrix3d GpsTransform::Ecef2EnuMatrix(const Eigen::Vector3d &ref_xyz, double ref_lat, double ref_lon)
{
    ref_lat = ref_lat * M_PI / 180.0; // 转弧度表示
    ref_lon = ref_lon * M_PI / 180.0; // 转弧度表示
    // 计算参考点的大地坐标系参数
    double W = 1 - e2 * sin(ref_lat) * sin(ref_lat);
    double N = EARTH_RADIUS / sqrt(W); // 卯酉圈曲率半径
    double sin_lat = sin(ref_lat);
    double cos_lat = cos(ref_lat);
    double sin_lon = sin(ref_lon);
    double cos_lon = cos(ref_lon);

    // 计算ENU坐标系的旋转矩阵
    static Eigen::Matrix3d R;
    R << -sin_lon, cos_lon, 0,
        -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
        cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;
    return R ;
}

// 计算enu到body的旋转矩阵
void GpsTransform::Enu2BodyMatrix(const double &current_deg_with_north){

    float tmpNorthDeg = current_deg_with_north - 90;

    float northRadians = tmpNorthDeg * PI / 180;

    float matrix_00 = cos(northRadians);
    float matrix_01 = -sin(northRadians);
    float matrix_02 = 0.0;

    float matrix_10 = sin(northRadians);
    float matrix_11 = cos(northRadians);
    float matrix_12 = 0.0;

    float matrix_20 = 0.0;
    float matrix_21 = 0.0;
    float matrix_22 = 1.0;

    matrix_enu_2_body_ << matrix_00, matrix_01, matrix_02,
                          matrix_10, matrix_11, matrix_12,
                          matrix_20, matrix_21, matrix_22;
}

Eigen::Vector3d GpsTransform::Enu2Body(const double &before_x, const double &before_y, const double &before_z){

    Eigen::Vector3d beforePose  = Eigen::Vector3d::Identity();
    Eigen::Vector3d afterPose   = Eigen::Vector3d::Identity();

    beforePose << before_x, before_y, before_z;
    afterPose = matrix_enu_2_body_ * beforePose;
    // afterPose = beforePose;

    // std::cout<<"==========================="<<std::endl;
    // std::cout<<"x:"<<before_x<<"; y:"<<before_y<<"; z:"<<before_z<<std::endl;
    // std::cout<<"x:"<<afterPose.x()<<"; y:"<<afterPose.y()<<"; z:"<<afterPose.z()<<std::endl;

    return afterPose;
}

Eigen::Vector3d GpsTransform::Ecef2Enu(const Eigen::Vector3d &xyz)
{
    return matrix_ecef_2_enu_*(xyz- world_orign_.xyz_ecef);
}

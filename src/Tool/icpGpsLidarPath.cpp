#include <ros/ros.h>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZI PointType;

int main( int argc, char **argv ) {
    ros::init( argc, argv, "icpGpsLidarPath_node" );
    ros::NodeHandle nh;

    std::string savePCDDirectory = "";
    std::string loadPCDDirectory = "";
    nh.param<std::string>( "lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/" );
    nh.param<std::string>( "lio_sam/loadPCDDirectory", loadPCDDirectory, "/Downloads/LOAM/" );

    // gpsPath
    pcl::PointCloud<PointType>::Ptr gpsPath( new pcl::PointCloud<PointType> );
    if ( pcl::io::loadPCDFile<PointType>( savePCDDirectory + "gpsPath.pcd", *gpsPath ) == -1 )
    {
        PCL_ERROR( "无法读取文件 gpsPath.pcd" );
        return -1;
    }

    // lidarPath
    pcl::PointCloud<PointType>::Ptr lidarPath( new pcl::PointCloud<PointType> );
    if ( pcl::io::loadPCDFile<PointType>( savePCDDirectory + "trajectory.pcd", *lidarPath ) == -1 )
    {
        PCL_ERROR( "无法读取文件 gpsPath.pcd" );
        return -1;
    }

    for ( auto &i : gpsPath->points )
    {
        i.z = 0;
    }
    for ( auto &i : lidarPath->points )
    {
        i.z = 0;
    }

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputSource( gpsPath );
    icp.setInputTarget( lidarPath );
    //设置对应点对之间的最大距离（此值对配准结果影响较大）。
    icp.setMaxCorrespondenceDistance( 10 );
    // 设置两次变化矩阵之间的差值（一般设置为1e-10即可）；步长
    icp.setTransformationEpsilon( 1e-10 );
    // 设置收敛条件是均方误差和小于阈值， 停止迭代；
    icp.setEuclideanFitnessEpsilon( 0.001 );

    //最大迭代次数，icp是一个迭代的方法，最多迭代这些次（若结合可视化并逐次显示，可将次数设置为1）；
    icp.setMaximumIterations( 2000 );

    // 运行ICP匹配
    pcl::PointCloud<PointType>::Ptr aligned( new pcl::PointCloud<PointType> );
    icp.align( *aligned );

    if ( icp.hasConverged() )
    {
        std::cout << "ICP 匹配成功，得分为：" << icp.getFitnessScore() << std::endl;
        std::cout << "变换矩阵为：" << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
    }
    else
    {
        std::cout << "ICP 匹配失败" << std::endl;
        return -1;
    }

    *aligned = *aligned + *lidarPath;

    pcl::io::savePCDFileBinary( savePCDDirectory + "icpRsut.pcd", *aligned );

    ros::shutdown();

    return 0;
}
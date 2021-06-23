#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>

#include "boost/date_time/posix_time/posix_time.hpp"
#include <thread>

using pcl::NormalEstimation;
using pcl::search::KdTree;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//Point cloud visualization
void visualize_pcd(PointCloud::Ptr pcd_src,
                   PointCloud::Ptr pcd_tgt,
                   PointCloud::Ptr pcd_final)
{
    //int vp_1, vp_2;
    // Create a PCLVisualizer object
    pcl::visualization::PCLVisualizer viewer("registration Viewer");
    // viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    // viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 0, 0, 255);
    // viewer.addPointCloud(pcd_src, src_h, "source cloud");
    viewer.addPointCloud(pcd_tgt, tgt_h, "tgt cloud");
    viewer.addPointCloud(pcd_final, final_h, "final cloud");
    //viewer.addCoordinateSystem(1.0);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        // std::this_thread::sleep_for(boost::posix_time::microseconds(100000));
    }
}

//Calculate the rotation angle from the rotation and translation matrix
void matrix2angle(Eigen::Matrix4f &result_trans, Eigen::Vector3f &result_angle)
{
    double ax, ay, az;
    if (result_trans(2, 0) == 1 || result_trans(2, 0) == -1)
    {
        az = 0;
        double dlta;
        dlta = atan2(result_trans(0, 1), result_trans(0, 2));
        if (result_trans(2, 0) == -1)
        {
            ay = M_PI / 2;
            ax = az + dlta;
        }
        else
        {
            ay = -M_PI / 2;
            ax = -az + dlta;
        }
    }
    else
    {
        ay = -asin(result_trans(2, 0));
        ax = atan2(result_trans(2, 1) / cos(ay), result_trans(2, 2) / cos(ay));
        az = atan2(result_trans(1, 0) / cos(ay), result_trans(0, 0) / cos(ay));
    }
    result_angle << ax, ay, az;
}

int main(int argc, char **argv)
{
    //Load point cloud file
    PointCloud::Ptr cloud_src_o(new PointCloud); //Origin cloud, to be registered
    pcl::io::loadPCDFile("../data/STN6xyzi.txt.pcd", *cloud_src_o);
    // pcl::io::loadPCDFile("../data/STN6xyzi.txt.pcd", *cloud_src_o);
    PointCloud::Ptr cloud_tgt_o(new PointCloud); //Target point cloud
    pcl::io::loadPCDFile("../data/STN7xyzi.txt.pcd", *cloud_tgt_o);
    // pcl::io::loadPCDFile("../data/STN7xyzi.txt.pcd", *cloud_tgt_o);
    // pcl::io::loadPCDFile("../data/transformed_cloud.pcd", *cloud_tgt_o);
    std::cout << "load data finish" << std::endl;

    //kevin trans cloud
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    // Define a 2.5 m translation on the X axis.
    transform_2.translation() << 0, -5.5, 0.0;
    // the same rotation as before; rotate theta radians on the Z axis
    transform_2.rotate(Eigen::AngleAxisf(-0.18, Eigen::Vector3f::UnitZ()));
    // print transformation matrix
    printf("\nMethod #2: using an Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_tgt_o, *cloud_tgt_o, transform_2);

    //icp registration
    PointCloud::Ptr icp_result(new PointCloud);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_src_o);
    icp.setInputTarget(cloud_tgt_o);

    int iterations = 100;
    icp.setTransformationEpsilon (1e-10);   //Set the minimum conversion difference for the termination condition
	icp.setMaxCorrespondenceDistance(100000); //Set the maximum distance between corresponding point pairs (this value has a greater impact on the registration result).
	icp.setEuclideanFitnessEpsilon(0.1);  //Set the convergence condition that the sum of the mean square error is less than the threshold, and stop the iteration;
	icp.setMaximumIterations(iterations); //Maximum number of iterations, icp is an iterative method, at most these times (if combined with visualization and display successively, the number can be set to 1);  
	
    //Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    //Perform alignment
    std::cout << "icp align ..." << std::endl;
    icp.align(*output_cloud);
    std::cout << "icp align finish" << std::endl;

    std::cout << "ICP has converged:" << icp.hasConverged()
              << " score: " << icp.getFitnessScore() << std::endl;
    Eigen::Matrix4f icp_trans;
    icp_trans = icp.getFinalTransformation();
    //cout<<"ransformationProbability"<<icp.getTransformationProbability()<<endl;
    std::cout << icp_trans << endl;

    pcl::transformPointCloud(*cloud_src_o, *output_cloud, icp_trans);

    //Visualization
    visualize_pcd(cloud_src_o, cloud_tgt_o, output_cloud);
    return (0);
}

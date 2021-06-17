#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/time.h> // There are many functions for calculating the running time of the program in pcl, among which the time of the console is used to calculate
using namespace std;
int main(int argc, char **argv)
{
	pcl::console::TicToc time;
	// Load the first scan point cloud data as the target cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("data/STN6xyzi.pcd", *target_cloud);

	// Load the second scan point cloud data obtained from the new perspective as the source point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("data/STN7xyzi.pcd", *input_cloud);

	cout << "Read from the target point cloud" << target_cloud->size() << "Points" << endl;
	cout << "Read from the source point cloud" << input_cloud->size() << "Points" << endl;
	time.tic();
	//Initialize the ICP object
	int iterations = 35;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; // Set ICP related parameters according to the scale of the input data
	icp.setInputCloud(input_cloud);								  //Source point cloud
	icp.setInputTarget(target_cloud);							  //Target point cloud
	icp.setTransformationEpsilon(1e-10);						  //Set the minimum conversion difference for the termination condition
	icp.setMaxCorrespondenceDistance(100);						  //Set the maximum distance between corresponding point pairs (this value has a greater impact on the registration result).
	icp.setEuclideanFitnessEpsilon(0.1);						  //Set the convergence condition that the sum of the mean square error is less than the threshold, and stop the iteration;
	icp.setMaximumIterations(iterations);						  //Maximum number of iterations, icp is an iterative method, at most these times (if combined with visualization and display successively, the number can be set to 1);

	// Calculate the required rigid body transformation to match the input source point cloud to the target point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align(*output_cloud);
	cout << "Applied " << 35 << " ICP iterations in " << time.toc() << " ms" << endl;
	cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
	cout << "Transformation matrix:\n"
		 << icp.getFinalTransformation() << endl;
	// Use the created transform to transform the input point cloud
	pcl::transformPointCloud(*input_cloud, *output_cloud, icp.getFinalTransformation());

	// Save the converted source point cloud as the final transformed output
	//  pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

	// Initialize the point cloud visualization object
	boost::shared_ptr<pcl::visualization::PCLVisualizer>
		viewer_final(new pcl::visualization::PCLVisualizer("Registration Results"));
	viewer_final->setBackgroundColor(0, 0, 0); //Set the background color to black

	// Color and visualize the target point cloud (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		target_color(target_cloud, 255, 0, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
												   1, "target cloud");
	// Color and visualize the source point cloud (blue).
	/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		input_color(input_cloud, 0, 0, 255);
	viewer_final->addPointCloud<pcl::PointXYZ>(input_cloud, input_color, "input cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		1, "input cloud");
		*/
	// Color the converted source point cloud (green) and visualize it.
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
		output_color(output_cloud, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
												   1, "output cloud");

	while (!viewer_final->wasStopped())
	{
		viewer_final->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return (0);
}

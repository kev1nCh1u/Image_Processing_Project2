
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h> // pcl::transformPointCloud uses this header file
#include <pcl/visualization/pcl_visualizer.h>

// helper function
void showHelp(char *program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}

// main function
int main(int argc, char **argv)
{

    // If you do not enter the expected parameters, the program will display help
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        showHelp(argv[0]);
        return 0;
    }

    // Find point cloud data files (.PCD|.PLY) from the main function parameters
    std::vector<int> filenames;
    bool file_is_pcd = false;

    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");

    if (filenames.size() != 1)
    {
        filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

        if (filenames.size() != 1)
        {
            showHelp(argv[0]);
            return -1;
        }
        else
        {
            file_is_pcd = true;
        }
    }

    // Load point cloud data files
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (file_is_pcd)
    {
        if (pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud) < 0)
        {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                      << std::endl;
            showHelp(argv[0]);
            return -1;
        }
    }
    else
    {
        if (pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud) < 0)
        {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                      << std::endl;
            showHelp(argv[0]);
            return -1;
        }
    }

    /* Tip: How the transformation matrix works:
                       |-------> Transformation matrix column
    | 1 0 0 x |  \
         | 0 1 0 y | }-> The left side is a 3rd order unit array (no rotation)
    | 0 0 1 z |  /
         | 0 0 0 1 | -> This line is not used (this line holds 0,0,0,1)
         Method One #1: Using Matrix4f
         This is a "manual method" that can be perfectly understood but is error-prone!
  */
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    float theta = M_PI / 4; // radians angle
    transform_1(0, 0) = cos(theta);
    transform_1(0, 1) = -sin(theta);
    transform_1(1, 0) = sin(theta);
    transform_1(1, 1) = cos(theta);
    // (row, column)

    // Define a 2.5 m translation on the X axis.
    transform_1(0, 3) = 2.5;

    // print transformation matrix
    printf("Method #1: using a Matrix4f\n");
    std::cout << transform_1 << std::endl;

    /* Method 2 #2: Using Affine3f
         This method is simple and not easy to make mistakes.
  */
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    // Define a 2.5 m translation on the X axis.
    transform_2.translation() << 2.5, 0.0, 0.0;

    // the same rotation as before; rotate theta radians on the Z axis
    transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

    // print transformation matrix
    printf("\nMethod #2: using an Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;

    // Perform the transformation and save the result in the newly created transformed_cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // can use transform_1 or transform_2; t they are the same
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

    // Visualization
    // Visualizes the original point cloud as white, the transformed point cloud is red, and sets the axis, background color, and point display size.
    printf("\nPoint cloud colors :  white  = original point cloud\n"
           "                        red  = transformed point cloud\n");
    pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

    // Define R, G, B colors for point clouds
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
    // Output point cloud to viewer, use color management
    viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // red
    viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Set the background to dark gray
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    //viewer.setPosition(800, 400); // Set the window position

    while (!viewer.wasStopped())
    { // The window will always be displayed until the "q" key is pressed
        viewer.spinOnce();
    }

    return 0;
}
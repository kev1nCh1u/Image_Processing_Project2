#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;
using namespace pcl;

typedef struct tagPOINT_3D
{
	double x; //mm world coordinate x
	double y; //mm world coordinate y
	double z; //mm world coordinate z
	double r;
} POINT_WORLD;

int main(int argc, char *argv[])
{
	string filePath;

	printf ("argc: %d\n", argc);
	if (argc < 2)
  	{
  	  printf ("input a txt file.\n");
  	  PCL_ERROR ("Provide one txt file.\n");
  	  return (-1);
  	}
	else
	{
		// filePath = "../data/STN7xyzi.txt";
		filePath = argv[1];
		cout << "filePath: " << filePath << endl;
	}

	

	/////Load txt data
	int number_Txt;
	FILE *fp_txt;
	tagPOINT_3D TxtPoint;
	double empty;
	vector<tagPOINT_3D> m_vTxtPoints;
	fp_txt = fopen((filePath).c_str(), "r"); //The location of the file filled in this place
	if (fp_txt)
	{
		while (fscanf(fp_txt, "%lf %lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z, &empty) != EOF)
		{
			m_vTxtPoints.push_back(TxtPoint);
		}
	}
	else
		cout << "Failed to load txt data!" << endl;
	number_Txt = m_vTxtPoints.size();
	pcl::PointCloud<pcl::PointXYZ> cloud;

	// Fill in the cloud data
	cloud.width = number_Txt;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);
	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		cloud.points[i].x = m_vTxtPoints[i].x;
		cloud.points[i].y = m_vTxtPoints[i].y;
		cloud.points[i].z = m_vTxtPoints[i].z;
	}
	pcl::io::savePCDFileASCII(filePath + ".pcd", cloud); //This place fills the output path
	std::cerr << "Saved " << cloud.points.size() << " data points to txt2pcd.pcd." << std::endl;
	for (size_t i = 0; i < cloud.points.size(); ++i)
		std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

	std::cout << "save at" << filePath << ".pcd" << std::endl;

	return 0;
}
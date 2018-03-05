#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <string>
#include <fstream>
using namespace std;

void loadData(string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	ifstream data_infile(path.c_str());
	float x, y, z;
	while(data_infile >> x >> y >> z)
	{
		pcl::PointXYZ pt(x,y,z);
		cloud->points.push_back(pt);
		cout<<x<<" "<<y<<" "<<z<<endl;
	}
	data_infile.close();
	return;
}

void loadNorm(string path, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	ifstream norm_infile(path.c_str());
	float x, y, z, dummy;
	while(norm_infile >> x >> y >> z)
	{
		pcl::Normal pt(x,y,z);
		cloud_normals->points.push_back(pt);
		cout<<x<<" "<<y<<" "<<z<<endl;
	}
	norm_infile.close();
	return;
}

int main (int argc, char** argv)
{
    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	string data_txt_path = "data.txt";
	string norm_txt_path = "norm.txt";
	loadData(data_txt_path, cloud);
	loadNorm(norm_txt_path, cloud_normals);
	cout<<cloud->size()<<endl;
	cout<<cloud_normals->size()<<endl;
	// visualize normals
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0,0,255);
	viewer.setBackgroundColor (1, 1, 1);
	viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals, 1, 0.05, "norm");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.0, 0.8, "norm");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "norm");
	viewer.addPointCloud<pcl::PointXYZ>(cloud, single_color, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "cloud");
	while (!viewer.wasStopped ())
	{
	  viewer.spinOnce ();
	}
	return 0;
}

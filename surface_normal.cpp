#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
using namespace std;
void loadData(string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	ifstream data_infile(path.c_str());
	float x, y, z;
	while(data_infile >> x >> y >> z)
	{
		pcl::PointXYZ pt(x,y,z);
		cloud->points.push_back(pt);
		// cout<<x<<" "<<y<<" "<<z<<endl;
	}
	data_infile.close();
	return;
}


int main (int argc, char** argv)
{
    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPCDFile (argv[1], *cloud);
    string data_txt_path = "ply_data_all_10_xyz_norm_origin_0_cloud.txt";
	loadData(data_txt_path, cloud);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);
	
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	
	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	
	// Use all neighbors in a sphere of radius 3cm
	float th_r = atof(argv[1]);
	cout<<"th_r is:"<<th_r<<endl;
	ne.setRadiusSearch (th_r);

	// ne.setKSearch(atoi(argv[1]));
	// Compute the features
	ne.compute (*cloud_normals);	
	std::cout<<cloud_normals->size()<<std::endl;
	// visualize normals
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor (0.0, 0.0, 0.5);

	
	viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals, 1, 0.02, "normal");
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	while (!viewer.wasStopped ())
	{
	  viewer.spinOnce ();
	}
	return 0;
}

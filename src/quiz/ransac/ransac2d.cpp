/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

#include <cmath>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../../src/sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

pcl::PointXYZ cross_product(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
  pcl::PointXYZ result;
  result.x = p1.y * p2.z - p1.z * p2.y;
  result.y = p1.z * p2.x - p1.x * p2.z;
  result.z = p1.x * p2.y - p1.y * p2.x;
  return result;
}

float dot_product(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
  return  p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	size_t max_num_inliers = 0;
	// For max iterations 
	while(maxIterations--) {
		std::unordered_set<int> inliers;
		// Randomly sample subset
		while(inliers.size() < 3) {
			inliers.insert(rand() % cloud->points.size());
		}

		auto itr = inliers.begin();
		auto x1 = cloud->points[*itr].x;
		auto y1 = cloud->points[*itr].y;
		auto z1 = cloud->points[*itr].z;
		itr++;
		auto x2 = cloud->points[*itr].x;
		auto y2 = cloud->points[*itr].y;
		auto z2 = cloud->points[*itr].z;
		itr++;
		auto x3 = cloud->points[*itr].x;
		auto y3 = cloud->points[*itr].y;
		auto z3 = cloud->points[*itr].z;

		auto v1 = pcl::PointXYZ{x2 - x1, y2 - y1, z2 - z1};
		auto v2 = pcl::PointXYZ{x3 - x1, y3 - y1, z3 - z1};

		auto plane_normal = cross_product(v1, v2);

		auto a = plane_normal.x;
		auto b = plane_normal.y;
		auto c = plane_normal.z;
		auto d = -1 * dot_product(plane_normal, pcl::PointXYZ{x1, y1, z1});

		for(size_t idx = 0; idx < cloud->points.size(); idx++) {
			const auto& point = cloud->points[idx];
			auto distace = std::abs(a * point.x + b * point.y + c * point.z + d) / std::sqrt(std::pow(a, 2) + std::pow(b, 2) +  std::pow(c, 2));
			if(distace < distanceTol) {
				inliers.insert(idx);
			}
		}

		if(inliers.size() > max_num_inliers) {
			max_num_inliers = inliers.size();
			inliersResult.clear();
			inliersResult = inliers;
		}
	}

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}

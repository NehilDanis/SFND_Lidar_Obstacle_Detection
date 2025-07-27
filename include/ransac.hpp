#pragma once

#include "pointUtils.h"
#include <unordered_set>
#include <cmath>

namespace utils {

template <HasXYZ PointT>
PointT cross_product(const PointT& p1, const PointT& p2) {
  PointT result;
  result.x = p1.y * p2.z - p1.z * p2.y;
  result.y = p1.z * p2.x - p1.x * p2.z;
  result.z = p1.x * p2.y - p1.y * p2.x;
  return result;
}

template <HasXYZ PointT>
float dot_product(const PointT& p1, const PointT& p2) {
  return  p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}

template <HasXYZ PointT>
std::unordered_set<int> ransac_self_impl(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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

}
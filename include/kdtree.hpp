#pragma once
#include <memory>
#include <vector>
#include <ranges> 
#include "pointUtils.h"

namespace utils {

template <HasXYZ PointT>
float getCoord(const PointT& pt, size_t axis_idx) {
    switch (axis_idx) {
        case 0: return pt.x;
        case 1: return pt.y;
        case 2: return pt.z;
        default: throw std::out_of_range("Invalid axis index");
    }
}

// Structure to represent node of kd tree
template <HasXYZ PointT>
struct Node
{
	PointT point;
	int id;
	std::unique_ptr<Node<PointT>> left{nullptr};
	std::unique_ptr<Node<PointT>> right{nullptr};

	Node(PointT arr, int setId)
	:	point(arr), id(setId)
	{}

	~Node()
	{
		// do nothing
	}
};


template <HasXYZ PointT>
struct KdTree
{
	std::unique_ptr<Node<PointT>> root{nullptr};

	KdTree()
	{}

	~KdTree()
	{
	}

	void insertHelper(std::unique_ptr<Node<PointT>>& node, PointT point, int id, size_t depth) {
		if(not node) {
			node = std::make_unique<Node<PointT>>(point, id);
			return;
		}

		size_t axis_idx = depth % 3;
		if(getCoord(point, axis_idx) < getCoord(node->point, axis_idx)) {
			insertHelper(node->left, point, id, depth + 1);
		}
		else {
			insertHelper(node->right, point, id, depth + 1);
		}
	}

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, point, id, 0U);
	}

	auto search_is_in_box(const std::array<pcl::PointXYZ, 2>& area, const PointT& point) -> bool {
		const auto& min_point = area.at(0);
		const auto& max_point = area.at(1);

		if(min_point.x <= point.x && point.x <= max_point.x) {
			if(min_point.y <= point.y && point.y <= max_point.y) {
				if(min_point.z <= point.z && point.z <= max_point.z) {
                    return true;
                }
			}
		}
		return false;

	}

	void search_helper(const std::unique_ptr<Node<PointT>>& node, const std::array<pcl::PointXYZ, 2>& area, std::vector<int>& ids, size_t depth) {

		if(not node) return;
		// check if the node is in the target_area
		if(search_is_in_box(area, node->point)) {
			// add it to the ids list
			ids.emplace_back(node->id);
		}

		const auto& area_min_pt = area.at(0); 
		const auto& area_max_pt = area.at(1); 

		// decide on the next search space
		auto axis = depth % 3; // 0 for x axis 1 for y axis 2 for axis z
		if(getCoord(area_max_pt, axis) < getCoord(node->point, axis)) {
			// box lays on the left of the axis node->left
			search_helper(node->left, area, ids, depth+1);
		}
		else if(getCoord(area_min_pt, axis) > getCoord(node->point, axis)) {
			// box lays on the right of the axis node->right
			search_helper(node->right, area, ids, depth+1);
		}
		else {
			// box lay in the middle both left and right
			search_helper(node->left, area, ids, depth+1);
			search_helper(node->right, area, ids, depth+1);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids{};
		std::array<pcl::PointXYZ, 2> area{pcl::PointXYZ{target.x - distanceTol, target.y - distanceTol, target.z - distanceTol},
		 pcl::PointXYZ{target.x + distanceTol, target.y + distanceTol, target.z + distanceTol}};
		search_helper(root, area, ids, 0);
		return ids;
	}
};

template <HasXYZ PointT>
auto proximity(size_t curr_idx, typename pcl::PointCloud<PointT>::Ptr point_cloud, const std::unique_ptr<KdTree<PointT>>& tree, float distanceTol, std::vector<int>& cluster, std::vector<bool>& is_processed) -> void {
	const auto& points = point_cloud->points;
	is_processed.at(curr_idx) = true;
	cluster.emplace_back(curr_idx);

	const auto& point = points.at(curr_idx);

	// use kd search and get all the nearest points
	const auto& nearest_indices = tree->search(point, distanceTol);
	// then for each unprocessed point call the proximity
	for(const auto& idx : nearest_indices) {
		if(not is_processed.at(idx)) {
			proximity(static_cast<size_t>(idx), point_cloud, tree, distanceTol, cluster, is_processed);
		}
	}

}

template <HasXYZ PointT>
std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr point_cloud, const std::unique_ptr<KdTree<PointT>>& tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	const auto points = point_cloud->points;

	std::vector<std::vector<int>> clusters;
	std::vector<bool> is_processed(points.size(), false);
	auto indices_view = std::views::iota(0U, points.size());

	std::ranges::for_each(indices_view, [&](size_t idx){
		if(not is_processed.at(idx)) {
			std::vector<int> cluster{};
			proximity<PointT>(idx, point_cloud, tree, distanceTol, cluster, is_processed);
			clusters.emplace_back(cluster);
		}
	});
	
 
	return clusters;

}

}
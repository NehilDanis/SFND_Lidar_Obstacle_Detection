/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <memory>


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	std::unique_ptr<Node> left{nullptr};
	std::unique_ptr<Node> right{nullptr};

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId)
	{}

	~Node()
	{
		// do nothing
	}
};

struct KdTree
{
	std::unique_ptr<Node> root{nullptr};

	KdTree()
	{}

	~KdTree()
	{
	}

	void insertHelper(std::unique_ptr<Node>& node, std::vector<float> point, int id, size_t depth) {
		if(not node) {
			node = std::make_unique<Node>(point, id);
			return;
		}

		size_t axis_idx = depth % 2;
		if(point.at(axis_idx) < node->point.at(axis_idx)) {
			insertHelper(node->left, point, id, depth + 1);
		}
		else {
			insertHelper(node->right, point, id, depth + 1);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, point, id, 0U);
	}

	auto search_is_in_box(const std::array<std::vector<float>, 2>& area, const std::vector<float>& point) -> bool {
		const auto& min_point = area.at(0);
		const auto& max_point = area.at(1);

		if(min_point.at(0) <= point.at(0) && point.at(0) <= max_point.at(0)) {
			if(min_point.at(1) <= point.at(1) && point.at(1) <= max_point.at(1)) {
				return true;
			}
		}
		return false;

	}

	void search_helper(const std::unique_ptr<Node>& node, const std::array<std::vector<float>, 2>& area, std::vector<int>& ids, size_t depth) {

		if(not node) return;
		// check if the node is in the target_area
		if(search_is_in_box(area, node->point)) {
			// add it to the ids list
			ids.emplace_back(node->id);
		}

		const auto& area_min_pt = area.at(0); 
		const auto& area_max_pt = area.at(1); 

		// decide on the next search space
		auto axis = depth % 2; // 0 for x axis 1 for y axis
		if(area_max_pt.at(axis) < node->point.at(axis)) {
			// box lays on the left of the axis node->left
			search_helper(node->left, area, ids, depth+1);
		}
		else if(area_min_pt.at(axis) > node->point.at(axis)) {
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
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids{};
		std::array<std::vector<float>, 2> area{std::vector<float>{target.at(0) - distanceTol, target.at(1) - distanceTol},
		 std::vector<float>{target.at(0) + distanceTol, target.at(1) + distanceTol}};
		search_helper(root, area, ids, 0);
		return ids;
	}
};





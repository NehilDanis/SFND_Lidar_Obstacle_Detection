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

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
};





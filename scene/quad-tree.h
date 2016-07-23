//////////////////////////////////////////////////////
// @fileoverview Defination of quad tree to manage
//				 physics space.
// @author	ysd
//////////////////////////////////////////////////////

#ifndef _QUAD_TREE_H_
#define _QUAD_TREE_H_

#include <vector>
#include <memory>

#include "../math/vector_2.h"
#include "../math/bound.h"
#include "../colliders/collider.h"

namespace ysd_phy_2d
{

//////////////////
// Space partition
//   1   |   0
// --------------
//   2   |   3
//////////////////

// Node structure in the quad tree.
struct TreeNode
{
	// Rectangle bound.
	Bound bound;

	// 4 children.
	std::shared_ptr<TreeNode> children[4];

	// Colliders in this node.
	std::vector<std::shared_ptr<BaseCollider>> colliders;

};

class QuadTree final
{
public:

	QuadTree(uint8_t max_deep)
		:root_(new TreeNode), max_deep_(max_deep)
	{}

	// Insert a new collider in the tree.
	// @param[in] 	id  		The collider's id.
	// @param[in] 	collider 	The collider need to insert in.
	void Insert(uint16_t id, const BaseCollider& collider);

	// Remove a collider from the tree with the given id and its AABB bound.
	// @param[in]	id		The collider's id.
	// @param[in]	bound	The collider's AABB bound.
	void Remove(uint16_t id, const Bound& bound);

private:
	void InsertNode(uint16_t id, uint8_t deep, const BaseCollider& collider, std::shared_ptr<TreeNode> root);

	// If remove the node successful, return true.
	bool RemoveNode(uint16_t id, const Bound& bound, std::shared_ptr<TreeNode> root);

	// Create bound for four quadrants.
	// @param[in]	qr 	[0, 3].
	inline const Bound CreateBound(const TreeNode* root, uint8_t qr) const;

	// Create 1/4 rect in root's bound.
	inline void CreateChild(std::shared_ptr<TreeNode> root, uint8_t qr, const Bound& bound);

	std::shared_ptr<TreeNode> root_;

	uint8_t max_deep_;
};
}

#endif
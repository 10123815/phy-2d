//////////////////////////////////////////////////////////
// @fileoverview Defination of quad tree to manage the node
//				 in physics world.
// @author	ysd
//////////////////////////////////////////////////////////

#ifndef _QUAD_TREE_H_
#define _QUAD_TREE_H_

#include <vector>
#include <memory>

#include "../math/vector_2.h"
#include "../math/bound.h"
#include "../colliders/collider.h"
#include "../common/un-copy-move-interface.h"

namespace ysd_phy_2d
{

/////////////////////////////////////////////////////////
// A QuadTree hold all colliders in a scene. It can not be
// coped.
// Space partition
//   1   |   0
// --------------
//   2   |   3
/////////////////////////////////////////////////////////
class QuadTree final : public IUncopyable
{
public:

	// Node structure in the quad tree.
	struct TreeNode : public IUnCopyMovable
	{
		// Rectangle bound.
		Bound bound;

		// The tree's 4 children.
		TreeNode* children[4];

		// Colliders in this node.
		std::vector<std::shared_ptr<BaseCollider>> colliders;

	};

	QuadTree(float width, float length, const Vector2& center, uint8_t deep = 8)
		:root_(new TreeNode), max_deep_(deep)
	{
		Vector2 max(width / 2, length / 2);
		root_->bound.max = max + center;
		root_->bound.min = center - max;
	}

	QuadTree(float width, float length, uint8_t deep = 8)
		:root_(new TreeNode), max_deep_(deep)
	{
		Vector2 max(width / 2, length / 2);
		root_->bound.max = max;
		root_->bound.min = -max;
	}

	// Move construct
	QuadTree(QuadTree&& other)
		:max_deep_(other.max_deep_)
	{
		root_.reset(other.root_.release());
	}

	// Insert a new collider in the tree.
	// @param[in] 	id  		The collider's id.
	// @param[in] 	collider 	The collider need to insert in.
	void Insert(const std::shared_ptr<BaseCollider> collider);

	// Remove a collider from the tree with the given id and its AABB bound.
	// @param[in]	id		The collider's id.
	// @param[in]	bound	The collider's AABB bound.
	void Remove(uint16_t id, const Bound& bound);

	void set_max_deep(uint8_t value) { max_deep_ = value; }

private:
	void InsertNode(const std::shared_ptr<BaseCollider> collider, TreeNode* root, uint8_t deep = 0);

	// If remove the node successful, return true.
	bool RemoveNode(uint16_t id, const Bound& bound, TreeNode* root);

	// A collider move for a movement.
	bool MoveNode(const std::shared_ptr<BaseCollider> collider, const Vector2 & movement, TreeNode* root);

	// A collider scale.
	bool ScaleNode(const std::shared_ptr<BaseCollider> collider, const float scale, TreeNode* root);

	// A polygon collider rotate.
	bool RotateNode(const std::shared_ptr<PolygonCollider> collider, const float angle, TreeNode* root);

	// Create bound for four quadrants.
	// @param[in]	qr 	[0, 3].
	inline const Bound CreateBound(const TreeNode* root, uint8_t qr) const;

	// Create 1/4 rect in root's bound.
	inline void CreateChild(TreeNode* root, uint8_t qr, const Bound& bound);

	std::unique_ptr<TreeNode> root_;

	uint8_t max_deep_;

};
}

#endif
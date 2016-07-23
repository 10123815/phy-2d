#include "quad-tree.h"

using namespace ysd_phy_2d;

void QuadTree::Insert(uint16_t id, const BaseCollider& collider)
{
	InsertNode(id, 0, collider, root_);
}

void QuadTree::InsertNode(uint16_t id, uint8_t deep, const BaseCollider& collider, std::shared_ptr<TreeNode> root)
{
	// The collider is in the rectangle bound.
	if (BoundinBound(root->bound, collider.bound()))
	{
		// If reach the max deep
		if (deep >= max_deep_)
		{
			// The collider is belong to this root area.
			root->colliders.push_back(std::shared_ptr<BaseCollider>(&collider));
			return;
		}

		// If the axis go across the bound.
		const Bound& root_bound = root->bound;
		Vector2 center = (root_bound.max + root_bound.min) / 2;
		if (VertivalAxisCrossBound(collider.bound(), center.x()) ||
			HorizentalAxisCrossBound(collider.bound(), center.y()))
		{
			// The collider is belong to this root area.
			root->colliders.push_back(std::shared_ptr<BaseCollider>(&collider));
			return;
		}

		// Insert the node in one of four child nodes.
		for (uint8_t i = 0; i < 4; ++i)
		{
			std::shared_ptr<TreeNode>* const children = root->children;
			if (children[i] != nullptr)
			{
				if (BoundinBound(children[i]->bound, collider.bound()))
				{
					// The collider is in child's bound.
					InsertNode(id, deep + 1, collider, children[i]);
					return;
				}
				else
				{
					// The collider is out of the bound.
					continue;
				}
			}
			else
			{
				Bound bound = this->CreateBound(root.get(), i);
				if (BoundinBound(bound, collider.bound()))
				{
					// The collider is in the bound.
					// The child is null, create it first.
					CreateChild(root, i, bound);
					InsertNode(id, deep + 1, collider, children[i]);
					return;
				}
				else
				{
					// The collider is out of the bound.
					continue;
				}
			}
		}
	}
}

inline const Bound QuadTree::CreateBound(const TreeNode* root, uint8_t qr) const
{
	assert(qr <= 3);

	// If collider is in the first quadrant, need to do nothing.
	static const int8_t opx[] = { 0, 1, 1, 0 };
	static const int8_t opy[] = { 0, 0, 1, 1 };

	Bound bound;
	// The first quadrant.
	bound.max = root->bound.max;
	bound.min = (root->bound.max + root->bound.min) / 2;

	Vector2 right = Vector2(bound.min.x() - bound.max.x(), 0);
	Vector2 down = Vector2(0, bound.min.y() - bound.max.y());

	bound.min += right * opx[qr];
	bound.min += down * opy[qr];
	bound.max += right * opx[qr];
	bound.max += down * opy[qr];

	return bound;
}

inline void ysd_phy_2d::QuadTree::CreateChild(std::shared_ptr<TreeNode> root, uint8_t qr, const Bound & bound)
{
	(root->children)[qr] = std::make_shared<TreeNode>();
	(root->children)[qr]->bound = bound;
}

void QuadTree::Remove(uint16_t id, const Bound& bound)
{
	this->RemoveNode(id, bound, root_);
}

// Find the node recursively with given id and remove id.
bool QuadTree::RemoveNode(uint16_t id, const Bound& bound, std::shared_ptr<TreeNode> root)
{
	if (root == nullptr)
		return false;

	// Find the collider in this tree node.
	std::vector<std::shared_ptr<BaseCollider>>& colliders = root->colliders;
	for (std::size_t i = 0, l = colliders.size(); i < l; i++)
	{
		if (colliders[i]->id() == id)
		{
			// We found it! Remove it.
			colliders[i].reset();
			colliders.erase(colliders.cbegin() + i);
			return true;
		}
	}

	// Find the collider in one of the four children.
	for (std::size_t i = 0; i < 4; i++)
	{
		const std::shared_ptr<TreeNode> child = root->children[i];
		if (child != nullptr)
		{
			const Bound& big_bound = root->children[i]->bound;
			if (BoundinBound(big_bound, bound))
			{
				// It is here!! Find it in this child.
				return this->RemoveNode(id, bound, root->children[i]);
			}
		}

	}

	// The node with the given id is not found.
	return false;
}

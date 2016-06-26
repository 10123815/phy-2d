#include "quad-tree.h"

using namespace ysd_phy_2d;

void QuadTree::Insert(uint16_t id, const BaseCollider& collider)
{
	InsertNode(id, 0, collider, root_);
}

void QuadTree::InsertNode(uint16_t id, uint8_t deep, const BaseCollider& collider, std::shared_ptr<TreeNode> root)
{
	// The collider is in the rectangle bound.
	if (BoundinBound(root->bound, collider.bound()) >= 0)
	{
		// If reach the max deep
		if (deep >= max_deep_)
		{
			// The collider is belong to this root area.
			root->colliders.push_back(std::shared_ptr<BaseCollider>(&collider));
			return;
		}

		// If the axis go across the bound.
		Bound& root_bound = root->bound;
		Vector2 center = (root_bound.max + root_bound.min) / 2;
		if (AxisCrossBound(collider.bound(), center.x(), false) ||
		        AxisCrossBound(collider.bound(), center.y()))
		{
			// The collider is belong to this root area.
			root->colliders.push_back(std::shared_ptr<BaseCollider>(&collider));
			return;
		}

		for (uint8_t i = 0; i < 4; ++i)
		{
			if (root->children[i] != nullptr)
			{
				if (BoundinBound(root->children[i]->bound, collider.bound()))
				{
					// The collider is in child's bound.
					InsertNode(id, deep + 1, collider, root->children[i]);
					return;
				}
				else
					// The collider is out of the bound.
					continue;
			}
			else
			{
				Bound bound = CreateBound(root.get(), i);
				if (BoundinBound(bound, collider.bound()))
				{
					// The collider is in the bound.
					// The child is null, create it first.
					CreateQuadrant(root, i, bound);
					InsertNode(id, deep + 1, collider, root->children[i]);
					return;
				}
				else
					// The collider is out of the bound.
					continue;

			}
		}
	}
}

const Bound QuadTree::CreateBound(const TreeNode* root, uint8_t qr) const
{
	assert(qr <= 3);

	// If collider is in the first quadrant, need to do nothing.
	static const int8_t opx[] = {0, 1, 1, 0};
	static const int8_t opy[] = {0, 0, 1, 1};

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

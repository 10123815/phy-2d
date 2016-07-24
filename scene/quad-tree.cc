#include "quad-tree.h"

using namespace ysd_phy_2d;

void QuadTree::Insert(const std::shared_ptr<BaseCollider> collider)
{
	InsertNode(collider, root_.get());
}

// Note that this function must not delete root.
void QuadTree::InsertNode(const std::shared_ptr<BaseCollider> collider, TreeNode* root, uint8_t deep = 0)
{
	// The collider is in the rectangle bound.
	if (BoundinBound(root->bound, collider->bound()))
	{
		// If reach the max deep
		if (deep >= max_deep_)
		{
			// The collider is belong to this root area.
			// Push back the copy of the shared pointer.
			root->colliders.push_back(collider);
			return;
		}

		// If the axis go across the bound.
		const Bound& root_bound = root->bound;
		Vector2 center = (root_bound.max + root_bound.min) / 2;
		if (VertivalAxisCrossBound(collider->bound(), center.x()) ||
			HorizentalAxisCrossBound(collider->bound(), center.y()))
		{
			// The collider is belong to this root area.
			// Push back the copy of the shared pointer.
			root->colliders.push_back(collider);
			return;
		}

		// Insert the node in one of four child nodes.
		for (uint8_t i = 0; i < 4; ++i)
		{
			TreeNode* const* const children = root->children;
			//auto children = root->children;
			if (children[i] != nullptr)
			{
				if (BoundinBound(children[i]->bound, collider->bound()))
				{
					// The collider is in child's bound.
					InsertNode(collider, children[i], deep + 1);
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
				Bound bound = this->CreateBound(root, i);
				if (BoundinBound(bound, collider->bound()))
				{
					// The collider is in the bound.
					// The child is null, create it first.
					CreateChild(root, i, bound);
					InsertNode(collider, children[i], deep + 1);
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

bool ysd_phy_2d::QuadTree::RotateNode(const std::shared_ptr<PolygonCollider> collider, const float angle, TreeNode * root)
{
	return false;
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

inline void ysd_phy_2d::QuadTree::CreateChild(TreeNode* root, uint8_t qr, const Bound & bound)
{
	(root->children)[qr] = new TreeNode;
	(root->children)[qr]->bound = bound;
}

void QuadTree::Remove(uint16_t id, const Bound& bound)
{
	this->RemoveNode(id, bound, root_.get());
}

bool ysd_phy_2d::QuadTree::MoveNode(const std::shared_ptr<BaseCollider> collider, const Vector2 & movement, TreeNode* root)
{
	if (root == nullptr)
		return false;

	// Find the collider in this tree node.
	std::vector<std::shared_ptr<BaseCollider>>& colliders = root->colliders;
	for (std::size_t i, length = colliders.size(); i < length; ++i)
	{
		if (colliders[i] == collider)
		{
			// Move the collider and reset its bound.
			collider->Translate(movement);
			// The collider may no longer belong to this quad.
			if (!BoundinBound(root->bound, collider->bound))
			{
				// Remove it and insert it again.
				colliders.erase(colliders.cbegin() + i);
				this->InsertNode(collider, root);
			}
			return true;
		}
	}

	// Find the collider in one of the four children.
	for (std::size_t i = 0; i < 4; i++)
	{
		TreeNode* child = root->children[i];
		if (child != nullptr)
		{
			const Bound& big_bound = root->children[i]->bound;
			if (BoundinBound(big_bound, collider->bound))
			{
				// It is here!! Find it in this child.
				return this->MoveNode(collider, movement, child);
			}
		}
	}

	// The node with the given id is not found.
	return false;
}

bool ysd_phy_2d::QuadTree::ScaleNode(const std::shared_ptr<BaseCollider> collider, const float scale, TreeNode * root)
{
	return false;
}

// Find the node recursively with given id and remove id.
// Note that this function must not delete root.
bool QuadTree::RemoveNode(uint16_t id, const Bound& bound, TreeNode* root)
{
	if (root == nullptr)
		return false;

	// Find the collider in this tree node.
	std::vector<std::shared_ptr<BaseCollider>>& colliders = root->colliders;
	for (std::size_t i = 0, l = colliders.size(); i < l; i++)
	{
		if (colliders[i]->id() == id)
		{
			// We found it! Delete and remove it.
			colliders[i].reset();
			colliders.erase(colliders.cbegin() + i);
			return true;
		}
	}

	// Find the collider in one of the four children.
	for (std::size_t i = 0; i < 4; i++)
	{
		TreeNode* child = root->children[i];
		if (child != nullptr)
		{
			const Bound& big_bound = root->children[i]->bound;
			if (BoundinBound(big_bound, bound))
			{
				// It is here!! Find it in this child.
				return this->RemoveNode(id, bound, child);
			}
		}
	}

	// The node with the given id is not found.
	return false;
}

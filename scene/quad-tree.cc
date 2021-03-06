#include "quad-tree.h"

using namespace ysd_phy_2d;

void QuadTree::Insert(const std::shared_ptr<BaseCollider> collider)
{
	InsertNode(collider, root_.get());
}

// Note that this function must not delete root.
bool QuadTree::InsertNode(const std::shared_ptr<BaseCollider> collider, TreeNode* root, uint8_t deep = 0)
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
			return true;
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
			return true;
		}

		// Insert the node in one of four child nodes.
		InsertNodeInChildren(collider, root, deep);

	}
	else if (deep == 0 && BoundContactBound(collider->bound, root->bound))
	{
		// The collider is belong to this root area.
		// Push back the copy of the shared pointer.
		root->colliders.push_back(collider);
		return true;
	}

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

// The tree may need update when a node move.
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
				this->InsertNode(collider, root_.get());
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

// @param[in]	deep	In which deep we find the rotated collider.
bool ysd_phy_2d::QuadTree::ScaleNode(const std::shared_ptr<BaseCollider> collider, const Vector2& scale, TreeNode * root, uint8_t deep = 0)
{

	if (root == nullptr)
		return false;

	// Find the colldier in this root.
	std::vector<std::shared_ptr<BaseCollider>>& colliders = root->colliders;
	for (std::size_t i, length = colliders.size(); i < length; ++i)
	{
		if (colliders[i] == collider)
		{
			// Scale the collider
			collider->ScaleFor(scale);

			const Bound& root_bound = root->bound;
			// If the collider became smaller, it may go down to the root's child nodes.
			Vector2 center = (root_bound.max + root_bound.min) / 2;
			if (deep < max_deep_ &&
				!VertivalAxisCrossBound(collider->bound, center.x()) &&
				!HorizentalAxisCrossBound(collider->bound, center.y()))
			{
				// The collider was already leave the root's bound's axis.
				// It may go down into one of the four children of the root.
				colliders.erase(colliders.cbegin() + i);

				// Insert the removed collider to one of the four children.
				return InsertNodeInChildren(collider, root, deep);
			}

			// If the collider became bigger, it may go up to the root's parent node.
			if (!BoundinBound(root_bound, collider->bound))
			{
				// Reinsert the collider.
				colliders.erase(colliders.cbegin() + i);
				return InsertNode(collider, root_.get());
			}
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
				return this->ScaleNode(collider, scale, child, deep + 1);
			}
		}
	}

	return false;
}

// @param[in]	deep	In which deep we find the rotated collider.
bool ysd_phy_2d::QuadTree::RotateNode(const std::shared_ptr<BaseCollider> collider, const float angle, TreeNode * root, uint8_t deep = 0)
{

	if (root == nullptr)
		return false;

	// Find the collider first in this tree node.
	std::vector<std::shared_ptr<BaseCollider>>& colliders = root->colliders;
	for (std::size_t i, length = colliders.size(); i < length; ++i)
	{
		if (colliders[i] == collider)
		{

			// Rotate the collider and reset its bound.
			collider->Rotate(angle);

			const Bound& root_bound = root->bound;
			// The collider may up to the root's father node.
			if (!BoundinBound(root_bound, collider->bound))
			{
				// Reinsert the node.
				colliders.erase(colliders.cbegin() + i);
				return InsertNode(collider, root_.get());
			}

			// The collider may down to this colliders' one child.
			Vector2 center = (root_bound.max + root_bound.min) / 2;
			if (deep < max_deep_ &&
				!VertivalAxisCrossBound(collider->bound, center.x()) &&
				!HorizentalAxisCrossBound(collider->bound, center.y()))
			{
				// The collider was already leave the root's bound's axis.
				// It may go down into one of the four children of the root.
				colliders.erase(colliders.cbegin() + i);

				// Insert the removed collider to one of the four children.
				return InsertNodeInChildren(collider, root, deep);
			}
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
				return this->RotateNode(collider, angle, child, deep + 1);
			}
		}
	}

	return false;

}

bool ysd_phy_2d::QuadTree::InsertNodeInChildren(const std::shared_ptr<ysd_phy_2d::BaseCollider> collider, TreeNode* root, uint8_t deep)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		TreeNode* const* const children = root->children;
		//auto children = root->children;
		if (children[i] != nullptr)
		{
			if (BoundinBound(children[i]->bound, collider->bound()))
			{
				// The collider is in child's bound.
				InsertNode(collider, children[i], deep + 1);
				return true;
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
				return true;
			}
			else
			{
				// The collider is out of the bound.
				continue;
			}
		}
	}
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

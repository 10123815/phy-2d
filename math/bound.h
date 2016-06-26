#ifndef _BOUND_H_
#define _BOUND_H_

#include "vector_2.h"

namespace ysd_phy_2d
{

// AABB of an arbitrary collider.
struct Bound
{
	// Left down point.
	Vector2 min;

	// Right up point.
	Vector2 max;
};

Bound CreateBound(Vector2 center, float length, float height)
{
	Bound b;
	Vector2 v = Vector2(length / 2, height / 2);
	b.min = center - v;
	b.max = center + v;
}

// Space relationship of two bounds.
// @return Return true if small is inside of big.
bool BoundinBound(const Bound& big, const Bound& small)
{
	return big.min < small.min && big.max > small.max;
}

bool BoundContactBound(const Bound& b1, const Bound& b2)
{

}

bool AxisCrossBound(const Bound& bound, const float value, const bool h = true)
{
	return (h && value < bound.max.y() && value > bound.min.y()) ||
	       (!h && value < bound.max.x() && value > bound.min.x());
}

}

#endif
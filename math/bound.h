//////////////////////////////////////////////////////
// @fileoverview Defination of AABB bound.
// @author	ysd
//////////////////////////////////////////////////////

#ifndef _BOUND_H_
#define _BOUND_H_

#include <algorithm>
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

// Space relationship of two bounds.
// @return Return true if small is inside of big.
inline bool BoundinBound(const Bound& big, const Bound& small)
{
	return big.min < small.min && big.max > small.max;
}

// Space relationship of two bounds.
// @return Return true if two bounds were contacted.
inline bool BoundContactBound(const Bound& b1, const Bound& b2)
{
	float min_x = std::max(b1.min.x, b2.min.x);
	float max_x = std::min(b1.max.x, b2.max.x);
	if (min_x < max_x)
	{
		float min_y = std::max(b1.min.y, b2.min.y);
		float max_y = std::min(b1.max.y, b2.max.y);
		return min_y < max_y;
	}
	return false;
}

// @return Return true if the x axis is cross the bound.
inline bool HorizentalAxisCrossBound(const Bound& bound, const float y)
{
	return y <= bound.max.y() && y >= bound.min.y();
}

// @return Return true if the y axis is cross the bound.
inline bool VertivalAxisCrossBound(const Bound& bound, const float x)
{
	return x <= bound.max.x() && x >= bound.min.y();
}

}

#endif
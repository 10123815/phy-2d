#ifndef _TRANSFORM_H_
#define _TRANSFORM_H_

#include <cmath>

#include "../math/vector_2.h"

namespace ysd_phy_2d
{
////////////////////////////////////////////////////////////////
// A Transform class descripe the space transform of a collider.
// Every collider has its own transform includes position,
// rotation and scale. In 2d space, the rotation can represent 
// by a single angle.
////////////////////////////////////////////////////////////////
class Transform
{
public:
	const Vector2 TransformVector(const Vector2& vec)
	{
		Vector2 v;

		// Scale
		float x = vec.x() * scale_.x();
		float y = vec.y() * scale_.y();

		// Rotate. [x*cosA-y*sinA  x*sinA+y*cosA] 
		x = x * cosf(angle_) - y * sinf(angle_);
		y = y * sinf(angle_) + y * cosf(angle_);

		// Move
		x += position_.x();
		y += position_.y();

		return v;

	}

private:
	Vector2 position_;
	Vector2 scale_;
	float angle_;
};
}

#endif
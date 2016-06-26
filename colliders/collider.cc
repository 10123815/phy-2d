#include "collider.h"

using namespace ysd_phy_2d;

Vector2 PolygonCollider::TransformVector(const Vector2& vec) const
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

void PolygonCollider::ResetBound(bool transformed) const
{
	float xmin, xmax, ymin, ymax;
	const std::vector<Vector2>& verts = pshared_shape_->vertices();
	std::size_t count = verts.size();
	std::size_t ini = 0;
	Vector2 p0;
	Vector2 p1;

	// Initialize.
	if (count % 2 == 0)
	{
		ini = 2;

		if (transformed)
		{
			p0 = TransformVector(verts[0]);
			p1 = TransformVector(verts[1]);
		}
		else
		{
			p0 = verts[0];
			p1 = verts[1];
		}

		xmin = std::min(p0.x(), p1.x());
		xmax = std::max(p0.x(), p1.x());

		ymin = std::min(p0.y(), p1.y());
		ymax = std::max(p0.y(), p1.y());
	}
	else
	{
		ini = 1;

		if (transformed)
			p0 = TransformVector(verts[0]);
		else
			p0 = verts[0];

		xmin = xmax = p0.x();
		ymin = ymax = p0.y();
	}

	// Find the bound points.
	for (std::size_t i = ini; i < count; i += 2)
	{
		if (transformed)
		{
			p0 = TransformVector(verts[ini]);
			p1 = TransformVector(verts[ini + 1]);
		}
		else
		{
			p0 = verts[ini];
			p1 = verts[ini + 1];
		}

		float xb = p1.x(), xs = p0.x(), yb = p1.y(), ys = p0.y();
		if (xb < xs)
		{
			xb = p0.x();
			xs = p1.x();
		}
		if (yb < ys)
		{
			yb = p0.y();
			ys = p1.y();
		}

		xmin = std::min(xmin, xs);
		xmax = std::max(xmax, xb);
		ymin = std::min(ymin, ys);
		ymax = std::max(ymax, yb);
	}

	bound_.min = Vector2(xmin, ymin);
	bound_.max = Vector2(xmax, ymax);

}


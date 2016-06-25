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

void PolygonCollider::ResetBound() const
{
	bound_.center = position_;
	float xmin, xmax, ymin, ymax;
	const std::vector<Vector2>& verts = pshared_shape_->vertices();
	std::size_t count = verts.size();
	std::size_t ini = 0;
	if (count % 2 == 0)
	{
		ini = 2;

		Vector2 p0 = TransformVector(verts[0]);
		Vector2 p1 = TransformVector(verts[1]);

		// Init
		if (p0.x() <= p1.x())
		{
			xmin = p0.x();
			xmax = p1.x();
		}
		else
		{
			xmin = p1.x();
			xmax = p0.x();
		}

		if (p0.y() <= p1.y())
		{
			ymin = p0.y();
			ymax = p1.y();
		}
		else
		{
			ymin = p1.y();
			ymax = p0.y();
		}
	}
	else
	{
		ini = 1;

		Vector2 p = TransformVector(verts[0]);

		xmin = xmax = p.x();
		ymin = ymax = p.y();
	}

	// Find the bound points.
	for (std::size_t i = ini; i < count; i += 2)
	{
		Vector2 p0 = TransformVector(verts[ini]);
		Vector2 p1 = TransformVector(verts[ini + 1]);

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

	bound_.size.set_x(xmax - xmin);
	bound_.size.set_y(ymax - ymin);

}
#include "collider.h"

using namespace ysd_phy_2d;

// Check the detection between 2 polygons by gjk.
inline bool ysd_phy_2d::DoCheck(const PolygonCollider& collider1, const PolygonCollider& collider2)
{
	// Initial direction: from collider1's center to collider2's center.
	Vector2 c1 = collider1.TransformVector(collider1.pshared_shape_->Center());
	Vector2 c2 = collider2.TransformVector(collider2.pshared_shape_->Center());
	Vector2 dir = c1 - c2;
	if (dir == Vector2::kZero)
	{
		dir.set_x(1);
	}

	// Vertices transform from model coordinates to world coordinates.
	const std::vector<Vector2> verts1 = collider1.pshared_shape_->vertices();
	std::size_t size1 = verts1.size();
	Vector2* vert_arr1 = new Vector2[size1];
	for (std::size_t i = 0; i < size1; ++i)
	{
		vert_arr1[i] = collider1.TransformVector(verts1[i]);
	}

	const std::vector<Vector2> verts2 = collider2.pshared_shape_->vertices();
	std::size_t size2 = verts2.size();
	Vector2* vert_arr2 = new Vector2[size2];
	for (std::size_t i = 0; i < size2; ++i)
	{
		vert_arr2[i] = collider2.TransformVector(verts2[i]);
	}

	// Simplex that used to check whether it contain the origin.
	Vector2 simplex[3];
	std::size_t index = 0;

	// The first point along the initial direction.
	simplex[index] = Support(vert_arr1, size1, vert_arr2, size2, dir);

	if (Vector2::Dot(simplex[0], dir) <= 0)
	{
		return false;
	}

	// Negate the direction to make the search area larger.
	dir = -dir;

	for (;;)
	{
		simplex[++index] = Support(vert_arr1, size1, vert_arr2, size2, dir);
		if (Vector2::Dot(simplex[0], dir) <= 0)
		{
			return false;
		}

		Vector2 ao = -simplex[index];

		// New direction to search the origin.
		if (index < 2)
		{
			Vector2 ab = simplex[1] - simplex[0];
			// ab x ao will get a vector p that is perpendicular to the plane of abo,
			// p x ab will get a vector that os perpendicular to ab.
			dir = Vector2::TripleCross(ab, ao, ab);
			if (dir == Vector2::kZero)
			{
				dir = Vector2::Perpendicular(ab);
			}
			continue;
		}

		Vector2 ab = simplex[1] - simplex[2];
		Vector2 ac = simplex[0] - simplex[2];

		// A vector perpendicular to ac.
		Vector2 acp = Vector2::TripleCross(ab, ac, ac);
		if (Vector2::Dot(acp, ao) >= 0)
		{
			// The origin and the acp are on the same side of ac.
			// Set the new direction.
			dir = acp;
			// Retain a and c, discard b.
			simplex[1] = simplex[2];
			index--;
		}
		else
		{
			Vector2 abp = Vector2::TripleCross(ac, ab, ab);
			if (Vector2::Dot(abp, ao) >= 0)
			{
				// The origin and abp are on the same side of ab.
				// Set the new direction.
				dir = abp;
				// Retain a and b, discard c.
				simplex[0] = simplex[1];
				simplex[1] = simplex[2];
				index--;
			}
			else
			{
				// The origin is contained inside the triangle abc.
				return true;
			}
		}

	}

	delete[] vert_arr1;
	delete[] vert_arr2;

	return false;
}

// Check the detection between a circle and a polygon.
inline bool ysd_phy_2d::DoCheck(const CircleCollider& collider1, const PolygonCollider& collider2)
{
	Vector2 center = collider1.Center();
	// Two collider contact just need that one corner of the polygon is in the circle.
	const std::vector<Vector2> corners = collider2.pshared_shape_->vertices();
	for (const Vector2& corner : corners)
	{
		float distance = Vector2::Distance(corner, center);
		if (distance < collider1.Radius())
			return true;
	}
	return false;
}

// Check the detection between two circle colliders.
inline bool ysd_phy_2d::DoCheck(const CircleCollider& collider1, const CircleCollider& collider2)
{
	// Just need to check if the distance of two circle is smaller than the sum of their radius.
	Vector2 c1 = collider1.Center();
	Vector2 c2 = collider2.Center();
	float distance = Vector2::Distance(c1, c2);
	return distance < collider1.Radius() + collider2.Radius();
}

inline const Vector2& PolygonCollider::TransformVector(const Vector2& vec) const
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

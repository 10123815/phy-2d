//////////////////////////////////////////////////////
// @fileoverview Defination of 2d colliders.
// @author	ysd
//////////////////////////////////////////////////////

#ifndef _COLLIDER_H_
#define _COLLIDER_H_

#include <memory>
#include <functional>

#include "../math/vector_2.h"
#include "../common/un-copy-move-interface.h"
#include "shape.h"
#include "collision.h"

namespace ysd_phy_2d
{

class BaseCollider;
class CircleCollider;
class PolygonCollider;

// Minkowski sum support function for GJK.
Vector2 Support(const Vector2* vertices1,
                const std::size_t size1,
                const Vector2* vertices2,
                const std::size_t size2,
                const Vector2& dir);

// template function to check if two collider collide.
template <typename CT1, typename CT2>
bool DoCheck(const CT1& collider1, const CT2& collider2);

template <>
bool DoCheck(const CircleCollider& collider1, const CircleCollider& collider2);

// Use GJK algorithm to check if two convex polygon collide each other.
template <>
bool DoCheck(const PolygonCollider& collider1, const PolygonCollider& collider2);

// Check if a convex polygon collider circle.
template <>
bool DoCheck(const CircleCollider& collider1, const PolygonCollider& collider2);
template <>
bool DoCheck(const PolygonCollider& collider1, const CircleCollider& collider2);

// Callback when collision is detected.
typedef std::function<void(std::shared_ptr<Collision>)> OnColliderEnter;

////////////////////////////////////////////////////////////////
// A BaseCollider contain a shape infomation and a transform.
//
// The shape information can be shared among not only one colliders.
// The class include a ptr to point the shared shape information.
//
// The trnasform descripe the space transform of a collider.
// Every collider has its own transform includes position,
// rotation and scale. In 2d space, the rotation can represent
// by a single angle.
//
////////////////////////////////////////////////////////////////
class BaseCollider : IUncopyable
{
public:
	BaseCollider(uint16_t id)
		: id_(id) {}

	uint16_t id() const { return id_; }

	// Check if two collider collide.
	// @param[in] 	callback 	Callback when collision is occur.
	virtual bool Check(std::shared_ptr<BaseCollider> collider, OnColliderEnter callback) = 0;

	void Move(Vector2 position)
	{
		position_ += position;
	}

	// Transform a vector/point from shape's self space to world space.
	// @return 	The transformed vector/point.
	virtual Vector2 TransformVector(const Vector2& vec) const = 0;

protected:

	// Transform.
	Vector2 position_;

	uint16_t id_;

};

const bool operator== (const BaseCollider& vec1, const BaseCollider& vec2)
{
	return vec1.id() == vec2.id();
}

class CircleCollider : public BaseCollider
{
public:
	CircleCollider(uint8_t id, std::shared_ptr<Circle> c)
		: BaseCollider(id), pshared_shape_(c)
	{}

	friend bool DoCheck<CircleCollider, CircleCollider>
	(const CircleCollider& collider1,
	 const CircleCollider& collider2);

	bool Check(std::shared_ptr<BaseCollider> collider, OnColliderEnter callback) override
	{
		return DoCheck(*this, *collider);
	}

	Vector2 TransformVector(const Vector2& vec) const override
	{
		return vec + position_;
	}

	float Radius() const
	{
		return pshared_shape_->radius();
	}

	Vector2 Center() const
	{
		return position_;
	}

protected:
	std::shared_ptr<Circle> pshared_shape_;
};

class PolygonCollider : public BaseCollider
{
public:
	PolygonCollider(uint8_t id, std::shared_ptr<ConvexPolygon> pss)
		: BaseCollider(id), pshared_shape_(pss)
	{}

	// template <typename CT1, typename CT2>
	// friend bool DoCheck(const CT1& collider1, const CT2& collider2);

	// Only the PolygonCollider specialization will be friend
	friend bool DoCheck<PolygonCollider, PolygonCollider>
	(const PolygonCollider& collider1,
	 const PolygonCollider& collider2);

	friend bool DoCheck<PolygonCollider, CircleCollider>
	(const PolygonCollider& collider1,
	 const CircleCollider& collider2);

	bool Check(std::shared_ptr<BaseCollider> collider, OnColliderEnter callback) override
	{
		return DoCheck(*this, *collider);
	}

	Vector2 TransformVector(const Vector2& vec) const override
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

	void Rotate(float angle)
	{
		angle_ += angle;
	}

	void Scale(Vector2 scale)
	{
		scale_.Scale(scale);
	}

protected:
	std::shared_ptr<ConvexPolygon> pshared_shape_;

	Vector2 scale_;
	float angle_;

};

template <typename CT1, typename CT2>
bool DoCheck(const CT1& collider1, const CT2& collider2)
{
	// TODO(ysd): Default.
	return true;
}
template <>
bool DoCheck(const PolygonCollider& collider1, const PolygonCollider& collider2)
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
	Vector2 vert_arr1[size1];
	for (std::size_t i = 0; i < size1; ++i)
	{
		vert_arr1[i] = collider1.TransformVector(verts1[i]);
	}

	const std::vector<Vector2> verts2 = collider2.pshared_shape_->vertices();
	std::size_t size2 = verts2.size();
	Vector2 vert_arr2[size2];
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

	return false;
}

template <>
bool DoCheck(const CircleCollider& collider1, const PolygonCollider& collider2)
{
	return true;
}

template <>
bool DoCheck(const PolygonCollider& collider1, const CircleCollider& collider2)
{
	return DoCheck(collider2, collider1);
}

template <>
bool DoCheck(const CircleCollider& collider1, const CircleCollider& collider2)
{
	Vector2 c1 = collider1.Center();
	Vector2 c2 = collider2.Center();
	float distance = Vector2::Distance(c1, c2);
	return distance < collider1.Radius() + collider2.Radius();
}

}

#endif
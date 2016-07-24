//////////////////////////////////////////////////////
// @fileoverview Defination of 2d colliders.
// @author	ysd
//////////////////////////////////////////////////////

#ifndef _COLLIDER_H_
#define _COLLIDER_H_

#include <memory>
#include <algorithm>
#include <functional>

#include "../math/vector_2.h"
#include "../math/bound.h"
#include "../common/un-copy-move-interface.h"
#include "shapes.h"
#include "collision.h"

namespace ysd_phy_2d
{

// Forward declaration.
class BaseCollider;
class CircleCollider;
class PolygonCollider;

// Get the furthest point along the certain direction.
inline std::size_t IndexOfFurthestPoint(const Vector2* vertices,
										const std::size_t size,
										const Vector2& dir)
{
	std::size_t fi = 0;
	// Dot product of two vectors.
	float fdot = Vector2::Dot(dir, vertices[0]);
	for (std::size_t i = 1; i < size; ++i)
	{
		float dot = Vector2::Dot(dir, vertices[i]);
		if (dot > fdot)
		{
			fdot = dot;
			fi = i;
		}
	}
	return fi;
}

// Minkowski sum support function for GJK.
inline Vector2 Support(const Vector2* vertices1,
					   const std::size_t size1,
					   const Vector2* vertices2,
					   const std::size_t size2,
					   const Vector2& dir)
{
	std::size_t i = IndexOfFurthestPoint(vertices1, size1, dir);
	std::size_t j = IndexOfFurthestPoint(vertices2, size2, -dir);
	return vertices1[i] - vertices2[j];
}

// Narrow phase detection.
// Check if two circle collide each other.
bool DoCheck(const CircleCollider& collider1, const CircleCollider& collider2);

// Use GJK algorithm to check if two convex polygon collide each other.
bool DoCheck(const PolygonCollider& collider1, const PolygonCollider& collider2);

// Check if a convex polygon collide circle.
bool DoCheck(const CircleCollider& collider1, const PolygonCollider& collider2);

// Callback when collision is detected.
typedef std::function<void(std::shared_ptr<Collision>)> OnDetectedCallback;
enum OnDetectedCallbackType
{
	kOnColliderEnter = 1,
	kOnColliderStay = 2,
	kOnColliderExit = 3
};

////////////////////////////////////////////////////////////////
// A BaseCollider contain a shape infomation and a transform.
//
// The shape information can be shared among not only one colliders.
// The class include a ptr to point the shared shape information.
//
// The transform descripe the space transform of a collider.
// Every collider has its own transform includes position,
// rotation and scale. In 2d space, the rotation can represent
// by a single angle.
// 
// The bound also owned by a collider. When a collider move,
// rotate or scale, its bound need to be updated.
//
////////////////////////////////////////////////////////////////
class BaseCollider : IUncopyable
{
public:
	BaseCollider(uint16_t id)
		:bound_(), position_(Vector2::kZero), scale_(Vector2::kOne), id_(id)
	{}

	BaseCollider(uint16_t id, Vector2 position)
		:bound_(), position_(position), scale_(Vector2::kOne), id_(id)
	{}

	virtual ~BaseCollider()
	{

	}

	uint16_t id() const { return id_; }

	virtual const Bound& bound() const { return bound_; }

	virtual void Translate(const Vector2& movement)
	{
		position_ += movement;
		// Bound move follow the transform.
		bound_.min += movement;
		bound_.max += movement;
	}

	virtual void ScaleFor(const Vector2& scale)
	{
		scale_.Scale(scale);

		// Update the bound.
		Vector2 center = (bound_.max - bound_.min) / 2;
		Vector2 offset = center - bound_.min;
		offset.Scale(scale);
		bound_.min = center - offset;
		bound_.max = center + offset;
	}

	virtual void Rotate(const float angle)
	{
		// Do nothing.
	}

	// Transform a vector/point from shape's self space to world space.
	// We must to transform the collider first in narrow phase.
	// @return 	The transformed vector/point.
	virtual const Vector2& TransformVector(const Vector2& vec) const = 0;

	// Overload functions to check if two BaseCollider contact.
	virtual bool Check(const BaseCollider&, OnDetectedCallback* callbacks) const = 0;
	virtual bool Check(const CircleCollider&, OnDetectedCallback* callbacks) const = 0;
	virtual bool Check(const PolygonCollider&, OnDetectedCallback* callbacks) const = 0;

protected:

	mutable Bound bound_;

	// Transform.
	Vector2 position_;
	Vector2 scale_;

	uint16_t id_;

};

// Is two colliders the same one.
const bool operator== (const BaseCollider& vec1, const BaseCollider& vec2)
{
	return vec1.id() == vec2.id();
}


///////////////////////////////////////////////////////
// Defination of circle collider.
// A circle collider only have translate information.
// It can not rotate.
///////////////////////////////////////////////////////
class CircleCollider : public BaseCollider
{
public:
	CircleCollider(uint8_t id, std::shared_ptr<Circle> c)
		: BaseCollider(id), pshared_shape_(c)
	{
		// Initailize bound.
		Vector2 v = Vector2(Radius(), Radius());
		bound_.min = -v;
		bound_.max = v;
	}

	void ScaleFor(const Vector2& scale) override
	{
		// Only need x.
		scale_.set_x(scale.x());

		// TODO(ysd): Optimize scaling of a circle.
	}

	// A circle do not have rotation and scale.
	const Vector2& TransformVector(const Vector2& vec) const override
	{
		return vec + position_;
	}

	float Radius() const
	{
		return pshared_shape_->radius() * scale_.x();
	}

	const Vector2& Center() const
	{
		return position_;
	}

	// Overload functions to check if two BaseCollider contact.
	bool Check(const BaseCollider& other, OnDetectedCallback* callback) const override
	{
		other.Check(*this, callback);
	}
	bool Check(const CircleCollider& other, OnDetectedCallback* callback) const override
	{
		DoCheck(*this, other);
	}
	bool Check(const PolygonCollider& other, OnDetectedCallback* callback) const override
	{
		DoCheck(*this, other);
	}

protected:
	const std::shared_ptr<Circle> pshared_shape_;

private:
	// Do the collistion detection.
	friend bool DoCheck(const CircleCollider& collider1, const CircleCollider& collider2);
	friend bool DoCheck(const CircleCollider& collider1, const CircleCollider& collider2);

};

////////////////////////////////////////////////////////////////
// Arbitrary convex polygon collider.
//
////////////////////////////////////////////////////////////////
class PolygonCollider : public BaseCollider
{
public:
	PolygonCollider(uint8_t id, std::shared_ptr<ConvexPolygon> pss)
		: BaseCollider(id), pshared_shape_(pss), angle_(0)
	{
		// Initailize bound.
		ResetBound(false);
	}

	// The generic DoCheck function is not friend.
	// template <typename CT1, typename CT2>
	// friend bool DoCheck(const CT1& collider1, const CT2& collider2);

	// Rotate the collider anticlockwise by given angle.
	void Rotate(float angle)
	{
		angle_ += angle;
		ResetBound();
	}

	const Vector2& TransformVector(const Vector2& vec) const override;

	// Overload functions to check if two BaseCollider contact.
	bool Check(const BaseCollider& other, OnDetectedCallback* callback) const override
	{
		other.Check(*this, callback);
	}
	bool Check(const CircleCollider& other, OnDetectedCallback* callback) const override
	{
		DoCheck(other, *this);
	}
	bool Check(const PolygonCollider& other, OnDetectedCallback* callback) const override
	{
		DoCheck(*this, other);
	}

protected:
	const std::shared_ptr<ConvexPolygon> pshared_shape_;

	// Transform.
	float angle_;

private:
	void ResetBound(bool transformed = true) const;

	// Only the PolygonCollider overload will be friend
	friend bool DoCheck(const CircleCollider& collider1, const PolygonCollider& collider2);
	friend bool DoCheck(const PolygonCollider& collider1, const PolygonCollider& collider2);
};

}

#endif
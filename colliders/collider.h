#ifndef _COLLIDER_H_
#define _COLLIDER_H_

#include <memory>
#include <functional>

#include "../math/vector_2.h"
#include "shape.h"
#include "transform.h"

namespace ysd_phy_2d
{

enum ColliderType
{
	kCircle = 0,
	kPolygon = 1
};

class BaseCollider
{
public:

protected:
	Transform transform_;

};

typedef std::function<void(BaseCollider)> OnColliderEnter;

class CircleCollider : public BaseCollider
{
public:
	bool Check(BaseCollider collider, OnColliderEnter callback);
protected:
	std::shared_ptr<Circle> pshared_shape_;
};

class PolygonCollider : public BaseCollider
{
public:

protected:
	std::shared_ptr<ConvexPolygon> pshared_shape_;

private:
	// Use GJK algorithm to check if two convex polygon collide each other.
	bool GJK(const PolygonCollider& collider1,
	         const PolygonCollider& collider2);

	// Get the furthest point along the certain direction.
	static std::size_t IndexOfFurthestPoint(const Vector2* vertices,
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
	static Vector2 Support(const Vector2* vertices1,
	                       const std::size_t size1,
	                       const Vector2* vertices2,
	                       const std::size_t size2,
	                       const Vector2& dir)
	{
		std::size_t i = IndexOfFurthestPoint(vertices1, size1, dir);
		std::size_t j = IndexOfFurthestPoint(vertices2, size2, -dir);
		return vertices1[i] - vertices2[j];
	}

};

}

#endif
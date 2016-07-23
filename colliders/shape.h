//////////////////////////////////////////////////////
// @fileoverview Defination of 2d shape.
// @author	ysd
//////////////////////////////////////////////////////

#ifndef _SHAPE_H_
#define _SHAPE_H_

#include <vector>
#include <assert.h>

#include "../math/vector_2.h"
#include "../common/un-copy-move-interface.h"

namespace ysd_phy_2d
{

//////////////////////////////////////////////////////
// A Shape class descrip the shape of a collider.
// Some colliders can share a same shape but they will
// have different transforms.
// We cannot copy-construct or move-construct a Shape.
//////////////////////////////////////////////////////
class Shape : IUnCopyMovable
{
public:
	virtual ~Shape() = default;
	virtual const Vector2 Center() const = 0;
};

class Circle : public Shape
{
public:
	Circle(float r) : radius_(r) {}
	// Inhibit cppy and move.
	// Circle(const Circle& other) = delete;
	// Circle(Circle&& other) = delete;

	// A circle's center is always at the origin.
	const Vector2 Center() const override
	{
		return Vector2::kZero;
	}

	float radius() const { return radius_; }
	void set_radius(float v) { radius_ = v; }

private:
	float radius_;

	// Vector2 center_ = Vector2::kZero;
};

class Rectangle : public Shape
{
public:
	Rectangle(Vector2 lu, Vector2 rd)
		: left_up_(lu), right_down_(rd)
	{}

	const Vector2 Center() const override
	{
		return (left_up_ + right_down_) / 2;
	}

	// Get lenght and height of the square.
	const Vector2 Size() const
	{
		return Vector2(right_down_.x() - left_up_.x(),
		               left_up_.y() - right_down_.y());
	}

	float Xmin() const { return left_up_.x(); }
	float Xmax() const { return right_down_.x(); }
	float Ymin() const { return right_down_.y(); }
	float Ymax() const { return left_up_.y(); }

private:
	Vector2 left_up_;
	Vector2 right_down_;
};

class ConvexPolygon : public Shape
{
public:

	ConvexPolygon(const Vector2* vertices, std::size_t size)
		: vertices_(vertices, vertices + size), changed_(true)
	{
		// At least three points.
		assert(size >= 3);
	}

	void ModifyVertex(const Vector2& vec, std::size_t index)
	{
		vertices_[index] = vec;
		changed_ = true;
	}

	// Calculate the average center roughly.
	const Vector2 Center() const override
	{
		if (!changed_)
			return center_;

		float x, y;
		for (std::size_t i = 0, l = vertices_.size(); i < l; i++)
		{
			x += vertices_[i].x();
			y += vertices_[i].y();
		}
		x /= vertices_.size();
		y /= vertices_.size();
		center_ = Vector2(x, y);
		return center_;
	}

	// @return Reference to const vertices.
	const std::vector<Vector2>& vertices() const
	{
		return vertices_;
	}

private:

	std::vector<Vector2> vertices_;

	mutable Vector2 center_;
	mutable bool changed_;
};
}

#endif
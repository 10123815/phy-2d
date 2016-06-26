//////////////////////////////////////////////////////
// @fileoverview Defination of 2d vector.
// @author	ysd
//////////////////////////////////////////////////////

#ifndef _VECTOR_2_H_
#define _VECTOR_2_H_

#include <cmath>

namespace ysd_phy_2d
{

class Vector2;
class Vector2 final
{

public:
	Vector2() = default;

	Vector2(float x, float y)
		: x_(x), y_(y),
		  length_(hypotf(x, y)), 		// √x2+y2
		  changed_(false)
	{}

	float x() const { return x_; }
	void set_x(float v)
	{
		x_ = v;
		changed_ = true;
	}

	float y() const { return y_; }
	void set_y(float v)
	{
		y_ = v;
		changed_ = true;
	}

	float length() const
	{
		if (changed_)
		{
			changed_ = false;
			length_ = hypotf(x_, y_);
		}
		return length_;
	}

	void Scale(const Vector2& vec2)
	{
		x_ *= vec2.x_;
		y_ *= vec2.y_;
	}

	// Returns this vector with a magnitude of 1 (Read Only).
	Vector2& Normalize()
	{
		this->operator/=(this->length());
		return *this;
	}

	Vector2& operator+= (const Vector2& vec)
	{
		changed_ = true;
		this->x_ += vec.x_;
		this->y_ += vec.y_;
		return *this;
	}

	Vector2& operator-= (const Vector2& vec)
	{
		changed_ = true;
		this->x_ -= vec.x_;
		this->y_ -= vec.y_;
		return *this;
	}

	Vector2& operator*= (float m)
	{
		changed_ = true;
		this->x_ *= m;
		this->y_ *= m;
		return *this;
	}

	Vector2& operator/= (float d)
	{
		changed_ = true;
		if (d == 0)
			return *this;
		else
			return this->operator*=(1 / d);
	}

	// Dot Product of two vectors.
	static float Dot(const Vector2& vec1, const Vector2& vec2)
	{
		return vec1.x_ * vec2.x_ + vec1.y_ * vec2.y_;
	}

	static Vector2 Perpendicular(const Vector2& vec)
	{
		return Vector2(vec.y_, -vec.x_);
	}

	static float Distance(const Vector2& vec1, const Vector2& vec2)
	{
		float x = vec1.x_ - vec2.x_;
		float y = vec1.y_ - vec2.y_;
		return hypotf(x, y);
	}

	// Perform a x b x c.
	// @return A perpendicular vector of c. This vector is still on the plane of abc.
	static Vector2 TripleCross(const Vector2& a, const Vector2& b, const Vector2& c)
	{
		Vector2 r;

		// Perform a.dot(c)
		float ac = a.x_ * c.x_ + a.y_ * c.y_;
		// Perform b.dot(c)
		float bc = b.x_ * c.x_ + b.y_ * c.y_;

		// Perform b * a.dot(c) - a * b.dot(c)
		r.x_ = b.x_ * ac - a.x_ * bc;
		r.y_ = b.y_ * ac - a.y_ * bc;
		return r;
	}

	static const Vector2 kZero;
	static const Vector2 kLeft;
	static const Vector2 kUp;
	static const Vector2 kRight;
	static const Vector2 kDown;
	static const Vector2 kOne;
	static const float kEpsinon;

private:

	friend const Vector2 operator+ (const Vector2& vec1, const Vector2& vec2);
	friend const Vector2 operator- (const Vector2& vec1, const Vector2& vec2);
	friend const Vector2 operator- (const Vector2& vec);
	friend const Vector2 operator* (const Vector2& vec1, float m);
	friend const bool operator== (const Vector2& vec1, const Vector2& vec2);
	friend const bool operator< (const Vector2& vec1, const Vector2& vec2);

	float x_;
	float y_;

	// Length of the vector. It is logic const but not bitwise const.
	mutable float length_;

	mutable bool changed_;

};

const Vector2 operator+ (const Vector2& vec1, const Vector2& vec2)
{
	float x = vec1.x_ + vec2.x_;
	float y = vec1.y_ + vec2.y_;
	return Vector2(x, y);
}

const Vector2 operator- (const Vector2& vec1, const Vector2& vec2)
{
	float x = vec1.x_ - vec2.x_;
	float y = vec1.y_ - vec2.y_;
	return Vector2(x, y);
}

const Vector2 operator- (const Vector2& vec)
{
	float x = -vec.x_;
	float y = -vec.y_;
	return Vector2(x, y);
}

const Vector2 operator* (const Vector2& vec, float m)
{
	float x = vec.x_ * m;
	float y = vec.y_ * m;
	return Vector2(x, y);
}

const Vector2 operator/ (const Vector2& vec, float d)
{
	if (d == 0)
		return vec;
	else
		return vec * (1 / d);
}

const bool operator== (const Vector2& vec1, const Vector2& vec2)
{
	Vector2 vec = vec1 - vec2;
	return (vec.x_ >= -Vector2::kEpsinon &&
	        vec.x_ <= Vector2::kEpsinon &&
	        vec.y_ >= -Vector2::kEpsinon &&
	        vec.y_ <= Vector2::kEpsinon);
}

const bool operator!= (const Vector2& vec1, const Vector2& vec2)
{
	return !(vec1 == vec2);
}

const bool operator< (const Vector2& vec1, const Vector2& vec2)
{
	return vec1.x() < vec2.x() && vec1.y() < vec2.y();
}

const bool operator> (const Vector2& vec1, const Vector2& vec2)
{
	return (!(vec1 < vec2)) && (vec1 != vec2);
}

}
#endif
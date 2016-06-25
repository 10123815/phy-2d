#include "collider.h"

using namespace ysd_phy_2d;

// Get the furthest point along the certain direction.
std::size_t IndexOfFurthestPoint(const Vector2* vertices,
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
Vector2 Support(const Vector2* vertices1,
                const std::size_t size1,
                const Vector2* vertices2,
                const std::size_t size2,
                const Vector2& dir)
{
	fuck();
	std::size_t i = IndexOfFurthestPoint(vertices1, size1, dir);
	std::size_t j = IndexOfFurthestPoint(vertices2, size2, -dir);
	return vertices1[i] - vertices2[j];
}

int main(int argc, char const *argv[])
{
	
}
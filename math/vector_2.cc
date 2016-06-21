//////////////////////////////////////////////////////
// @fileoverview Defination of 2d vector.
// @author	ysd
//////////////////////////////////////////////////////

#include "vector_2.h"

using namespace ysd_phy_2d;

const Vector2 Vector2::kZero 	= Vector2(0, 0);
const Vector2 Vector2::kLeft 	= Vector2(-1, 0);
const Vector2 Vector2::kUp 		= Vector2(0, 1);
const Vector2 Vector2::kRight 	= Vector2(1, 0);
const Vector2 Vector2::kDown 	= Vector2(0, -1);
const float Vector2::kEpsinon   = 0.00001f;

////////////////////////////////////////////////////////
// @fileoverview API of this collision detection system.
// @author	ysd
////////////////////////////////////////////////////////

#include <map>
#include <memory>

#include "./colliders/collider.h"
#include "./scene/quad-tree.h"

using namespace ysd_phy_2d;

QuadTree g_quad_tree;

///////////////////////////////////////////////////////
// Add a circle collider in the physics world.
///////////////////////////////////////////////////////
void AddCircleCollider(uint16_t id, float pos_x, float pos_y, float radius)
{
	std::shared_ptr<Circle> pcircle_shape = std::make_shared<Circle>(radius);
	std::shared_ptr<CircleCollider> pcircle_collider = std::make_shared<CircleCollider>(id, pcircle_collider);
	g_quad_tree.Insert(pcircle_collider);
}

///////////////////////////////////////////////////////
// Add a convex polygon collider in the physics world.
///////////////////////////////////////////////////////
void AddRectangleCollider(uint16_t id, float min_x, float min_y, float max_x, float max_y)
{
}

///////////////////////////////////////////////////////
// Add a convex polygon collider in the physics world.
// @param[in]	xy		Corners of the polygon, [x1, y1, x2, y2...]
// @param[in]	size 	Size of the array.
////////////////////////////////////////////////////////
void AddPolygonCollider(uint16_t id, float pos_x, float pos_y, float* xy, std::size_t size)
{

}

///////////////////////////////////////////////////////
// Update the physical world, trigger collistin events. 
///////////////////////////////////////////////////////
void Update()
{
	// Check all possible collision in the game scene.
}
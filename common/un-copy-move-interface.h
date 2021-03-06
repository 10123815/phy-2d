//////////////////////////////////////////////////////
// @fileoverview Interface of uncopyable class and
//				 unmovable class. If class want to
//				 inhibt copy-construct and move-construct,
//				 just let it inherit the class in this file.
// @author	ysd
//////////////////////////////////////////////////////

#ifndef _UN_COPY_MOVE_INTERFACE_H_
#define _UN_COPY_MOVE_INTERFACE_H_

namespace ysd_phy_2d
{
class IUncopyable
{
protected:
	IUncopyable() = default;
	~IUncopyable() = default;

	// Inhibit copying.
	IUncopyable(const IUncopyable& other) = delete;
	IUncopyable& operator=(const IUncopyable& other) = delete;

	// Allow movement.
	IUncopyable(IUncopyable&& other) = default;
	IUncopyable& operator=(IUncopyable&& other) = default;

};

class IUnmovable
{
protected:
	IUnmovable() = default;
	~IUnmovable() = default;

	// Allow copying
	IUnmovable(const IUnmovable& other) = default;
	IUnmovable& operator=(const IUnmovable& other) = default;

	// Inhibit movement
	IUnmovable(IUnmovable&& other) = delete;
	IUnmovable& operator=(IUnmovable&& other) = delete;
};

class IUnCopyMovable
{
protected:
	IUnCopyMovable() = default;
	~IUnCopyMovable() = default;

	// Inhibit copying.
	IUnCopyMovable(const IUnCopyMovable& other) = delete;
	IUnCopyMovable& operator=(const IUnCopyMovable& other) = delete;

	// Inhibit movement
	IUnCopyMovable(IUnCopyMovable&& other) = delete;
	IUnCopyMovable& operator=(IUnCopyMovable&& other) = delete;
};
}

#endif
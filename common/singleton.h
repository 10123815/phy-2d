//////////////////////////////////////////////////////
// @fileoverview Template class defination of singleton.
// @author	ysd
//////////////////////////////////////////////////////

#ifndef _SINGLETON_H_
#define _SINGLETON_H_

#include "un-copy-move-interface.h"

namespace ysd_phy_2d
{

////////////////////////////////////////////////////////////
// Use it like Singleton<#classname#>::Inst().#functionname#
// The class need to provide a default constructor.
////////////////////////////////////////////////////////////
template<typename T>
class Singleton : public IUnCopyMovable
{
public:
	static T& Inst()
	{
		static T obj;
		// Make sure that the constructor of obj_ctor is called.
		obj_ctor.do_nothing();
		return obj;
	}

private:
	struct ObjectCreator
	{
		ObjectCreator()
		{
			Singleton<T>::instance();
		}
		inline void do_nothing() const {}
	};

	static ObjectCreator obj_ctor;
};

template <typename T> Singleton<T>::ObjectCreator Singleton<T>::obj_ctor;

}

#endif // !_SINGLETON_H_
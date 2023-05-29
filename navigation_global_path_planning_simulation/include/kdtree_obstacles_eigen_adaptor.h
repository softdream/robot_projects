#ifndef __KDTREE_OBSTACLES_EIGEN_ADAPTOR_H
#define __KDTREE_OBSTACLES_EIGEN_ADAPTOR_H

#include "nanoflann.hpp"
#include "obstacles.h"

namespace kdtree
{

template<typename Derived>
struct KdTreeObstaclesEigenAdaptor
{
	using coord_t = typename Derived::value_type;

	const Derived& obj_;

	KdTreeObstaclesEigenAdaptor() = delete;

	KdTreeObstaclesEigenAdaptor( const Derived& obj ) : obj_( obj )
	{
	
	}

	inline const Derived& derived() const
	{
		return obj_;
	}

	inline size_t kdtree_get_point_count() const
	{
		return derived().getSize();
	}

	inline coord_t kdtree_get_pt( const size_t idx, const size_t dim ) const
	{
		return derived()[idx]( dim );
	}

	template<class BBOX>
	bool kdtree_get_bbox( BBOX&  ) const
	{
		return false;
	}
};

template<typename T>
using KdTreeObstaclesType = KdTreeObstaclesEigenAdaptor<apf::Obstacles<T>>;

template<typename T>
using KdTreeType = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<T, KdTreeObstaclesType<T>>, KdTreeObstaclesType<T>, 2>;

}

#endif

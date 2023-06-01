#ifndef __KDTREE_VECTOR_EIGEN_ADAPTOR_H
#define __KDTREE_VECTOR_EIGEN_ADAPTOR_H

#include "nanoflann.hpp"
#include <Eigen/Dense>
#include <vector>

namespace slam
{

template<typename Derived>
struct KdTreeVectorEigenAdaptor
{
	using coord_t = typename Derived::value_type::value_type;

	const Derived& obj_;

	KdTreeVectorEigenAdaptor() = delete;

	KdTreeVectorEigenAdaptor( const Derived& obj ) : obj_( obj )
	{
	
	}

	inline const Derived& derived() const
        {
                return obj_;
        }

	inline size_t kdtree_get_point_count() const
        {
                return derived().size();
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

template<typename T, int Dimension>
using VectorEigenType = typename std::vector<Eigen::Matrix<T, Dimension, 1>>;

template<typename T, int Dimension>
using KdTreeVectorEigenType = KdTreeVectorEigenAdaptor<VectorEigenType<T, Dimension>>;

template<typename T, int Dimension>
using KdTreeType = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<T, KdTreeVectorEigenType<T, Dimension>>, KdTreeVectorEigenType<T, Dimension>, Dimension>;

}

#endif

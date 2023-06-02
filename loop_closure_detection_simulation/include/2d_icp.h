#ifndef __2D_ICP_H
#define __2D_ICP_H

#include "data_container.h"
#include "nanoflann.hpp"

namespace slam
{

template<typename Derived>
struct KdTreeScanContainerAdaptor
{
        using coord_t = typename Derived::value_type;

        const Derived& obj_;

        KdTreeScanContainerAdaptor() = delete;

        KdTreeScanContainerAdaptor( const Derived& obj ) : obj_( obj )
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

using KdTreeScanContainerType = KdTreeScanContainerAdaptor<sensor::ScanContainer>;

template<typename T>
using KdTreeType = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<T, KdTreeScanContainerType>, KdTreeScanContainerType, 2>;


template<typename T>
class ICP
{
public:
	using ValueType = T;
	using PointType = typename Eigen::Matrix<ValueType, 2, 1>;
	using PoseType = typename Eigen::Matrix<ValueType, 3, 1>;

	using RotaionType = typename Eigen::Matrix<ValueType, 2, 2>;
	using TranslationType = typename Eigen::Matrix<ValueType, 2, 1>;

	ICP()
	{
	
	}

	~ICP()
	{
	
	}

	void solveICP( const sensor::ScanContainer& first_scan, 
		       const sensor::ScanContainer& second_scan,
		       PoseType& p, 
		       const int max_iterations = 50 )
	{
		if ( first_scan.isEmpty() || second_scan.isEmpty() ) {
			return;
		}

		// 2. construct the kd tree of the first scan
		KdTreeScanContainerType kd_first_scan( first_scan );
		kd_tree_ptr_ = std::make_unique<KdTreeType<ValueType>>( 2, kd_first_scan, 10 );

		// 3. get the initial transformation
		pose2Transformation( p, rotation_matrix_, translation_vector_);

		// 4. start iteration
		int iter = 0;
		while ( iter < max_iterations ) {
			mse_ = 0;

			estimateOnce( first_scan, second_scan, p );


			iter ++;
		}
		std::cout<<"mse_ = "<<mse_<<std::endl;
	}

private:
	void estimateOnce( const sensor::ScanContainer& first_scan,
			   const sensor::ScanContainer& second_scan,
			   PoseType& p )
	{
		Hessian_.setZero();
		B_.setZero();

		// 1. for every point int the second scan
		for ( int i = 0; i < second_scan.getSize(); i ++ ) {
			auto pt_in_second = second_scan[i];

			// 2. transform the second frame point to the first frame coordinate system
			auto pt_in_transformed = rotation_matrix_ * pt_in_second + translation_vector_;

			// 3. for point transformed, find the closed point in the first scan
			ValueType min_dist = 0;
			size_t closed_pt_idx = -1;
			nanoflann::KNNResultSet<ValueType> ret_set(1);

			ValueType query_pt[2] = { pt_in_transformed[0], pt_in_transformed[1] };
			ret_set.init( &closed_pt_idx, &min_dist );

			kd_tree_ptr_->findNeighbors( ret_set, query_pt );
			
			// 4. 
			if ( min_dist > 0.5 ) {
				continue;
			}

			// 5. get the closed point in the first scan
			auto closed_pt_in_first = first_scan[closed_pt_idx];

			// 6. caculate the error vector
			auto error = pt_in_transformed - closed_pt_in_first;

			mse_ += error.norm();

			// 7. caculate the Jacobian Matrix
			Eigen::Matrix<ValueType, 2, 3> Jacobian = Eigen::Matrix<ValueType, 2, 3>::Zero();
			Jacobian << 1, 0, -pt_in_second[0] * ::sin( p[2] ) - pt_in_second[1] * ::cos( p[2] ),
				    0, 1,  pt_in_second[0] * ::cos( p[2] ) - pt_in_second[1] * ::sin( p[2] ); 

			Hessian_ += Jacobian.transpose() * Jacobian;
			B_ += -Jacobian.transpose() * error;
		}

		if ( Hessian_.determinant() == 0 ) return;

		// 8. caculate the increment of the transformation
		PoseType delta_p = Hessian_.inverse() * B_;

		// 9. update the pose p
		p += delta_p;
		angleNormalize( p[2] );

		// 10. update the rotation matrix & translation vector
		pose2Transformation( p, rotation_matrix_, translation_vector_ );
	}

	void pose2Transformation( const PoseType& p, RotaionType& rotation, TranslationType& translation )
	{
		rotation.setZero();
		translation.setZero();

		rotation << ::cos( p[2] ), -::sin( p[2] ),
			    ::sin( p[2] ),  ::cos( p[2] );

		translation = p.template head<2>();
	}

	void angleNormalize( ValueType& angle )
	{
		if ( angle >= M_PI ) angle -= 2 * M_PI;
		if ( angle <= -M_PI ) angle += 2 * M_PI;
	}

private:
	std::unique_ptr<KdTreeType<ValueType>> kd_tree_ptr_;

	Eigen::Matrix<ValueType, 3, 3> Hessian_ = Eigen::Matrix<ValueType, 3, 3>::Zero();
	Eigen::Matrix<ValueType, 3, 1> B_ = Eigen::Matrix<ValueType, 3, 1>::Zero();

	RotaionType rotation_matrix_ = RotaionType::Zero();
        TranslationType	translation_vector_ = TranslationType::Zero();

	ValueType mse_ = 0;
};

}

#endif

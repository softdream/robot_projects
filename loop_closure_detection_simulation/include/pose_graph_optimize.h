#ifndef __POSE_GRAPH_OPTIMIZE_H
#define __POSE_GRAPH_OPTIMIZE_H

#include "nano_pgo.hpp"
#include "2d_icp.h"

namespace slam
{

template<typename T>
class PoseOptimization
{
public:
	using ValueType = T;
	using PoseType = typename Eigen::Matrix<ValueType, 3, 1>;

	PoseOptimization()
	{
	
	}

	~PoseOptimization()
	{
	
	}

	const bool optimize( const std::vector<PoseType>& key_pose,
                             const std::vector<sensor::ScanContainer>& key_scans )
	{
		PoseType constraint = PoseType::Zero();

		int loop_id = getLoopClosureConstraint( key_pose, key_scans, constraint );
		if ( loop_id == -1 ) return false;		

		for ( int i = 0; i < key_pose.size() - 1; i ++ ) {
			pgo_.addVertex( key_pose[i], i );
			Eigen::Matrix<ValueType, 3, 1> V = pgo_.homogeneousCoordinateTransformation( key_pose[i], key_pose[i + 1] );
			Eigen::Matrix<ValueType, 3, 3> info_matrix = Eigen::Matrix<ValueType, 3, 3>::Zero();
			pgo_.addEdge( V, i, i + 1, info_matrix );
		}
		pgo_.addVertex( key_pose.back(), key_pose.size() - 1 );

		Eigen::Matrix<ValueType, 3, 3> info_matrix = Eigen::Matrix<ValueType, 3, 3>::Zero();
		pgo_.addEdge( constraint, key_pose.size() - 1, loop_id, info_matrix );

		pgo_.execuGraphOptimization( 2 );

	}

	const int getLoopClosureConstraint( const std::vector<PoseType>& key_pose,
					     const std::vector<sensor::ScanContainer>& key_scans,
		       			     PoseType& constraint )
	{
		auto curr_pose = key_pose.back();

		int loop_id = loopClosureDetect( key_pose, curr_pose );

		if ( loop_id == -1 ) return -1;

		std::cout<<"curr pose = "<<curr_pose.transpose()<<std::endl;
		std::cout<<"looped pose = "<<key_pose[loop_id].transpose()<<std::endl;
		//for ( const auto& it : key_pose ) {
		//	std::cout<<it.transpose()<<std::endl;
		//}
		std::cout<<std::endl;

		//constraint = curr_pose - key_pose[loop_id];
		//constraint = key_pose[loop_id] - curr_pose;
		constraint = PoseType::Zero();

		slam::ICP<ValueType> icp;
		sensor::ScanContainer looped_scan = key_scans[loop_id];
		std::cout<<"looped scan size = "<<looped_scan.getSize()<<std::endl;

		sensor::ScanContainer curr_scan = key_scans.back();
		std::cout<<"curr scan size = "<<curr_scan.getSize()<<std::endl;

		icp.solveICP( looped_scan, curr_scan, constraint );
		std::cout<<"constraint = ( "<<constraint.transpose()<<" )"<<std::endl;

		return loop_id;
	}

	const int loopClosureDetect( const std::vector<PoseType>& key_pose, const PoseType& curr_pose )
	{
		key_pose_excluded_.clear();
		key_pose_excluded_.assign( key_pose.begin(), key_pose.end() - NUM_EXCLUDE_RECENT );

		ValueType min_dist = 10000;
		int closed_idx = -1;
		for ( int i = 0; i < key_pose_excluded_.size(); i ++ ) {
			auto dist = ( key_pose_excluded_[i] - curr_pose ).norm();

			if ( dist < min_dist ) {
				min_dist = dist;
				closed_idx = i;
			}
		}	
		
		if ( min_dist > 0.3 ) return -1;
		
		std::cout<<"min_dist ===================================== "<<min_dist<<std::endl;
                std::cout<<"closed_idx =================================== "<<closed_idx<<std::endl;

		return closed_idx;
	}

private:
	std::vector<PoseType> key_pose_excluded_;

	const int NUM_EXCLUDE_RECENT = 10;

	pgo::GraphOptimizer<ValueType> pgo_;
};

}

#endif

#ifndef __SCAN_CONTEXT_PCA_H
#define __SCAN_CONTEXT_PCA_H

#include <iostream>
#include <Eigen/Dense>

#include "data_type.h"

#include "KDTreeVectorOfVectorsAdaptor.h"

#define NUM_RING 20
#define NUM_SECTOR 60

namespace scancontext 
{


template<typename T>
static constexpr T MAX_RADIUS()
{
	return static_cast<T>(10.0);
}
	
template<typename T>
class ScanContextPCA
{
public:
	using DataType = T;
	using Vectors = std::vector<Eigen::Matrix<T, NUM_RING, 1>>;
	using KDTree = KDTreeVectorOfVectorsAdaptor<Vectors, T>;
 

	ScanContextPCA()
	{

	}

	const Eigen::Matrix<DataType, NUM_RING, NUM_SECTOR>& getDesc() const
        {
                return desc;
        }


	void nomilize()
	{
		Eigen::Matrix<DataType, NUM_RING, 1> mean = desc.rowwise().mean();
		desc.colwise() -= mean;
	}

	void caculateCovarince()
	{
		covarince = desc.transpose() * desc * ( 1 / ( desc.rows() - 1 ) );
	}

	void caculateEigenValuesAndEigenVectors(Eigen::Matrix<DataType, NUM_SECTOR, 1> &values, Eigen::Matrix<DataType, NUM_SECTOR, NUM_SECTOR> &vectors)
	{
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix<DataType, NUM_SECTOR, NUM_SECTOR>> eigensolver( covarince );
		values = eigensolver.eigenvalues();
		vectors = eigensolver.eigenvectors();
	}
	
	void caculateResult( const sensor::LaserScan &scan )
	{
		desc = Eigen::Matrix<DataType, NUM_RING, NUM_SECTOR>::Zero();
		covarince = Eigen::Matrix<DataType, NUM_SECTOR, NUM_SECTOR>::Zero();		

		makeScanContext( scan );

		nomilize();
		caculateCovarince();

		Eigen::Matrix<DataType, NUM_SECTOR, 1> values;
		Eigen::Matrix<DataType, NUM_SECTOR, NUM_SECTOR> vectors;
		caculateEigenValuesAndEigenVectors( values, vectors );

		Eigen::Matrix<DataType, NUM_RING, 1> ret = desc * vectors.rightCols(1);
		keys.push_back( ret );
	}

	
	void makeScanContext( const sensor::LaserScan &scan )
        {
                int ring_idx = 0, sctor_idx = 0;
                DataType radians = -3.141592653;

                for( int i = 0; i < scan.size(); i ++ ){
                        DataType dist = scan.ranges[i];
                        DataType angle = rad2deg<DataType>( radians ) + 180.0f;

                        if( dist >= 0.009999998 && dist <= MAX_RADIUS<DataType>() ){
                                ring_idx = std::max( std::min( NUM_RING - 1, static_cast<int>( ceil( ( dist / MAX_RADIUS<DataType>() ) * NUM_RING ) ) ), 0 );

                                sctor_idx = std::max( std::min( NUM_SECTOR - 1, static_cast<int>(ceil( ( angle / 360.0) * NUM_SECTOR ) ) ), 0 );


                                desc( ring_idx, sctor_idx ) += 1;
                        }

                        radians += 0.0043633231;
                }
        }

	const int detectLoopClosure( const sensor::LaserScan &scan, const int curr_index, const std::vector<Eigen::Matrix<DataType, 3, 1>> &key_poses )
        {
                int loop_id = -1;

                caculateResult( scan );
                auto curr_key = keys.back();
                std::cout<<"current key : "<<std::endl<<curr_key<<std::endl;

                if( keys.size() < NUM_EXCLUDE_RECENT + 1 ){
                        return loop_id;
                }

                if( tree_making_period_count % ( TREE_MAKING_PERIOD ) == 0 ){
                        std::cout<<"======================= RECONSTRUCT THE KD TREE ==================="<<std::endl;
                        keysVec.clear();
                        keysVec.assign( keys.begin(), keys.end() - NUM_EXCLUDE_RECENT );
                        kdtree.reset();
                        kdtree = std::make_unique<KDTree>( NUM_RING, keysVec, 10 );
                        std::cout<<"================= FINISHED CONSTRUCT THE KD TREE =================="<<std::endl;
                }
                tree_making_period_count ++;

                std::vector<size_t> candidate_indexes( 10 );
                std::vector<DataType> out_dists_sqr( 10 );

                nanoflann::KNNResultSet<DataType> knnsearch_result( 10 );
                knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
                kdtree->index->findNeighbors( knnsearch_result, &curr_key[0], nanoflann::SearchParams(10) );

                std::cout <<"------------------- KNN SEARCH RESULT -----------------"<<std::endl;
                DataType min_dist = 65536.0;
		Eigen::Vector2f curr_pose( key_poses[curr_index](0), key_poses[curr_index](1) );
		for( size_t i = 0; i < 10; i ++ ){
                        std::cout<<"ret index : [ "<<i<<" ] = "<<candidate_indexes[i]<<", ouot dist sqr = "<<out_dists_sqr[i]<<std::endl;
                	
			Eigen::Vector2f candidate_pose( key_poses[candidate_indexes[i]](0), key_poses[candidate_indexes[i]](1) );
			DataType dist = ( curr_pose - candidate_pose ).norm();
			if( dist < min_dist ){
				min_dist = dist;
				loop_id = candidate_indexes[i];
			}
		}
                std::cout <<"------------------- KNN SERARCH END -------------------"<<std::endl;
	
		std::cout<<"loop id = "<<loop_id<<", min dist = "<<min_dist<<std::endl;
		std::cout<<"looped pose: "<<std::endl<<key_poses[loop_id]<<std::endl<<"current pose : "<<std::endl<<key_poses[curr_index]<<std::endl;

		if( min_dist > 0.4f ){
			return -1;
		}
                return loop_id;
        }


private:
	template<typename TT>
        const TT rad2deg( const TT radians)
        {
                return radians * 180.0 / M_PI;
        }

        template<typename TT>
        const TT deg2rad( const TT angle )
        {
                return angle * M_PI / 180.0;
        }


private:

	Eigen::Matrix<DataType, NUM_RING, NUM_SECTOR> desc = Eigen::Matrix<DataType, NUM_RING, NUM_SECTOR>::Zero();
	Eigen::Matrix<DataType, NUM_SECTOR, NUM_SECTOR> covarince = Eigen::Matrix<DataType, NUM_SECTOR, NUM_SECTOR>::Zero();

	std::vector<Eigen::Matrix<DataType, NUM_RING, 1>> keys;
	Vectors keysVec;

	std::unique_ptr<KDTree> kdtree;

	int NUM_EXCLUDE_RECENT = 15;
	int TREE_MAKING_PERIOD = 5;
	int tree_making_period_count = 0;
};

}

#endif

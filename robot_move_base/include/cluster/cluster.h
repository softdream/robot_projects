#ifndef __CLUSTER_H
#define __CLUSTER_H

#include "data_container.h"

namespace cluster
{


template<typename T>
class Cluster
{
public:
	using DataType = T;
	
	Cluster()
	{

	}

	~Cluster()
	{

	}

	const int extractEuclideanClusters( const sensor::ScanContainer<DataType>& scan_container,
					    std::vector<std::vector<typename sensor::ScanContainer<DataType>::type>>& clusters )
	{
		if( scan_container.isEmpty() ){
			return 0;
		}		

		int nn_start_idx = 0;
		std::vector<bool> processed( scan_container.getSize(), false );
		std::vector<int> nn_indices;

		for( int i = 0; i < scan_container.getSize(); i ++ ){
			if( processed[i] ) continue;
		
			std::vector<int> seed_queue;
			int sq_idx = 0;
			seed_queue.push_back( i );
			processed[i] = true;
				
			while( sq_idx < seed_queue.size() ){
				if( !radiusSearch( scan_container, seed_queue[sq_idx], nn_indices ) ){
					sq_idx ++;
					continue;
				}

				for( int j = nn_start_idx; j < nn_indices.size(); j ++ ){
					if( processed[ nn_indices[j] ] ){
						continue;
					}
				
					seed_queue.push_back( nn_indices[j] );
					processed[nn_indices[j]] = true;
				}
		
				sq_idx ++;
			}

			if( seed_queue.size() > min_pts_per_cluster && seed_queue.size() < max_pts_per_cluster ){
				std::vector<typename sensor::ScanContainer<DataType>::type> cluster;
				for( int j = 0; j < seed_queue.size(); j ++ ){
					typename sensor::ScanContainer<DataType>::type pt = scan_container.getIndexData( seed_queue[j] );//scan_container[seed_queue[j]];
					cluster.push_back( pt );				
				}
				
				clusters.push_back( cluster );
			}
		}
	
		return clusters.size();
	}


	const typename sensor::ScanContainer<DataType>::type getMaxDistAndMean( const std::vector<typename sensor::ScanContainer<DataType>::type>& cluster )
	{
		typename sensor::ScanContainer<DataType>::type pt_all = typename sensor::ScanContainer<DataType>::type::Zero();
		for( auto& pt : cluster ){
			pt_all += pt;
		}	
		pt_all /= static_cast<DataType>( cluster.size() );

		return pt_all;
	}

private:

	const int radiusSearch( const sensor::ScanContainer<DataType>& scan_container, 
				const int curr_idx, 
				std::vector<int>& indices_vec )
	{
		int count = 0;
		typename sensor::ScanContainer<DataType>::type curr_pt = scan_container[ curr_idx ];

		for( int i = 0; i < scan_container.getSize(); i ++ ){
			typename sensor::ScanContainer<DataType>::type candidate_pt = scan_container[ i ];
			if( i != curr_idx && ( curr_pt - candidate_pt ).norm() < min_radius ){
				indices_vec.push_back( i );
				count ++;
			}
		}

		return count;
	}

	
private:
	static int min_pts_per_cluster;
	static int max_pts_per_cluster;
	
	static DataType min_radius;
};

template<typename T>
int Cluster<T>::min_pts_per_cluster = 10;

template<typename T>
int Cluster<T>::max_pts_per_cluster = 160;

template<typename T>
typename Cluster<T>::DataType Cluster<T>::min_radius = 0.1; 

}

#endif

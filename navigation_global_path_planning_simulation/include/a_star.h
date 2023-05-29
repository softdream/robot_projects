#ifndef __A_STAR_H
#define __A_STAR_H

#include <iostream>
#include <set>
#include <vector>
#include <deque>
#include <algorithm>
#include <list>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

namespace planner
{

using MapPoseType = typename Eigen::Matrix<int, 2, 1>;

template<typename T>
struct Cell_
{
	using ValueType = T;

	Cell_()
	{
	
	}

	Cell_( const MapPoseType& pose_ ) : pose( pose_ )
	{
	
	}


	Cell_( const MapPoseType& pose_, const ValueType G_, const ValueType H_, Cell_* parent_ = nullptr ) : pose( pose_ ), G( G_ ), H( H_ ), parent( parent_ )
        {

        }

	bool operator==( const Cell_& rhs ) 
	{
		return ( pose == rhs.pose );
	}

	const ValueType getScore() const
	{
		return H + G;
	}

	ValueType G = 0;
	ValueType H = 0;
	MapPoseType pose = MapPoseType::Zero();

	Cell_* parent = nullptr;
};

template<typename T>
using Cell = Cell_<T>;

template<typename T>
struct Manhattan
{
	using ValueType = T;

	static ValueType distance( const MapPoseType& src, const MapPoseType& dst )
	{
		return static_cast<ValueType>( std::abs( src[0] - dst[0] ) + std::abs( src[1] - dst[1] ) );
	}
};

template<typename T>
struct Euclidean
{
	using ValueType = T;

	static ValueType distance( const MapPoseType& src, const MapPoseType& dst ) 
	{
		return static_cast<ValueType>( ( src - dst ).norm() );
	}
};

template<typename T, typename DistancePolicy = Manhattan<T>>
class AStar
{
public:
	using ValueType = T;
	using WorldPoseType = typename Eigen::Matrix<ValueType, 2, 1>;

	using CellType = Cell<T>;
	
	AStar()
	{
	
	}

	~AStar()
	{
	
	}
	
	void setMap( const cv::Mat& map )
	{
		map_ = map;
	}

	bool findPath( const MapPoseType& src, const MapPoseType& target )
	{
		bool is_path_generated_ = false;

		// 1. initilize the open list and the closed list
		std::vector<CellType*> open_list;
		std::vector<MapPoseType> closed_list;

		// 2. put the starting node on the open list
		open_list.push_back( new CellType( src ) );
		
		// 3. while the open list is not empty
		CellType* current_node = nullptr;
		while ( !open_list.empty() ) {
			auto current_node_iter = open_list.begin();
			current_node = *current_node_iter;

			// find the node with the least 'F' in the open list
			for ( auto it = open_list.begin(); it != open_list.end(); it ++ ) {
				if ( (*it)->getScore() < current_node->getScore() ) {
					current_node = *it;
					current_node_iter = it;
				}
			}

			// if the current node is the target, stop search
			if ( current_node->pose == target ) {
				std::cout<<"find the best path to the target !"<<std::endl;
				is_path_generated_ = true;
				break;
			}

			// remove this node from the open_list
			open_list.erase( current_node_iter );
				
			// add this node to the closed_list
			closed_list.push_back( current_node->pose );

			// generate all the 8 successors of this node/cell
			// 	( i-1, j-1 ) ( i, j-1 ) ( i+1, j-1 )
			// 		   \	    |       /
			// 		    \	    |	   /
			// 		     \      |     /
			// 		      \     |    /
			// 		       \    |   /
			// 	( i-1, j ) ---- ( i, j ) ---- ( i+1, j )
			// 		       /    |   \
			// 		      /     |    \
			// 		     /      |     \
			// 		    /	    |      \
			// 		   /	    |       \
			// 	( i-1, j+1 )  ( i, j+1 ) ( i+1, j+1 )
			
			for ( int i = 0; i < directions_.size(); i ++ ) {
				MapPoseType successor_pose = current_node->pose + directions_[i].first;

				// if the successor is on a occupied cell, continue;
				if ( isOccupied( successor_pose ) || isInUnknowArea( successor_pose ) ){
					//std::cout<<"point is occupied or in unkonw area !"<<std::endl; 
					continue;
				}

				// if the successor is already in the closed list, continue;
				if ( auto it = std::find( closed_list.begin(), closed_list.end(), successor_pose ); it != closed_list.end() ) {
					continue;
				}
				auto total_cost = current_node->G + directions_[i].second;
				
				// if this successor is already in the open list,
				if ( auto successor = findNodeInOpenList( open_list, successor_pose ) ) {
					if ( total_cost < successor->G ) { // if the cost of this direction is lower than the old one, substitute it
						successor->G = total_cost; // update the cost
						successor->parent = current_node; // update the parent
					}
				}	
				else { // else : this successor is not in the open list
					auto H = DistancePolicy::distance( successor_pose, target ); // caculae the value of H
					
					CellType* new_node = new CellType( successor_pose, total_cost, H, current_node ); // create a node and put into the open list
					//std::cout<<"new node .pose = "<<new_node->pose.transpose()<<", .H = "<<new_node->H<<std::endl;

					open_list.push_back( new_node );
				}
			}
		}	

		path_.clear();
		if ( !is_path_generated_ ) {
			std::cout<<"Can not find the path to the goal !"<<std::endl;	
			return false;
		}

		// get the path
		while ( current_node != nullptr ) {
			path_.push_back( current_node->pose );

			current_node = current_node->parent;
		}	
		return true;
	}

	const std::vector<WorldPoseType> bezierSmoothness( )
	{
		std::cout<<"path_.size() ==== "<<path_.size()<<std::endl;

		std::vector<WorldPoseType> pose_world_vec;
		std::vector<WorldPoseType> pose_curve_vec;

		// 1. sample
		int i = path_.size() - 1;
		for ( ; i >= 0; i -= 2 ) {
			pose_world_vec.push_back( WorldPoseType( ( path_[i][0] - 250 ) * 0.1, ( path_[i][1] - 250 ) * 0.1 ) );

		}
		if ( i == -1 ) {
			pose_world_vec.push_back( WorldPoseType( ( path_[0][0] - 250 ) * 0.1, ( path_[0][1] - 250 ) * 0.1 ) );
		}	

		std::cout<<"pose_world_vec.size() ======= "<<pose_world_vec.size()<<std::endl;
		// 2. smooth

		for ( float t = 0; t < 1; t += 0.01 ) {
                	auto pt = cacuBezierCurvePoint( pose_world_vec, t );
			pose_curve_vec.push_back( pt );
        	}
	

		return pose_curve_vec;
	}

	const std::vector<MapPoseType>& getPath() const
	{
		return path_;
	}

private:
	const WorldPoseType cacuBezierCurvePoint( const std::vector<WorldPoseType>& vec, const ValueType t )
	{
		std::vector<WorldPoseType> vec_tmp( vec.size() );

		for ( ValueType i = 1; i < vec.size(); i ++ ) {
			for ( ValueType j = 0; j < vec.size() - 1; j ++ ) {
				if ( i == 1 ) {
					vec_tmp[j] = vec[j] * ( 1 - t ) + vec[j + 1] * t;
					continue;
				}

				vec_tmp[j] = vec_tmp[j] * ( 1 - t ) + vec_tmp[j + 1] * t;
			}
		}

		return vec_tmp[0];
	}

private:	
	const bool isOccupied( const MapPoseType& pose ) 
	{
		return ( map_.at<uchar>( pose[0], pose[1] ) >= 0 && map_.at<uchar>( pose[0], pose[1] ) <= 50  );
	}

	const bool isInUnknowArea( const MapPoseType& pose  ) 
	{
		return ( map_.at<uchar>( pose[0], pose[1] ) <= 150 && map_.at<uchar>( pose[0], pose[1] ) >= 60 );
	}

	CellType* findNodeInOpenList( const std::vector<CellType*>& open_list, const MapPoseType& pose )
	{
		for ( const auto& node : open_list ) {
			if ( node->pose == pose ) {
				return node;
			}
		}

		return nullptr;
	}

private:
	//std::vector<std::vector<uint8_t>> map_; 
	cv::Mat map_;

	std::vector<MapPoseType> path_;

	const std::vector<std::pair<MapPoseType, ValueType>> directions_ = { { { -1, -1 }, 1.414 }, { { 0, -1 }, 1.0 }, { { 1, -1 }, 1.414 }, { { 1, 0 }, 1.0 }, 
							  			       { { 1, 1 }, 1.414 }, { { 0, 1 }, 1.0 }, { { -1, 1 }, 1.414 }, { { -1, 0 }, 1.0 } };
};

}

#endif

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
		return ( src.cast<T>() - dst.cast<T>() ).norm();
	}
};

template<typename T, typename DistancePolicy = Manhattan<T>>
class AStar
{
public:
	using ValueType = T;
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

	void findPath( const MapPoseType& src, const MapPoseType& target )
	{
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

			std::cout<<"current_node.pose = "<<current_node->pose.transpose()<<std::endl;

			// if the current node is the target, stop search
			if ( current_node->pose == target ) break;

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
				//std::cout<<"successor_pose = "<<successor_pose.transpose()<<std::endl;

				// if the successor is in a occupied cell or in unknow area,  continue;
				if ( isOccupied( successor_pose ) || isInUnknowArea( successor_pose ) ) continue;
	
	
				// if the successor is already in the closed list, continue;
				if ( auto it = std::find( closed_list.begin(), closed_list.end(), successor_pose ); it != closed_list.end() ) continue;

				auto total_cost = current_node->G + directions_[i].second;
				std::cout<<"total cost = "<<total_cost<<std::endl;
				// if this successor is already in the open list,
				//if ( auto successor = std::find( open_list.begin(), open_list.end(), CellType( successor_pose ) ); successor != open_list.end() ) {
				if ( auto successor = findNodeInOpenList( open_list, successor_pose ) ) {
					std::cout<<"found in open list"<<std::endl;
					if ( total_cost < successor->G ) { // if the cost of this direction is lower than the old one, substitute it
						successor->G = total_cost; // update the cost
						successor->parent = current_node; // update the parent
					}
				}	
				else { // else : this successor is not in the open list
					std::cout<<"not found in open list"<<std::endl;
					auto H = DistancePolicy::distance( successor_pose, target ); // caculae the value of H
					
					CellType* new_node = new CellType( successor_pose, total_cost, H, current_node ); // create a node and put into the open list
					std::cout<<"new node .pose = "<<new_node->pose.transpose()<<", .H = "<<new_node->H<<std::endl;

					open_list.push_back( new_node );
				}

			}
		}	

		std::cout<<"find the best path to the target !"<<std::endl;
		// get the path
		std::cout<<"current_node.pose = "<<current_node->pose.transpose()<<std::endl;
	
		while ( current_node != nullptr ) {
			//std::cout<<"current_node.pose = "<<current_node->pose.transpose()<<std::endl;
		
			path_.push_back( current_node->pose );
			current_node = current_node->parent;
		}	
	}

	const std::vector<MapPoseType>& getPath() const
	{
		return path_;
	}

private:	
	const bool isOccupied( const MapPoseType& pose ) 
	{
		return ( map_.at<uchar>( pose[0], pose[1] ) == 0 );
	}

	const bool isInUnknowArea( const MapPoseType& pose )
	{
		return ( map_.at<uchar>( pose[0], pose[1] ) == 125 );
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

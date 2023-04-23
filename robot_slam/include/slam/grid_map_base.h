#ifndef __GRID_MAP_BASE_H
#define __GRID_MAP_BASE_H

#include "grid_cell.h"
#include <Eigen/Dense>

#include <iostream>

namespace grid
{

template<typename T, template<typename, typename = void>class GridCellType, 
	 template<typename, typename = void>class GridCellOperateType, 
	 typename = typename std::enable_if<std::is_same<GridCellType<T>, GridCell<T>>::value>::type,
	 typename = typename std::enable_if<std::is_same<GridCellOperateType<T>, GridCellOperations<T>>::value>::type>
class GridMapBase
{
public:
	using DataType =  T;
	using CellType =  GridCellType<T>;

	GridMapBase()
	{
		mapArrayAllocate();
	}

	GridMapBase( const int size_x, const int size_y, const DataType cell_length ) : size_x_( size_x ), size_y_( size_y ), cell_length_( cell_length )
        {
		mapArrayAllocate();
        }

	~GridMapBase()
	{

	}

	void setMapInfo( const int size_x, const int size_y, const DataType cell_length )
	{
		size_x_ = size_x;
		size_y_ = size_y;
		cell_length_ = cell_length;
	}

	const int getSizeX() const 
	{
		return size_x_;
	}

	const int getSizeY() const 
	{
		return size_y_;
	}

	const DataType getCellLength() const
	{
		return cell_length_;
	}

	const DataType getScale() const
	{
		return 1.0 / cell_length_;
	}
	
	const Eigen::Vector2i getMapCenter() const
	{
		return Eigen::Vector2i( static_cast<int>( size_x_ * 0.5 ), static_cast<int>( size_y_ * 0.5 ) );
	}

	CellType& getCell( const int x, const int y ) 
	{
		return map_array_[ y * size_x_ + x ];
	}

	CellType& getCell( const int index ) 	
	{
		return map_array_[ index ];
	}

	const Eigen::Matrix<DataType, 2, 1> observedPointWorld2Map( const Eigen::Matrix<DataType, 2, 1> &point_in_world ) const
	{
		//return point_in_world * getScale() + getMapCenter().cast<DataType>();
		Eigen::Vector2i map_center = getMapCenter();
		return point_in_world * getScale() + map_center.cast<DataType>();
	}

	const Eigen::Matrix<DataType, 2, 1> observedPointMap2World( const Eigen::Matrix<DataType, 2, 1> &point_in_map ) const
	{
		Eigen::Vector2i map_center = getMapCenter();
		return ( point_in_map - map_center.cast<DataType>() ) * cell_length_;
	}

	const Eigen::Matrix<DataType, 3, 1> robotPoseWorld2Map( const Eigen::Matrix<DataType, 3, 1> &pose_in_world ) const
	{
		Eigen::Vector2i map_center = getMapCenter();
		Eigen::Matrix<DataType, 2, 1> pose_in_world_head( pose_in_world[0], pose_in_world[1] );
		Eigen::Matrix<DataType, 2, 1> tmp( pose_in_world_head * getScale() + map_center.cast<DataType>() );
		return Eigen::Matrix<DataType, 3, 1>( tmp[0], tmp[1], pose_in_world[2] );
	}
	
	const Eigen::Matrix<DataType, 3, 1> robotPoseMap2World( const Eigen::Matrix<DataType, 3, 1> &pose_in_map ) const
	{
		Eigen::Vector2i map_center = getMapCenter();
		Eigen::Matrix<DataType, 2, 1> pose_in_map_head( pose_in_map[0], pose_in_map[1] );
		Eigen::Matrix<DataType, 2, 1> tmp( ( pose_in_map_head - map_center.cast<DataType>() ) * cell_length_ );
		return Eigen::Matrix<DataType, 3, 1>( tmp[0], tmp[1], pose_in_map[2] );
	}
	
	bool isPointOutOfRange( const Eigen::Matrix<DataType, 2, 1> &point ) const
	{
		return ( point[0] < 0.0 || point[1] < 0.0 || point[0] > static_cast<DataType>( size_x_ ) || point[1] > static_cast<DataType>( size_y_ ) );
	}

	bool isPointOutOfRange( const DataType x, const DataType y ) const
        {
                return ( x < 0.0 || y < 0.0 || x > static_cast<DataType>( size_x_ ) || y > static_cast<DataType>( size_y_ ) );
        }

	void printMapInfo() const
	{
		std::cout<<"---------- Map Information ----------"<<std::endl;
        	std::cout<<"Map Size X      : "<<size_x_<<std::endl;
	        std::cout<<"Map Size Y      : "<<size_y_<<std::endl;
        	std::cout<<"Map Cell Length : "<<cell_length_<<std::endl;
	        std::cout<<"Map Center      : ( "<<getMapCenter()[0]<<", "<<getMapCenter()[1]<<" )"<<std::endl;
        	std::cout<<"Map Scale       : "<<getScale()<<std::endl;
		std::cout<<"-------------------------------------"<<std::endl;
	}

	const size_t getCellsNumber() const
	{
		return map_array_.size();
	}

	void setCellOccupied( const int x, const int y )
	{
		return cell_operate_.setCellOccupied( getCell( x, y ) );	
	}

	void setCellOccupied( const int index )
	{
		return cell_operate_.setCellOccupied( getCell( index ) );
	}

	void setCellFree( const int x, const int y )
	{
		return cell_operate_.setCellFree( getCell( x, y ) );
	}
	
	void setCellFree( const int index )
        {
                return cell_operate_.setCellFree( getCell( index ) );
        }
		
	void setCellUnFree( const int x, const int y )
	{
		return cell_operate_.setCellUnFree( getCell( x, y ) );
	}

	void setCellUnFree( const int index )
        {
                return cell_operate_.setCellUnFree( getCell( index ) );
        }

	const DataType getCellOccupiedProbability( const int x, const int y )
	{
		return cell_operate_.getCellProbability( getCell( x, y ) );
	}

	const DataType getCellOccupiedProbability( const int index )
        {
                return cell_operate_.getCellProbability( getCell( index ) );
        }

	void setLogOddsPoccValue( const DataType p_occ )
	{
		return cell_operate_.setLogOddsPocc( p_occ );
	}

	void setLogOddsPfreeValue( const DataType p_free )
	{
		return cell_operate_.setLogOddsPfree( p_free );
	}

	const DataType getLogOddsPoccValue() const
	{
		return cell_operate_.getLogOddsPocc();
	}

	const DataType getLogOddsPfreeValue() const
	{
		return cell_operate_.getLogOddsPfree();
	}

	bool isCellOccupied( const int x, const int y )
	{
		return getCell( x, y ).isOccupied();
	}

	bool isCellOccupied( const int index )
        {
                return getCell( index ).isOccupied();
        }

	bool isCellFree( const int x, const int y )
	{
		return getCell( x, y ).isFree();
	}
	
	bool isCellFree( const int index )
        {
                return getCell( index ).isFree();
        }

	bool isCellUnknow( const int x, const int y )
	{
		return getCell( x, y ).isUnknow();
	}

	bool isCellUnknow( const int index )
        {
                return getCell( index ).isUnknow();
        }

	const DataType getCellLogOdds( const int x, const int y )
	{
		return getCell( x, y ).log_odds_value_;
	}

	const DataType getCellLogOdds( const int index ) 
        {
                return getCell( index ).log_odds_value_;
        }

	const std::vector<CellType>& getMapArray() const
	{
		return map_array_;
	}

	void setMapArray( const std::vector<CellType> &map_array )
	{
		map_array_ = map_array;
	}
	
protected:
	void mapArrayAllocate()
	{
		map_array_.resize( size_x_ * size_y_ );
		
		if( map_array_.size() != size_x_ * size_y_ ){
			std::cerr<<"allocate the memory for the map failed !"<<std::endl;
			exit(-1);
		}
		std::cerr<<"allocate the memory for the map !"<<std::endl;
	}

	void mapArrayClear()
	{
		return map_array_.clear();
	}
	

protected:
	std::vector<CellType> map_array_;
	GridCellOperateType<T> cell_operate_;
	
	int size_x_ = 1001;
	int size_y_ = 1001;
	DataType cell_length_ = 0.1;
};

}

#endif

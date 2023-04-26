#ifndef __OCCUPIED_GRID_MAP
#define __OCCUPIED_GRID_MAP

#include "grid_map_base.h"

#include "dataContainer.h"

namespace grid
{

template<typename T>
using GridMapBaseType = GridMapBase<T, GridCell, GridCellOperations>;

template<typename T, typename = typename std::enable_if<is_double_or_float<T>::value>::type>
class OccupiedGridMap : public GridMapBaseType<T>
{
public:
	using ConstSharedPtr = const std::shared_ptr<OccupiedGridMap<T>>;
	using SharedPtr = std::shared_ptr<OccupiedGridMap<T>>;
	using DataType = T;
	using CellType = GridCell<T>;	

	OccupiedGridMap()
	{

	}

	~OccupiedGridMap()
	{
	
	}

	OccupiedGridMap( const int map_size_x, const int map_size_y, const DataType cell_length ) : GridMapBaseType<T>( map_size_x, map_size_y, cell_length )
	{
		
	}

	const Eigen::Matrix<DataType, 2, 1> observedPointLaser2World( const Eigen::Matrix<DataType, 2, 1> &point_in_laser, const Eigen::Matrix<DataType, 3, 1> &robot_pose_in_world ) const
	{
		Eigen::Matrix<DataType, 2, 2> rotate;
		rotate << ::cos( robot_pose_in_world[2] ), -::sin( robot_pose_in_world[2] ),
			  ::sin( robot_pose_in_world[2] ),  ::cos( robot_pose_in_world[2] );
		Eigen::Matrix<DataType, 2, 1> trans( robot_pose_in_world[0], robot_pose_in_world[1] );
		return rotate * point_in_laser + trans;
	}

	const Eigen::Matrix<DataType, 2, 1> observedPointWorld2Laser( const Eigen::Matrix<DataType, 2, 1> &point_in_world, const Eigen::Matrix<DataType, 3, 1> &robot_pose_in_world ) const
	{
		Eigen::Matrix<DataType, 2, 2> rotate;
                rotate << ::cos( robot_pose_in_world[2] ), -::sin( robot_pose_in_world[2] ),
                          ::sin( robot_pose_in_world[2] ),  ::cos( robot_pose_in_world[2] );
		Eigen::Matrix<DataType, 2, 1> trans( robot_pose_in_world[0], robot_pose_in_world[1] );
		return rotate.inverse() * point_in_world - trans;
	}

	template<typename = typename std::enable_if<std::is_same<DataType, sensor::ScanContainer::value_type>::value>::type>
	void updateMapByScan( const sensor::ScanContainer &scan, const Eigen::Matrix<DataType, 3, 1> &robot_pose_in_world )
	{
		curr_mark_free_index_ = curr_update_index_ + 1;
        	curr_mark_occ_index_ = curr_update_index_ + 2;

		// 1. Transform robot Pose In world Coordinate to Map Coordinat
        	Eigen::Matrix<DataType, 3, 1> robot_pose_in_map = this->robotPoseWorld2Map( robot_pose_in_world );
		// std::cout<<"Robot Pose In Map Coordinate: "<<std::endl;
        	// std::cout<<robotPoseInMap<<std::endl;
	
		// 2. Get the start point of the laser data in Map Coordinate
        	Eigen::Vector2i begin_point_in_map_i( static_cast<int>( robot_pose_in_map[0] ), static_cast<int>( robot_pose_in_map[1] ) );
		
		size_t numberOfBeams = scan.getSize();
        	//std::cout<<"Number Of Beams: "<<numberOfBeams<<std::endl;
		for( size_t i = 0; i < numberOfBeams; i ++ ){
			//std::cout<<"----------------------------"<<std::endl;
			// 3. Get the End point of Every Laser Beam in Laser Coordinate
                	Eigen::Matrix<DataType, 2, 1> end_point_in_laser = scan.getIndexData( i ) ;
			//std::cout<<"end point in laser: "<<i<<std::endl<<end_point_in_laser<<std::endl;
		
			// 4. Transform the End Point from Laser Coordinate to World Coordinate
                	Eigen::Matrix<DataType, 2, 1> end_point_in_world( this->observedPointLaser2World( end_point_in_laser, robot_pose_in_world ) );
			//std::cout<<"end point in world: "<<i<<std::endl<<end_point_in_world<<std::endl;		

			// 5. Transform the End Point from World Coordinate to Map Coordinate
                	Eigen::Matrix<DataType, 2, 1> end_point_in_map( this->observedPointWorld2Map( end_point_in_world ) );
			//std::cout<<"end point in map: "<<i<<std::endl<<end_point_in_map<<std::endl;

			// 6. Convert float to interger
			Eigen::Vector2i end_point_in_map_i( static_cast<int>( ::round( end_point_in_map[0] ) ), static_cast<int>( ::round( end_point_in_map[1] ) ) );

			// 7. execuate Inverse Model algorithm
                	if( end_point_in_map_i != begin_point_in_map_i ){
                        	this->inverseModel( begin_point_in_map_i, end_point_in_map_i );
                	}
		}

		curr_update_index_ += 3;
	}

        void clearGridMap()
	{
		return this->mapArrayClear();
	}
	
	void resetGridMap()
	{
		for( size_t i = 0; i < this->map_array_.size(); i ++ ){
			this->map_array_[i].log_odds_value_ = 0;
		}
	}

	const int getCurrUpdateIndex() const
	{
		return curr_update_index_;
	}

	const int getCurrMarkOccIndex() const
	{
		return curr_mark_occ_index_;
	}

	const int getCurrMarkFreeIndex() const
	{
		return curr_mark_free_index_;
	}
	
private:	
	void inverseModel( const int x0, const int y0, const int x1, const int y1 )
        {
		// 1. set the end point occupied first
        	bresenhamCellOccupied( x1, y1 );
		
		//std::cout<<"Occupied Cell Pose In Map: ( "<< x1 <<", "<< y1<<" )"<<std::endl;
		// 2. execute the bresenham algorithm, find the points of free, and set them free
        	bresenHam( x0, y0, x1, y1 );
        }

        void inverseModel( const Eigen::Vector2i &p0, const Eigen::Vector2i &p1 )
        {
         	return inverseModel( p0[0], p0[1], p1[0], p1[1] );
        }
	

	void bresenHam( const int x0, const int y0, const int x1, const int y1 )
	{
		int dx = ::abs( x1 - x0 );
        	int dy = ::abs( y1 - y0 );

        	bool interChange = false;

        	int e = -dx;// error

        	int signX = x1 > x0 ? 1 : ( ( x1 < x0 ) ? -1 : 0 );
        	int signY = y1 > y0 ? 1 : ( ( y1 < y0 ) ? -1 : 0 );

        	if (dy > dx) {
                	int temp = dx;
                	dx = dy;
                	dy = temp;
                	interChange = true;
        	}
	
		int x = x0, y = y0;
        	for (int i = 1; i <= dx; i++) { // not include the end point
                	//operate( x, y, img1 );
                	// add operations for the point 
                	bresenhamCellFree( x, y );

                	if (!interChange)
                        	x += signX;
                	else
                        	y += signY;

                	e += 2 * dy;

                	if (e >= 0) {
                        	if (!interChange)
                                	y += signY;
                        	else
                                	x += signX;

                        	e -= 2 * dx;
                	}
        	}
	}
        
	void bresenHam( Eigen::Vector2i &p0, Eigen::Vector2i &p1)
	{
		return bresenHam( p0[0], p0[1], p1[0], p1[1] );
	}

        void bresenhamCellFree( const int index )	
	{
		grid::GridCell<T> &cell = this->getCell( index );

	        if( cell.update_index_ < curr_mark_free_index_ ){
        	        this->setCellFree( index );

                	cell.update_index_ = curr_mark_free_index_; // avoid reUpdate
        	}
	}

	void bresenhamCellFree( const int x, const int y )
        {
		grid::GridCell<T>  &cell = this->getCell( x, y );

                if( cell.update_index_ < curr_mark_free_index_ ){
                        this->setCellFree( x, y );

                        cell.update_index_ = curr_mark_free_index_; // avoid reUpdate
                }
        }

        void bresenhamCellOccupied( const int index )
	{
		grid::GridCell<T>  &cell = this->getCell( index );

	        if( cell.update_index_ < curr_mark_occ_index_ ){
        	        if( cell.update_index_ == curr_mark_occ_index_ ){
                	        this->setCellUnFree( index );
                	}
                	this->setCellOccupied( index );

                	cell.update_index_ = curr_mark_occ_index_; // avoid reUpdate
        	}

	}

        void bresenhamCellOccupied( const int x, const int y )
	{
		grid::GridCell<T> &cell = this->getCell( x, y );

                if( cell.update_index_ < curr_mark_occ_index_ ){
                        if( cell.update_index_ == curr_mark_occ_index_ ){
                                this->setCellUnFree( x, y );
                        }
                        this->setCellOccupied( x, y );

                        cell.update_index_ = curr_mark_occ_index_; // avoid reUpdate
                }

	}

private:
	int curr_update_index_ = 0;
	int curr_mark_occ_index_ = -1;
	int curr_mark_free_index_ = -1;
};

}


#endif

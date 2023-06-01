#ifndef __LOOP_CLOSURE_DETECT_H
#define __LOOP_CLOSURE_DETECT_H

#include <iostream>
#include "data_type.h"

#define LASER_DIST 5.0
#define ROWS 25
#define COLS 36
#define DIMENSION ( ROWS * 4 )

namespace slam
{

template<typename T>
struct NDCell_
{
	using ValueType = T;
	using Vector2 = typename Eigen::Matrix<ValueType, 2, 1>;
	using Matrix2 = typename Eigen::Matrix<ValueType, 2, 2>;


	Vector2 mean = Vector2::Zero();
	Matrix2 covarince = Matrix2::Identity();
	std::vector<Vector2> points;
};

template<typename T>
using NDCell = NDCell_<T>;

template<typename T, template<typename > class CellType = NDCell>
class LoopDetect
{
public:
	using ValueType = T;
	using PointType = typename CellType<ValueType>::Vector2;
	using CovarinceType = typename CellType<ValueType>::Matrix2;
	using MapPointType = Eigen::Vector2i;
	
	using DescriptorType = typename Eigen::Matrix<ValueType, DIMENSION, 1>;

	LoopDetect()
	{
	
	}

	~LoopDetect()
	{
	
	}

	void makeScanContext( const sensor::LaserScan& scan )
	{
		// 1. cacluate the normal distribution
		cacuNDGrid( scan );

		// 2. caculate the descriptor
		desc_.setZero();
		for ( int i = 0; i < ROWS; i ++ ) {
			for ( int j = 0; j < COLS; j ++ ) {
				desc_[i * 4 + 0] += grid_[i][j].mean(0);
				desc_[i * 4 + 1] += grid_[i][j].mean(1);
				desc_[i * 4 + 2] += grid_[i][j].covarince( 0, 0 );
				desc_[i * 4 + 3] += grid_[i][j].covarince( 1, 1 );
			}
		}

		desc_ /= static_cast<ValueType>( COLS );
		std::cout<<"desc_ : "<<std::endl<<desc_<<std::endl;
		
	}

private:
	void cacuNDGrid( const sensor::LaserScan& scan )
	{
		grid_.resize( ROWS, std::vector<CellType<ValueType>>( COLS ) );

		ValueType radians = -M_PI;

		for ( int i = 0; i < scan.size(); i ++ ) {
			auto dist = scan.ranges[i];
			ValueType angle = radians * 180 / M_PI + 180.0;

			if ( dist >= 0.1 && dist < LASER_DIST ) {
				int idx_x = std::max( std::min( ROWS - 1, static_cast<int>( std::ceil( ( dist / LASER_DIST ) * ROWS ) ) ), 0 );
				int idx_y = std::max( std::min( COLS - 1, static_cast<int>( std::ceil( ( angle / 360.0 ) * COLS ) ) ), 0 );

				grid_[idx_x][idx_y].mean += PointType( dist, radians );
				grid_[idx_x][idx_y].points.push_back( PointType( dist, radians ) );
			}

			radians += scan.angle_increment;
		}

		for ( int i = 0; i < ROWS; i ++ ) {
			for ( int j = 0; j < COLS; j ++ ) {
				int pt_num = grid_[i][j].points.size();
				if ( pt_num >= 3 ) {
					auto average = grid_[i][j].mean / static_cast<ValueType>( pt_num );
					grid_[i][j].mean = average;

					for ( const auto& pt : grid_[i][j].points ) {
						CovarinceType sigma = ( pt - grid_[i][j].mean ) * ( pt - grid_[i][j].mean ).transpose();
						grid_[i][j].covarince += sigma;
					}

					grid_[i][j].covarince /= static_cast<ValueType>( pt_num );
				}
				else {
					grid_[i][j].covarince.setZero();
					grid_[i][j].mean.setZero();
				}

			}
		}

	}

	const ValueType cacuProbability( const PointType& x, const PointType& mean, const CovarinceType& covarince ) const
	{
		return ::exp( -( ( x - mean ).transpose() * covarince.inverse() * ( x - mean ) ) * 0.5 );
	}

private:
	std::vector<std::vector<CellType<ValueType>>> grid_;
	
	DescriptorType desc_ = DescriptorType::Zero();
};

}

#endif

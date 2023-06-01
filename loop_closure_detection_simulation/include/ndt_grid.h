#ifndef __NDT_GRID_H
#define __NDT_GRID_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <type_traits>

#include <opencv2/opencv.hpp>

#include <limits>

#include "dataContainer.h"

#define WINDOW_WIDTH 1200
#define WINDOW_HEIGHT 1200

#define LASER_DIST 10.0
#define ROWS 20
#define COLUMNS 20
#define HALF_ROWS 10
#define HALF_COLUMNS 10


namespace ndt{

template<typename T>
struct is_double_or_float
{
        static const bool value = false;
};

template<>
struct is_double_or_float<float>
{
        static const bool value = true;
};

template<>
struct is_double_or_float<double>
{
        static const bool value = true;
};

template<typename T, int Rows, int Cols,
        template<typename U, int R, int C, int Option, int MaxR, int MaxC>
        class EigenType>
struct is_Eigen_type
{
        static const bool value = false;
};

template<typename T, int Rows, int Cols>
struct is_Eigen_type<T, Rows, Cols, Eigen::Matrix>
{
        using type = Eigen::Matrix<T, Rows, Cols>;
        static const bool value = true;
};

template<typename T, typename = typename std::enable_if<is_double_or_float<T>::value>::type>
static constexpr T CONST_PI()
{
        return static_cast<T>( 3.14159265358979323846 );
}

template<typename T, typename = typename std::enable_if<is_double_or_float<T>::value>::type>
static constexpr T CONST_TWO_PI()
{
	return CONST_PI<T>() * 2.0;
}


template<typename T, typename = typename std::enable_if<is_double_or_float<T>::value>::type>
static constexpr T laser_max_dist()
{
	return static_cast<T>( LASER_DIST );
}

template<typename T, typename = typename std::enable_if<is_double_or_float<T>::value>::type>
static constexpr T get_row_range()
{
	return laser_max_dist<T>() / HALF_ROWS;
}

template<typename T, typename = typename std::enable_if<is_double_or_float<T>::value>::type>
static constexpr T get_column_range()
{
	return laser_max_dist<T>() / HALF_COLUMNS;
}

template<typename T, typename = typename std::enable_if<is_double_or_float<T>::value>::type>
static void angleNormalize( T &angle )
{
	if( angle >= CONST_PI<T>() ) {
		angle -= CONST_TWO_PI<T>();
	}

	if( angle <= -CONST_PI<T>() ){
		angle += CONST_TWO_PI<T>();
	}
}


template<typename DataType,
                typename = typename std::enable_if<is_Eigen_type<typename DataType::value_type, DataType::RowsAtCompileTime, DataType::ColsAtCompileTime, Eigen::Matrix>::value>::type>
struct GridCell
{
	using type = DataType;
	using value_type = typename DataType::value_type;
	using covarince_type = typename Eigen::Matrix<value_type, 2, 2>;
	
	GridCell()
	{

	}
	
	~GridCell()
	{

	}

	GridCell( const DataType &mean, const DataType &covarince, const int number ) : mean_( mean ), covarince_( covarince ), number_( number )
	{
		
	}
	
	DataType mean_ = DataType::Zero();;
	covarince_type covarince_ = covarince_type::Zero();
	int number_ = 0;
	
	std::vector<DataType> points;

	typename DataType::value_type probablity_ = 0; 
};

template<typename T>
using GridCellType = GridCell<Eigen::Matrix<T, 2, 1>>;

template<typename T, template<typename U> class GridType = GridCellType>
class NdtGrid
{
public:
	using PointType = typename GridType<T>::type;
	using DataType = typename GridType<T>::value_type;
	using PoseType = typename Eigen::Matrix<DataType, 3, 1>;
	using CovarinceType = typename GridType<T>::covarince_type;

	NdtGrid() {  }
	~NdtGrid() {  } 

	bool ndtProcess( const sensor::ScanContainer &first_scan,
			  const sensor::ScanContainer &second_scan,
			  PoseType &p,
			  const int max_iterations = 5 )
	{
		if( first_scan.isEmpty() || second_scan.isEmpty() ){
			return false;
		}	

		caculateNDTByFirstScan( first_scan );
		
		int iteration = 0;
		for( ; iteration < max_iterations; iteration ++ ){
			
			//showTwoScanFrame( first_scan, second_scan, p );	
			
			estimateTransformationOnce( second_scan, p );
			
			if ( std::isnan( delta_norm ) || delta_norm > 1 || delta_norm < -1 ) {
				std::cout << "nan ..." << std::endl;
				score = 65536;
				return false;
			}	
		}
		
		angleNormalize( p[2] );
		//std::cout<<"score = "<<score<<std::endl;

		return true;
	}	

	const DataType& getMatchScore() const
	{
		return score;
	}

private:
	void caculateNDTByFirstScan( const sensor::ScanContainer &scan )
	{
		//memset( grid.data(), 0, grid.size() * sizeof( grid[0] ) );	
		for( int i = 0; i < grid.size(); i ++ ){
                        grid[i].number_ = 0;
			grid[i].mean_ = PointType::Zero();
			grid[i].covarince_ = CovarinceType::Zero();
                        grid[i].points.clear();	
                }
		score = 0;
		delta_norm = 0;

		for( size_t i = 0; i < scan.getSize(); i ++ ){
			PointType point = scan.getIndexData( i );
			int index = pointMapToGrid( point );
			
			grid[index].number_ ++;
			grid[index].mean_ += point;
			grid[index].points.push_back( point );
		}	

		for( size_t i = 0; i < grid.size(); i ++ ){
			if( grid[i].number_ >= 3 ){
				PointType average = grid[i].mean_ / static_cast<DataType>( grid[i].number_ );
				grid[i].mean_ = average;
				for( auto item : grid[i].points ){
					CovarinceType sigma = ( item - grid[i].mean_ ) * ( item - grid[i].mean_ ).transpose();
					grid[i].covarince_ += sigma;
				}
				
				grid[i].covarince_ /= static_cast<DataType>( grid[i].number_  - 1 );
			}
			else {
				CovarinceType cov;
				cov << std::numeric_limits<DataType>::max(), 0, 0, std::numeric_limits<DataType>::max();
				grid[i].covarince_ = cov;
			}
		}
		
	}

	void getHessianDerived( const sensor::ScanContainer &scan, 
	    	           const PoseType &p,
			   Eigen::Matrix<DataType, 3, 3> &H,
			   Eigen::Matrix<DataType, 3, 1> &b )
	{
		H = Eigen::Matrix<DataType, 3, 3>::Zero();
		b = Eigen::Matrix<DataType, 3, 1>::Zero();

		for( size_t i = 0; i < scan.getSize(); i ++ ){
			// for all points in second scan frame, transform
                        PointType point = scan.getIndexData( i );
			PointType point_in_first_frame = pointCoordinateTransform( point, p );                
			
		        int index = pointMapToGrid( point_in_first_frame );

			PointType e = point_in_first_frame - grid[index].mean_;
			CovarinceType sigma_inverse = ( grid[index].covarince_ ).inverse();

			DataType tmp1 = -point[0] * ::sin( p[2] ) - point[1] * ::cos( p[2] );
			DataType tmp2 =  point[0] * ::cos( p[2] ) - point[1] * ::sin( p[2] );
			Eigen::Matrix<DataType, 2, 3> Jacobian;
			Jacobian << 1, 0,  tmp1,
				    0, 1,  tmp2;


			b += ( e.transpose() * sigma_inverse * Jacobian ).transpose();
			H += Jacobian.transpose() * sigma_inverse * Jacobian;			
			score += caculateProbability( point_in_first_frame, grid[index].mean_,  grid[index].covarince_ );
		}
		
	}	

	void estimateTransformationOnce( const sensor::ScanContainer &scan,
                           		 PoseType &p )	
	{
		getHessianDerived( scan, p, H, b );

		PoseType delta_p = -H.inverse() * b;
		p += delta_p;
		//std::cout<<"delta_p.norm() = "<<delta_p.norm()<<std::endl;
		delta_norm = delta_p.norm();
	}

private:	
	const int pointMapToGrid( const PointType &point ) const
	{
		int x = ( point[0] / get_column_range<DataType>() );
		int y = ( point[1] / get_row_range<DataType>() );
	
		if( point[1] >= 0 ){
			if( point[0] >= 0 ){
				return ( HALF_ROWS - y - 1 ) * COLUMNS + ( x + HALF_COLUMNS + 1 );
			}
			else {
				return ( HALF_ROWS - y - 1 ) * COLUMNS + ( x + HALF_COLUMNS );
			}
		}
		else {
			if( point[0] >= 0 ){
				return ( HALF_ROWS - y ) * COLUMNS + ( HALF_COLUMNS + x ) + 1;
			}
			else {
				return ( HALF_ROWS - y ) * COLUMNS + ( HALF_COLUMNS + x );
			}
		}
	}
	
	const DataType caculateProbability( const PointType &x, const PointType &mean, const CovarinceType &covarince )
	{
		DataType tmp = ( x - mean ).transpose() * covarince.inverse() * ( x - mean );
		return ::exp( -( tmp * 0.5 ) );
	}	

	const PoseType poseCoordinateTransform( const PoseType &pose_old, const PoseType &delta ) const
	{
		Eigen::Matrix<DataType, 3, 3> trans;
		trans << ::cos( delta[2] ), -::sin( delta[2] ), delta[0],
			 ::sin( delta[2] ),  ::cos( delta[2] ), delta[1],
				0	  ,	    0	      ,    1	;
	
		return trans * pose_old;
	}

	const PointType pointCoordinateTransform( const PointType &point_old, const PoseType &delta ) const
	{
		Eigen::Matrix<DataType, 2, 2> rotate;
		rotate << ::cos( delta[2] ), -::sin( delta[2] ), 
			  ::sin( delta[2] ),  ::cos( delta[2] );
	
		Eigen::Matrix<DataType, 2, 1> trans( delta[0], delta[1] );
		return rotate * point_old + trans;
	}

public:
	void showTwoScanFrame( const sensor::ScanContainer &container1, const sensor::ScanContainer &container2, const PoseType &delta, const float scale = 20 ) const
	{
		cv::Mat image = cv::Mat::zeros( WINDOW_WIDTH, WINDOW_HEIGHT, CV_8UC3 );

	        cv::Point2d center( WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2 );
        	cv::circle(image, center, 1, cv::Scalar(0, 255, 0), 1);
	        cv::line( image, cv::Point( WINDOW_WIDTH / 2, 0 ), cv::Point( WINDOW_WIDTH / 2, WINDOW_HEIGHT ), cv::Scalar( 67, 128, 94 ), 1 );
        	cv::line( image, cv::Point( 0, WINDOW_HEIGHT / 2 ), cv::Point( WINDOW_WIDTH, WINDOW_HEIGHT / 2 ), cv::Scalar( 67, 128, 94 ), 1 );

	        for( int i = 0; i < container1.getSize(); i ++ ){
        	        cv::Point2d point( container1.getIndexData(i)[0] * scale + WINDOW_WIDTH / 2, container1.getIndexData(i)[1] * scale + WINDOW_HEIGHT / 2 );
                	cv::circle(image, point, 1, cv::Scalar(0, 0, 255), -1);
        	}
			
		for( int i = 0; i < container2.getSize(); i ++ ){
                	Eigen::Matrix<float, 2, 2> rotate;
                	rotate << ::cos( delta[2] ), -::sin( delta[2] ),
                          	  ::sin( delta[2] ),  ::cos( delta[2] );
                	Eigen::Matrix<float, 2, 1> trans( delta[0], delta[1] );

                	Eigen::Vector2f point_old( container2.getIndexData(i)[0], container2.getIndexData(i)[1] );
                	Eigen::Vector2f point_new = rotate * point_old + trans;

                	cv::Point2d point( point_new[0] * scale + WINDOW_WIDTH / 2, point_new[1] * scale + WINDOW_HEIGHT / 2 );
                	cv::circle(image, point, 1, cv::Scalar(0, 255, 0), -1);
        	}

        	cv::imshow( "scan", image );
        	cv::waitKey(10);
	}

private:
	DataType delta_norm = 0.0;
	DataType score = 0.0;

	std::vector<GridType<T>> grid = std::vector<GridType<T>>( COLUMNS * ROWS + 1 );
	
	Eigen::Matrix<DataType, 3, 3> H;
        Eigen::Matrix<DataType, 3, 1> b;
};

using NDT = NdtGrid<float>;

}

#endif



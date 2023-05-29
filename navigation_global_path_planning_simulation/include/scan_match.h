#ifndef __SCAN_MATCH_H
#define __SCAN_MATCH_H

#include "occupied_grid_map.h"
#include <iostream>

namespace match
{

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

template<typename T, typename = typename std::enable_if<is_double_or_float<T>::value>::type>
class ScanMatchMethod
{
public:
	using DataType = T;
	using OccupiedMap = typename grid::OccupiedGridMap<T>;
	

	ScanMatchMethod()
	{

	}

	~ScanMatchMethod()
	{

	}

	const DataType bilinearInterpolation( const OccupiedMap &occ_map, const Eigen::Matrix<DataType, 2, 1> &coords )
	{
		if( occ_map.isPointOutOfRange( coords ) ){
                	return 0.0;
        	}	

		Eigen::Vector2i indMin( static_cast<int>( coords[0] ), static_cast<int>( coords[1] ) );
		
		DataType factor0 = coords[0] - static_cast<DataType>( indMin[0] );
        	DataType factor1 = coords[1] - static_cast<DataType>( indMin[1] );
		
		int sizeX = occ_map.getSizeX();
        	int index = indMin[1] * sizeX + indMin[0];
	
		mP00 = occ_map.getCellOccupiedProbability( index );
		
		index ++;
		mP10 = occ_map.getCellOccupiedProbability( index );

		index += sizeX - 1;
		mP01 = occ_map.getCellOccupiedProbability( index );
		
		index ++;
        	mP11 = occ_map.getCellOccupiedProbability( index );
	
		DataType factorInv0 = 1.0f - factor0;
        	DataType factorInv1 = 1.0f - factor1;		
		
		return ( factor1 * ( factor0 * mP11 + factorInv0 * mP01 ) ) + ( factorInv1 * ( factor0 * mP10 + factorInv0 * mP00 ) );
	}

	const Eigen::Matrix<DataType, 3, 1> bilinearInterpolationWithDerivative( OccupiedMap &occ_map, const Eigen::Matrix<DataType, 2, 1> &coords )
	{
		if( occ_map.isPointOutOfRange( coords ) ){
        	        return Eigen::Matrix<DataType, 3, 1>( 0.0f, 0.0f, 0.0f );
	        }

		Eigen::Vector2i indMin( static_cast<int>( coords[0] ), static_cast<int>( coords[1] ) );
	
		DataType factor0 = coords[0] - static_cast<DataType>( indMin[0] );
        	DataType factor1 = coords[1] - static_cast<DataType>( indMin[1] );
		
		int sizeX = occ_map.getSizeX();
        	int index = indMin[1] * sizeX + indMin[0];
		
		mP00 = occ_map.getCellOccupiedProbability( index );
		
		index ++;
        	mP10 = occ_map.getCellOccupiedProbability( index );
	
		index += sizeX - 1;
        	mP01 = occ_map.getCellOccupiedProbability( index );

		index ++;
       	 	mP11 = occ_map.getCellOccupiedProbability( index );

		DataType factorInv0 = 1.0f - factor0;
       	 	DataType factorInv1 = 1.0f - factor1;
	
		return Eigen::Matrix<DataType, 3, 1>( ( ( factor1 * ( factor0 * mP11 + factorInv0 * mP01 ) ) + ( factorInv1 * ( factor0 * mP10 + factorInv0 * mP00 ) ) ),
                                			( factor1 * ( mP11 - mP01 ) + factorInv1 * ( mP10 - mP00 ) ),
                                			( factor0 * ( mP11 - mP10 ) + factorInv0 * ( mP01 - mP00 ) ) );	
	
	}	

	void getHessianDerivative( OccupiedMap &occ_map,
				   const Eigen::Matrix<DataType, 3, 1> &robot_pose_in_world,
				   const sensor::ScanContainer &scan,
				   Eigen::Matrix<DataType, 3, 3> &H, 
				   Eigen::Matrix<DataType, 3, 1> &dTr )
	{
		size_t size = scan.getSize();
		
		DataType sinRot = ::sin( robot_pose_in_world[2] );
        	DataType cosRot = ::cos( robot_pose_in_world[2] );	
	
		H = Eigen::Matrix<DataType, 3, 3>::Zero();
        	dTr = Eigen::Matrix<DataType, 3, 1>::Zero();

		for( size_t i = 0; i < size; i ++ ){
			// 1. get the current point in laser coordinate
                	Eigen::Matrix<DataType, 2, 1> curr_point_in_laser( scan.getIndexData( i ) );
                	Eigen::Matrix<DataType, 2, 1> curr_point_in_scaled_laser( scan.getIndexData( i ) * occ_map.getScale() );

			Eigen::Matrix<DataType, 2, 1> curr_point_in_world( occ_map.observedPointLaser2World( curr_point_in_laser, robot_pose_in_world ) );

			Eigen::Matrix<DataType, 2, 1> curr_point_in_map( occ_map.observedPointWorld2Map( curr_point_in_world ) );
		
			Eigen::Matrix<DataType, 3, 1> interpolated_value( bilinearInterpolationWithDerivative( occ_map, curr_point_in_map ) );		
			
			DataType func_value = 1.0 - interpolated_value[0];

			dTr[0] += interpolated_value[1] * func_value;
                	dTr[1] += interpolated_value[2] * func_value;

			DataType rotDeriv = ( interpolated_value[1] * ( -sinRot * curr_point_in_scaled_laser[0] - cosRot * curr_point_in_scaled_laser[1] ) ) + ( interpolated_value[2] * ( cosRot * curr_point_in_scaled_laser[0] - sinRot * curr_point_in_scaled_laser[1] ) );

			dTr[2] += rotDeriv * func_value;
				
			H( 0, 0 ) += sqr( interpolated_value[1] );
	                H( 1, 1 ) += sqr( interpolated_value[2] );
        	        H( 2, 2 ) += sqr( rotDeriv );
	
        	        H( 0, 1 ) += interpolated_value[1] * interpolated_value[2];
                	H( 0, 2 ) += interpolated_value[1] * rotDeriv;
                	H( 1, 2 ) += interpolated_value[2] * rotDeriv;

		}

		H( 1, 0 ) = H( 0, 1 );
	        H( 2, 0 ) = H( 0, 2 );
        	H( 2, 1 ) = H( 1, 2 );
	}

	bool estimateTransformationOnce( OccupiedMap &occ_map,
					 Eigen::Matrix<DataType, 3, 1> &estimate_in_world,
					 const sensor::ScanContainer &scan )
	{
		getHessianDerivative( occ_map, estimate_in_world, scan, H, dTr ) ;
		if ( ( H(0, 0) != 0.0f ) && ( H(1, 1) != 0.0f ) ){
			Eigen::Matrix<DataType, 3, 1> delta_cauchy( H.inverse() * dTr );
			if( delta_cauchy[2] > 0.2f ){
	                        delta_cauchy[2] = 0.2f;
        	                std::cout<<"delta Cauchy angle change too large"<<std::endl;
                	}
	                else if( delta_cauchy[2] < -0.2f ){
        	                delta_cauchy[2] = -0.2f;
                	        std::cout<<"delta Cauchy angle change too small"<<std::endl;
                	}

			updateEstimatedPose( occ_map, estimate_in_world, delta_cauchy );
			return true;
		}
	
		return false;
	}

	void updateEstimatedPose( const OccupiedMap &occ_map, 
				  Eigen::Matrix<DataType, 3, 1> &estimate_in_world, Eigen::Matrix<DataType, 3, 1> &delta )
	{
		Eigen::Matrix<DataType, 3, 1> estimate_in_map( occ_map.robotPoseWorld2Map( estimate_in_world ) );
		
		estimate_in_map += delta;
		
		estimate_in_world = occ_map.robotPoseMap2World( estimate_in_map );

	}

	const Eigen::Matrix<DataType, 3, 1> scanToMap( OccupiedMap &occ_map,
                        			 const Eigen::Matrix<DataType, 3, 1> &begin_estimated_pose_in_world,
                        			 const sensor::ScanContainer &scan,
                        			 Eigen::Matrix<DataType, 3, 3> &covarince_matrix,
                        			 const int max_interations = 100 )	
	{
		Eigen::Matrix<DataType, 3, 1> estimate_pose( begin_estimated_pose_in_world );
		if( scan.getSize() == 0 ){
        	        return begin_estimated_pose_in_world;
	        }
	
		estimateTransformationOnce( occ_map, estimate_pose, scan );

		for( size_t i = 0; i < max_interations - 1; i ++ ){
                	estimateTransformationOnce( occ_map, estimate_pose, scan );
        	}	

		estimate_pose[2] = normalize_angle( estimate_pose[2] );
		covarince_matrix = H;

		return estimate_pose;
	}

private:
	inline const DataType normalize_angle_pos( DataType angle )
	{
        	return fmod( fmod( angle, 2.0 * M_PI ) + 2.0 * M_PI, 2.0 * M_PI );
	}

	inline const DataType normalize_angle(const DataType angle)
	{
        	DataType a = normalize_angle_pos(angle);

        	if (a > M_PI){
                	a -= 2.0 * M_PI;
        	}

        	return a;
	}

	inline const DataType sqr( const DataType val )
	{
		return val * val;
	}

private:
	Eigen::Matrix<DataType, 3, 3> H;
	Eigen::Matrix<DataType, 3, 1> dTr;

	DataType mP00 = 0;
	DataType mP11 = 0;
	DataType mP01 = 0;
	DataType mP10 = 0;
};


}

#endif

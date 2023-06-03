#ifndef __SLAM_PROCESS_H
#define __SLAM_PROCESS_H

#include <opencv2/opencv.hpp>

#include "scan_match.h"

namespace slam
{

template<typename T>
class SlamProcessor
{
public:
	using DataType = T;

	SlamProcessor(  )
	{
		grid_map_ = new grid::OccupiedGridMap<T>();
		if( grid_map_ == nullptr ){
			std::cerr<<"Construct Occupied Grid Map Failed !"<<std::endl;	
			exit( -1 );
		}
		std::cerr<<"Construct The Occupied Grid Map !"<<std::endl;
	
		scan_match_ = new match::ScanMatchMethod<T>();
		if( scan_match_ == nullptr ){
			std::cerr<<"Construct Scan Match Method Failed !"<<std::endl;
			exit(-1);
		}
		std::cerr<<"Construct The Scan Match Method !"<<std::endl;
	}

	SlamProcessor( const int size_x, const int size_y, const DataType cell_length )
	{
		grid_map_ = new grid::OccupiedGridMap<T>();
                if( grid_map_ == nullptr ){
                        std::cerr<<"Construct Occupied Grid Map Failed !"<<std::endl;            
                        exit( -1 );
                }
                std::cerr<<"Construct The Occupied Grid Map !"<<std::endl;

                scan_match_ = new match::ScanMatchMethod<T>();
                if( scan_match_ == nullptr ){
                        std::cerr<<"Construct Scan Match Method Failed !"<<std::endl;            
                        exit(-1);
                }
                std::cerr<<"Construct The Scan Match Method !"<<std::endl;

	}

	~SlamProcessor(  )
	{
		if( grid_map_ != nullptr )
			delete grid_map_;
		if( scan_match_ != nullptr )
			delete scan_match_;
	}	
	
	void printMapInfo() const
	{
		return grid_map_->printMapInfo();
	}

	const int getSizeX() const
	{
		return grid_map_->getSizeX();
	} 

	const int getSizeY() const
	{
		return grid_map_->getSizeY();
	}

	const DataType getCellLength() const
	{
		return grid_map_->getCellLength();
	}

	// ------------ Set Parameters ---------- //
	void setMinDistanceDiffForMapUpdate( const DataType min_dist )
	{
		minDistanceDiffForMapUpdate = min_dist;
	}

	void setMinAngleDiffForMapUpdate( const DataType min_angle )	
	{
		minAngleDiffForMapUpdate = min_angle;
	}

	// ------------- Update --------------//
        void update( const Eigen::Matrix<DataType, 3, 1> &robot_pose_in_world,
                     const sensor::ScanContainer &scan,
                     bool map_without_matching = false )
	{
		is_key_frame_ = false;
		
		Eigen::Matrix<DataType, 3, 1> new_pose_estimated;
		
		if( !map_without_matching ){
			new_pose_estimated = scan_match_->scanToMap( *grid_map_, robot_pose_in_world, scan, covarince_matrix_, 10 );
		}
		else {
			new_pose_estimated = robot_pose_in_world;
		}

		last_scan_match_pose_ = new_pose_estimated;

		// map update
		if( poseDiffLargerThan( last_map_update_pose_, new_pose_estimated ) ){
                	is_key_frame_ = true;

			grid_map_->updateMapByScan( scan, new_pose_estimated );
                	last_map_update_pose_ = new_pose_estimated;
		}

	}

	void processTheFirstScan( const Eigen::Matrix<DataType, 3, 1> &robot_pose_in_world,
				  const sensor::ScanContainer &scan )
	{
		grid_map_->updateMapByScan( scan, robot_pose_in_world );
	}
	
	const Eigen::Matrix<DataType, 3, 1>& getLastScanMatchPose() const
	{
		return last_scan_match_pose_;
	}

	const Eigen::Matrix<DataType, 3, 1>& getLastMapUpdatePose() const
	{
		return last_map_update_pose_;
	}

	const Eigen::Matrix<DataType, 3, 3>& getCovarianceMatrix() const
	{
		return covarince_matrix_;
	} 

	bool isKeyFrame() const
	{
		return is_key_frame_;
	}

	//
	const grid::OccupiedGridMap<T>& getOccupiedGridMap() const
	{
		return * grid_map_;
	}

	void generateCvMap( cv::Mat& map, cv::Mat& costmap, int inflate_radius = 3 )
        {
                if ( map.type() != CV_8UC1 || costmap.type() != CV_8UC1 ) return;

                for( int i = 0; i < grid_map_->getSizeX(); i ++ ) {
                        for( int j = 0; j < grid_map_->getSizeY(); j ++ ) {
                                if( grid_map_->isCellFree( i, j ) ) {
                                        map.at<uchar>( i, j ) = 255;
                                        costmap.at<uchar>( i, j ) = 255;
                                }
                                else if( grid_map_->isCellOccupied( i, j ) ) {
                                        map.at<uchar>( i, j ) = 0;
                                        cv::circle( costmap, cv::Point2d( j, i ), inflate_radius, cv::Scalar(0), -1 );
                                }
                        }
                }
        }

        void generateCvMap( cv::Mat& map )
        {
                if ( map.type() != CV_8UC1 ) return;

                for( int i = 0; i < grid_map_->getSizeX(); i ++ ) {
                        for( int j = 0; j < grid_map_->getSizeY(); j ++ ) {
                                if( grid_map_->isCellFree( i, j ) ) {
                                        map.at<uchar>( i, j ) = 255;
                                }
                                else if( grid_map_->isCellOccupied( i, j ) ) {
                                        map.at<uchar>( i, j ) = 0;
                                }
                        }
                }
        }


private:
	bool poseDiffLargerThan( const Eigen::Matrix<DataType, 3, 1> &pose_old, const Eigen::Matrix<DataType, 3, 1> &pose_new )	
	{
		pose_diff_ = pose_new - pose_old;
		Eigen::Matrix<DataType, 2, 1> trans_diff( pose_diff_[0], pose_diff_[1] );
	
		if( trans_diff.norm()  > minDistanceDiffForMapUpdate ){
                	return true;
        	}

		DataType angle_diff = ( pose_new[2] - pose_old[2] );

		if( angle_diff > M_PI ){
                	angle_diff -= M_PI * 2.0f;
        	}
        	else if( angle_diff < -M_PI ){
                	angle_diff += M_PI * 2.0f;
        	}

        	if( ::abs( angle_diff ) > minAngleDiffForMapUpdate ){
                	return true;
        	}

        	return false;
	}

private:
	match::ScanMatchMethod<T> *scan_match_;
	grid::OccupiedGridMap<T> *grid_map_;

	// ------------ Parameters -------------- //
	DataType minDistanceDiffForMapUpdate = 0.2;
	DataType minAngleDiffForMapUpdate = 0.035;
	
	Eigen::Matrix<DataType, 3, 3> covarince_matrix_;
	Eigen::Matrix<DataType, 3, 1> last_scan_match_pose_;
	Eigen::Matrix<DataType, 3, 1> last_map_update_pose_;	

	Eigen::Matrix<DataType, 3, 1> pose_diff_;
	Eigen::Matrix<DataType, 3, 1> pose_trans_;

	bool is_key_frame_ = false;
	
};


}

#endif

#ifndef __UTILS_H
#define __UTILS_H

#include "data_type.h"
#include "data_container.h"
#include "obstacles.h"

#include "kdtree_obstacles_eigen_adaptor.h"

#include <opencv2/opencv.hpp>

class Utils
{
public:
	static void laserData2Container( const sensor::LaserScan& scan, sensor::ScanContainer& container )
{
        container.clear();

        float angle = 0.0;
        for ( size_t i = 0; i < scan.size(); i ++ ) {
                auto dist = scan.ranges[i];
                if ( dist >= 0.1 && dist < 10.0 ) {
                        auto pt = Eigen::Vector2f( ::cos( angle ) * dist, ::sin( angle ) * dist );
                        Eigen::Matrix2f R;
                        R << ::cos( M_PI ), -::sin( M_PI ),
                             ::sin( M_PI ),  ::cos( M_PI );

                        container.addData( R * pt );
                }

                angle += scan.angle_increment;
        }

        std::cout<<"Scan Container size : "<<container.getSize()<<std::endl;
}

	static void displayScan( sensor::ScanContainer& container, const float scale = 100 )
	{
        	cv::Mat image = cv::Mat::zeros( 800, 800, CV_8UC3 );

	        cv::line( image, cv::Point( 400, 0 ), cv::Point( 400, 800 ), cv::Scalar( 0, 255, 0 ), 2 );
        	cv::line( image, cv::Point( 0, 400 ), cv::Point( 800, 400 ), cv::Scalar( 255, 0, 0 ), 2 );

	        for ( size_t i = 0; i < container.getSize(); i ++ ) {
        	        auto pt = container.getIndexData( i );
                	cv::Point2f point( pt(0) * scale + 400, pt(1) * scale + 400 );
	                cv::circle( image, point, 3, cv::Scalar( 0, 0, 255 ), -1 );
        	}

	        cv::imshow( "scan", image );
        	cv::waitKey(5);
	}

	template<typename T>
	static void angleNormalize( T&& angle ) 
	{
		if ( angle >= M_PI ) angle -= 2 * M_PI;

		if ( angle <= -M_PI ) angle += 2 * M_PI;
	}

	template<typename T>
	static const Eigen::Matrix<T, 2, 1> coordinateTransformMap2World( const Eigen::Vector2i& pt_in_map, const Eigen::Vector2i& map_center, const T cell_len ) 
	{
		return ( pt_in_map - map_center ).cast<T>() * cell_len;
	}

	template<typename T>
	static const Eigen::Vector2i coordinateTransformWorld2Map( const Eigen::Matrix<T, 2, 1>& pt_in_world, const Eigen::Vector2i& map_center, const T cell_len )
	{
		return Eigen::Vector2i( pt_in_world[0] / cell_len + map_center[0],
			       	        pt_in_world[1] / cell_len + map_center[1] );
	}

	template<typename T>
	static void cvMap2ObstaclesVec( const cv::Mat& map, apf::Obstacles<T>& obs_vec, const Eigen::Vector2i& map_center, const T cell_len )
	{
		obs_vec.clearAll();

		Eigen::Matrix<T, 2, 1> pre_pt = Eigen::Matrix<T, 2, 1>::Zero();
		int cnt = 0;

		for ( int i = 0; i < map.cols; i ++ ) {
			for ( int j = 0; j < map.rows; j ++ ) {
				if ( map.at<uchar>( i, j ) == 0 ) { // obstacles
					auto pt = coordinateTransformMap2World( Eigen::Vector2i( i, j ), map_center, cell_len );
					//std::cout<<"pt["<<cnt<<"] = ( "<<pt.transpose()<<" )"<<std::endl;

					if ( cnt == 0 ) {
						pre_pt = pt;
						obs_vec.addObstacle( pt );
					}		
					else {
						if ( ( pt - pre_pt ).norm() > 0.3 ) {
							kdtree::KdTreeObstaclesType<T> obs( obs_vec );
							kdtree::KdTreeType<T> kd_tree( 2, obs, 10 );

							T min_dist = 0;
							size_t closed_pt_idx = -1;
							nanoflann::KNNResultSet<T> ret_set( 1 );
							T query_pt[2] = { pt[0], pt[1] };
							ret_set.init( &closed_pt_idx, &min_dist );
       							kd_tree.findNeighbors( ret_set, query_pt );

							//std::cout<<"closed_pt_idx = "<<closed_pt_idx<<std::endl;
        						//std::cout<<"min dist = "<<min_dist<<std::endl;
        						//std::cout<<"close pt = "<<obs_vec[closed_pt_idx].transpose()<<std::endl;

							if ( ( obs_vec[closed_pt_idx] - pt ).norm() > 0.3 ) {
								obs_vec.addObstacle( pt );
								pre_pt = pt;
								//std::cout<<"pt in obstacle : ("<<pt.transpose()<<" )"<<std::endl;
							}
						}

					}

					cnt ++;
				}
			}
		}
	}

	template<typename T>
       	static void convertObstacles2PoseXYVec( const apf::Obstacles<T>& obs_vec, std::vector<geometry::PoseXY<T>>& vec )
	{
		for ( int i = 0; i < obs_vec.getSize(); i ++ ) {
			vec.push_back( geometry::PoseXY<T>( obs_vec[i](0), obs_vec[i](1) ) );
		}
	}

};

#endif

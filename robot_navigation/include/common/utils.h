#ifndef __UTILS_H
#define __UTILS_H

#include "data_type.h"
#include "data_container.h"
#include "obstacles.h"

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
	static void cvMap2ObstaclesVec( const cv::Mat& map, apf::Obstacles<T>& obs_vec, const Eigen::Vector2i& map_center, const T cell_len )
	{
		obs_vec.clearAll();

		Eigen::Matrix<T, 2, 1> pre_pt = Eigen::Matrix<T, 2, 1>::Zero();
		int cnt = 0;

		for ( int i = 0; i < map.rows; i ++ ) {
			for ( int j = 0; j < map.cols; j ++ ) {
				if ( map.at<uchar>( j, i ) == 0 ) { // obstacles
					auto pt = coordinateTransformMap2World( Eigen::Vector2i( i, j ), map_center, cell_len );
					if ( cnt == 0 ) {
						pre_pt = pt;
						obs_vec.addObstacle( pt );
					}		
					else {
						if ( ( pt - pre_pt ).norm() > 0.3 ) {
							obs_vec.addObstacle( pt );
							pre_pt = pt;
						}
					}
				}
			}
		}
	}
};


#endif

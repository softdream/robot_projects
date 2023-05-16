#ifndef __UTILS_H
#define __UTILS_H

#include "data_type.h"
#include "data_container.h"

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
};


#endif

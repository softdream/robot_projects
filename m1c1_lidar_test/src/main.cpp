#include "lidar_drive.h"
#include "data_type.h"
#include "data_container.h"
#include <opencv2/opencv.hpp>

// -------- global data --------- //
sensor::ScanContainer container;

void laserData2Container( const sensor::LaserScan& scan, sensor::ScanContainer& container )
{
	container.clear();

	float angle = 0.0;
	for ( size_t i = 0; i < scan.size(); i ++ ) {
		auto dist = scan.ranges[i];
		if ( dist >= 0.1 && dist < 10.0 ) {
			container.addData( Eigen::Vector2f( ::cos( angle ) * dist, ::sin( angle ) * dist ) );
		}

		angle += scan.angle_increment;
	}

	std::cout<<"Scan Container size : "<<container.getSize()<<std::endl;
}

void displayScan( sensor::ScanContainer& container, const float scale = 100 )
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

void lidarCallback( const sensor::LaserScan& scan )
{
	std::cout<<"lidar data callback ..."<<std::endl;

	std::cout<<"angle_min : "<<scan.angle_min<<std::endl;
	std::cout<<"angle_max : "<<scan.angle_max<<std::endl;
	std::cout<<"angle_increment : "<<scan.angle_increment<<std::endl;
	std::cout<<"range_min : "<<scan.range_min<<std::endl;
	std::cout<<"range_max : "<<scan.range_max<<std::endl;
	std::cout<<"scan_time : "<<scan.scan_time<<std::endl;

	std::cout<<"ranges : ";
	for ( size_t i = 0; i < 10; i ++ ) {
		std::cout<<scan.ranges[i]<<" ";
	}
	std::cout<<std::endl;

	laserData2Container( scan, container );
	displayScan( container );
}

int main(int argc, char *argv[])
{
	std::cout<<"----------------- LIDAR TEST ----------------"<<std::endl;

	lidar::Lidar m1c1_lidar;

	m1c1_lidar.spin<sensor::LaserScan>( lidarCallback );

	return 0;
}

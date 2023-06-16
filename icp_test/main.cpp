#include <iostream>
#include "laserSimulation.h"
#include "data_container.h"

#include <opencv2/opencv.hpp>

#include "2d_icp.h"


void laserData2Container( const sensor::LaserScan& scan, sensor::ScanContainer& container )
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
        cv::waitKey(0);
}

int main()
{
	std::cout<<"------------------- ICP TEST ----------------------"<<std::endl;

	simulation::Simulation simulation;
	simulation.openSimulationFile("/groupdata/share/ddf/Test/robot_test/robot_projects/icp_test/laser_data3.txt");

	sensor::LaserScan scan;
       	sensor::ScanContainer scan_container1;

	simulation.readAFrameData( scan );
        laserData2Container( scan, scan_container1 );
	displayScan( scan_container1 );
	std::cout<<"container 1 size = "<<scan_container1.getSize()<<std::endl;

	sensor::ScanContainer scan_container2;

	simulation.readAFrameData( scan );
        laserData2Container( scan, scan_container2 );
	displayScan( scan_container2 );
	std::cout<<"container 2 size = "<<scan_container2.getSize()<<std::endl;

	// ----------- icp match ------------ //
	slam::ICP<float> icp;
	Eigen::Vector3f constraint = Eigen::Vector3f::Zero();
	icp.solveICP( scan_container1, scan_container2, constraint );
	std::cout<<"constraint = "<<constraint.transpose()<<std::endl;

	return 0;
}

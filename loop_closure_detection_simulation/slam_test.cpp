#include <iostream>
#include "slam_process.h"

#include "laserSimulation.h"

#include <opencv2/opencv.hpp>

#include <chrono>

#include <fstream>

#include <thread>
#include <mutex>

#include "loop_closure_detect.h"

// -------------------------- GLOBAL DATA -------------------------- //
slam::SlamProcessor<float> slam_processor;
cv::Mat image = cv::Mat( slam_processor.getSizeX(), slam_processor.getSizeY(), CV_8UC1, cv::Scalar(125));
Eigen::Vector3f robot_pose( 0.0f, 0.0f, 0.0f );

slam::LoopDetect<float> loop_detect;
// ----------------------------------------------------------------- //

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
        cv::waitKey(5);
}


void threadSlam()
{
	simulation::Simulation simulation;
	simulation.openSimulationFile( "/groupdata/share/ddf/Test/robot_test/robot_projects/loop_closure_detection_simulation/test_data/laser_test_data2.txt" );


	int img_cnt = 0;
	while( !simulation.endOfFile() ){
		sensor::LaserScan scan;
		sensor::ScanContainer scan_container;
	
		simulation.readAFrameData( scan );
		laserData2Container( scan, scan_container );
	
		std::cout<<"frame count: "<<simulation.getFrameCount()<<std::endl;
		//displayScan( scan_container );

		// generate the scan contex
		loop_detect.makeScanContext( scan );

		/*if( simulation.getFrameCount() <= 10  ){
			slam_processor.processTheFirstScan( robot_pose, scan_container );
			if( simulation.getFrameCount() == 10 ){
				slam_processor.generateMap( image );

			}
		}
		else {
			auto beforeTime = std::chrono::steady_clock::now();
			slam_processor.update( robot_pose, scan_container );
			robot_pose = slam_processor.getLastScanMatchPose();
			auto afterTime = std::chrono::steady_clock::now();
			double duration_millsecond = std::chrono::duration<double, std::milli>(afterTime - beforeTime).count();
			std::cout<<"duration : " << duration_millsecond << "ms" << std::endl;
			
			std::cout<<"robot pose now: "<<std::endl;
                        std::cout<<robot_pose<<std::endl;
                        std::cout<<"------------------"<<std::endl;

			if( slam_processor.isKeyFrame() ){
				slam_processor.generateMap( image );
			}
		}
		*/

		if ( img_cnt > 1 ) break;
		img_cnt ++;

		cv::waitKey(100);
	}

	simulation.closeSimulationFile();
}


int main()
{
	std::cout<<"---------------------- PATH PLANNING TEST ----------------------"<<std::endl;

	std::thread t1( threadSlam );

	t1.join();

	while ( 1 ) {
	
	}

	return 0;
}

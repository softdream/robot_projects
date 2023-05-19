#include <iostream>
#include "slam_process.h"

#include "laserSimulation.h"

#include <opencv2/opencv.hpp>

#include <chrono>

#include <map>

#include <fstream>

#include "map_manage.h"

#include "data_transport.h"

#include "apf_process.h"

#include <thread>
#include <mutex>

// -------------------------- GLOBAL DATA -------------------------- //
slam::SlamProcessor<float> slam_processor;
cv::Mat image = cv::Mat( slam_processor.getSizeX(), slam_processor.getSizeY(), CV_8UC1, cv::Scalar(125));
Eigen::Vector3f robot_pose( 0.0f, 0.0f, 0.0f );

cv::Mat obs_map = cv::Mat( 500, 500, CV_8UC3, cv::Scalar(255, 255, 255 ) );
std::mutex mux;
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

template<typename T>
void drawObstacles( const apf::Obstacles<T>& obstacles, T scale = 50 )
{
	for( int i = 0; i < obstacles.getSize() - 1; i ++ ){
        	cv::circle( obs_map, cv::Point( obstacles[i][0] * scale + 250, obstacles[i][1] * scale + 250 ), 15, cv::Scalar( 0, 0, 255 ), -1 );
       	
	}

}


void threadSlam()
{
	simulation::Simulation simulation;
	simulation.openSimulationFile( "/groupdata/share/ddf/Test/robot_test/robot_projects/navigation_simu/test_data/laser_test_data2.txt" );


	int img_cnt = 0;
	while( !simulation.endOfFile() ){
		sensor::LaserScan scan;
		sensor::ScanContainer scan_container;
	
		simulation.readAFrameData( scan );
		laserData2Container( scan, scan_container );
	
		std::cout<<"frame count: "<<simulation.getFrameCount()<<std::endl;
		//displayScan( scan_container );

		if( simulation.getFrameCount() <= 10  ){
			slam_processor.processTheFirstScan( robot_pose, scan_container );
			if( simulation.getFrameCount() == 10 ){
				mux.lock();
				slam_processor.displayMap( image );
				mux.unlock();
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
				mux.lock();
				slam_processor.displayMap( image );
				mux.unlock();

				//cv::imwrite( std::to_string( img_cnt ++ ) + ".jpg", image );
				apf::Obstacles<float> obs_vec;
				Utils::cvMap2ObstaclesVec( image, obs_vec, Eigen::Vector2i( 250, 250 ), 0.1f );
				obs_map = cv::Mat( 500, 500, CV_8UC3, cv::Scalar(255, 255, 255 ) );
                		drawObstacles( obs_vec );

				cv::imshow( "obstacles", obs_map );

			}
		}
		
		cv::waitKey(100);
	}

	simulation.closeSimulationFile();
}

void pathPlanning()
{
	apf::APFProcess<float> apf_processor;
	apf::Obstacles<float> obs_vec;

	int img_cnt = 0;
	while ( 1 ) {
		mux.lock();
		Utils::cvMap2ObstaclesVec( image, obs_vec, Eigen::Vector2i( 250, 250 ), 0.1f );
		mux.unlock();
		std::cout<<"obs_vec.size() ============= "<<obs_vec.getSize()<<std::endl;
	
		obs_map = cv::Mat( 500, 500, CV_8UC3, cv::Scalar(255, 255, 255 ) );
		drawObstacles( obs_vec );
		//cv::imshow( "obstacles", obs_map );
		//
		cv::imwrite( std::to_string( img_cnt ++ ) + ".jpg", image );
		cv::waitKey(200);
	}
}

int main()
{
	std::cout<<"---------------------- PATH PLANNING TEST ----------------------"<<std::endl;

	std::thread t1( threadSlam );
	//std::thread t2( pathPlanning );

	t1.join();
	//t2.join();

	while ( 1 ) {
	
	}

	return 0;
}

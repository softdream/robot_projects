#include <iostream>
#include "slam_process.h"

#include "laserSimulation.h"

#include <opencv2/opencv.hpp>

#include <chrono>

#include <map>

#include <fstream>

#include "map_manage.h"

#include "data_transport.h"

// global data
transport::Sender scan_sender( "192.168.137.211", 2337 );

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

int main()
{
	std::cout<<" --------------- SLAM TEST --------------"<<std::endl;
	std::ofstream odom_out( "./key_poses.txt", std::ios::app );
	

	slam::SlamProcessor<float> slam;

	// print the map information
	slam.printMapInfo();
	cv::Mat image = cv::Mat(slam.getSizeX(), slam.getSizeY(), CV_8UC1, cv::Scalar(125));
	cv::imshow("map", image);

	simulation::Simulation simulation;
	simulation.openSimulationFile( "/home/riki/Test/robot_projects/robot_projects/slam_simu/test_data/laser_test_data2.txt" );

	Eigen::Vector3f robot_pose( 0.0f, 0.0f, 0.0f );
	
	std::map<int, Eigen::Vector3f> key_poses;

	while( !simulation.endOfFile() ){
	//while( simulation.getFrameCount() < 20 ){
		sensor::LaserScan scan;
		sensor::ScanContainer scan_container;
	
		simulation.readAFrameData( scan );
		laserData2Container( scan, scan_container );
	
		std::cout<<"frame count: "<<simulation.getFrameCount()<<std::endl;
		//displayScan( scan_container );

		if( simulation.getFrameCount() <= 10  ){
			slam.processTheFirstScan( robot_pose, scan_container );
			if( simulation.getFrameCount() == 10 ){
				slam.displayMap( image );
				
				key_poses.insert( std::make_pair( simulation.getFrameCount(), robot_pose ) );
				odom_out <<"odom "<< robot_pose[0]<< " "<<robot_pose[1]<<" "<<robot_pose[2]<<" "<<simulation.getFrameCount()<<std::endl;
				
			}
		}
		else {
			auto beforeTime = std::chrono::steady_clock::now();
			slam.update( robot_pose, scan_container );
			robot_pose = slam.getLastScanMatchPose();
			auto afterTime = std::chrono::steady_clock::now();
			double duration_millsecond = std::chrono::duration<double, std::milli>(afterTime - beforeTime).count();
			std::cout<<"duration : " << duration_millsecond << "ms" << std::endl;
			
			std::cout<<"robot pose now: "<<std::endl;
                        std::cout<<robot_pose<<std::endl;
			odom_out <<"odom "<< robot_pose[0]<< " "<<robot_pose[1]<<" "<<robot_pose[2]<<" "<<simulation.getFrameCount()<<std::endl;
                        std::cout<<"------------------"<<std::endl;

			if( slam.isKeyFrame() ){
				key_poses.insert( std::make_pair( simulation.getFrameCount(), robot_pose ) );
			
				slam.displayMap( image );

				// send map
				std::vector<unsigned char> encode_data;
				cv::imencode(".jpg", image, encode_data);
				int ret  = scan_sender.send( encode_data );
				std::cout<<"map send : "<<ret<<std::endl;
			}
		}
		
		cv::waitKey(0);
	}

	map::MapManagement<float>::saveOccupiedGridMap( "test.map", slam.getOccupiedGridMap() );

	simulation.closeSimulationFile();

	return 0;
}

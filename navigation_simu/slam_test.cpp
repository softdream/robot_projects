#include <iostream>
#include "slam_process.h"

#include "laserSimulation.h"

#include <opencv2/opencv.hpp>

#include <chrono>

#include <map>

#include <fstream>

#include "map_manage.h"

#include "data_transport.h"

#include "target_planner.h"

#include "a_star.h"

#include <thread>
#include <mutex>


// -------------------------- GLOBAL DATA -------------------------- //
slam::SlamProcessor<float> slam_processor;
cv::Mat image = cv::Mat( slam_processor.getSizeX(), slam_processor.getSizeY(), CV_8UC1, cv::Scalar(125));
Eigen::Vector3f robot_pose( 0.0f, 0.0f, 0.0f );

cv::Mat costmap = cv::Mat( slam_processor.getSizeX(), slam_processor.getSizeY(), CV_8UC1, cv::Scalar(125));

cv::Mat costmap2 = cv::Mat( 800, 800, CV_8UC3, cv::Scalar( 255, 255, 255 ) );

std::vector<Eigen::Vector2f> visited_robot_pose_vec;

planner::AStar<float> a_star;
bool planned_flag = false;
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

void map2CostMap2( const cv::Mat& map, cv::Mat& costmap2 )
{
	costmap2 = cv::Mat( 800, 800, CV_8UC3, cv::Scalar( 255, 255, 255 ) );

	for ( int i = 0; i < map.cols; i ++ ) {
		for ( int j = 0; j < map.rows; j ++ ) {
		
			if ( map.at<uchar>( i, j ) == 0 ) {
				Eigen::Vector2f pt_map( i, j );
			
				Eigen::Vector2f pt_world = ( pt_map - Eigen::Vector2f( 250, 250 ) ) * 0.1;
				cv::circle( costmap2, cv::Point( pt_world[0] * 50 + 400, pt_world[1] * 50 + 400 ), 12, cv::Scalar( 255, 0, 0 ), -1 );
			}
		}
	}
}

void drawPath( cv::Mat& costmap2, const std::vector<Eigen::Vector2i>& path, const Eigen::Vector2i& target )
{
	std::cout<<"path.size ====== == === "<<path.size()<<std::endl;
	
	Eigen::Vector2f dst_map( target[0], target[1] );
        Eigen::Vector2f dst_world = ( dst_map - Eigen::Vector2f( 250, 250 ) ) * 0.1;
	cv::circle( costmap2, cv::Point( dst_world[0] * 50 + 400, dst_world[1] * 50 + 400 ), 5, cv::Scalar( 0, 0, 255 ), -1 );

	for ( const auto& pt : path ) {
		Eigen::Vector2f pt_map( pt[0], pt[1] );
		Eigen::Vector2f pt_world = ( pt_map - Eigen::Vector2f( 250, 250 ) ) * 0.1;

		cv::circle( costmap2, cv::Point( pt_world[0] * 50 + 400, pt_world[1] * 50 + 400 ), 3, cv::Scalar( 0, 255, 0 ), -1 );
	} 
}

void drawPath( cv::Mat& costmap2, const std::vector<Eigen::Vector2f>& path, const Eigen::Vector2i& target, const Eigen::Vector2f& start )
{
        std::cout<<"path.size ====== == === "<<path.size()<<std::endl;

        Eigen::Vector2f dst_map( target[0], target[1] );
        Eigen::Vector2f dst_world = ( dst_map - Eigen::Vector2f( 250, 250 ) ) * 0.1;
        cv::circle( costmap2, cv::Point( dst_world[0] * 50 + 400, dst_world[1] * 50 + 400 ), 5, cv::Scalar( 0, 0, 255 ), -1 );

	cv::circle( costmap2, cv::Point( start[0] * 50 + 400, start[1] * 50 + 400 ), 5, cv::Scalar( 0, 255, 255 ), -1 );


        for ( const auto& pt : path ) {
                cv::circle( costmap2, cv::Point( pt[0] * 50 + 400, pt[1] * 50 + 400 ), 3, cv::Scalar( 0, 255, 0 ), -1 );
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
				slam_processor.displayMap( image );

//				map2CostMap( image, costmap );
//				cv::imshow( "costmap", costmap );
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
				slam_processor.generateMap( image, costmap );
				
				//cv::imshow( "costmap", costmap );

				a_star.setMap( costmap );
				auto robot_pose_map = Eigen::Vector2i( robot_pose[0] * 10 + 250, robot_pose[1] * 10 + 250 );
				Eigen::Vector2i target = TargetPlanner::generatePlannedTargetGoal( costmap, visited_robot_pose_vec, planned_flag );
				
				visited_robot_pose_vec.push_back( Eigen::Vector2f( ( target[0] - 250 ) * 0.1, ( target[0] - 250 ) * 0.1 ) );
				
				if ( planned_flag ) {
					std::cout<<"Finished the Travel !!!!!!!!!!!!!!!!!!!!!!!! "<<std::endl;
					break;
				}

				if ( !a_star.findPath( robot_pose_map, target ) )  continue;

				map2CostMap2( image, costmap2 );

				auto path = a_star.bezierSmoothness();
					
				drawPath( costmap2, path, target, robot_pose.head( 2 ) );
				cv::imshow( "costmap", costmap2 );
				cv::waitKey(0);
				//cv::imwrite(std::to_string( img_cnt ++ ) + ".jpg", costmap);
			}
		}
		
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

#include <iostream>
#include "slam_process.h"

#include "laserSimulation.h"

#include <opencv2/opencv.hpp>

#include <chrono>

#include <fstream>

#include <thread>
#include <mutex>

#include "pose_graph_optimize.h"

// -------------------------- GLOBAL DATA -------------------------- //
slam::SlamProcessor<float> slam_processor;
cv::Mat image = cv::Mat( slam_processor.getSizeX(), slam_processor.getSizeY(), CV_8UC1, cv::Scalar(125));
Eigen::Vector3f robot_pose( 0.0f, 0.0f, 0.0f );

slam::PoseOptimization<float> pgo_detect;
std::vector<Eigen::Vector3f> key_poses;
std::vector<sensor::ScanContainer> key_scans;

pgo::GraphOptimizer<float> pose_optimizer;

std::ofstream out;
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
	int vertex_cnt = 0;
	int constriant_cnt = 0;
	while( !simulation.endOfFile() ){
		sensor::LaserScan scan;
		sensor::ScanContainer scan_container;
	
		simulation.readAFrameData( scan );
		laserData2Container( scan, scan_container );
	
		std::cout<<"frame count: "<<simulation.getFrameCount()<<std::endl;


		if( simulation.getFrameCount() <= 10  ){
			slam_processor.processTheFirstScan( robot_pose, scan_container );
			if( simulation.getFrameCount() == 10 ){
				slam_processor.generateMap( image );
				robot_pose = slam_processor.getLastScanMatchPose();
			
				// for pgo
				key_poses.push_back( robot_pose );
				key_scans.push_back( scan_container );

				// record
				out << "pose "<< robot_pose[0]<<" "<<robot_pose[1]<<" "<<robot_pose[2]<<std::endl;
				vertex_cnt ++;
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
                        std::cout<<"------------------"<<std::endl<<std::endl;

			if( slam_processor.isKeyFrame() ){
				slam_processor.generateMap( image );
			
				key_poses.push_back( robot_pose );
				key_scans.push_back( scan_container );

				// record
                                out << "pose "<< robot_pose[0]<<" "<<robot_pose[1]<<" "
<<robot_pose[2]<<std::endl;
				vertex_cnt ++;

				if ( key_poses.size() > 10 ) {
					Eigen::Vector3f constraint = Eigen::Vector3f::Zero();
					auto ret = pgo_detect.getLoopClosureConstraint( key_poses, key_scans, constraint );
					if ( ret != -1 ) {
						out<<"constraint "<<vertex_cnt<<" "<<ret<<" "<<constraint[0]<<" "<<constraint[1]<<" "<<constraint[2]<<std::endl;
						constriant_cnt ++;
				
						if ( constriant_cnt == 1 ) {
							for ( int i = 0; i < key_poses.size() - 1; i ++ ) {
								pose_optimizer.addVertex( key_poses[i], i );


					                	Eigen::Vector3f V = pose_optimizer.homogeneousCoordinateTransformation( key_poses[i], key_poses[i + 1] );

                						Eigen::Matrix3f info_matrix = Eigen::Matrix3f::Identity();
                						pose_optimizer.addEdge( V, i, i + 1, info_matrix );
							}
							pose_optimizer.addVertex( key_poses.back(), key_poses.size() - 1 );
							Eigen::Matrix3f info_matrix = Eigen::Matrix3f::Identity();

                					pose_optimizer.addEdge( constraint, vertex_cnt, ret, info_matrix );
							pose_optimizer.execuGraphOptimization( 2 );

							slam_processor.reConstructMap(  key_poses, key_scans );

							slam_processor.generateMap( image );
							cv::imshow( "map", image );
							cv::waitKey(0);
						}
				
					}
				}

			}
		}
		

		img_cnt ++;

		cv::waitKey(100);
	}

	simulation.closeSimulationFile();
}


int main()
{
	std::cout<<"---------------------- PATH PLANNING TEST ----------------------"<<std::endl;

	out.open("key_poses.txt", std::ios::out);
	if ( !out.is_open() ) {
		std::cerr<<"Can not open the file !"<<std::endl;
		return 0;
	}

	std::thread t1( threadSlam );

	t1.join();

	while ( 1 ) {
	
	}

	return 0;
}

#ifndef __SIMULATION_H
#define __SIMULATION_H

#include <opencv2/opencv.hpp>

#include "laser_simulation.h"
#include "utils.h"

#define WORLD_WIDTH 800
#define WORLD_HEIGHT 800

#define BINARY_MAP_WIDTH 800
#define BINARY_MAP_HEIGHT 800


namespace simulation
{

template<typename T>
class PlannerSimulation
{
public:
	using DataType = T;
        using Vector2 = typename Eigen::Matrix<T, 2, 1>;

	PlannerSimulation() {}

	~PlannerSimulation() {}

	void initMap()
        {
                simulation::Simulation simu;
                simu.openSimulationFile( "/home/riki/Test/robot_projects/robot_projects/global_planner/test_data/laser_test_data2.txt" );
                sensor::LaserScan scan;
                sensor::ScanContainer scan_container;

                simu.readAFrameData( scan );

                Utils::laserData2Container( scan, scan_container );

		drawBinaryMap( scan_container );

		cv::imshow( "binarymap", binary_map );
		cv::waitKey(0);

		setCostMap();
		cv::imshow( "binary_cost_map", binary_map );
		cv::waitKey(0);

		drawMap();
		cv::imshow( "map", map );
                cv::waitKey(0);
        }

private:
	void setCostMap()
	{
		/*cv::Mat kernel = cv::Mat( 9, 9, CV_8UC1, cv::Scalar(255) );
		for ( int i = 0; i < 9; i ++ ) {
                        for ( int j = 0; j < 9; j ++ ) {
                                std::cout<<(int)kernel.at<uchar>( i, j )<<" ";
                        }
                        std::cout<<std::endl;
                }*/

		int cnt = 0;

		int kernel_size = 10;
		for ( int i = 0; i < BINARY_MAP_WIDTH; i ++ ) {
			for ( int j = 0; j < BINARY_MAP_HEIGHT; j ++ ) {
				if ( binary_map.at<uchar>( i, j ) == 1 ) {
					//std::cout<<"i = "<<i<<", j = "<<j<<std::endl;
					cnt ++;					

					/*for ( int u = i - kernel_size / 4; u <= i + kernel_size / 4; u ++ ) {
						for ( int v = j - kernel_size / 4; v <= j + kernel_size / 4; v ++ ) {
							binary_map.at<uchar>( u, v ) = 0;
						}
					}*/

					circle( binary_map, cv::Point( j, i ), 5, cv::Scalar(0), -1 );
				}
			}
		}

		std::cout<<"count = "<<cnt<<std::endl;
	}

private:
	/*void drawMap( const sensor::ScanContainer& scan_container )
	{
		for ( int i = 0; i < scan_container.getSize(); i ++ ) {
			auto pt = scan_container.getIndexData( i );
			cv::circle( map, cv::Point( pt[0] * scale + 400, pt[1] * scale + 400 ), 5, cv::Scalar( 0, 0, 255 ), -1 );

		}
	}*/

	void drawMap()
	{
		for ( int i = 0; i < BINARY_MAP_WIDTH; i ++ ) { 
                        for ( int j = 0; j < BINARY_MAP_HEIGHT; j ++ ) {
				if ( binary_map.at<uchar>( i, j ) == 0 ) { 
					cv::circle( map, cv::Point( j, i ), 4, cv::Scalar( 0, 0, 255 ), -1 );
				}
			}
		}
	}
	

	void drawBinaryMap( const sensor::ScanContainer& scan_container )
	{
		for ( int i = 0; i < scan_container.getSize(); i ++ ) {
                        auto pt = scan_container.getIndexData( i );
			binary_map.at<uchar>( pt[0] * scale + 400, pt[1] * scale + 400 ) = 1;
			std::cout<<"i = "<<pt[0] * scale + 400<<", j = "<<pt[1] * scale + 400 <<std::endl;
		}
	}

private:
	cv::Mat map = cv::Mat( WORLD_WIDTH, WORLD_HEIGHT, CV_8UC3, cv::Scalar(255, 255, 255 ) );

	cv::Mat binary_map = cv::Mat(  BINARY_MAP_HEIGHT, BINARY_MAP_WIDTH, CV_8UC1, cv::Scalar(125) );

	cv::Mat costmap = cv::Mat( WORLD_WIDTH, WORLD_HEIGHT, CV_8UC3, cv::Scalar(255, 255, 255 ) );
	cv::Mat binary_costmap = cv::Mat(  BINARY_MAP_HEIGHT, BINARY_MAP_WIDTH, CV_8UC1, cv::Scalar(125) );


	DataType scale = 50;
};

}

#endif

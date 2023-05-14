#ifndef __APF_SIMULATOR_H
#define __APF_SIMULATIO_H

#include "apf.h"
#include <opencv2/opencv.hpp>

#include "laser_simulation.h"
#include "utils.h"

#define WORLD_WIDTH 800
#define WORLD_HEIGHT 800


namespace apf
{

template<typename T>
class APFSimulator
{
public:
	using DataType = T;
	using Vector2 = typename Eigen::Matrix<T, 2, 1>;	

	APFSimulator()
	{

	}

	~APFSimulator()
	{

	}

	void setTargetPose( const Vector2& target_pose_  )
	{
		drawTarget( target_pose_ );
		cv::imshow( "map", map );
                cv::waitKey(0);
		return apf.setTargetPose( target_pose_ );
	}

	void initMap()
	{
		simulation::Simulation simu;
		simu.openSimulationFile( "/home/riki/Test/robot_projects/robot_projects/apf_planner/test_data/laser_test_data2.txt" );
		sensor::LaserScan scan;
                sensor::ScanContainer scan_container;

                simu.readAFrameData( scan );	

		Utils::laserData2Container( scan, scan_container );

		Vector2 pre_pt = Vector2::Zero();
		for ( int i = 0; i < scan_container.getSize(); i ++ ) {
			auto pt = scan_container.getIndexData( i );
			if ( i == 0 ) {
				pre_pt = pt;
				obstacles.addObstacle( pt );
			}	
			if ( ( pt - pre_pt ).norm() > 0.3 ) {
				obstacles.addObstacle( pt );
				pre_pt = pt;
			}
			//obstacles.addObstacle( pt );
		}

		drawObstacles();

		cv::imshow( "map", map );
		cv::waitKey(0);

	}

	const bool runApfOnce( Vector2& robot_pose )
	{
		drawRobotPose( robot_pose );		

		// attraction
		Vector2 f_att_vec = apf.cacuFatt( robot_pose );	
		drawVector( robot_pose, f_att_vec, 2 );
	
		// repulsion
		Vector2 f_rep_vec = apf.cacuFrep( robot_pose, obstacles );
		std::cout<<"repulsion vector = "<<f_rep_vec.transpose()<<std::endl;
	
		std::vector<Vector2> repulsion_vec1 = apf.getRepulsionVec1();
		drawRepulsionVec1( repulsion_vec1 );

		apf.clearRepulsionVec1();

		drawVector( robot_pose, f_rep_vec, 3 );

		// total 
		Vector2 f_total = f_att_vec + f_rep_vec;
		drawVector( robot_pose, f_total, 5 );

		// update the robot pose
		//Vector2 new_pose = robot_pose + f_total;
		///robot_pose = new_pose;		
		robot_pose += f_total.normalized() * 0.1;

		cv::imshow("map", map);
                cv::waitKey(0);
	}

private:
	void drawRepulsionVec1( const std::vector<Vector2>& repulsion_vec1 ) 
	{
		for( int i = 0; i < repulsion_vec1.size(); i ++ ){
			drawVector( obstacles[i], repulsion_vec1[i] );
		}
	}

	void drawRobotPose( const Vector2& robot_pose )
	{
		cv::circle( map, cv::Point( robot_pose[0] * scale + 400, robot_pose[1] * scale + 400 ), 5, cv::Scalar( 255, 0, 0 ), -1 );
	}

	void drawVector( const Vector2& start_point, const Vector2& vec, const int color = 1 )
	{
		Vector2 end_point = start_point + vec;
		
		switch( color ){
			case 1 : {
				cv::line( map, cv::Point( start_point[0] * scale + 400, start_point[1] * scale + 400 ), cv::Point( end_point[0] * scale + 400, end_point[1] * scale + 400 ), cv::Scalar( 0, 255, 0 ), 1 );	
				break;
			}
			case 2 : {
				cv::line( map, cv::Point( start_point[0] * scale + 400, start_point[1] * scale + 400 ), cv::Point( end_point[0] * scale + 400, end_point[1] * scale + 400 ), cv::Scalar( 0, 0, 255 ), 1 );		
				break;
			}
			case 3 : {
				 cv::line( map, cv::Point( start_point[0] * scale + 400, start_point[1] * scale + 400 ), cv::Point( end_point[0] * scale + 400, end_point[1] * scale + 400 ), cv::Scalar( 0, 255, 255 ), 1 );
				break;
			}
			case 4 : {
				cv::line( map, cv::Point( start_point[0] * scale + 400, start_point[1] * scale + 400 ), cv::Point( end_point[0] * scale + 400, end_point[1] * scale + 400 ), cv::Scalar( 255, 0, 0 ), 2 );	
				break;
			}
			case 5 : {
				cv::line( map, cv::Point( start_point[0] * scale + 400, start_point[1] * scale + 400 ), cv::Point( end_point[0] * scale + 400, end_point[1] * scale + 400 ), cv::Scalar( 255, 255, 0 ), 2 );
                                break;
			}
			default : break;
		}
	}

	void drawTarget( const Vector2& target_pose )
	{
		cv::circle( map, cv::Point( target_pose[0] * scale + 400, target_pose[1] * scale + 400 ), 5, cv::Scalar( 0, 255, 0 ), -1 );
	}

	void drawObstacles()
	{
		for( int i = 0; i < obstacles.getSize(); i ++ ){
			cv::circle( map, cv::Point( obstacles[i][0] * scale + 400, obstacles[i][1] * scale + 400 ), 5, cv::Scalar( 0, 0, 255 ), -1 );
		}
	}

private:
	APF<DataType> apf;
	Obstacles<DataType> obstacles;

	DataType scale = 100; 
	cv::Mat map = cv::Mat( WORLD_WIDTH, WORLD_HEIGHT, CV_8UC3, cv::Scalar(255, 255, 255 ) );
};

}

#endif

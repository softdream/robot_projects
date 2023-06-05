#include "odometry.h"
#include "utils.h"
#include "lidar_drive.h"
#include "slam_process.h"
#include "time_synchronize.h"

#include "keyboard.h"

#include "data_transport.h"

//#include "apf_process.h"
#include "a_star.h"

#include "target_planner.h"

#include "pid_tracking.h"

#include <thread>
#include <mutex>

// -------------------------------------- GLOBAL DATA ---------------------------------------- //
odom::Odometry<float> odometry; // 1. odometry
long scan_frame_cnt = 0; // 3. scan frame counter

time_manage::Synchronize<time_manage::TimeManageData<Eigen::Vector3f>> sync_list; // 4. time synchronization

odom::Odometry<float>::Vector3 pre_odom_pose = odom::Odometry<float>::Vector3::Zero(); // 5. previous odom pose

Eigen::Vector3f robot_pose = Eigen::Vector3f::Zero(); // 6. robot pose

slam::SlamProcessor<float> slam_processor; // 7. slam processor

bool is_initialized = false; // 8. slam initialized flag

cv::Mat map_image = cv::Mat( slam_processor.getSizeX(), slam_processor.getSizeY(), CV_8UC1, cv::Scalar(125) ); // 9. global map image

cv::Mat cost_map_image = cv::Mat( slam_processor.getSizeX(), slam_processor.getSizeY(), CV_8UC1, cv::Scalar(125) ); // 10. global cost map image

bool is_map_ready_flag = false; // 11. global map ready flag

std::mutex map_image_mux;
std::mutex robot_pose_mux;

//transport::Sender odom_sender( "192.168.3.27", 2335 ); // send odometry data
//transport::Sender scan_sender( "192.168.3.27", 2336 ); // send lidar scan data
transport::Sender map_sender( "192.168.3.27", 2337 ); // send map data
transport::Sender pose_sender( "192.168.3.27", 2338 ); // send robot pose data
transport::Sender trajectory_sender( "192.168.3.27", 2339 ); // send the trajectory data
// ------------------------------------------------------------------------------------------- //

void sendMapImage( const cv::Mat& image )
{
	std::vector<unsigned char> encode_data;
        cv::imencode(".jpg", image, encode_data);
        map_sender.send( encode_data ); // send the map image data
}

void keyWPressed()
{
        std::cout<<"key W pressed !"<<std::endl;
        odometry.sendControlVector( 0.15, 0.0 );
}

void keyWReleased()
{
        std::cout<<"key W released !"<<std::endl;
        odometry.sendControlVector( 0.0, 0.0 );
}

void keyAPressd()
{
        std::cout<<"key A pressed !"<<std::endl;
        odometry.sendControlVector( 0.0, 1.2 );
}

void keyAReleased()
{
        std::cout<<"key A released !"<<std::endl;
        odometry.sendControlVector( 0.0, 0.0 );
}

void keyDPressed()
{
        std::cout<<"key D pressed !"<<std::endl;
        odometry.sendControlVector( 0.0, -1.2 );
}

void keyDReleased()
{
        std::cout<<"key D released !"<<std::endl;
        odometry.sendControlVector( 0.0, 0.0 );
}

// thread 1 : keyboard control
void keyboardControl()
{
        keyboard::Keyboard keyboard;

        keyboard.init();
        keyboard.registCallBackFunc( keyWPressed, keyWReleased, 17, keyAPressd, keyAReleased, 30, keyDPressed, keyDReleased, 32);

        keyboard.spin();
}

// 20 Hz update frequency
void odometryCallback( const odom::Odometry<float>::Vector3& pose )
{
#ifdef LOG_ON
        //std::cout<<"odometry call back function : "<<std::endl;
        std::cout<<"odom_pose : "<<pose.transpose()<<std::endl;
#endif

	auto stamp = time_manage::TimeManage::getTimeStamp();
	sync_list.addData( stamp, pose );

        // send to 
        //geometry::Pose2f pose_2( pose[0], pose[1], pose[2] );
        //odom_sender.send( pose_2 ); // send the odometry data
}

// thread 2 : odometry 
void odometryThread()
{
        odometry.initDevice();

        odometry.spin( odometryCallback );

        odometry.releaseDevice();
}

// 10 Hz update frequency
void lidarCallback( const sensor::LaserScan& scan )
{
	//scan_sender.send( scan ); // send lidar scan data

	sensor::ScanContainer scan_container;
        Utils::laserData2Container( scan, scan_container );
	
	// caculate odometry delta pose
	Eigen::Vector3f odom_delta_pose = Eigen::Vector3f::Zero();
	auto stamp = time_manage::TimeManage::getTimeStamp();
	auto odom_pose = sync_list.getSynchronizedData( stamp );

	if ( !is_initialized ) {
		pre_odom_pose = odom_pose;
		is_initialized = true;
		return ;
	}
	else {
		odom_delta_pose = odom_pose - pre_odom_pose;
		std::cout<<"odom_delta_pose = "<<odom_delta_pose.transpose()<<std::endl;	

		pre_odom_pose = odom_pose; // update the old value
	}

	// slam process
	if ( scan_frame_cnt <= 10 ) { // initialize the map using previous 10 frames
		slam_processor.processTheFirstScan( robot_pose, scan_container );
	
		if ( scan_frame_cnt == 10 ) {
			std::lock_guard<std::mutex> map_guard( map_image_mux );
			{
				slam_processor.generateCvMap( map_image, cost_map_image ); // update the map image
			}

			sendMapImage( map_image ); // send initialized map

			is_map_ready_flag = true; 
			std::cout<<"the map is ready now !!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
		}	
	}
	else {
		if ( odom_delta_pose.norm() < 0.2 ) {
			robot_pose += odom_delta_pose; // update the robot pose by odometry data
		}
		else {
			std::cerr<<"odometry data error : delta pose is too large !"<<std::endl;
		}

		// pose estimated by scan to map optimization
		slam_processor.update( robot_pose, scan_container );

		std::lock_guard<std::mutex> pose_guard( robot_pose_mux );
		{
			robot_pose = slam_processor.getLastScanMatchPose(); // update the robot pose
		}
		std::cout<<"robot pose : "<<robot_pose.transpose()<<std::endl;
		
		geometry::Pose2f pose_2( robot_pose[0], robot_pose[1], robot_pose[2] );
        	pose_sender.send( pose_2 ); // send the global pose of the robot
	
		if ( slam_processor.isKeyFrame() ) {  // key pose
			std::lock_guard<std::mutex> map_guard( map_image_mux );
			{
				slam_processor.generateCvMap( map_image, cost_map_image ); // update the map image		
			}

			// send map
			sendMapImage( map_image );
		}
	}
	
	scan_frame_cnt ++;

}

// thread 3 : lidar process
void lidarThread()
{
	lidar::Lidar m1c1_lidar;

        m1c1_lidar.spin<sensor::LaserScan>( lidarCallback );
}

// thread 4 : path planner
void pathPlannerThread()
{

	planner::AStar<float> a_star;
	pt::Tracking<float> tracking;

	bool is_initialized = false;
	bool is_plan_completed = false;

	std::vector<Eigen::Vector2f> visited_robot_pose_vec;
        Eigen::Vector2i target = Eigen::Vector2i::Zero();
	Eigen::Vector2f target_world = Eigen::Vector2f::Zero();

	std::vector<Eigen::Vector2f> trajectory;
	std::vector<geometry::PoseXY<float>> geometry_trajectory;

	while ( 1 ) {
                usleep( 200000 );
                std::cout<<"----------------------------- path planning ----------------------- "<<std::endl;
	
		Eigen::Vector2f robot_pose_xy = robot_pose.head(2);
                Eigen::Vector2i robot_pose_map = Utils::coordinateTransformWorld2Map( robot_pose_xy, Eigen::Vector2i( 250, 250 ), 0.1f );

		// 1. generate the  target 
		if ( !is_initialized && is_map_ready_flag ) {
			visited_robot_pose_vec.push_back( target_world );

			target = TargetPlanner::generatePlannedTargetGoal( cost_map_image, visited_robot_pose_vec, is_plan_completed );
		
			target_world = Utils::coordinateTransformMap2World( target, Eigen::Vector2i( 250, 250 ), 0.1f );
			std::cout<<"target world = ( "<<target_world.transpose()<<" )"<<std::endl;
	
			if ( is_plan_completed ) {
                                std::cout<<"the robot has traveled all around the world !"<<std::endl;
                                break;
                        }


			a_star.setMap( cost_map_image );
			if ( a_star.findPath( robot_pose_map, target ) ) {
				std::cout<<"find a path to the goal !"<<std::endl;
				trajectory = a_star.getSmoothedPath();		
				
				Utils::convertEigenVec2PoseXYVec( trajectory, geometry_trajectory );
				trajectory_sender.send( geometry_trajectory );

				is_initialized = true;
			}

			continue;
		}

		if ( !is_map_ready_flag ) continue;

		// 2. path tracking
		auto u = tracking.cacuControlVector( trajectory, robot_pose );
		std::cout<<"u = ( "<<u.first<<", "<<u.second<<" )"<<std::endl;
		
		odometry.sendControlVector( u.first, u.second );

		// 3. arrived the target goal
		if ( ( robot_pose_xy - target_world ).norm() < 0.2 || ( u.first == 0 && u.second == 0 ) ) {
			usleep( 100000 );
                        odometry.sendControlVector( 0.0, 0.0 ); // stop the robot
                        std::cout<<"--------------------------- target goal is arrived ! --------------------------"<<std::endl;

			sleep(2); // delay 2 seconds, and then generate the next target goal
			visited_robot_pose_vec.push_back( target_world );


			// need regenerate the target goal
			is_initialized = false;
		}
	}
}

int main()
{
	std::cout<<"---------------------- ROBOT SLAM TEST --------------------"<<std::endl;
	
	slam_processor.printMapInfo();

	std::thread keyboard_control_thread( keyboardControl );
        std::thread odometry_thread( odometryThread );
	std::thread lidar_thread( lidarThread );
	std::thread pathPlanningThread( pathPlannerThread );

        keyboard_control_thread.join();
        odometry_thread.join();
	lidar_thread.join();
	pathPlanningThread.join();


        while ( 1 ) {

        }

	
	return 0;
}

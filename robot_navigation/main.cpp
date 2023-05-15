#include "odometry.h"
#include "utils.h"
#include "lidar_drive.h"
#include "slam_process.h"
#include "time_synchronize.h"

#include "keyboard.h"

#include "data_transport.h"

#include <thread>

// -------------------------------------- GLOBAL DATA ---------------------------------------- //
odom::Odometry<float> odometry; // 1. odometry
sensor::ScanContainer scan_container; // 2. scan container
long scan_frame_cnt = 0; // 3. scan frame counter

time_manage::Synchronize<time_manage::TimeManageData<Eigen::Vector3f>> sync_list; // 4. time synchronization

odom::Odometry<float>::Vector3 pre_odom_pose = odom::Odometry<float>::Vector3::Zero(); // 5. previous odom pose

Eigen::Vector3f robot_pose = Eigen::Vector3f::Zero(); // 6. robot pose

slam::SlamProcessor<float> slam_processor; // 7. slam processor

bool is_initialized = false; // 8. slam initialized flag

cv::Mat map_image = cv::Mat(slam_processor.getSizeX(), slam_processor.getSizeY(), CV_8UC1, cv::Scalar(125)); // 9. global map image

transport::Sender odom_sender( "192.168.3.27", 2335 );
transport::Sender scan_sender( "192.168.3.27", 2336 );
transport::Sender map_sender( "192.168.3.27", 2337 );
transport::Sender pose_sender( "192.168.3.27", 2338 );
// ------------------------------------------------------------------------------------------- //

void sendMapImage()
{
	std::vector<unsigned char> encode_data;
        cv::imencode(".jpg", map_image, encode_data);
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
        odometry.sendControlVector( 0.0, 1.5 );
}

void keyAReleased()
{
        std::cout<<"key A released !"<<std::endl;
        odometry.sendControlVector( 0.0, 0.0 );
}

void keyDPressed()
{
        std::cout<<"key D pressed !"<<std::endl;
        odometry.sendControlVector( 0.0, -1.5 );
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
        geometry::Pose2f pose_2( pose[0], pose[1], pose[2] );
        odom_sender.send( pose_2 ); // send the odometry data
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
#ifdef LOG_ON
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
#endif
	scan_sender.send( scan ); // send lidar scan data

	auto stamp = time_manage::TimeManage::getTimeStamp();

        Utils::laserData2Container( scan, scan_container );
        //Utils::displayScan( scan_container );
	
	// caculate odometry delta pose
	Eigen::Vector3f odom_delta_pose = Eigen::Vector3f::Zero();
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
			sendMapImage(); // send initialized map
		}	
	}
	else {
		if ( odom_delta_pose.norm() < 0.1 ) {
			robot_pose += odom_delta_pose; // update the robot pose by odometry data
		}
		else {
			std::cerr<<"odometry data error : delta pose is too large !"<<std::endl;
		}

		// pose estimated by scan to map optimization
		slam_processor.update( robot_pose, scan_container );
		robot_pose = slam_processor.getLastScanMatchPose(); // update the robot pose
		std::cout<<"robot pose : "<<robot_pose.transpose()<<std::endl;
		geometry::Pose2f pose_2( robot_pose[0], robot_pose[1], robot_pose[2] );
        	pose_sender.send( pose_2 ); // send the global pose of the robot
	
		if ( slam_processor.isKeyFrame() ) {  // key pose
			slam_processor.displayMap( map_image, false ); // update the map image		
			// send map
			sendMapImage();
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
	while ( 1 ) {
		

	}
}

int main()
{
	std::cout<<"---------------------- ROBOT SLAM TEST --------------------"<<std::endl;
	
	slam_processor.printMapInfo();

	std::thread keyboard_control_thread( keyboardControl );
        std::thread odometry_thread( odometryThread );
	std::thread lidar_thread( lidarThread );

        keyboard_control_thread.join();
        odometry_thread.join();
	lidar_thread.join();


        while ( 1 ) {

        }

	
	return 0;
}

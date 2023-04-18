#include "keyboard.h"
#include "odometry.h"

#include <thread>
#include <chrono>

// -------------------------------------- GLOBAL DATA ---------------------------------------- //
odom::Odometry<float> odometry;
odom::Odometry<float>::Vector3 odom_pose = odom::Odometry<float>::Vector3::Zero();


// ------------------------------------------------------------------------------------------- //

void keyWPressed()
{
	std::cout<<"key W pressed !"<<std::endl;
	
}

void keyWReleased()
{	
	std::cout<<"key W released !"<<std::endl;
}

void keyAPressd()
{
	std::cout<<"key A pressed !"<<std::endl;
}

void keyAReleased()
{
	std::cout<<"key A released !"<<std::endl;
}

void keyDPressed()
{
	std::cout<<"key D pressed !"<<std::endl;
}

void keyDReleased()
{	
	std::cout<<"key D released !"<<std::endl;
}

void keyboardControl()
{
	keyboard::Keyboard keyboard;
	
	keyboard.init();
        keyboard.registCallBackFunc( keyWPressed, keyWReleased, 17, keyAPressd, keyAReleased, 30, keyDPressed, keyDReleased, 32);

        keyboard.spin();
}

void odometryCallback( const odom::Odometry<float>::Vector3& measure )
{
	std::cout<<"odometry call back function : "<<std::endl;
}

void odometryThread()
{
	odometry.initDevice();
	odometry.registerCallbackFunc( odometryCallback );

	odometry.spin();
	
	odometry.releaseDevice();
}

int main()
{
	std::cout<<"---------------------- DESKTOP ROBORT LOCALIZATION --------------------"<<std::endl;

	std::thread keyboard_control_thread( keyboardControl );
	//std::thread odometry_thread( odometryThread );


	keyboard_control_thread.join();
	// odometry_thread.join();


	while(1){

	}

	return 0;
}

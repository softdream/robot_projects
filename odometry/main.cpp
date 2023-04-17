#include "keyboard.h"
#include "odometry.h"

#include <thread>
#include <chrono>

// -------------------------------------- GLOBAL DATA ---------------------------------------- //

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

int main()
{
	std::cout<<"---------------------- DESKTOP ROBORT LOCALIZATION --------------------"<<std::endl;

	std::thread keyboard_control_thread( keyboardControl );

	keyboard_control_thread.join();

	while(1){

	}

	return 0;
}

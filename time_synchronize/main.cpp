#include "time_synchronize.h"
#include <iostream>
#include <thread>
#include <Eigen/Dense>

#include <unistd.h>

time_manage::Synchronize<Eigen::Vector2f> sync_list;

void testThread1()
{
	float cnt = 0;

	while ( 1 ) {
		float stamp = static_cast<float>( time_manage::TimeManage::getTimeStamp() );
		Eigen::Vector2f vec( stamp, cnt ++ );
		std::cout<<vec.transpose()<<std::endl;
		
		sync_list.addData( vec );
		usleep( 50000 );
	}
}

void testThread2()
{
	float cnt = 0;
	
	while ( 1 ) {
		float stamp = static_cast<float>( time_manage::TimeManage::getTimeStamp() );
		std::cout<<"stamp : "<<stamp<<std::endl;
		auto ret = sync_list.getSynchronizedData( stamp );
		std::cout<<"ret : "<<ret.transpose()<<std::endl;

		usleep( 100000 );
	}
}

int main()
{
	std::thread t1( testThread1 );
	std::thread t2( testThread2 );
	t1.join();
	t2.join();
	

	return 0;
}


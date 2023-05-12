#include "time_synchronize.h"
#include <iostream>
#include <thread>
#include <Eigen/Dense>

#include <unistd.h>

time_manage::Synchronize<time_manage::TimeManageData<Eigen::Vector2f>> sync_list;

void testThread1()
{
	float cnt = 0;

	while ( 1 ) {
		auto stamp = time_manage::TimeManage::getTimeStamp();
		Eigen::Vector2f data( cnt, cnt );
		std::cout<<"thread 1 stamp : "<<stamp<<", data : "<<data.transpose()<<std::endl;
		
		sync_list.addData( stamp, data );
		usleep( 50000 );
		
		cnt ++;
	}
}

void testThread2()
{
	float cnt = 0;
	
	while ( 1 ) {
		auto stamp = time_manage::TimeManage::getTimeStamp();
		auto ret = sync_list.getSynchronizedData( stamp );
		std::cout<<"thread 2 stamp : "<<stamp<<", sync data : "<<ret.transpose()<<std::endl;
	
		std::cout<<std::endl;

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


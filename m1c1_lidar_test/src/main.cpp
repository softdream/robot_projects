#include "lidar_drive.h"

void lidarCallback( int a )
{
	
}

int main(int argc, char *argv[])
{
	std::cout<<"----------------- LIDAR TEST ----------------"<<std::endl;

	lidar::Lidar laser;

	laser.spin<int>( lidarCallback );

	return 0;
}

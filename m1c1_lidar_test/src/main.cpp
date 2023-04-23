#include "lidar_drive.h"
#include "data_type.h"

void lidarCallback( const sensor::LaserScan& scan )
{
	std::cout<<"lidar data callback ..."<<std::endl;

	std::cout<<"angle_min : "<<scan.angle_min<<std::endl;
	std::cout<<"angle_max : "<<scan.angle_max<<std::endl;
	std::cout<<"angle_increment : "<<scan.angle_increment<<std::endl;
	std::cout<<"range_min : "<<scan.range_min<<std::endl;
	std::cout<<"range_max : "<<scan.range_max<<std::endl;
	std::cout<<"scan_time : "<<scan.scan_time<<std::endl;
}

int main(int argc, char *argv[])
{
	std::cout<<"----------------- LIDAR TEST ----------------"<<std::endl;

	lidar::Lidar m1c1_lidar;

	m1c1_lidar.spin<sensor::LaserScan>( lidarCallback );

	return 0;
}

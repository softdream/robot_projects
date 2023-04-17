#include "uart.h"
#include <string.h>

#pragma pack(4)

char buffer[100];

typedef struct IMU_
{
	float ax;
	float ay;
	float gz;
}IMU;

typedef struct Measurement_
{
	long time_stamp;
	float v;
	float delta_s;
	float delta_angle;

	IMU imu;
}Measurement;

int main()
{
	std::cout<<"------------ UART TEST ------------"<<std::endl;

	uart::Uart u;

	while ( 1 ) {
		int ret = u.readData( buffer, 100 );

		if ( ret > 0 ) {
			std::cout<<"ret = "<<ret<<std::endl;
			std::cout<<buffer<<std::endl;
		}
		usleep(40 * 1000);
	}

	//std::cout<<"sizeof  = "<<sizeof(Measurement)<<std::endl;

	return 0;
}

#ifndef __LIDAR_DRIVE_H
#define __LIDAR_DRIVE_H

#include "C_CSPC_Lidar.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <cspclidar_protocol.h>

#include <functional>


namespace lidar
{

using namespace cspclidar;

template<typename T>
using CallBackRef = std::function<void( const T& )>;

class Lidar
{
public:
	Lidar()
	{
		init();
	}
	
        ~Lidar()
        {
                laser_->turnOff();
                laser_->disconnecting();
                delete laser_;
        }


	void init()
	{
		int argc; 
		char **argv;
		cspclidar::init(argc, argv);		

		laser_ = new C_CSPC_Lidar();
	
		laser_->setSerialPort(port_);
	        laser_->setSerialBaudrate(baudrate_);
        	laser_->setFixedResolution(/*true*/false);
	        laser_->setReversion(/*false*/false);
        	laser_->setInverted(false);
	        laser_->setAutoReconnect(true);
        	laser_->setSingleChannel(isSingleChannel_);
	        laser_->setLidarType(isTOFLidar_ ? TYPE_TOF : TYPE_TRIANGLE);
        	laser_->setMaxAngle(360);
	        laser_->setMinAngle(-0);
        	laser_->setMinRange(0.01);
	        laser_->setMaxRange(10.0);
        	laser_->setScanFrequency(frequency_);
	        std::vector<float> ignore_array;
	        ignore_array.clear();
        	laser_->setIgnoreArray(ignore_array);

		ret = laser_->initialize();
        	if ( ret ) {
                	ret = laser_->turnOn();
        	}
	}

	
	template<typename T>
	void spin( CallBackRef<T>&& cb )
	{
		while ( ret && cspclidar::ok() ) {
	                bool hardError;
        	        LaserScan scan;
                	if ( laser_->doProcessSimple(scan, hardError) ) {
                        	std::cout<<"Scan recved : "<<scan.stamp<<", size : "<<scan.points.size()<<", frequency : "<<1.0 / scan.config.scan_time<<std::endl;
                	
				T scan_data;
				//scan_data.angle_min = -M_PI;
				//scan_data.angle_max = M_PI;
				scan_data.angle_increment = ( 2.0 * M_PI / scan.points.size() );
				for ( size_t i = 0; i < 380; i ++ ) {
					scan_data.ranges[i] = scan.points[i].range;
					scan_data.intensities[i] = scan.points[i].intensity;
				}
				cb( scan_data );
			}
                	else {
                        	std::cerr<<"Failed to get lidar data !"<<std::endl;
                	}
	
        	}
	}

private:
	const std::string port_ = "/dev/ttyUSB0";//"/dev/sc_mini";
	const int baudrate_ = 115200;

        bool isSingleChannel_ = true;
        bool isTOFLidar_ = false;
        std::string input_channel_;
        std::string input_tof_;
        std::string input_frequency_;

        const float frequency_ = 10.0;

	C_CSPC_Lidar* laser_ = nullptr;

	bool ret = false;
};

}

#endif

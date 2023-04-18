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
using CallBackRef = std::function<void( T& )>;

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
        	laser_->setFixedResolution(false);
	        laser_->setReversion(false);
        	laser_->setInverted(false);
	        laser_->setAutoReconnect(true);
        	laser_->setSingleChannel(isSingleChannel_);
	        laser_->setLidarType(isTOFLidar_ ? TYPE_TOF : TYPE_TRIANGLE);
        	laser_->setMaxAngle(360);
	        laser_->setMinAngle(0);
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
				cb( scan_data );
			}
                	else {
                        	std::cerr<<"Failed to get lidar data !"<<std::endl;
                	}
	
        	}
	}

private:
	const std::string port_ = "/dev/sc_mini";
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

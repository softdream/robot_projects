#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "ekf_fusion.h"
#include "EpollEvent.h"

#include "uart.h"

#include <mutex>
#include <thread>

#define SIZE 100

namespace odom
{

using namespace std::placeholders;

typedef enum MessageType_
{
	none,
	measurement,
	sensor
}MessageType;

template<typename T>
class Odometry
{
public:
        using DataType = T;
        using Vector2 = typename Eigen::Matrix<DataType, 2, 1>;
        using Vector3 = typename Eigen::Matrix<DataType, 3, 1>;
        using Vector4 = typename Eigen::Matrix<DataType, 4, 1>;
	using Vector5 = typename Eigen::Matrix<DataType, 5, 1>;
        using Vector6 = typename Eigen::Matrix<DataType, 6, 1>;

        using Matrix3x3 = typename Eigen::Matrix<DataType, 3, 3>;

	Odometry()
	{

	}

	~Odometry()
	{

	}

	void initDevice()
	{
		uart_ = new uart::Uart();
	
		Event odom_event;
		odom_event.fd = uart_->getFd();
		odom_event.event |= EPOLLIN;
                odom_event.event |= EPOLLERR;
                odom_event.event |= EPOLLET;
                odom_event.arg = NULL;
		
		FUNC recv_cb = std::bind( &Odometry::odomRecvCallback, this, _1, _2 );
		odom_event.callback = recv_cb;

		event_base_.addEvent( odom_event );
	}

	void releaseDevice()
	{
		uart_->closeDevice();
		delete uart_;
	}

	void spin()
        {
                while(true){
                        event_base_.dispatcher();
                }
        }

	void* odomRecvCallback( int fd, void* arg )
	{
		if ( uart_->readData( recv_buffer_, SIZE ) ) {
			MessageType type = parseData();

			// millis(), velocity, delta_s, delta_angle, imu.gz
			if ( type == measurement ) {

				if ( measure_[1] != 0 ) is_static_ = true;
				else is_static_ = false;
	
				Vector2 u( measure_[2], measure_[3] );
				odom_ekf_.predict( u );
	
				DataType gz = -measure_[4] * ( M_PI / 180 );
	
				if ( is_static_ ) gz = 0;
				
				DataType now_time = measure_[0];
                                if ( !is_init_ ) {
                                        pre_time_ = now_time;
                                        pre_gz_ = gz;
					is_init_ = true;

					return nullptr;
                                }   

				// integrate
				DataType delta_t = ( now_time - pre_time_ ) / 1000;
				DataType theta = delta_t * ( pre_gz_ + gz ) * 0.5;
				odom_ekf_.update( theta );

				// update the pre parameters
                                pre_gz_ = gz;
                                pre_time_ = now_time;		

				// get the key pose of the robot
				if ( odom_ekf_.isKeyFrame() ) {
					pose_mux_.lock();
					robot_pose_ = odom_ekf_.getStateX();
					pose_mux_.unlock();
				}
				
			}
			// humidity, temperature, distance 
			else if ( type == sensor ) {
				
			} 
		}
	
		return nullptr;
	}

private:
	MessageType parseData()
	{
		std::string str = this->recv_buffer_;
		
		std::istringstream iss(str);

		std::string tag;
                iss >> tag;

		if ( tag.compare( "meas" ) == 0 ) {
			measure_.setZero();
		
			std::string num;
			for ( size_t i = 0; i < 5; i ++ ) {
				iss >> num;
				measure_(i) = static_cast<DataType>( std::stod( num ) );
			}

			return measurement;
		}
		else if ( tag.compare( "sens" ) == 0 ) {
			sensors_.setZero();
	
			std::string num;
                        for ( size_t i = 0; i < 3; i ++ ) {
                                iss >> num;
                                sensors_(i) = static_cast<DataType>( std::stod( num ) );
                        }

			return sensor;
		}

		return none;
	}

private:
	// 
	EpollEvent event_base_;
	uart::Uart *uart_ = nullptr;

	// ekf fusion
        ekf::EKF<DataType> odom_ekf_;
        DataType pre_time_ = 0;
        bool is_init_ = false;
        DataType pre_gz_ = 0;

        bool is_static_ = false;

        // pose
        Vector3 robot_pose_ = Vector3::Zero();
        std::mutex pose_mux_;

	// recv buffer
	char recv_buffer_[SIZE];

	// data
	Vector5 measure_ = Vector5::Zero();
	Vector3 sensors_ = Vector3::Zero();
};

}

#endif

#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "ekf_fusion.h"
#include "EpollEvent.h"

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

	void init()
	{
		
	}

	void spin()
        {
                while(true){
                        event_base.dispatcher();
                }
        }

	void* odomRecvCallback( int fd, void* arg )
	{

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
	EpollEvent event_base_;

	// ekf fusion
        ekf::EKF<DataType> odom_ekf_;
        DataType pre_time_ = 0;
        bool is_init_ = false;
        DataType pre_gz_ = 0;
        DataType theta_ = 0;

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

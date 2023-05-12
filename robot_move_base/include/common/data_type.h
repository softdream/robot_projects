#ifndef __DATA_TYPE_H
#define __DATA_TYPE_H

#include <string.h>

namespace geometry
{

template<typename T>
struct Pose2_
{
	using Type = T;	

        Pose2_()
        {

        }

        Pose2_( const T x_, const T y_, const T theta_ ) :
                 x( x_ ), y( y_ ), theta( theta_ )
        {

        }

        T x = 0.0;
        T y = 0.0;
        T theta = 0.0;

};

using Pose2d = Pose2_<double>;
using Pose2f = Pose2_<float>;
using Pose2i = Pose2_<int>;

}

namespace sensor
{

template<typename T, int Size>
struct LaserScan_
{
	using Type = T;

	LaserScan_()
	{

	}

	~LaserScan_()
	{

	}

	LaserScan_( const LaserScan_& obj ) : angle_min( obj.angle_min ),
					    angle_max( obj.angle_max ),
					    angle_increment( obj.angle_increment ),
					    scan_time( obj.scan_time ),
					    time_increment( obj.time_increment ),
					    range_min( obj.range_min ),
					    range_max( obj.range_max )
	{
		memcpy( this->ranges, obj.ranges, Size );
		memcpy( this->intensities, obj.intensities, Size );
	}

	LaserScan_& operator=( const LaserScan_& other )
	{
		if( &other == this )
			return *this;

		angle_min = other.angle_min;
		angle_max = other.angle_max;
		angle_increment = other.angle_increment;
		scan_time = other.scan_time;
		time_increment = other.time_increment;
		range_min = other.range_min;
		range_max = other.range_max;
		memcpy( this->ranges, other.ranges, Size );
                memcpy( this->intensities, other.intensities, Size );
	
		return *this;
	}

	void setParameters( const T angle_min, 
			    const T angle_max, 
			    const T angle_increment,
			    const T range_min,
			    const T range_max, 
			    const T scan_time = 0,
			    const T time_increment = 0 )
	{
		this->angle_min = angle_min;
		this->angle_max = angle_max;
		this->angle_increment = angle_increment;
		this->range_min = range_min;
		this->range_max = range_max;
		this->scan_time = scan_time;
		this->time_increment = time_increment;
	}

	int size() const
	{
		return Size;
	}
	
	T angle_min = 0;
        T angle_max = 2 * M_PI;
        T angle_increment = 0.0162356209f;
        T scan_time = 0;
        T time_increment = 0;
        T range_min = 0.1f;
        T range_max = 10.0;
        T ranges[Size] = { 0 };
        T intensities[Size] = { 0 };
	
};


using LaserScan = LaserScan_<float, 380>;
using LaserScanD = LaserScan_<double, 380>;

}


#endif

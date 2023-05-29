#ifndef __DATA_TYPE_H
#define __DATA_TYPE_H

#include <string.h>


namespace sensor{

#ifndef M_PI
#define M_PI 3.141592653
#endif

template<int Size>
struct LidarScan{
	LidarScan(){}

	~LidarScan(){}

	LidarScan( const LidarScan& obj ) : angle_min( obj.angle_min ),
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

	LidarScan& operator=( const LidarScan& other )
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

	void setParameters( const float angle_min, 
			    const float angle_max, 
			    const float angle_increment,
			    const float range_min,
			    const float range_max, 
			    const float scan_time = 0,
			    const float time_increment = 0 )
	{
		this->angle_min = angle_min;
		this->angle_max = angle_max;
		this->angle_increment = angle_increment;
		this->range_min = range_min;
		this->range_max = range_max;
		this->scan_time = scan_time;
		this->time_increment = time_increment;
	}

	const int size()
	{
		return Size;
	}

	const int size() const{
		return Size;
	}
	
	float angle_min = 0;
        float angle_max = 2 * M_PI;
        float angle_increment = 0.0162356209f;
        float scan_time = 0;
        float time_increment = 0;
        float range_min = 0.1f;
        float range_max = 10.0;
        float ranges[Size] = { 0 };
        float intensities[Size] = { 0 };
	
};

typedef struct LidarScan<380> LaserScan;

}


#endif

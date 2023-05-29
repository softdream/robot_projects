#ifndef __PID_TRACKING_H
#define __PID_TRACKING_H

#include "pid.h"
#include "utils.h"

#include <Eigen/Dense>

#include <vector>
#include <cmath>
#include <limits>

namespace pt
{

template<typename T>
class Tracking
{
public:
	using DataType = T;
	using Vector2 = typename Eigen::Matrix<DataType, 2, 1>;
	using Vector3 = typename Eigen::Matrix<DataType, 3, 1>;

	Tracking()
	{
		yaw_pid_ = new PID<DataType>(0.2, 1.3, -1.3, 0.8, 0.5, 0.03);
	}

	~Tracking()
	{
		delete yaw_pid_;
	}

	const DataType cacuYaw(const Vector2& p1, const Vector2& p2)
	{
		Vector2 tmp = p2 - p1;
		DataType yaw = 0;

		if ( tmp[0] < 0 && tmp[1] > 0 ) {
			yaw = M_PI + ::atan2(tmp[1], tmp[0]);
		}
		else if ( tmp[0] < 0 && tmp[1] < 0 ) {
			yaw = ::atan2(tmp[1], tmp[0]) - M_PI;
		}
		else {
			yaw = ::atan2(tmp[1], tmp[0]);
		}

		Utils::angleNormalize( yaw );
			
		return yaw;
	}

	const DataType cacuDist(const Vector2& p1, const Vector2& p2)
	{
		return (p1 - p2).norm();
	}

	const DataType yawPidProcess(const DataType& current_yaw, const DataType& dst_yaw)
	{
		return yaw_pid_->caculate(dst_yaw, current_yaw);
	}

	const int findClosedPtIndex( const std::vector<Vector2>& trajectory, const Vector2& curr_pose )
	{
		int min_dist = std::numeric_limits<DataType>::max();
		int min_pt_idx = -1;

		for ( int i = 0; i < trajectory.size(); i ++ ) {
			auto dist = ( curr_pose - trajectory[i] ).norm();
			if ( dist < min_dist  ) {
				min_dist = dist;
				min_pt_idx = -1;
			}	
		}

		return min_pt_idx;
	}

	// u = ( v, w )
	const std::pair<DataType, DataType> cacuControlVector( const std::vector<Vector2>& trajectory, const Vector3& curr_pose )
	{	
		auto curr_pose_xy = curr_pose.head(2);
		auto curr_yaw = curr_pose[2];

		auto closed_idx = findClosedPtIndex( trajectory, curr_pose_xy );	
		
		if ( closed_idx == -1 ) {
			std::cout<<"there is no closed point around the current pose !"<<std::endl;
			return { 0.0, 0.0 };
		}

		if ( closed_idx == trajectory.size() - 1 ) {
			std::cout<<"arrived at the target goal !"<<std::endl;
			return { 0.0, 0.0 };
		}

		auto target_yaw = cacuYaw( trajectory[closed_idx], trajectory[closed_idx + 1] );
		if ( std::abs( curr_yaw - target_yaw ) >= ( M_PI * 0.5 ) ) {
			auto w = yawPidProcess( curr_yaw, target_yaw );
			return { 0.0, -w };
		}
		else{
			auto w = yawPidProcess( curr_yaw, target_yaw );
			return { 0.15, -w };
		}
		
		return { 0.0, 0.0 };
	}

	// u = ( v, w )
        const std::pair<DataType, DataType> cacuControlVector( const DataType target_yaw, const DataType curr_yaw )
	{
		if ( std::abs( curr_yaw - target_yaw ) >= ( M_PI * 0.5 ) ) {
                        auto w = yawPidProcess( curr_yaw, target_yaw );
                        return { 0.0, -w };
                }
                else{
                        auto w = yawPidProcess( curr_yaw, target_yaw );
                        return { 0.15, -w };
                }

                return { 0.0, 0.0 };
	}

private:
	PID<DataType>* yaw_pid_;

};

}

#endif
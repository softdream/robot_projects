#ifndef __PID_TRACKING_H
#define __PID_TRACKING_H

#include "pid.h"

#include <Eigen/Dense>

#include <cmath>

namespace pt
{

	template<typename T>
	class Tracking
	{
	public:
		using DataType = T;
		using Vector2 = typename Eigen::Matrix<DataType, 2, 1>;


		Tracking()
		{
			yaw_pid = new PID<DataType>(0.4, 1, -1, 0.8, 0.5, 0.3);
			y_pose_pid = new PID<DataType>(0.5, 2, -2, 0.3, 0.3, 0.3);
			x_pose_pid = new PID<DataType>(0.5, 2, -2, 0.3, 0.3, 0.3);
		}

		~Tracking()
		{
			delete yaw_pid;
			delete y_pose_pid;
			delete x_pose_pid;
		}

		const DataType cacuYaw(const Vector2& p1, const Vector2& p2)
		{
			Vector2 tmp = p2 - p1;
			DataType yaw = ::atan2(tmp[1], tmp[0]);

			return yaw;
		}

		const DataType cacuDist(const Vector2& p1, const Vector2& p2)
		{
			return (p1 - p2).norm();
		}

		const DataType yawPidProcess(const DataType& current_yaw, const DataType& dst_yaw)
		{
			return yaw_pid->caculate(dst_yaw, current_yaw);
		}

		const DataType yPosePidProcess(const DataType& current_pose, const DataType& dst_pose)
		{
			return y_pose_pid->caculate(dst_pose, current_pose);
		}

		const DataType xPosePidProcess(const DataType& current_pose, const DataType& dst_pose)
		{
			return x_pose_pid->caculate(dst_pose, current_pose);
		}


	private:
		PID<DataType>* yaw_pid;
		PID<DataType>* y_pose_pid;
		PID<DataType>* x_pose_pid;

	};

}

#endif

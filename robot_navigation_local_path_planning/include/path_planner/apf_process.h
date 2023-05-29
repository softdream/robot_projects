#ifndef __APF_PROCESS_H
#define __APF_PROCESS_H

#include "apf.h"
#include "utils.h"

namespace apf
{
	
template<typename T>
class APFProcess
{
public:
	using DataType = T;
	using Vector2 = typename Eigen::Matrix<T, 2, 1>;
	using Vector3 = typename Eigen::Matrix<T, 3, 1>;

	APFProcess()
	{
	
	}

	~APFProcess()
	{
	
	}

	void setTargetPose( const Vector2& target_pose )
	{
		return apf_.setTargetPose( target_pose );
	}

	// return ( force, direction )
	const std::pair<DataType, DataType> runApfOnce( const Vector2& robot_pose, 
		       		   			const Obstacles<DataType>& obstacles	)
	{
		// 1. attraction vector
		Vector2 f_att_vec = apf_.cacuFatt( robot_pose );

		// 2. repulsion vector
		Vector2 f_rep_vec = apf_.cacuFrep( robot_pose, obstacles );
		apf_.clearRepulsionVec1();

		// 3. total force
		Vector2 f_total = f_att_vec + f_rep_vec;

		// 4. get target angle
		DataType target_theta = 0;
		if ( f_total[0] < 0 && f_total[1] > 0 ) {
			target_theta = M_PI + ::atan2( f_total[1], f_total[0] );
		}
		else if ( f_total[0] < 0 && f_total[1] < 0  ) {
			target_theta = ::atan2( f_total[1], f_total[0] ) - M_PI;
		}
		else {
			target_theta = ::atan2( f_total[1], f_total[0] );
		}

		Utils::angleNormalize( target_theta );

		return { f_total.norm() *  force_scale, target_theta };
	}

private:
	APF<DataType> apf_;
	const DataType force_scale = 0.1;
};

}


#endif

#ifndef __APF_H
#define __APF_H

#include "obstacles.h"

#include <cmath>


//#define DEBUG

namespace apf
{

template<typename T>
class APF
{
public:
	using DataType = T;
	using Vector2 = typename Eigen::Matrix<T, 2, 1>;

	APF()
	{

	}

	~APF()
	{

	}

	void setTargetPose( const Vector2& target_pose )
	{
		target_pose_ = target_pose;
	}

	// attraction
	const Vector2 cacuFatt( const Vector2& robot_pose )
	{
		DataType dist = distance( robot_pose, target_pose_ );

#ifdef DEBUG
		std::cout<<"dist attraction = "<<dist<<std::endl; 
#endif

		DataType f_att_norm = dist * eta_;
	
		//Vector2 direction = ( target_pose - robot_pose ) / ( target_pose - robot_pose ).norm();
		Vector2 direction = ( target_pose_ - robot_pose ).normalized();

#ifdef DEBUG
		std::cout<<"attraction direction = "<<direction.transpose()<<std::endl;	
		std::cout<<"attraction vec = "<<( f_att_norm * direction ).transpose()<<std::endl<<std::endl;
#endif

		return f_att_norm * direction;
	}

	// repulsion
	const Vector2 cacuFrep( const Vector2& robot_pose, const Obstacles<DataType>& obstacles )
	{
		if( obstacles.isEmpty() ){
			return Vector2::Zero();
		}
		
		DataType rho_g = distance( robot_pose, target_pose_ );

		// for each obstacle
		Vector2 f_rep_total = Vector2::Zero();
		for( int i = 0; i < obstacles.getSize(); i ++ ){
			DataType dist = distance( robot_pose, obstacles[i] );

#ifdef DEBUG
			std::cout<<"dist repulsion "<<i<<" : "<<dist<<std::endl;
#endif

			DataType f_rep1_norm = 0.0;
			DataType f_rep2_norm = 0.0;
			
			if( dist > rho0_ ){
				f_rep1_norm = 0.0;
				f_rep2_norm = 0.0;
			}
			else {
				f_rep1_norm = cauchy_ * ( 1 / dist - 1 / rho0_ ) * ( ::pow( rho_g, n_ ) ) / ( ::pow( dist, 2 ) );
				f_rep2_norm = ( static_cast<DataType>( n_ ) * cauchy_ / 2 ) * ( ::pow( ( 1 / dist - 1 / rho0_ ), 2 ) ) * ( ::pow( rho_g, n_ - 1 ) );

#ifdef DEBUG
				std::cout<<"f_rep1_norm "<<i<<" : "<<f_rep1_norm<<std::endl;
#endif
			}
			
			//Vector2 f_rep1_direction = ( robot_pose - obstacles[i] ) * ( robot_pose - obstacles[i] ).norm();
			//Vector2 f_rep2_direction = ( target_pose - robot_pose ) * ( target_pose - robot_pose ).norm();

			Vector2 f_rep1_direction = ( robot_pose - obstacles[i] ).normalized();
			Vector2 f_rep2_direction = ( target_pose_ - robot_pose ).normalized();

#ifdef DEBUG
			std::cout<<"f_rep1 direction "<<i<<" : "<<f_rep1_direction.transpose()<<std::endl<<std::endl;			
#endif

			// for display
			repulsion_vec1_.push_back( f_rep1_norm * f_rep1_direction );
	
#ifdef DEBUG
			std::cout<<"f_rep1 "<<i<<" : "<<( f_rep1_norm * f_rep1_direction ).transpose()<<std::endl;
#endif

			Vector2 f_rep = ( f_rep1_norm * f_rep1_direction ) + ( f_rep2_norm * f_rep2_direction  );

			f_rep_total += f_rep;
	
		}	

		return f_rep_total;
	}

	const std::vector<Vector2>& getRepulsionVec1() const
	{
		return repulsion_vec1_;
	}

	void clearRepulsionVec1() 
	{
		return repulsion_vec1_.clear();
	}

private:
	const DataType distance( const Vector2& p1, const Vector2& p2 ) const
        {
                return ( p1 - p2 ).norm();
        }


private:
	DataType eta_ = 1.0;
        DataType cauchy_ = 0.12;
	DataType rho0_ = 0.4;
	int n_ = 3;

	Vector2 target_pose_ = Vector2::Zero();

	std::vector<Vector2> repulsion_vec1_;
};

}

#endif


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

	void setTargetPose( const Vector2& target_pose_ )
	{
		target_pose = target_pose_;
	}

	// attraction
	const Vector2 cacuFatt( const Vector2& robot_pose )
	{
		DataType dist = distance( robot_pose, target_pose );
		std::cout<<"dist attraction = "<<dist<<std::endl; 

		DataType f_att_norm = dist * eta;
	
		//Vector2 direction = ( target_pose - robot_pose ) / ( target_pose - robot_pose ).norm();
		Vector2 direction = ( target_pose - robot_pose ).normalized();

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
		
		DataType rho_g = distance( robot_pose, target_pose );

		// for each obstacle
		Vector2 f_rep_total = Vector2::Zero();
		for( int i = 0; i < obstacles.getSize(); i ++ ){
			DataType dist = distance( robot_pose, obstacles[i] );

#ifdef DEBUG
			std::cout<<"dist repulsion "<<i<<" : "<<dist<<std::endl;
#endif

			DataType f_rep1_norm = 0.0;
			DataType f_rep2_norm = 0.0;
			
			if( dist > rho0 ){
				f_rep1_norm = 0.0;
				f_rep2_norm = 0.0;
			}
			else {
				f_rep1_norm = cauchy * ( 1 / dist - 1 / rho0 ) * ( ::pow( rho_g, n ) ) / ( ::pow( dist, 2 ) );
				f_rep2_norm = ( static_cast<DataType>( n ) * cauchy / 2 ) * ( ::pow( ( 1 / dist - 1 / rho0 ), 2 ) ) * ( ::pow( rho_g, n - 1 ) );
				std::cout<<"f_rep1_norm "<<i<<" : "<<f_rep1_norm<<std::endl;
			}
			
			//Vector2 f_rep1_direction = ( robot_pose - obstacles[i] ) * ( robot_pose - obstacles[i] ).norm();
			//Vector2 f_rep2_direction = ( target_pose - robot_pose ) * ( target_pose - robot_pose ).norm();

			Vector2 f_rep1_direction = ( robot_pose - obstacles[i] ).normalized();
			Vector2 f_rep2_direction = ( target_pose - robot_pose ).normalized();

			std::cout<<"f_rep1 direction "<<i<<" : "<<f_rep1_direction.transpose()<<std::endl<<std::endl;			

			// for display
			repulsion_vec1.push_back( f_rep1_norm * f_rep1_direction );
	
			std::cout<<"f_rep1 "<<i<<" : "<<( f_rep1_norm * f_rep1_direction ).transpose()<<std::endl;

			Vector2 f_rep = ( f_rep1_norm * f_rep1_direction ) + ( f_rep2_norm * f_rep2_direction  );

			f_rep_total += f_rep;
	
		}	

		return f_rep_total;
	}

	const std::vector<Vector2>& getRepulsionVec1() const
	{
		return repulsion_vec1;
	}

	void clearRepulsionVec1() 
	{
		return repulsion_vec1.clear();
	}

private:
	const DataType distance( const Vector2& p1, const Vector2& p2 ) const
        {
                return ( p1 - p2 ).norm();
        }


private:
	DataType eta = 1.0;
        DataType cauchy = 0.08;
	DataType rho0 = 0.6;
	int n = 3;

	Vector2 target_pose = Vector2::Zero();

	std::vector<Vector2> repulsion_vec1;
};

}

#endif


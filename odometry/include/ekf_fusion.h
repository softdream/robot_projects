#ifndef __EKF_FUSION_H
#define __EKF_FUISION_H

#include <iostream>
#include <Eigen/Dense>
#include <cmath>

#ifndef M_PI 
#define M_PI 3.141592653
#endif

namespace ekf
{

template<typename T>
class EKF
{
public:
	using DataType = T;
	using Vector2 = typename Eigen::Matrix<T, 2, 1>;
	using Vector3 = typename Eigen::Matrix<T, 3, 1>;
	using Matrix2 = typename Eigen::Matrix<T, 2, 2>;
	using Matrix3 = typename Eigen::Matrix<T, 3, 3>;
	using Matrix1x3 = typename Eigen::Matrix<T, 1, 3>;
	using Matrix3x1 = typename Eigen::Matrix<T, 3, 1>;

	EKF()
	{

	}

	~EKF()
	{

	}

	void predict( const Vector2 &u_now )  // input control vector: ( delta_s, delta_theta )
	{
		if( is_init == false ){
			is_init = true;
			
			return;
		}
#ifdef DEBUG
		std::cout<<" start predict "<<std::endl;
#endif
		// 1. state prediction
		x_now(0) = x_pre(0) + u_now(0) * ::cos( x_pre(2) + 0.5 * u_now(1) ); // x
		x_now(1) = x_pre(1) + u_now(0) * ::sin( x_pre(2) + 0.5 * u_now(1) ); // y
		x_now(2) = x_pre(2) + u_now(1); // theta
		// added
		angleNormalize( x_now[2] );

		// 2. caculate state Jacobian matrix	
		F = Matrix3::Identity();
		F(0, 2) = -u_now(0) * ::sin( x_pre(2) + 0.5 * u_now(1) );
		F(1, 2) =  u_now(0) * ::cos( x_pre(2) + 0.5 * u_now(1) );
	
		// 3. state covarince prediction
		P_now = F * P_pre * F.transpose() + Q;
	
		//std::cout<<"delta_theta = "<<x_now(2) - x_pre(2)<<std::endl;
		// 4. update the old value
		//x_pre = x_now;
		//P_pre = P_now;
		//std::cout<<"x estimated : "<<std::endl<<x_now<<std::endl;
#ifdef DEBUG
		std::cout<<" prediction end "<<std::endl;
#endif
	}	

	void update( const DataType z ) // measurement value : delta_theta
	{
		if( is_init == false ){
                        return;
        }
#ifdef DEBUG
        std::cout<<" update "<<std::endl;
#endif

		// 1. measurement estimate
		H << 0, 0, 1;
		DataType h = H * x_now - x_pre(2);
		//std::cout<<"h = "<<h<<std::endl;
		//std::cout<<"z = "<<z<<std::endl;		
		
		// 2. measurement error
		//DataType error = z - h;
		DataType error = z - h;
	//	std::cout<<"error = "<<error<<std::endl;
	
		// 3. Kalman Gain
		DataType K_tmp = H * P_now * H.transpose() + R;
		Matrix3x1 K = P_now * H.transpose() * ( 1 / K_tmp );

		//std::cout<<"K * error = "<<std::endl<<K * error<<std::endl;
		// 4. state update
		x_now += K * error;
		
		// 5. state covarince matrix update
		P_now = ( Matrix3::Identity() - K * H ) * P_now;
#ifdef DEBUG	
		std::cout<<" update end "<<std::endl;
#endif
		// 6. update the old value
		angleNormalize( x_now[2] );

		// 7. judge if the pose is a key frame pose
		if( poseDiffLargerThan( x_now, last_keyframe_pose ) ){
			is_key_frame = true;
#ifdef DEBUG
			std::cout<<"pose larger true"<<std::endl;
#endif
			last_keyframe_pose = x_now;
		}	
		else {
#ifdef DEBUG
			std::cout<<"pose larger false"<<std::endl;
#endif
			is_key_frame = false;
		}
	
                x_pre = x_now;
                P_pre = P_now;
	}

	const bool isKeyFrame() const
	{
		return is_key_frame;
	}

	const Vector3& getStateX() const
	{
		return x_now;
	}

	void setStateGaussianNoise( const Matrix1x3& H_ )
	{
		H = H_;
	}

	void setMeasurementGaussianNoise( const DataType R_ )
	{
		R = R_;
	}

	void updateXPre( const Vector3& x_pre_ )
	{
		x_pre = x_pre_;
		x_now = x_pre;	

		// added
		P_pre = Matrix3::Identity();
		P_now = Matrix3::Identity();
	}

	const Matrix3& getCovarinceMatrixP()
	{
		return P_now;
	}
	
private:
	void angleNormalize( DataType &angle )
        {
                if( angle >= M_PI ) {
                        angle -= 2 * M_PI;
                }

                if( angle <= -M_PI ){
                        angle += 2 * M_PI;
                }
        }

	bool poseDiffLargerThan( const Vector3 &pose_now, const Vector3 &pose_pre ) const
	{
		Vector3 pose_diff = pose_now - pose_pre;
		Vector2 trans_diff( pose_diff[0], pose_diff[1] );

		if( trans_diff.norm() > min_dist_diff ){
			return true;
		}
	
		DataType angle_diff = pose_now[2] - pose_pre[2];
		if( std::abs( angle_diff ) > min_angle_diff ){
			return true;
		}
		
		return false;
	}
	
private:
	// state vector at k-1 moment, ( x, y, theta )
	Vector3 x_pre = Vector3::Zero();
	
	// state vector at k moment, ( x, y, theta )
	Vector3 x_now = Vector3::Zero();

	// state covarince matrix
	Matrix3 P_pre = Matrix3::Identity();
	Matrix3 P_now = Matrix3::Identity();

	// state Jacobian matrix
	Matrix3 F = Matrix3::Identity();

	// measurement update matrix
	Matrix1x3 H = Matrix1x3::Zero();


	// state Gaussian Noise
	Matrix3 Q = Matrix3::Identity() * 1000;

	// measurement Gaussian Noise
	DataType R = 10;
	
	// previous time moment
	DataType time_pre = 0;
	// 
	bool is_init = false;

	// for key frame identification
	const DataType min_dist_diff = 0.03;
	const DataType min_angle_diff = 0.03490658;

	Vector3 last_keyframe_pose = Vector3::Zero();
	bool is_key_frame = false;
};

}

#endif

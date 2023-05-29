#ifndef __PID_H
#define __PID_H

#include <iostream>

namespace pt
{

	template<typename T>
	class PID
	{
	public:
		using DataType = T;

		PID() = delete;

		PID(const DataType& dt, const DataType& max, const DataType& min, const DataType& kp, const DataType& ki, const DataType & kd) :
			dt_(dt), max_(max), min_(min), kp_(kp), ki_(ki), kd_(kd)
		{

		}

		~PID()
		{

		}

		const DataType caculate(const DataType& target, const DataType& current)
		{
			// error
			DataType error = target - current;

			// Proportional
			DataType p_out = kp_ * error;

			// Integral
			integral_ += error * dt_;
			DataType i_out = ki_ * integral_;

			// Derivative
			DataType derivative = (error - pre_error_) / dt_;
			DataType d_out = kd_ * derivative;

			// total output 
			DataType out = p_out + i_out + d_out;

			// limit
			if (out > max_) {
				out = max_;
			}
			else if (out < min_) {
				out = min_;
			}

			// update the pre_error
			pre_error_ = error;

			return out;
		}

	private:
		DataType dt_;
		DataType max_;
		DataType min_;
		DataType kp_;
		DataType ki_;
		DataType kd_;
		DataType pre_error_ = 0;
		DataType integral_ = 0;


	};

}


#endif

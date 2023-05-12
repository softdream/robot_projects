#ifndef __TIME_SYNCHRONIZE_H
#define __TIME_SYNCHRONIZE_H

#include <iostream>
#include <chrono>

#include <deque>

namespace time_manage
{

class TimeManage
{
public:
	static long getTimeStamp()
	{
		std::chrono::steady_clock::time_point stamp = std::chrono::steady_clock::now();
		return  std::chrono::time_point_cast<std::chrono::milliseconds>( stamp ).time_since_epoch().count();
	}
};


template<typename T, int QUEUE_SIZE = 10>
class Synchronize
{
public:

	Synchronize()
	{

	}

	~Synchronize()
	{

	}

	void addData( const T& elem )
	{
		if ( que_.size() < QUEUE_SIZE ) {
			que_.push_back( elem );
		}
		else {
			que_.pop_front();
			que_.push_back( elem );
		}
	}

	const T getSynchronizedData( const typename T::value_type stamp )
	{
		using ValueType = typename T::value_type;
		
		ValueType dist = std::numeric_limits<ValueType>::max();

		T ret = T::Zero();
		for ( const auto& it : que_ ) {
			if ( std::abs( stamp - it( 0 ) ) <= dist ) {
				dist = std::abs( stamp - it( 0 ) );
				ret = it;
			}
		}

		return ret;
	}
	
private:
	std::deque<T> que_;
};

}

#endif

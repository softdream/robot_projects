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

template<typename T>
struct TimeManageData_
{
	using TimeStampType = long long;
	using DataType = T;

	TimeManageData_()
	{
	
	}

	TimeManageData_( const long long stamp_, const T& data_ ) : stamp( stamp_ ), data( data_ )
	{
	
	}


	long long stamp = 0;

	T data = T::Zero();
};

template<typename T>
using TimeManageData = TimeManageData_<T>;

template<typename T, int BUFFER_SIZE = 10>
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
		if ( que_.size() < BUFFER_SIZE ) {
			que_.push_back( elem );
		}
		else {
			que_.pop_front();
			que_.push_back( elem );
		}
	}

	void addData( const typename T::TimeStampType stamp, const typename T::DataType& data ) 
	{
		return addData( T( stamp, data ) );
	}

	const typename T::DataType getSynchronizedData( const typename T::TimeStampType stamp )
	{
		using ValueType = typename T::TimeStampType;
		
		ValueType dist = std::numeric_limits<ValueType>::max();

		typename T::DataType ret;
		for ( const auto& it : que_ ) {
			if ( std::abs( stamp - it.stamp ) <= dist ) {
				dist = std::abs( stamp - it.stamp );
				ret = it.data;
			}
		}

		return ret;
	}
	
private:
	std::deque<T> que_;
};

}

#endif

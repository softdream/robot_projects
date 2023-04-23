#ifndef __DATA_CONTAINER_H_
#define __DATA_CONTAINER_H_

#include <vector>
#include <cmath>
#include <memory>

#include <Eigen/Dense>

namespace sensor
{

template<typename T, int Rows, int Cols,
        template<typename U, int R, int C, int Option, int MaxR, int MaxC>
        class EigenType>
struct is_Eigen_type
{
        static const bool value = false;
};

template<typename T, int Rows, int Cols>
struct is_Eigen_type<T, Rows, Cols, Eigen::Matrix>
{
        using type = Eigen::Matrix<T, Rows, Cols>;
        static const bool value = true;
};

template<typename DataType,
                typename = typename std::enable_if<is_Eigen_type<typename DataType::value_type, DataType::RowsAtCompileTime, DataType::ColsAtCompileTime, Eigen::Matrix>::value>::type>
class DataContainer
{
public:
	using type = DataType;
	using value_type = typename DataType::value_type;

	using ConstSharedPtr = const std::shared_ptr<DataType>;
	using SharedPtr = std::shared_ptr<DataType>;

	DataContainer() {  }
	~DataContainer() {  }

	void addData( const DataType &data )
	{
		return data_vec_.push_back( data );	
	}

	void clear()
	{
		return data_vec_.clear();
	}
	
	const DataType& getIndexData( const int index ) const
	{
		return data_vec_[index];
	}

	
	const int getSize() const	
	{
		return data_vec_.size();
	}


	bool isEmpty() const
	{
		return data_vec_.empty();
	}

	const DataType& operator[] (const int i)
	{
		return data_vec_[i];
	}

private:
	std::vector<DataType> data_vec_;
	
};

using ScanContainer = DataContainer<Eigen::Vector2f>;

}

#endif

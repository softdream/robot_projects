#ifndef __OBSTACLES_H
#define __OBSTACLES_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>

namespace apf
{

template<typename T>
class Obstacles
{
public:
	using value_type = T;
	using PoseType = typename Eigen::Matrix<T, 2, 1>;

	Obstacles()
	{

	}

	~Obstacles()
	{

	}

	void addObstacle( const PoseType& pose )
	{
		return obstacles.push_back( pose );
	}

	void clearAll()
	{
		return obstacles.clear();
	}

	const std::vector<PoseType>& getObstacles() const 
	{
		return obstacles;
	}

	void updateObstacles( const std::vector<PoseType>& obs )
	{
		obstacles = obs;
	}
	
	const int getSize() const
	{
		return obstacles.size();
	}

	const PoseType& operator[]( const int i ) const
	{
		return obstacles[i];
	}
	
	const bool isEmpty() const
	{
		return obstacles.empty();
	}
	
private:
	std::vector<PoseType> obstacles;
};

}

#endif

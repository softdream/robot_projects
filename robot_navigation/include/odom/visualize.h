#ifndef __VISUALIZE_H
#define __VISUALIZE_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace visualize
{

class Visualization
{
public:
	Visualization()
	{
		mapInit();
	}

	Visualization( const int width, const int height ) : map_width_( width ), map_height_( height )
	{
		mapInit();
	}

	~Visualization()
	{

	}

	template<typename T>
	void setOdometryPose( const Eigen::Matrix<T, 3, 1>& pose )
	{
		cv::circle(map_, cv::Point2f(map_width_ / 2 - static_cast<float>(pose[0]) * scale_, map_height_ / 2 - static_cast<float>(pose[1]) * scale_), 3, cv::Scalar(0, 0, 255), -1);
	}

	void onMapDisplay()
	{
		cv::imshow( "map", map_ );
		cv::waitKey(5);
	}
	
	void resetMap()
	{
		map_ = cv::Mat::zeros( map_width_, map_height_, CV_8UC3 );
	}

private:
	void mapInit()
	{
		map_ = cv::Mat::zeros( map_width_, map_height_, CV_8UC3 );
	}

private:
	cv::Mat map_;
	int map_width_ = 800;
	int map_height_ = 800;
	const float scale_ = 400;
};
	
}

#endif

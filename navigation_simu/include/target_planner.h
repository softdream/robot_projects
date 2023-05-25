#ifndef __TARGET_PLANNER_H
#define __TARGET_PLANNER_H

#include "utils.h"
#include <random>
#include <vector>

class TargetPlanner
{
public:
	static void randomPointGenerate( int& img_pose_x, int& img_pose_y )
	{
        	std::random_device rd;
	        std::mt19937 gen;

        	gen.seed( rd() );

	        std::uniform_int_distribution<int> u_x( 0, 500 );
        	std::uniform_int_distribution<int> u_y( 0, 500 );

	        img_pose_x = u_x( gen );
        	img_pose_y = u_y( gen );
	}

	static auto generatePlannedTargetGoal( const cv::Mat& map, 
                		               bool& plan_complete_flag )
	{
        	int img_pose_x = -1;
	        int img_pose_y = -1;


	        bool generate_flag = false;

        	int iter_cnt = 0;

	        while ( !generate_flag ) {
        	        if ( iter_cnt > 100 ) {
                        	plan_complete_flag = true;

	                        break;
        	        }
			randomPointGenerate( img_pose_x, img_pose_y );

                	if ( map.at<uchar>( img_pose_x, img_pose_y ) != 255 ) { // if the goal is not in the freespace, regenerate
                        	continue;
                	}
	                else {
                	        std::cout<<"found one =========================== ( "<<img_pose_x<<", "<<img_pose_y<<" )"<<std::endl;
				generate_flag = true;
                	}

        	}

	        return Eigen::Vector2i( img_pose_x, img_pose_y );
	}
				
};

#endif

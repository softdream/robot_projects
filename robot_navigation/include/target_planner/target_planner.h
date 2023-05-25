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

	template<typename T>
	static auto generatePlannedTargetGoal( const cv::Mat& map, 
					       const apf::Obstacles<T>& obstacles,
        		                       std::vector<Eigen::Matrix<T, 2, 1>>& visited_robot_pose,
                		               bool& plan_complete_flag )
	{
        	int img_pose_x = -1;
	        int img_pose_y = -1;

        	Eigen::Matrix<T, 2, 1> target_goal_world_pose = Eigen::Matrix<T, 2, 1>::Zero();

	        bool generate_flag = false;

        	int iter_cnt = 0;

	        while ( !generate_flag ) {
        	        if ( iter_cnt > 100 ) {
                	        visited_robot_pose.clear();
                        	plan_complete_flag = true;

	                        break;
        	        }
			randomPointGenerate( img_pose_x, img_pose_y );
	                //std::cout<<"img_pose : ( "<<img_pose_x<<", "<<img_pose_y<<" )"<<std::endl;

        	        //std::cout<<"map.at<uchar>( "<<img_pose_x<<", "<<img_pose_y<<" ) = "<<int( map.at<uchar>( img_pose_x, img_pose_y ) )<<std::endl;
                	if ( map.at<uchar>( img_pose_x, img_pose_y ) != 255 ) { // if the goal is not in the freespace, regenerate
                        	continue;
                	}
	                else {
        	                target_goal_world_pose = Utils::coordinateTransformMap2World( Eigen::Vector2i( img_pose_x, img_pose_y ), Eigen::Vector2i( 250, 250 ), 0.1f );
                	        std::cout<<"found one =========================== "<<target_goal_world_pose.transpose()<<std::endl;

                        	bool is_visited_flag = false;
	                        for ( const auto& robot_pose : visited_robot_pose ) {
        	                        if ( ( target_goal_world_pose - robot_pose ).norm() < 0.4 ) {
                	                        is_visited_flag = true;
                        	                break;
                                	}
	                        }

        	                if ( is_visited_flag ) {
                	                iter_cnt ++;
                        	        continue;
                        	}

				bool has_obs_in_ranges_flag = false;
        	                for ( int i = 0; i < obstacles.getSize(); i ++ ) {
                	                auto obs_pose = obstacles[i];
                        	        //std::cout<<"distance = "<<( target_goal_world_pose - obs_pose ).norm()<<std::endl;
                                	// ensure that there is no obstacle in the range of the target goal
	                                if ( ( target_goal_world_pose - obs_pose ).norm() < 0.4 ) {
        	                                has_obs_in_ranges_flag = true;
                	                        break;
                        	        }
	                        }	

        	                // if there is a obstacle in the range of the target goal, regenerate
                	        if ( has_obs_in_ranges_flag ) {
                        	        //iter_cnt ++;
                                	continue;
	                        }
        	                else {
                	                generate_flag = true; // if not, end the loop
                        	        std::cout<<"the target goal = ( "<<img_pose_x<<", "<<img_pose_y<<" )"<<std::endl;
                        	}
                	}

	                // 
        	        std::cout<<"iter_cnt ============================ "<<iter_cnt<<std::endl;
        	}

	        return target_goal_world_pose;
	}
	
	template<typename T>
        static auto generatePlannedTargetGoal( const cv::Mat& map,
                                               std::vector<Eigen::Matrix<T, 2, 1>>& visited_robot_pose,
                                               bool& plan_complete_flag )
        {
                int img_pose_x = -1;
                int img_pose_y = -1;

                Eigen::Matrix<T, 2, 1> target_goal_world_pose = Eigen::Matrix<T, 2, 1>::Zero();

                bool generate_flag = false;

                int iter_cnt = 0;

                while ( !generate_flag ) {
                        if ( iter_cnt > 20 ) {
                                visited_robot_pose.clear();
                                plan_complete_flag = true;

                                break;
                        }
                        randomPointGenerate( img_pose_x, img_pose_y );

			if ( map.at<uchar>( img_pose_x, img_pose_y ) != 255 ) { // if the goal is not in the freespace, regenerate
                                continue;
                        }
                        else {
                                target_goal_world_pose = Utils::coordinateTransformMap2World( Eigen::Vector2i( img_pose_x, img_pose_y ), Eigen::Vector2i( 250, 250 ), 0.1f );
                                std::cout<<"found one =========================== "<<target_goal_world_pose.transpose()<<std::endl;

                                bool is_visited_flag = false;
                                for ( const auto& robot_pose : visited_robot_pose ) {
                                        if ( ( target_goal_world_pose - robot_pose ).norm() < 0.4 ) {
                                                is_visited_flag = true;
                                                break;
                                        }
                                }

                                if ( is_visited_flag ) {
                                        iter_cnt ++;
                                        continue;
                                }
                                else {
                                        generate_flag = true; // if not, end the loop
                                        std::cout<<"the target goal = ( "<<img_pose_x<<", "<<img_pose_y<<" )"<<std::endl;
                                }
                        }

                }

                return Eigen::Vector2i( img_pose_x, img_pose_y );
        }

		
};

#endif

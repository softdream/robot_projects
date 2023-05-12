#include "cluster.h"
#include "utils.h"
#include "laser_simulation.h"

int main()
{
	std::cout<<"----------------------- CLUSTER TEST ---------------------"<<std::endl;

	simulation::Simulation simu;
	simu.openSimulationFile( "/home/riki/Test/robot_projects/robot_projects/2d_cluster_test/test_data/laser_test_data2.txt" );

	cluster::Cluster<float> cluster;

	cv::Mat image = cv::Mat::zeros( 800, 800, CV_8UC3 );

	while( !simu.endOfFile() ) {  
		sensor::LaserScan scan;
                sensor::ScanContainer scan_container;

		simu.readAFrameData( scan );
	
		Utils::laserData2Container( scan, scan_container );
	
		Utils::displayScan( scan_container );

		std::vector<std::vector<typename sensor::ScanContainer::type>> clusters;
                int num = cluster.extractEuclideanClusters( scan_container, clusters );
                std::cout<<"cluster number = "<< num<<std::endl;

		cv::waitKey(0);

		for( int i = 0; i < clusters.size(); i ++ ) { 
			for( auto& pt : clusters[i] ) {
				float img_x = 400 + pt[0] * 100;
				float img_y = 400 + pt[1] * 100;
			
				if( i == 1 )
                                        cv::circle(image, cv::Point2f(img_x, img_y), 2, cv::Scalar(0, 0, 255), -1);
                                else if( i == 2 )
                                        cv::circle(image, cv::Point2f(img_x, img_y), 2, cv::Scalar(0, 255, 0), -1);
                                else if( i == 3 )
                                        cv::circle(image, cv::Point2f(img_x, img_y), 2, cv::Scalar(255, 0, 0), -1);
				else if( i == 4 )
                                        cv::circle(image, cv::Point2f(img_x, img_y), 2, cv::Scalar(255, 255, 0), -1);
				else if( i == 5 )
                                        cv::circle(image, cv::Point2f(img_x, img_y), 2, cv::Scalar(255, 0, 255), -1);
				else if( i == 6 )
                                        cv::circle(image, cv::Point2f(img_x, img_y), 2, cv::Scalar(125, 0, 0), -1);
				else if( i == 7 )
                                        cv::circle(image, cv::Point2f(img_x, img_y), 2, cv::Scalar(0, 125, 0), -1);
				else if( i == 8 )
                                        cv::circle(image, cv::Point2f(img_x, img_y), 2, cv::Scalar(0, 0, 125), -1);
                                else
                                        cv::circle(image, cv::Point2f(img_x, img_y), 2, cv::Scalar(0, 255, 255), -1);

			}
		}

		imshow("clusters", image);
                cv::waitKey(0);
	
		image = cv::Mat::zeros( 800, 800, CV_8UC3 );
	}

	simu.closeSimulationFile();

	return 0;
}

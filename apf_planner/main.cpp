#include "apf_simulator.h"


int main()
{
	std::cout<<"--------------- APF PLANNER ---------------"<<std::endl;

	apf::APFSimulator<float> apf;
	apf.initMap();

	apf.setTargetPose( apf::APF<float>::Vector2( 2.3, -0.5 ) );	
	
	apf::APF<float>::Vector2 robot_pose( 0, 0 );
	for( int i = 0; i < 100; i ++ ){
		apf.runApfOnce( robot_pose );
	}

	return 0;
}

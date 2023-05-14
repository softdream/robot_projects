#include "simulation.h"


int main()
{
	std::cout<<"--------------- GLOABAL PLANNER ---------------"<<std::endl;

	simulation::PlannerSimulation<float> simulator;
	simulator.initMap();

	return 0;
}

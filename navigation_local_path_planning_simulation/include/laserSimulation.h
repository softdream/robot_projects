#ifndef __LASER_SIMULATION_H_
#define __LASER_SIMULATION_H_

#include <iostream>
#include <vector>
#include <fstream>
#include <string.h>
#include <sstream>
#include <iomanip>

#include <stdlib.h>
#include <stdint.h>

#include "data_type.h"

namespace simulation{


class Simulation
{
public:
	Simulation();
	~Simulation();
	
	bool openSimulationFile( const std::string &inputFile );
	void closeSimulationFile();

	bool readAFrameData( sensor::LaserScan &scan );
	
	bool readLaserInfo( sensor::LaserScan &scan );

	inline const int filePointPose()
	{
		return input_file.tellg();
	}

	inline const int endOfFile()
	{
		return input_file.eof();
	}
	
	inline const long getFrameCount() const
	{
		return count;
	}
	
private:
	std::ifstream input_file;
	long count;
};


}

#endif

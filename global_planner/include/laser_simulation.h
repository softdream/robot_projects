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
	Simulation()
	{

	}
	
	~Simulation()
	{

	}
	
	bool openSimulationFile( const std::string &inputFile )
	{
		input_file_.open( inputFile.c_str(), std::ifstream::in );

	        if( !input_file_.is_open() ){
        	        std::cout<<"Failed to open the simulation file ..."<<std::endl;
                	return false;
        	}

	        std::cout<<"............Open the Simulation File ............."<<std::endl;

	}

	void closeSimulationFile()
	{
		return input_file_.close();
	}

	bool readAFrameData( sensor::LaserScan& scan )
	{
		memset( scan.ranges, 0, scan.size() );

		std::string line;
		std::getline(input_file_, line);

		{
                //std::cout << line << std::endl;
                std::istringstream iss(line);
                std::string tag;
                iss >> tag;
                std::string num;

                if (tag.compare("laser") == 0) {

                        for (int i = 0; i < scan.size(); i++) {
                                iss >> num;
                                //std::cout << num << "\t";
                                //iss >> scan[count].range[i];
                                if (!num.compare("inf")) {
                                        scan.ranges[i] = 65536.0f;
                                }
                                else{
                                        scan.ranges[i] = std::stof( num );
                                }
                        }
                }
	        }
	}
	

	inline const int filePointPose()
	{
		return input_file_.tellg();
	}

	inline const int endOfFile()
	{
		return input_file_.eof();
	}
	
	inline const long getFrameCount() const
	{
		return count_;
	}
	
private:
	std::ifstream input_file_;
	long count_;
};


}

#endif

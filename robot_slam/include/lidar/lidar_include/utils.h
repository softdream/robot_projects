
#pragma once

#ifdef WIN32
#ifdef cspclidar_IMPORTS
#define C_CSPCLIDAR_API __declspec(dllimport)
#else
#ifdef cspclidarStatic_IMPORTS
#define C_CSPCLIDAR_API
#else

#define C_CSPCLIDAR_API __declspec(dllexport)
#endif 
#endif

#else
#define C_CSPCLIDAR_API
#endif // ifdef WIN32

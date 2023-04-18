#include "C_CSPC_Lidar.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <cspclidar_protocol.h>

using namespace std;
using namespace cspclidar;

#if defined(_MSC_VER)
#pragma comment(lib, "cspclidar_driver.lib")
#endif

int main(int argc, char *argv[]) {
  printf("CSPC LIDAR START\n");
  fflush(stdout);
  std::string port = "/dev/sc_mini";
  cspclidar::init(argc, argv);

  /*
  std::map<std::string, std::string> ports =
    ydlidar::YDlidarDriver::lidarPortList();
  std::map<std::string, std::string>::iterator it;

  if (ports.size() == 1) {
    port = ports.begin()->second;
  } else {
    int id = 0;

    for (it = ports.begin(); it != ports.end(); it++) {
      printf("%d. %s\n", id, it->first.c_str());
      id++;
    }

    if (ports.empty()) {
      printf("Not Lidar was detected. Please enter the lidar serial port:");
      std::cin >> port;
    } else {
      while (ydlidar::ok()) {
        printf("Please select the lidar port:");
        std::string number;
        std::cin >> number;

        if ((size_t)atoi(number.c_str()) >= ports.size()) {
          continue;
        }

        it = ports.begin();
        id = atoi(number.c_str());

        while (id) {
          id--;
          it++;
        }

        port = it->second;
        break;
      }
    }
  }*/

  int baudrate = 115200;
  /*
  std::map<int, int> baudrateList;
  baudrateList[0] = 115200;
  baudrateList[1] = 128000;
  baudrateList[2] = 153600;
  baudrateList[3] = 230400;
  baudrateList[4] = 512000;

  printf("Baudrate:\n");

  for (std::map<int, int>::iterator it = baudrateList.begin();
       it != baudrateList.end(); it++) {
    printf("%d. %d\n", it->first, it->second);
  }

  while (ydlidar::ok()) {
    printf("Please select the lidar baudrate:");
    std::string number;
    std::cin >> number;

    if ((size_t)atoi(number.c_str()) > baudrateList.size()) {
      continue;
    }

    baudrate = baudrateList[atoi(number.c_str())];
    break;
  }

  if (!ydlidar::ok()) {
    return 0;
  }
*/
  bool isSingleChannel = true;
  bool isTOFLidar = false;
  std::string input_channel;
  std::string input_tof;
  /*
  printf("Whether the Lidar is one-way communication[yes/no]:");
  std::cin >> input_channel;
  std::transform(input_channel.begin(), input_channel.end(),
                 input_channel.begin(),
  [](unsigned char c) {
    return std::tolower(c);  // correct
  });

  if (input_channel.find("yes") != std::string::npos) {
    isSingleChannel = true;
  }

  if (!ydlidar::ok()) {
    return 0;
  }
  */
 /*
  printf("Whether the Lidar is a TOF Lidar [yes/no]:");
  std::cin >> input_tof;
  std::transform(input_tof.begin(), input_tof.end(),
                 input_tof.begin(),
  [](unsigned char c) {
    return std::tolower(c);  // correct
  });

  if (input_tof.find("yes") != std::string::npos) {
    isTOFLidar = true;
  }

  if (!ydlidar::ok()) {
    return 0;
  }
  */
  std::string input_frequency;

  float frequency = 10.0;
  /*
  while (ydlidar::ok() && !isSingleChannel) {
    printf("Please enter the lidar scan frequency[3-15.7]:");
    std::cin >> input_frequency;
    frequency = atof(input_frequency.c_str());

    if (frequency <= 15.7 && frequency >= 3.0) {
      break;
    }

    fprintf(stderr,
            "Invalid scan frequency,The scanning frequency range is 5 to 12 HZ, Please re-enter.\n");
  }

  if (!ydlidar::ok()) {
    return 0;
  }
  */
  

  C_CSPC_Lidar laser;
  //<! lidar port
  laser.setSerialPort(port);
  //<! lidar baudrate
  laser.setSerialBaudrate(baudrate);

  //<! fixed angle resolution
  laser.setFixedResolution(false);
  //<! rotate 180
  laser.setReversion(false); //rotate 180
  //<! Counterclockwise
  laser.setInverted(false);//ccw
  laser.setAutoReconnect(true);//hot plug
  //<! one-way communication
  laser.setSingleChannel(isSingleChannel);

  //<! tof lidar
  laser.setLidarType(isTOFLidar ? TYPE_TOF : TYPE_TRIANGLE);
  //unit: °
  laser.setMaxAngle(360);
  laser.setMinAngle(0);

  //unit: m
  laser.setMinRange(0.01);
  laser.setMaxRange(10.0);

  //unit: Hz
  laser.setScanFrequency(frequency);
  std::vector<float> ignore_array;
  ignore_array.clear();
  laser.setIgnoreArray(ignore_array);

  bool ret = laser.initialize();

  if (ret) {
    ret = laser.turnOn();
  }

  while (ret && cspclidar::ok()) {
    bool hardError;
    LaserScan scan;
    if (laser.doProcessSimple(scan, hardError)) {
      
      fprintf(stdout, "Scan received[%llu]: %u ranges is [%f]Hz\n",
              scan.stamp,
              (unsigned int)scan.points.size(), 1.0 / scan.config.scan_time);
      //int i=0;
      
      fflush(stdout);
    } else {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }
  }

  laser.turnOff();
  laser.disconnecting();

  return 0;
}

#include "C_CSPC_Lidar.h"
#include "common.h"
#include <map>
#include <angles.h>
#include <numeric>

using namespace std;
using namespace cspclidar;
using namespace impl;
using namespace angles;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
C_CSPC_Lidar::C_CSPC_Lidar(): lidarPtr(nullptr) {
  m_SerialPort        = "/dev/sc_mini";
  m_SerialBaudrate    = 115200;
  m_FixedResolution   = true;
  m_Reversion         = false;
  m_Inverted          = false;//
  m_AutoReconnect     = true;
  m_SingleChannel     = false;
  m_LidarType         = TYPE_TRIANGLE;
  m_MaxAngle          = 360.0;
  m_MinAngle          = 0.0;
  m_MaxRange          = 10.0;
  m_MinRange          = 0.10;
  m_SampleRate        = 4;//3.86
  defalutSampleRate   = 5;
  m_ScanFrequency     = 10;
  m_GlassNoise        = true;
  m_SunNoise          = true;
  isScanning          = false;
  m_FixedSize         = 720;
  frequencyOffset     = 0.4;
  m_AbnormalCheckCount  = 6;
  Major               = 0;
  Minjor              = 0;
  m_IgnoreArray.clear();
  m_PointTime         = 1e9 / 5000;
  m_OffsetTime        = 0.0;
  m_AngleOffset       = 0.0;
  lidar_model = CSPCLIDAR_G2B;
  last_node_time = getTime();
  global_nodes = new node_info[CSPClidarDriver::MAX_SCAN_NODES];
  m_ParseSuccess = false;
}

/*-------------------------------------------------------------
                    ~C_CSPC_Lidar
-------------------------------------------------------------*/
C_CSPC_Lidar::~C_CSPC_Lidar() {
  disconnecting();

  if (global_nodes) {
    delete[] global_nodes;
    global_nodes = NULL;
  }
}

void C_CSPC_Lidar::disconnecting() {
  if (lidarPtr) {
    lidarPtr->disconnect();
    delete lidarPtr;
    lidarPtr = nullptr;
  }

  isScanning = false;
}

//get zero angle offset value
float C_CSPC_Lidar::getAngleOffset() const {
  return m_AngleOffset;
}

bool C_CSPC_Lidar::isAngleOffetCorrected() const {
  return m_isAngleOffsetCorrected;
}

std::string C_CSPC_Lidar::getSoftVersion() const {
  return m_lidarSoftVer;
}

std::string C_CSPC_Lidar::getHardwareVersion() const {
  return m_lidarHardVer;
}

std::string C_CSPC_Lidar::getSerialNumber() const {
  return m_lidarSerialNum;
}

bool C_CSPC_Lidar::isRangeValid(double reading) const {
  if (reading >= m_MinRange && reading <= m_MaxRange) {
    return true;
  }

  return false;
}

bool C_CSPC_Lidar::isRangeIgnore(double angle) const {
  bool ret = false;

  for (uint16_t j = 0; j < m_IgnoreArray.size(); j = j + 2) {
    if ((angles::from_degrees(m_IgnoreArray[j]) <= angle) &&
        (angle <= angles::from_degrees(m_IgnoreArray[j + 1]))) {
      ret = true;
      break;
    }
  }

  return ret;
}


/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/

float angle_degress_format(float degress)
{
	while(degress >= 360.0f)
	{
		degress -= 360.0f;
	}
	while(degress < 0.0f)
	{
		degress += 360.0f;
	}
	return degress;
}

bool  C_CSPC_Lidar::doProcessSimple(LaserScan &outscan,
                                bool &hardwareError) {
  hardwareError			= false;

  // Bound?
  if (!checkHardware()) {
    hardwareError = true;
    delay(200 / m_ScanFrequency);
    return false;
  }

  size_t   count = CSPClidarDriver::MAX_SCAN_NODES;
  //wait Scan data:
  uint64_t tim_scan_start = getTime();
  uint64_t startTs = tim_scan_start;
  result_t op_result =  lidarPtr->grabScanData(global_nodes, count);
  uint64_t tim_scan_end = getTime();
  const float AngCorrect_fk = 19.16;
  const float AngCorrect_fx = 90.15;
  const float AngCorrect_fa = 12;

  // Fill in scan data:
  if (IS_OK(op_result)) {
    uint64_t scan_time = m_PointTime * (count - 1);
    tim_scan_end += m_OffsetTime * 1e9;
    tim_scan_end -= m_PointTime;
    tim_scan_end -= global_nodes[0].stamp;
    tim_scan_start = tim_scan_end -  scan_time ;

    if (tim_scan_start < startTs) {
      tim_scan_start = startTs;
      tim_scan_end = tim_scan_start + scan_time;
    }

    if ((last_node_time + m_PointTime) >= tim_scan_start) {
      tim_scan_start = last_node_time + m_PointTime;
      tim_scan_end = tim_scan_start + scan_time;
    }

    last_node_time = tim_scan_end;

    if (m_MaxAngle < m_MinAngle) {
      float temp = m_MinAngle;
      m_MinAngle = m_MaxAngle;
      m_MaxAngle = temp;
    }

    int all_node_count = count;

    outscan.config.min_angle = m_MinAngle;
    outscan.config.max_angle =  m_MaxAngle;
    outscan.config.scan_time =  static_cast<float>(scan_time * 1.0 / 1e9);
    outscan.config.time_increment = outscan.config.scan_time / (double)(count - 1);
    outscan.config.min_range = m_MinRange;
    outscan.config.max_range = m_MaxRange;
    outscan.stamp = tim_scan_start;
    outscan.points.clear();

    if (m_FixedResolution) {
      all_node_count = m_FixedSize;
    }

    outscan.config.angle_increment = (outscan.config.max_angle -
                                      outscan.config.min_angle) / (all_node_count - 1);

    float range = 0.0;
    float intensity = 0.0;
    float angle = 0.0;
    float angle_struct_c = 0.0;
    float range_temp;

    for (int i = 0; i < count; i++) {
      angle = static_cast<float>((global_nodes[i].angle_q6_checkbit >>
                                  LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f) + m_AngleOffset;

      if (isTOFLidar(m_LidarType)) {
        if (isOldVersionTOFLidar(lidar_model, Major, Minjor)) {
          range = static_cast<float>(global_nodes[i].distance_q2 / 2000.f);
        } else {
          range = static_cast<float>(global_nodes[i].distance_q2 / 1000.f);
        }
      } else {
        if (isOctaveLidar(lidar_model)) {
          range = static_cast<float>(global_nodes[i].distance_q2 / 2000.f);
        } else {
          range = static_cast<float>(global_nodes[i].distance_q2 / 4000.f);
        }
      }
      
      range_temp = global_nodes[i].distance_q2/4.0f;
      if(range_temp <10)
      {
        angle_struct_c = 0;
      }
      else
      {
        angle_struct_c = atan( AngCorrect_fk * (range_temp - AngCorrect_fx) / (AngCorrect_fx*range_temp) ) * 180 / PI;
      }
      
      intensity = static_cast<float>(global_nodes[i].sync_quality);
      //angle = angles::from_degrees(angle);


      //Rotate 180 degrees or not
      if (m_Reversion) {
        angle = angle + M_PI;
      }

      //Is it counter clockwise
      if (m_Inverted) {
        angle = 2 * M_PI - angle;
      }

      //angle = angles::normalize_angle(angle);

      //ignore angle
      if (isRangeIgnore(angle)) {
        range = 0.0;
      }

      //valid range
      
      if (!isRangeValid(range)) {
        range = 0.0;
        intensity = 0.0;
      }

      angle += AngCorrect_fa;
      angle -= angle_struct_c;

      angle = angle_degress_format(angle);

      if (angle >= m_MinAngle &&
          angle <= m_MaxAngle) {
        LaserPoint point;
        point.angle = angle;
        point.range = range;
        point.intensity = intensity;
        //printf("angle=%f,range=%f,angle_struct_c=%f,intensity=%f\n",point.angle,point.range,angle_struct_c,point.intensity);

        if (outscan.points.empty()) {
          outscan.stamp = tim_scan_start + i * m_PointTime;
        }

        outscan.points.push_back(point);
      }

      handleDeviceInfoPackage(count);
    }

    return true;
  } else {
    if (IS_FAIL(op_result)) {
      // Error? Retry connection
    }
  }

  return false;
  
}

void C_CSPC_Lidar::parsePackageNode(const node_info &node, LaserDebug &info) {
  switch (node.index) {
    case 0://W3F4CusMajor_W4F0CusMinor;
      info.W3F4CusMajor_W4F0CusMinor = node.debug_info[node.index];
      break;

    case 1://W4F3Model_W3F0DebugInfTranVer
      info.W4F3Model_W3F0DebugInfTranVer = node.debug_info[node.index];
      break;

    case 2://W3F4HardwareVer_W4F0FirewareMajor
      info.W3F4HardwareVer_W4F0FirewareMajor = node.debug_info[node.index];
      break;

    case 4://W3F4BoradHardVer_W4F0Moth
      info.W3F4BoradHardVer_W4F0Moth = node.debug_info[node.index];
      break;

    case 5://W2F5Output2K4K5K_W5F0Date
      info.W2F5Output2K4K5K_W5F0Date = node.debug_info[node.index];
      break;

    case 6://W1F6GNoise_W1F5SNoise_W1F4MotorCtl_W4F0SnYear
      info.W1F6GNoise_W1F5SNoise_W1F4MotorCtl_W4F0SnYear =
        node.debug_info[node.index];
      break;

    case 7://W7F0SnNumH
      info.W7F0SnNumH = node.debug_info[node.index];
      break;

    case 8://W7F0SnNumL
      info.W7F0SnNumL = node.debug_info[node.index];

      break;

    default:
      break;
  }

  if (node.index > info.MaxDebugIndex && node.index < 100) {
    info.MaxDebugIndex = static_cast<int>(node.index);
  }
}

void C_CSPC_Lidar::handleDeviceInfoPackage(int count) {
  if (m_ParseSuccess) {
    return;
  }

  LaserDebug debug;
  debug.MaxDebugIndex = 0;

  for (int i = 0; i < count; i++) {
    parsePackageNode(global_nodes[i], debug);
  }

  device_info info;

  if (ParseLaserDebugInfo(debug, info)) {
    if (info.firmware_version != 0 ||
        info.hardware_version != 0) {
      std::string serial_number;

      for (int i = 0; i < 16; i++) {
        serial_number += std::to_string(info.serialnum[i] & 0xff);
      }

      Major = (uint8_t)(info.firmware_version >> 8);
      Minjor = (uint8_t)(info.firmware_version & 0xff);
      std::string softVer =  std::to_string(Major & 0xff) + "." + std::to_string(
                               Minjor & 0xff);
      std::string hardVer = std::to_string(info.hardware_version & 0xff);

      m_lidarSerialNum = serial_number;
      m_lidarSoftVer = softVer;
      m_lidarHardVer = hardVer;

      if (!m_ParseSuccess) {
        printfVersionInfo(info);
      }
    }

  }
}


/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  C_CSPC_Lidar::turnOn() {
  if (isScanning && lidarPtr->isscanning()) {
    return true;
  }

  // start scan...
  result_t op_result = lidarPtr->startScan();

  if (!IS_OK(op_result)) {
    op_result = lidarPtr->startScan();

    if (!IS_OK(op_result)) {
      lidarPtr->stop();
      fprintf(stderr, "[C_CSPC_Lidar] Failed to start scan mode: %x\n", op_result);
      isScanning = false;
      return false;
    }
  }

  m_ParseSuccess = false;
  m_PointTime = lidarPtr->getPointTime();

  if (checkLidarAbnormal()) {
    lidarPtr->stop();
    fprintf(stderr,
            "[C_CSPC_Lidar] Failed to turn on the Lidar, because the lidar is blocked or the lidar hardware is faulty.\n");
    isScanning = false;
    return false;
  }

  if (m_SingleChannel && !m_ParseSuccess) {
    handleSingleChannelDevice();
  }

  m_PointTime = lidarPtr->getPointTime();
  isScanning = true;
  lidarPtr->setAutoReconnect(m_AutoReconnect);
  printf("[CSPCLIDAR INFO] Current Sampling Rate : %dK\n", m_SampleRate);
  printf("[CSPCLIDAR INFO] Now CSPCLIDAR is scanning ......\n");
  fflush(stdout);
  return true;
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  C_CSPC_Lidar::turnOff() {
  if (lidarPtr) {
    lidarPtr->stop();
  }

  if (isScanning) {
    printf("[CSPCLIDAR INFO] Now CSPCLIDAR Scanning has stopped ......\n");
  }

  isScanning = false;
  return true;
}

/*-------------------------------------------------------------
            checkLidarAbnormal
-------------------------------------------------------------*/
bool C_CSPC_Lidar::checkLidarAbnormal() {

  size_t   count = CSPClidarDriver::MAX_SCAN_NODES;
  int check_abnormal_count = 0;

  if (m_AbnormalCheckCount < 2) {
    m_AbnormalCheckCount = 2;
  }

  result_t op_result = RESULT_FAIL;
  std::vector<int> data;
  int buffer_count  = 0;

  while (check_abnormal_count < m_AbnormalCheckCount) {
    //Ensure that the voltage is insufficient or the motor resistance is high, causing an abnormality.
    if (check_abnormal_count > 0) {
      delay(check_abnormal_count * 1000);
    }

    float scan_time = 0.0;
    uint32_t start_time = 0;
    uint32_t end_time = 0;
    op_result = RESULT_OK;

    while (buffer_count < 10 && (scan_time < 0.05 ||
                                 !lidarPtr->getSingleChannel()) && IS_OK(op_result)) {
      start_time = getms();
      count = CSPClidarDriver::MAX_SCAN_NODES;
      op_result =  lidarPtr->grabScanData(global_nodes, count);
      end_time = getms();
      scan_time = 1.0 * static_cast<int32_t>(end_time - start_time) / 1e3;
      buffer_count++;

      if (IS_OK(op_result)) {
        handleDeviceInfoPackage(count);

        if (CalculateSampleRate(count, scan_time)) {
          if (!lidarPtr->getSingleChannel()) {
            return !IS_OK(op_result);
          }
        }
      }
    }

    if (IS_OK(op_result) && lidarPtr->getSingleChannel()) {
      data.push_back(count);
      int collection = 0;

      while (collection < 5) {
        count = CSPClidarDriver::MAX_SCAN_NODES;
        start_time = getms();
        op_result =  lidarPtr->grabScanData(global_nodes, count);
        end_time = getms();


        if (IS_OK(op_result)) {
          if (std::abs(static_cast<int>(data.front() - count)) > 10) {
            data.erase(data.begin());
          }

          handleDeviceInfoPackage(count);
          scan_time = 1.0 * static_cast<int32_t>(end_time - start_time) / 1e3;
          data.push_back(count);

          if (CalculateSampleRate(count, scan_time)) {

          }

          if (scan_time > 0.05 && scan_time < 0.5 && lidarPtr->getSingleChannel()) {
            m_SampleRate = static_cast<int>((count / scan_time + 500) / 1000);
            m_PointTime = 1e9 / (m_SampleRate * 1000);
            lidarPtr->setPointTime(m_PointTime);
          }

        }

        collection++;
      }

      if (data.size() > 1) {
        int total = accumulate(data.begin(), data.end(), 0);
        int mean =  total / data.size(); //mean value
        m_FixedSize = (static_cast<int>((mean + 5) / 10)) * 10;
        printf("[CSPCLIDAR]:Fixed Size: %d\n", m_FixedSize);
        printf("[CSPCLIDAR]:Sample Rate: %dK\n", m_SampleRate);
        return false;
      }

    }

    check_abnormal_count++;
  }

  return !IS_OK(op_result);
}


/** Returns true if the device is connected & operative */
bool C_CSPC_Lidar::getDeviceHealth() {
  if (!lidarPtr) {
    return false;
  }

  lidarPtr->stop();
  result_t op_result;
  device_health healthinfo;
  printf("[CSPCLIDAR]:SDK Version: %s\n", CSPClidarDriver::getSDKVersion().c_str());
  op_result = lidarPtr->getHealth(healthinfo);

  if (IS_OK(op_result)) {
    printf("[CSPCLIDAR]:Lidar running correctly ! The health status: %s\n",
           (int)healthinfo.status == 0 ? "good" : "bad");

    if (healthinfo.status == 2) {
      fprintf(stderr,
              "Error, CSPC Lidar internal error detected. Please reboot the device to retry.\n");
      return false;
    } else {
      return true;
    }

  } else {
    fprintf(stderr, "Error, cannot retrieve CSPC Lidar health code: %x\n", op_result);
    return false;
  }

}

bool C_CSPC_Lidar::getDeviceInfo() {
  if (!lidarPtr) {
    return false;
  }

  device_info devinfo;
  result_t op_result = lidarPtr->getDeviceInfo(devinfo);

  if (!IS_OK(op_result)) {
    fprintf(stderr, "get Device Information Error\n");
    return false;
  }

  if (!isSupportLidar(devinfo.model)) {
    printf("[CSPCLIDAR INFO] Current SDK does not support current lidar models[%s]\n",
           lidarModelToString(devinfo.model).c_str());
    return false;
  }

  frequencyOffset     = 0.4;
  std::string model = "G2";
  lidar_model = devinfo.model;
  model = lidarModelToString(devinfo.model);
  bool intensity = hasIntensity(devinfo.model);
  defalutSampleRate = lidarModelDefaultSampleRate(devinfo.model);

  std::string serial_number;
  lidarPtr->setIntensities(intensity);
  printfVersionInfo(devinfo);

  for (int i = 0; i < 16; i++) {
    serial_number += std::to_string(devinfo.serialnum[i] & 0xff);
  }

  if (devinfo.firmware_version != 0 ||
      devinfo.hardware_version != 0) {
    m_lidarSerialNum = serial_number;
    m_lidarSoftVer = std::to_string(Major & 0xff) + "." + std::to_string(
                       Minjor & 0xff);
    m_lidarHardVer = std::to_string(devinfo.hardware_version & 0xff);
  }

  if (hasSampleRate(devinfo.model)) {
    checkSampleRate();
  } else {
    m_SampleRate = defalutSampleRate;
  }

  if (hasScanFrequencyCtrl(devinfo.model)) {
    checkScanFrequency();
  }

  if (hasZeroAngle(devinfo.model)) {
    checkCalibrationAngle(serial_number);
  }

  return true;
}

void C_CSPC_Lidar::handleSingleChannelDevice() {
  if (!lidarPtr || !lidarPtr->getSingleChannel()) {
    return;
  }

  device_info devinfo;
  result_t op_result = lidarPtr->getDeviceInfo(devinfo);

  if (!IS_OK(op_result)) {
    return;
  }

  printfVersionInfo(devinfo);
  return;
}

void C_CSPC_Lidar::printfVersionInfo(const device_info &info) {
  if (info.firmware_version == 0 &&
      info.hardware_version == 0) {
    return;
  }

  m_ParseSuccess = true;
  lidar_model = info.model;
  Major = (uint8_t)(info.firmware_version >> 8);
  Minjor = (uint8_t)(info.firmware_version & 0xff);
  printf("[CSPCLIDAR] Connection established in [%s][%d]:\n"
         "Firmware version: %u.%u\n"
         "Hardware version: %u\n"
         "Model: %s\n"
         "Serial: ",
         m_SerialPort.c_str(),
         m_SerialBaudrate,
         Major,
         Minjor,
         (unsigned int)info.hardware_version,
         lidarModelToString(lidar_model).c_str());

  for (int i = 0; i < 16; i++) {
    printf("%01X", info.serialnum[i] & 0xff);
  }

  printf("\n");
}

void C_CSPC_Lidar::checkSampleRate() {
  sampling_rate _rate;
  _rate.rate = 3;
  int _samp_rate = 9;
  int try_count = 0;
  m_FixedSize = 1440;
  result_t ans = lidarPtr->getSamplingRate(_rate);

  if (IS_OK(ans)) {
    _samp_rate = ConvertUserToLidarSmaple(lidar_model, m_SampleRate, _rate.rate);

    while (_samp_rate != _rate.rate) {
      ans = lidarPtr->setSamplingRate(_rate);
      try_count++;

      if (try_count > 6) {
        break;
      }
    }

    _samp_rate = ConvertLidarToUserSmaple(lidar_model, _rate.rate);
  }

  m_SampleRate = _samp_rate;
  defalutSampleRate = m_SampleRate;
}


bool C_CSPC_Lidar::CalculateSampleRate(int count, double scan_time) {
  if (count < 1) {
    return false;
  }

  if (global_nodes[0].scan_frequence != 0) {
    double scanfrequency  = global_nodes[0].scan_frequence / 10.0;

    if (isTOFLidar(m_LidarType)) {
      if (!isOldVersionTOFLidar(lidar_model, Major, Minjor)) {
        scanfrequency  = global_nodes[0].scan_frequence / 10.0 + 3.0;
      }
    }

    int samplerate = static_cast<int>((count * scanfrequency + 500) / 1000);
    int cnt = 0;

    if (SampleRateMap.find(samplerate) != SampleRateMap.end()) {
      cnt = SampleRateMap[samplerate];
    }

    cnt++;
    SampleRateMap[samplerate] =  cnt;

    if (isValidSampleRate(SampleRateMap) || defalutSampleRate == samplerate) {
      m_SampleRate = samplerate;
      m_PointTime = 1e9 / (m_SampleRate * 1000);
      lidarPtr->setPointTime(m_PointTime);

      if (!m_SingleChannel) {
        m_FixedSize = m_SampleRate * 1000 / (m_ScanFrequency - 0.1);
        printf("[CSPCLIDAR]:Fixed Size: %d\n", m_FixedSize);
        printf("[CSPCLIDAR]:Sample Rate: %dK\n", m_SampleRate);
      }

      return true;
    } else {
      if (SampleRateMap.size() > 1) {
        SampleRateMap.clear();
      }
    }
  } else {
    if (scan_time > 0.04 && scan_time < 0.4) {
      int samplerate = static_cast<int>((count / scan_time + 500) / 1000);

      if (defalutSampleRate == samplerate) {
        m_SampleRate = samplerate;
        m_PointTime = 1e9 / (m_SampleRate * 1000);
        lidarPtr->setPointTime(m_PointTime);
        return true;
      }
    }

  }


  return false;
}
/*-------------------------------------------------------------
                        checkScanFrequency
-------------------------------------------------------------*/
bool C_CSPC_Lidar::checkScanFrequency() {
  float frequency = 7.4f;
  scan_frequency _scan_frequency;
  float hz = 0;
  result_t ans = RESULT_FAIL;

  if (isSupportScanFrequency(lidar_model, m_ScanFrequency)) {
    m_ScanFrequency += frequencyOffset;
    ans = lidarPtr->getScanFrequency(_scan_frequency) ;

    if (IS_OK(ans)) {
      frequency = _scan_frequency.frequency / 100.f;
      hz = m_ScanFrequency - frequency;

      if (hz > 0) {
        while (hz > 0.95) {
          lidarPtr->setScanFrequencyAdd(_scan_frequency);
          hz = hz - 1.0;
        }

        while (hz > 0.09) {
          lidarPtr->setScanFrequencyAddMic(_scan_frequency);
          hz = hz - 0.1;
        }

        frequency = _scan_frequency.frequency / 100.0f;
      } else {
        while (hz < -0.95) {
          lidarPtr->setScanFrequencyDis(_scan_frequency);
          hz = hz + 1.0;
        }

        while (hz < -0.09) {
          lidarPtr->setScanFrequencyDisMic(_scan_frequency);
          hz = hz + 0.1;
        }

        frequency = _scan_frequency.frequency / 100.0f;
      }
    }
  } else {
    m_ScanFrequency += frequencyOffset;
    fprintf(stderr, "current scan frequency[%f] is out of range.",
            m_ScanFrequency - frequencyOffset);
  }

  ans = lidarPtr->getScanFrequency(_scan_frequency);

  if (IS_OK(ans)) {
    frequency = _scan_frequency.frequency / 100.0f;
    m_ScanFrequency = frequency;
  }

  m_ScanFrequency -= frequencyOffset;
  m_FixedSize = m_SampleRate * 1000 / (m_ScanFrequency - 0.1);
  printf("[CSPCLIDAR INFO] Current Scan Frequency: %fHz\n", m_ScanFrequency);
  return true;
}

/*-------------------------------------------------------------
                        checkCalibrationAngle
-------------------------------------------------------------*/
void C_CSPC_Lidar::checkCalibrationAngle(const std::string &serialNumber) {
  m_AngleOffset = 0.0;
  result_t ans = RESULT_FAIL;
  offset_angle angle;
  int retry = 0;
  m_isAngleOffsetCorrected = false;

  while (retry < 2) {
    ans = lidarPtr->getZeroOffsetAngle(angle);

    if (IS_OK(ans)) {
      if (angle.angle > 720 || angle.angle < -720) {
        ans = lidarPtr->getZeroOffsetAngle(angle);

        if (!IS_OK(ans)) {
          continue;
          retry++;
        }
      }

      m_isAngleOffsetCorrected = (angle.angle != 720);
      m_AngleOffset = angle.angle / 4.0;
      printf("[CSPCLIDAR INFO] Successfully obtained the %s offset angle[%f] from the lidar[%s]\n"
             , m_isAngleOffsetCorrected ? "corrected" : "uncorrrected", m_AngleOffset,
             serialNumber.c_str());
      return;
    }

    retry++;
  }

  printf("[CSPCLIDAR INFO] Current %s AngleOffset : %f°\n",
         m_isAngleOffsetCorrected ? "corrected" : "uncorrrected", m_AngleOffset);
}



/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  C_CSPC_Lidar::checkCOMMs() {
  if (!lidarPtr) {
    // create the driver instance
    lidarPtr = new CSPClidarDriver();

    if (!lidarPtr) {
      fprintf(stderr, "Create Driver fail\n");
      return false;
    }
  }

  if (lidarPtr->isconnected()) {
    return true;
  }

  // Is it COMX, X>4? ->  "\\.\COMX"
  if (m_SerialPort.size() >= 3) {
    if (tolower(m_SerialPort[0]) == 'c' && tolower(m_SerialPort[1]) == 'o' &&
        tolower(m_SerialPort[2]) == 'm') {
      // Need to add "\\.\"?
      if (m_SerialPort.size() > 4 || m_SerialPort[3] > '4') {
        m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
      }
    }
  }

  // make connection...
  result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);

  if (!IS_OK(op_result)) {
    fprintf(stderr,
            "[C_CSPC_Lidar] Error, cannot bind to the specified serial port[%s] and baudrate[%d]\n",
            m_SerialPort.c_str(), m_SerialBaudrate);
    return false;
  }

  lidarPtr->setSingleChannel(m_SingleChannel);
  lidarPtr->setLidarType(m_LidarType);

  return true;
}

/*-------------------------------------------------------------
                        checkStatus
-------------------------------------------------------------*/
bool C_CSPC_Lidar::checkStatus() {

  if (!checkCOMMs()) {
    return false;
  }

  bool ret = getDeviceHealth();

  if (!ret) {
    delay(2000);
    ret = getDeviceHealth();

    if (!ret) {
      delay(1000);
    }
  }

  if (!getDeviceInfo()) {
    delay(2000);
    ret = getDeviceInfo();

    if (!ret) {
      return false;
    }
  }

  return true;
}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool C_CSPC_Lidar::checkHardware() {
  if (!lidarPtr) {
    return false;
  }

  if (isScanning && lidarPtr->isscanning()) {
    return true;
  }

  return false;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
bool C_CSPC_Lidar::initialize() {
  if (!checkCOMMs()) {
    fprintf(stderr,
            "[C_CSPC_Lidar::initialize] Error initializing CSPCLIDAR check Comms.\n");
    fflush(stderr);
    return false;
  }

  if (!checkStatus()) {
    fprintf(stderr,
            "[C_CSPC_Lidar::initialize] Error initializing CSPCLIDAR check status.\n");
    fflush(stderr);
    return false;
  }

  return true;
}

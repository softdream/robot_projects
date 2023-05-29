
#pragma once
#include "utils.h"
#include "cspclidar_driver.h"
#include <math.h>

using namespace cspclidar;
/*
struct offset_angle{
	int32_t angle;
}__attribute__((packed));
*/
class C_CSPCLIDAR_API C_CSPC_Lidar {
  PropertyBuilderByName(float, MaxRange,
                        private) ///< 设置和获取激光最大测距范围(m)
  PropertyBuilderByName(float, MinRange,
                        private) ///< 设置和获取激光最小测距范围(m)
  PropertyBuilderByName(float, MaxAngle,
                        private) ///< 设置和获取激光最大角度, 最大值180度(度)
  PropertyBuilderByName(float, MinAngle,
                        private) ///< 设置和获取激光最小角度, 最小值-180度(度)
  PropertyBuilderByName(int, SampleRate,
                        private) ///< 设置和获取激光采样频率
  PropertyBuilderByName(float, ScanFrequency,
                        private) ///< 设置和获取激光扫描频率(范围5HZ~12HZ)(HZ)
  PropertyBuilderByName(bool, FixedResolution,
                        private) ///< 设置和获取激光是否是固定角度分辨率
  PropertyBuilderByName(bool, Reversion,
                        private) ///< 设置和获取是否旋转激光180度
  PropertyBuilderByName(bool, Inverted,
                        private)///< 设置是否反转激光方向(顺时针，　逆时针）
  PropertyBuilderByName(bool, AutoReconnect,
                        private) ///< 设置异常是否开启重新连接
  PropertyBuilderByName(int, SerialBaudrate,
                        private) ///< 设置和获取激光通讯波特率
  PropertyBuilderByName(int, AbnormalCheckCount,
                        private) ///< Maximum number of abnormal checks
  PropertyBuilderByName(std::string, SerialPort,
                        private) ///< 设置和获取激光端口号
  PropertyBuilderByName(std::vector<float>, IgnoreArray,
                        private) ///< 设置和获取激光剔除点
  PropertyBuilderByName(float, OffsetTime,
                        private) ///<
  PropertyBuilderByName(bool, SingleChannel,
                        private) ///< 是否是单通信雷达
  PropertyBuilderByName(int, LidarType,
                        private) ///< 雷达类型

 public:
  C_CSPC_Lidar(); //!< Constructor
  virtual ~C_CSPC_Lidar();  //!< Destructor: turns the laser off.
  /*!
   * @brief initialize
   * @return
   */
  bool initialize();  //!< Attempts to connect and turns the laser on. Raises an exception on error.

  // Return true if laser data acquistion succeeds, If it's not
  bool doProcessSimple(LaserScan &outscan,
                       bool &hardwareError);

  //Turn on the motor enable
  bool  turnOn();  //!< See base class docs

  //Turn off the motor enable and close the scan
  bool  turnOff(); //!< See base class docs

  //Turn off lidar connection
  void disconnecting(); //!< Closes the comms with the laser. Shouldn't have to be directly needed by the user
  CSPClidarDriver *getCSPClidarDriver();
  //get zero angle offset value
  float getAngleOffset() const;

  //Whether the zero offset angle is corrected?
  bool isAngleOffetCorrected() const;

  //! get lidar software version
  std::string getSoftVersion() const;

  //! get lidar hardware version
  std::string getHardwareVersion() const;

  //! get lidar serial number
  std::string getSerialNumber() const;

 protected:
  /*! Returns true if communication has been established with the device. If it's not,
    *  try to create a comms channel.
    * \return false on error.
    */
  bool  checkCOMMs();

  /*! Returns true if health status and device information has been obtained with the device. If it's not,
    * \return false on error.
    */
  bool  checkStatus();

  /*! Returns true if the normal scan runs with the device. If it's not,
    * \return false on error.
    */
  bool checkHardware();

  /*! Returns true if the device is in good health, If it's not*/
  bool getDeviceHealth();

  /*! Returns true if the device information is correct, If it's not*/
  bool getDeviceInfo();

  /*!
   * @brief checkSampleRate
   */
  void checkSampleRate();

  /**
   * @brief CalculateSampleRate
   * @param count
   * @return
   */
  bool CalculateSampleRate(int count, double scan_time);

  /*! Retruns true if the scan frequency is set to user's frequency is successful, If it's not*/
  bool checkScanFrequency();

  /*! returns true if the lidar data is normal, If it's not*/
  bool checkLidarAbnormal();

  /*!
   * @brief checkCalibrationAngle
   * @param serialNumber
   */
  void checkCalibrationAngle(const std::string &serialNumber);

  /*!
    * @brief isRangeValid
    * @param reading
    * @return
    */
  bool isRangeValid(double reading) const;

  /*!
   * @brief isRangeIgnore
   * @param angle
   * @return
   */
  bool isRangeIgnore(double angle) const;

  /*!
   * @brief handleSingleChannelDevice
   */
  void handleSingleChannelDevice();

  /**
   * @brief parsePackageNode
   * @param node
   * @param info
   */
  void parsePackageNode(const node_info &node, LaserDebug &info);

  /**
   * @brief handleDeviceInfoPackage
   * @param count
   */
  void handleDeviceInfoPackage(int count);

  /**
   * @brief printfVersionInfo
   * @param info
   */
  void printfVersionInfo(const device_info &info);

 private:
  bool    isScanning;
  int     m_FixedSize ;
  float   m_AngleOffset;
  bool    m_isAngleOffsetCorrected;
  float   frequencyOffset;
  int   lidar_model;
  uint8_t Major;
  uint8_t Minjor;
  CSPClidarDriver *lidarPtr;
  uint64_t m_PointTime;
  uint64_t last_node_time;
  node_info *global_nodes;
  std::map<int, int> SampleRateMap;
  bool m_ParseSuccess;
  bool m_GlassNoise;
  bool m_SunNoise;
  std::string m_lidarSoftVer;
  std::string m_lidarHardVer;
  std::string m_lidarSerialNum;
  int defalutSampleRate;
};	// End of class


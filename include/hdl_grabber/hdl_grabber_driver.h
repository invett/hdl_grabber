 
#include <string>
#include <iostream>
#include <iomanip>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/io/grabber.h>
#include <pcl/io/impl/synchronized_queue.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>


#ifdef ISROS
	#include "hdl_grabber/quaternion_fnc.h"
#else
    #include "quaternion_fnc.h"
#endif

#define HDL_GrabberDriver_toRadians(x) ((x) * M_PI / 180.0)

#define HDL_VPL32C_FACTOR 2 

class HDLGrabberDriver: public pcl::Grabber
{
public:

      static const int HDL_DATA_PORT = 2368;
      static const int HDL_GPSDATA_PORT = 8308;
      static const int HDL_NUM_ROT_ANGLES = 36001;
      static const int HDL_LASER_PER_FIRING = 32;
      static const int HDL_MAX_NUM_LASERS = 64;
      static const int HDL_FIRING_PER_PKT = 12;
      static const int VLP_MAX_NUM_LASERS = 32;
      static const std::uint8_t VLP_DUAL_MODE = 0x39;
     // static const float CHANNEL_TDURATION = 2.304;
    
    //  static const float SEQ_TDURATION = 55.296;

      static const boost::asio::ip::address HDL_DEFAULT_NETWORK_ADDRESS;
//protected:
      enum HDLBlock
      {
        BLOCK_0_TO_31 = 0xeeff, BLOCK_32_TO_63 = 0xddff
      };

// Carlota, upgrade to VLP32C
     enum VELODYNE_MODEL
    {
      HDL_32E = 0,
      VLP_32C,
      HDL_64E,
      VLP_16  
    };


#pragma pack(push, 1)
      typedef struct HDLLaserReturn
      {
          unsigned short distance;
          unsigned char intensity;
      } HDLLaserReturn;
#pragma pack(pop)

      struct HDLFiringData
      {
          unsigned short blockIdentifier;
          unsigned short rotationalPosition;
          HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];
      };

      struct HDLDataPacket
      {
          HDLFiringData firingData[HDL_FIRING_PER_PKT];
          unsigned int gpsTimestamp;
          unsigned char blank1;
          unsigned char blank2;
      };
/*
      struct HDLIMUPacket
      {
	      unsigned char  blank1[14];
          unsigned short Giro1;
          unsigned short Temp1;
          unsigned short Acel1X;
          unsigned short Acel1Y;
          unsigned short Giro2;
          unsigned short Temp2;
          unsigned short Acel2X;
          unsigned short Acel2Y;
          unsigned short Giro3;
          unsigned short Temp3;
          unsigned short Acel3X;
          unsigned short Acel3Y;
	      unsigned char blank2[160];
          unsigned int gpsTimestamp;
          unsigned char blank3[4];
	      unsigned char NMEA[72];
	      unsigned char blank4[234];
      }__attribute__ ((packed));

      struct HDLIMUData
      {
          double Giro1;
          double Temp1;
          double Acel1X;
          double Acel1Y;
          double Giro2;
          double Temp2;
          double Acel2X;
          double Acel2Y;
          double Giro3;
          double Temp3;
          double Acel3X;
          double Acel3Y;
          unsigned int gpsTimestamp;
	  unsigned char NMEA[72];
      };
*/
      struct GyroTempAccelBlockRaw
      {
        uint16_t gyro;
        uint16_t temp;
        uint16_t accel_x;
        uint16_t accel_y;
      };
      struct GyroTempAccelBlock
      {
        float gyro;
        float temp;
        float accel_x;
        float accel_y;
      };
      struct HDLIMUPacket
      {
	      unsigned char  blank1[14];
          GyroTempAccelBlockRaw gyro_temp_accel[3];
	      unsigned char blank2[160];
          unsigned int gpsTimestamp;
          unsigned char blank3[4];
	      unsigned char NMEA[72];
	      unsigned char blank4[234];
      }__attribute__ ((packed));

      struct HDLIMUData
      {
        GyroTempAccelBlock gyro_temp_accel[3];
        GyroTempAccelBlock gyro_temp_accel_xyz[3];
        unsigned int gpsTimestamp;
	    unsigned char NMEA[72];
      };
     struct HDLLaserCorrection
      {
          double azimuthCorrection;
          double verticalCorrection;
          double distanceCorrection;
          double verticalOffsetCorrection;
          double horizontalOffsetCorrection;
          double sinVertCorrection;
          double cosVertCorrection;
          double sinVertOffsetCorrection;
          double cosVertOffsetCorrection;
      };
    struct HDLScanInterpolation //lineal interpolation
    {
        unsigned int sweepcounter;
        float dt_scan;
        float ppx[2];
        float ppy[2];
        float ppz[2];
        float ppyaw[2];
        float pppitch[2];
        float pproll[2];
    };
    struct HDLFiringCorrection //lineal interpolation
    {
        unsigned int sweepcounter;
        double x;
        double y;
        double z;
        double yaw;
        double pitch;
        double roll;
     };

    public:
      /** \brief Signal used for a single sector
       *         Represents 1 corrected packet from the HDL Velodyne
       */
      typedef void (sig_cb_velodyne_hdl_scan_point_cloud_xyz) (
          const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&,
          float, float);
      /** \brief Signal used for a single sector
       *         Represents 1 corrected packet from the HDL Velodyne.  Each laser has a different RGB
       */
      typedef void (sig_cb_velodyne_hdl_scan_point_cloud_xyzrgb) (
          const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&,
          float, float);
      /** \brief Signal used for a single sector
       *         Represents 1 corrected packet from the HDL Velodyne with the returned intensity.
       */
      typedef void (sig_cb_velodyne_hdl_scan_point_cloud_xyzi) (
          const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&,
          float startAngle, float);
      /** \brief Signal used for a 360 degree sweep
       *         Represents multiple corrected packets from the HDL Velodyne
       *         This signal is sent when the Velodyne passes angle "0"
       */
      typedef void (sig_cb_velodyne_hdl_sweep_point_cloud_xyz) (
          const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
      /** \brief Signal used for a 360 degree sweep
       *         Represents multiple corrected packets from the HDL Velodyne with the returned intensity
       *         This signal is sent when the Velodyne passes angle "0"
       */
      typedef void (sig_cb_velodyne_hdl_sweep_point_cloud_xyzi) (
          const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&);
      /** \brief Signal used for a 360 degree sweep
       *         Represents multiple corrected packets from the HDL Velodyne
       *         This signal is sent when the Velodyne passes angle "0".  Each laser has a different RGB
       */
      typedef void (sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgb) (
          const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&);

      /** \brief Signal used for a GPS/IMU packet decoded
       *         Represents 1 GYRO processed packet from the HDL Velodyne
       *         This signal is sent when the Velodyne GPS sends a packet
       */
      typedef void (sig_cb_velodyne_gps_packet_decoded) (
          const boost::shared_ptr<const HDLGrabberDriver::HDLIMUData> &);
      // Carlota
      /** \brief Signal used for a 360 degree sweep
         *         Represents multiple corrected packets from the HDL Velodyne
         *         This signal is sent when the Velodyne passes angle "0"
         */
        typedef void (sig_cb_velodyne_hdl_snap_sweep_point_cloud_xyz) (
            const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&,
	          float, float,unsigned int);
        /** \brief Signal used for a 360 degree sweep
         *         Represents multiple corrected packets from the HDL Velodyne with the returned intensity
         *         This signal is sent when the Velodyne passes angle "0"
         */
        typedef void (sig_cb_velodyne_hdl_snap_sweep_point_cloud_xyzi) (
            const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&,
	          float, float,unsigned int);
        /** \brief Signal used for a 360 degree sweep
         *         Represents multiple corrected packets from the HDL Velodyne
         *         This signal is sent when the Velodyne passes angle "0".  Each laser has a different RGB
         */
        typedef void (sig_cb_velodyne_hdl_snap_sweep_point_cloud_xyzrgb) (
            const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&,
	          float, float,unsigned int);


	public:
    std::string calibrationFile, pcapFile,sweepCorrectionFile_;

    HDLGrabberDriver (const std::string& correctionsFile = "",
           const std::string& pcapFile = "", const bool& use_gps=true, const bool& use_laser=true,
		       const std::string& sweepCorrectionFile= "" ,HDLGrabberDriver::VELODYNE_MODEL vmodel=HDL_32E);

       /** \brief Constructor taking a pecified IP/port and an optional path to an HDL corrections file.
        * \param[in] ipAddress IP Address that should be used to listen for HDL packets
        * \param[in] port UDP Port that should be used to listen for HDL packets
        * \param[in] correctionsFile Path to a file which contains the correction parameters for the HDL.  This field is mandatory for the HDL-64, optional for the HDL-32
        */
   HDLGrabberDriver (const boost::asio::ip::address& ipAddress,
    	   const unsigned short port, const unsigned short port_gps, const std::string& correctionsFile = "", const bool& use_gps=true, const bool& use_laser=true,
         HDLGrabberDriver::VELODYNE_MODEL vmodel=HDL_32E);
   virtual ~HDLGrabberDriver (void) throw();

   /** \brief Starts processing the Velodyne packets, either from the network or PCAP file. */
	 virtual void start ();

	 /** \brief Stops processing the Velodyne packets, either from the network or PCAP file */
	 virtual void stop ();

	 /** \brief Obtains the name of this I/O Grabber
	  *  \return The name of the grabber
	  */
	 virtual std::string getName () const;

	 /** \brief Check if the grabber is still running.
	  *  \return TRUE if the grabber is running, FALSE otherwise
	  */
	 virtual bool isRunning () const;

	 /** \brief Returns the number of frames per second.
	  */
	 virtual float getFramesPerSecond () const;

	 // carlota
	 void setPauseEventOn();
	 void setPauseEventOff();
	 void requestPauseOn();
     void setPcapFilter(std::string ip_address,int port);
     void setMaxDistanceThreshold(float max_thresh){max_distance_threshold_=max_thresh;}
     void setMinDistanceThreshold(float min_thresh){min_distance_threshold_=min_thresh;}
     void setTransformationParameters(double x,double y, double z, double roll, double pitch, double yaw);
     //VLP-16
     protected:

     //static const std::uint8_t VLP_MAX_NUM_LASERS = 16;
     

	 private:
     static double *cos_lookup_table_;
     static double *sin_lookup_table_;
     pcl::SynchronizedQueue<unsigned char *> hdl_data_;
     pcl::SynchronizedQueue<unsigned char *> gps_data_;
     boost::asio::ip::udp::endpoint udp_listener_endpoint_;
     boost::asio::ip::udp::endpoint udp_listener_gps_endpoint_;
     boost::asio::ip::address source_address_filter_;
     unsigned short source_port_filter_;
     boost::asio::io_service hdl_read_socket_service_;
     boost::asio::ip::udp::socket *hdl_read_socket_;
     boost::asio::io_service hdl_read_gps_socket_service_;
     boost::asio::ip::udp::socket *hdl_read_gps_socket_;
     std::string pcap_file_name_;
     boost::thread *queue_consumer_thread_;
     boost::thread *queue_consumer_IMU_thread_;
     boost::thread *hdl_read_packet_thread_;
     boost::thread *GPS_read_packet_thread_;
     HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];
     bool terminate_read_packet_thread_;
     bool use_gps_;
     bool use_laser_;
     bool read_packet_pcap_;
     boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > current_scan_xyz_,
         current_sweep_xyz_;
     boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > current_scan_xyzi_,
         current_sweep_xyzi_;
     boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > current_scan_xyzrgb_,
         current_sweep_xyzrgb_;
     unsigned int last_azimuth_;
     boost::shared_ptr<HDLIMUData> hdl_imu_data_;
     boost::signals2::signal<sig_cb_velodyne_hdl_sweep_point_cloud_xyz>* sweep_xyz_signal_;
     boost::signals2::signal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgb>* sweep_xyzrgb_signal_;
     boost::signals2::signal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi>* sweep_xyzi_signal_;
     boost::signals2::signal<sig_cb_velodyne_hdl_scan_point_cloud_xyz>* scan_xyz_signal_;
     boost::signals2::signal<sig_cb_velodyne_hdl_scan_point_cloud_xyzrgb>* scan_xyzrgb_signal_;
     boost::signals2::signal<sig_cb_velodyne_hdl_scan_point_cloud_xyzi>* scan_xyzi_signal_;
     boost::signals2::signal<sig_cb_velodyne_gps_packet_decoded>* gps_packet_signal_;
    pcl::RGB laser_rgb_mapping_[HDL_MAX_NUM_LASERS];
    float min_distance_threshold_;
    float max_distance_threshold_;

    void initialize (const std::string& correctionsFile);
    void loadCorrectionsFile (const std::string& correctionsFile);
    void loadHDL32Corrections ();
    void loadVPLC32Corrections ();

    void processVelodynePackets ();
    void processGPSIMUPackets ();

    void readHDLPacketsFromSocket ();
    void readGPSPacketsFromSocket ();

    void readPacketsFromPcap();
    void readPacketsFromPcap(unsigned long long uSecDelay);

    void enqueueHDLPacket (const unsigned char *data,std::size_t bytesReceived);
    unsigned char toPointClouds (HDLDataPacket *dataPacket);
    unsigned char toPointCloudsVPL (HDLDataPacket *dataPacket);

    bool isAddressUnspecified (const boost::asio::ip::address& ip_address);
    void fireCurrentSweep ();
	 void fireCurrentScan (const unsigned short startAngle, const unsigned short endAngle);
	 void computeXYZI (pcl::PointXYZI& pointXYZI, int azimuth, HDLLaserReturn laserReturn, HDLLaserCorrection correction);
	 void fireCurrentGPS ();

	 // carlota
	 bool                             is_pause_acquisition_on_;
	 bool                             is_set_pause_on_;
	 boost::mutex                     hdl_pause_event_mutex_;
	 boost::condition_variable        hdl_pause_event_cond_var_;
     unsigned int firecount;
	 void waitForPauseEventOff();
	 void loadScanInterpolationCorrectionData();
	 bool getDataForScanCorrection(unsigned int sweepcounter, struct HDLScanInterpolation* params);
	 bool getParamsForFiringCorrection(unsigned int scancounter,float azimuth_pos,struct HDLScanInterpolation* params, struct HDLFiringCorrection *firing_params);
	 bool doScanCorrection(pcl::PointXYZI& undistorXYZI, struct HDLFiringCorrection *firing_params);
 
	 boost::signals2::signal<sig_cb_velodyne_hdl_snap_sweep_point_cloud_xyz>* snap_sweep_xyz_signal_;
	 boost::signals2::signal<sig_cb_velodyne_hdl_snap_sweep_point_cloud_xyzrgb>* snap_sweep_xyzrgb_signal_;
	 boost::signals2::signal<sig_cb_velodyne_hdl_snap_sweep_point_cloud_xyzi>* snap_sweep_xyzi_signal_;
     boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > current_snap_sweep_xyz_;
     boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > current_snap_sweep_xyzi_;
     boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > current_snap_sweep_xyzrgb_;
	 void fireCurrentSnapSweep (const unsigned short startAngle, const unsigned short endAngle,unsigned int sweepCount);
	 // sync time stamp

	 FILE *stampFile_;
	 FILE *gpsFile_;
	 FILE *pclFile_;
	 time_t velodyneTimef1;
	 FILE *scanFile_;
	 FILE *rotationFile_;
     FILE *timeSyncFile_;
	 unsigned int scan_data_count_;
	 struct HDLScanInterpolation *scan_correction_LUT_;
	 struct HDLScanInterpolation scan_correction_params_;// lineal interpolation of velocity and yaw rate for a current sweep
	 struct HDLFiringCorrection  firing_correction_params_;// correction for a current firing/scan
	 bool is_scan_correction_on_;
     std::string pcap_ip_port_filter_;
     VELODYNE_MODEL  velodyne_model_;
     double model_dist_correction_;
     int16_t convert12bit2int16(uint16_t v);
     // VLP-16
     void initializeLaserMapping ();
     void loadVLP16Corrections ();
     void toPointCloudsVPL16 (HDLDataPacket *dataPacket);
     bool isTransformOn_;
     QuaternionFnc::euler_t rpyRotVelodyne_;
     QuaternionFnc::quat_t  quatVelodyne_;
     QuaternionFnc::point_t trasVelodyne_;
   
};

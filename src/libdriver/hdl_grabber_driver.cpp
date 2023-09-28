
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <pcl/io/boost.h>
#include <pcl/console/print.h>
#include <boost/version.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/math/special_functions.hpp>
#include <pcap.h>
#include <stdlib.h>
#include <string>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>

#ifndef ISROS
#define ISROS 1
#endif

#ifdef ISROS

#include "hdl_grabber/hdl_grabber_driver.h"

#else
#include "hdl_grabber_driver.h"
#endif

double *HDLGrabberDriver::cos_lookup_table_ = NULL;
double *HDLGrabberDriver::sin_lookup_table_ = NULL;

using boost::asio::ip::udp;

const boost::asio::ip::address HDLGrabberDriver::HDL_DEFAULT_NETWORK_ADDRESS = boost::asio::ip::address::from_string(
        "192.168.0.200");


HDLGrabberDriver::HDLGrabberDriver(const std::string &correctionsFile,
                                   const std::string &pcapFile, const bool &use_gps, const bool &use_laser,
                                   const std::string &sweepCorrectionFile, HDLGrabberDriver::VELODYNE_MODEL vmodel)
        : hdl_data_(), gps_data_(), udp_listener_endpoint_(HDL_DEFAULT_NETWORK_ADDRESS, HDL_DATA_PORT),
          udp_listener_gps_endpoint_(HDL_DEFAULT_NETWORK_ADDRESS, HDL_GPSDATA_PORT), source_address_filter_(),
          source_port_filter_(443), hdl_read_socket_service_(), hdl_read_socket_(NULL), hdl_read_gps_socket_service_(),
          hdl_read_gps_socket_(NULL), pcap_file_name_(pcapFile), queue_consumer_thread_(NULL),
          queue_consumer_IMU_thread_(NULL), hdl_read_packet_thread_(NULL), GPS_read_packet_thread_(NULL),
          use_gps_(use_gps), use_laser_(use_laser), current_scan_xyz_(new pcl::PointCloud<pcl::PointXYZ>()),
          current_sweep_xyz_(new pcl::PointCloud<pcl::PointXYZ>()),
          current_scan_xyzi_(new pcl::PointCloud<pcl::PointXYZI>()),
          current_sweep_xyzi_(new pcl::PointCloud<pcl::PointXYZI>()),
          current_scan_xyzrgb_(new pcl::PointCloud<pcl::PointXYZRGBA>()),
          current_sweep_xyzrgb_(new pcl::PointCloud<pcl::PointXYZRGBA>()), last_azimuth_(65000),
          hdl_imu_data_(new HDLIMUData), sweep_xyz_signal_(), sweep_xyzrgb_signal_(), sweep_xyzi_signal_(),
          scan_xyz_signal_(), scan_xyzrgb_signal_(), scan_xyzi_signal_(), gps_packet_signal_(),
          min_distance_threshold_(2.0), max_distance_threshold_(10000.0)
// carlota
        , is_pause_acquisition_on_(false), snap_sweep_xyz_signal_(), snap_sweep_xyzrgb_signal_(),
          snap_sweep_xyzi_signal_(), current_snap_sweep_xyz_(new pcl::PointCloud<pcl::PointXYZ>()),
          current_snap_sweep_xyzi_(new pcl::PointCloud<pcl::PointXYZI>()),
          current_snap_sweep_xyzrgb_(new pcl::PointCloud<pcl::PointXYZRGBA>()), firecount(0), is_set_pause_on_(false),
          scan_data_count_(0), is_scan_correction_on_(true), sweepCorrectionFile_(sweepCorrectionFile),
          pcap_ip_port_filter_(), velodyne_model_(vmodel), isTransformOn_(false) {

    printf("velodyne_model_ %d\n", velodyne_model_);
    printf("pcapfile: %s\n", pcapFile.c_str());
    initialize(correctionsFile);

}

/////////////////////////////////////////////////////////////////////////////
HDLGrabberDriver::HDLGrabberDriver(const boost::asio::ip::address &ipAddress,
                                   const unsigned short int port,
                                   const unsigned short int port_gps,
                                   const std::string &correctionsFile, const bool &use_gps, const bool &use_laser,
                                   HDLGrabberDriver::VELODYNE_MODEL vmodel)
        : hdl_data_(), gps_data_(), udp_listener_endpoint_(HDL_DEFAULT_NETWORK_ADDRESS, port),
          udp_listener_gps_endpoint_(HDL_DEFAULT_NETWORK_ADDRESS, port_gps), source_address_filter_(),
          source_port_filter_(443), hdl_read_socket_service_(), hdl_read_socket_(NULL), hdl_read_gps_socket_service_(),
          hdl_read_gps_socket_(NULL), pcap_file_name_(pcapFile), queue_consumer_thread_(NULL),
          queue_consumer_IMU_thread_(NULL), hdl_read_packet_thread_(NULL), GPS_read_packet_thread_(NULL),
          use_gps_(use_gps), use_laser_(use_laser), current_scan_xyz_(new pcl::PointCloud<pcl::PointXYZ>()),
          current_sweep_xyz_(new pcl::PointCloud<pcl::PointXYZ>()),
          current_scan_xyzi_(new pcl::PointCloud<pcl::PointXYZI>()),
          current_sweep_xyzi_(new pcl::PointCloud<pcl::PointXYZI>()),
          current_scan_xyzrgb_(new pcl::PointCloud<pcl::PointXYZRGBA>()),
          current_sweep_xyzrgb_(new pcl::PointCloud<pcl::PointXYZRGBA>()), last_azimuth_(65000),
          hdl_imu_data_(new HDLIMUData), sweep_xyz_signal_(), sweep_xyzrgb_signal_(), sweep_xyzi_signal_(),
          scan_xyz_signal_(), scan_xyzrgb_signal_(), scan_xyzi_signal_(), gps_packet_signal_(),
          min_distance_threshold_(2.0), max_distance_threshold_(10000.0)
// carlota
        , is_pause_acquisition_on_(false), snap_sweep_xyz_signal_(), snap_sweep_xyzrgb_signal_(),
          snap_sweep_xyzi_signal_(), current_snap_sweep_xyz_(new pcl::PointCloud<pcl::PointXYZ>()),
          current_snap_sweep_xyzi_(new pcl::PointCloud<pcl::PointXYZI>()),
          current_snap_sweep_xyzrgb_(new pcl::PointCloud<pcl::PointXYZRGBA>()), firecount(0), is_set_pause_on_(false),
          scan_data_count_(0), is_scan_correction_on_(true), velodyne_model_(vmodel), isTransformOn_(false) {
    
    ROS_INFO_STREAM("VELODYNE DRIVER starting");
    ROS_INFO("Selected Velodyne Sensor with variable velodyne_model_ %d", velodyne_model_);

    sweepCorrectionFile_.clear();
    initialize(correctionsFile);

    if (pcap_file_name_.empty())
        ROS_WARN_STREAM("NO PCAP");

}

/////////////////////////////////////////////////////////////////////////////
HDLGrabberDriver::~HDLGrabberDriver() throw() {
    stop();

    disconnect_all_slots<sig_cb_velodyne_hdl_sweep_point_cloud_xyz>();
    disconnect_all_slots<sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgb>();
    disconnect_all_slots<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi>();
    disconnect_all_slots<sig_cb_velodyne_hdl_scan_point_cloud_xyz>();
    disconnect_all_slots<sig_cb_velodyne_hdl_scan_point_cloud_xyzrgb>();
    disconnect_all_slots<sig_cb_velodyne_hdl_scan_point_cloud_xyzi>();
    // carlota
    disconnect_all_slots<sig_cb_velodyne_hdl_snap_sweep_point_cloud_xyz>();
    disconnect_all_slots<sig_cb_velodyne_hdl_snap_sweep_point_cloud_xyzrgb>();
    disconnect_all_slots<sig_cb_velodyne_hdl_snap_sweep_point_cloud_xyzi>();

    ROS_INFO_STREAM("VELODYNE DRIVER stopping");

}


void HDLGrabberDriver::initialize(const std::string &correctionsFile) {

    sweep_xyz_signal_ = createSignal<sig_cb_velodyne_hdl_sweep_point_cloud_xyz>();
    sweep_xyzrgb_signal_ = createSignal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzrgb>();
    sweep_xyzi_signal_ = createSignal<sig_cb_velodyne_hdl_sweep_point_cloud_xyzi>();
    scan_xyz_signal_ = createSignal<sig_cb_velodyne_hdl_scan_point_cloud_xyz>();
    scan_xyzrgb_signal_ = createSignal<sig_cb_velodyne_hdl_scan_point_cloud_xyzrgb>();
    scan_xyzi_signal_ = createSignal<sig_cb_velodyne_hdl_scan_point_cloud_xyzi>();
    gps_packet_signal_ = createSignal<sig_cb_velodyne_gps_packet_decoded>();

    // carlota
    snap_sweep_xyz_signal_ = createSignal<sig_cb_velodyne_hdl_snap_sweep_point_cloud_xyz>();
    snap_sweep_xyzrgb_signal_ = createSignal<sig_cb_velodyne_hdl_snap_sweep_point_cloud_xyzrgb>();
    snap_sweep_xyzi_signal_ = createSignal<sig_cb_velodyne_hdl_snap_sweep_point_cloud_xyzi>();

    current_scan_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    current_scan_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    current_sweep_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    current_sweep_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    current_snap_sweep_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    current_snap_sweep_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    loadCorrectionsFile(correctionsFile);
    if (cos_lookup_table_ == NULL && sin_lookup_table_ == NULL) {
        cos_lookup_table_ = static_cast<double *> (malloc(HDL_NUM_ROT_ANGLES * sizeof(*cos_lookup_table_)));
        sin_lookup_table_ = static_cast<double *> (malloc(HDL_NUM_ROT_ANGLES * sizeof(*sin_lookup_table_)));
        for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++) {
            double rad = (M_PI / 180.0) * (static_cast<double> (i) / 100.0);
            cos_lookup_table_[i] = std::cos(rad);
            sin_lookup_table_[i] = std::sin(rad);
        }
    }
    for (int i = 0; i < HDL_MAX_NUM_LASERS; i++) {
        HDLLaserCorrection correction = laser_corrections_[i];
        laser_corrections_[i].sinVertOffsetCorrection = correction.verticalOffsetCorrection
                                                        * correction.sinVertCorrection;
        laser_corrections_[i].cosVertOffsetCorrection = correction.verticalOffsetCorrection
                                                        * correction.cosVertCorrection;
    }
    for (int i = 0; i < HDL_MAX_NUM_LASERS; i++)
        laser_rgb_mapping_[i].r = laser_rgb_mapping_[i].g = laser_rgb_mapping_[i].b = 0;

    if (laser_corrections_[32].distanceCorrection == 0.0) {
        for (int i = 0; i < 16; i++) {
            laser_rgb_mapping_[i * 2].b = static_cast<uint8_t> (i * 6 + 64);
            laser_rgb_mapping_[i * 2 + 1].b = static_cast<uint8_t> ((i + 16) * 6 + 64);
        }
    } else {
        for (int i = 0; i < 16; i++) {
            laser_rgb_mapping_[i * 2].b = static_cast<uint8_t> (i * 3 + 64);
            laser_rgb_mapping_[i * 2 + 1].b = static_cast<uint8_t> ((i + 16) * 3 + 64);
        }
        for (int i = 0; i < 16; i++) {
            laser_rgb_mapping_[i * 2 + 32].b = static_cast<uint8_t> (i * 3 + 160);
            laser_rgb_mapping_[i * 2 + 33].b = static_cast<uint8_t> ((i + 16) * 3 + 160);
        }
    }
    switch (velodyne_model_) {
        case VLP_32C:
            model_dist_correction_ = 0.004;
            ROS_INFO_STREAM("Load VPLC32 Corrections");
            break;
        case VLP_16:
            model_dist_correction_ = 0.002;
            ROS_INFO_STREAM("Load VPL16 Corrections");
            break;
        case HDL_32E:
            model_dist_correction_ = 0.002;
            ROS_INFO_STREAM("Load HDL32 Corrections");
            break;
        default:
            ROS_INFO_STREAM("SENSOR NOT DEFINED");
            break;;
    }


}

void HDLGrabberDriver::loadCorrectionsFile(const std::string &correctionsFile) {
    if (correctionsFile.empty()) {
        switch (velodyne_model_) {
            case VLP_32C:
                model_dist_correction_ = 0.004;
                loadVPLC32Corrections();
                printf("loadVPLC32Corrections\n");
                break;
            case VLP_16:
                model_dist_correction_ = 0.002;
                loadVLP16Corrections();
                printf("loadVLP16Corrections\n");
                break;
            case HDL_32E:
            default:
                model_dist_correction_ = 0.002;
                loadHDL32Corrections();
                printf("loadHDL32Corrections\n");
                break;
        }

        return;
    }

    boost::property_tree::ptree pt;
    try {
        read_xml(correctionsFile, pt, boost::property_tree::xml_parser::trim_whitespace);
    }
    catch (boost::exception const &) {
        PCL_ERROR ("[pcl::HDLGrabber::loadCorrectionsFile] Error reading calibration file %s!\n",
                   correctionsFile.c_str());
        return;
    }
    BOOST_FOREACH (boost::property_tree::ptree::value_type &v, pt.get_child("boost_serialization.DB.points_")) {
                    if (v.first == "item") {
                        boost::property_tree::ptree points = v.second;
                        BOOST_FOREACH(boost::property_tree::ptree::value_type &px, points) {
                                        if (px.first == "px") {
                                            boost::property_tree::ptree calibrationData = px.second;
                                            int index = -1;
                                            double azimuth = 0, vertCorrection = 0, distCorrection = 0,
                                                    vertOffsetCorrection = 0, horizOffsetCorrection = 0;

                                            BOOST_FOREACH (boost::property_tree::ptree::value_type &item,
                                                           calibrationData) {
                                                            if (item.first == "id_")
                                                                index = atoi(item.second.data().c_str());
                                                            if (item.first == "rotCorrection_")
                                                                azimuth = atof(item.second.data().c_str());
                                                            if (item.first == "vertCorrection_")
                                                                vertCorrection = atof(item.second.data().c_str());
                                                            if (item.first == "distCorrection_")
                                                                distCorrection = atof(item.second.data().c_str());
                                                            if (item.first == "vertOffsetCorrection_")
                                                                vertOffsetCorrection = atof(item.second.data().c_str());
                                                            if (item.first == "horizOffsetCorrection_")
                                                                horizOffsetCorrection = atof(
                                                                        item.second.data().c_str());
                                                        }
                                            if (index != -1) {
                                                laser_corrections_[index].azimuthCorrection = azimuth;
                                                laser_corrections_[index].verticalCorrection = vertCorrection;
                                                laser_corrections_[index].distanceCorrection = distCorrection / 100.0;
                                                laser_corrections_[index].verticalOffsetCorrection =
                                                        vertOffsetCorrection / 100.0;
                                                laser_corrections_[index].horizontalOffsetCorrection =
                                                        horizOffsetCorrection / 100.0;

                                                laser_corrections_[index].cosVertCorrection = std::cos(
                                                        HDL_GrabberDriver_toRadians(
                                                                laser_corrections_[index].verticalCorrection));
                                                laser_corrections_[index].sinVertCorrection = std::sin(
                                                        HDL_GrabberDriver_toRadians(
                                                                laser_corrections_[index].verticalCorrection));
                                            }
                                        }
                                    }
                    }
                }


}

void HDLGrabberDriver::loadVLP16Corrections() {
    double vlp16_vertical_corrections[] = {-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15};
    for (std::uint8_t i = 0; i < VLP_MAX_NUM_LASERS; i++) {
        laser_corrections_[i].azimuthCorrection = 0.0;
        laser_corrections_[i].distanceCorrection = 0.0;
        laser_corrections_[i].horizontalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalCorrection = vlp16_vertical_corrections[i];
        laser_corrections_[i].sinVertCorrection = std::sin(HDL_GrabberDriver_toRadians(vlp16_vertical_corrections[i]));
        laser_corrections_[i].cosVertCorrection = std::cos(HDL_GrabberDriver_toRadians(vlp16_vertical_corrections[i]));
    }
}

void HDLGrabberDriver::loadHDL32Corrections() {
    double hdl32VerticalCorrections[] = {
            -30.67, -9.3299999, -29.33, -8, -28,
            -6.6700001, -26.67, -5.3299999, -25.33, -4, -24, -2.6700001, -22.67,
            -1.33, -21.33, 0, -20, 1.33, -18.67, 2.6700001, -17.33, 4, -16, 5.3299999,
            -14.67, 6.6700001, -13.33, 8, -12, 9.3299999, -10.67, 10.67};
    for (int i = 0; i < HDL_LASER_PER_FIRING; i++) {
        laser_corrections_[i].azimuthCorrection = 0.0;
        laser_corrections_[i].distanceCorrection = 0.0;
        laser_corrections_[i].horizontalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalCorrection = hdl32VerticalCorrections[i];
        laser_corrections_[i].sinVertCorrection = std::sin(HDL_GrabberDriver_toRadians(hdl32VerticalCorrections[i]));
        laser_corrections_[i].cosVertCorrection = std::cos(HDL_GrabberDriver_toRadians(hdl32VerticalCorrections[i]));
    }
    for (int i = HDL_LASER_PER_FIRING; i < HDL_MAX_NUM_LASERS; i++) {
        laser_corrections_[i].azimuthCorrection = 0.0;
        laser_corrections_[i].distanceCorrection = 0.0;
        laser_corrections_[i].horizontalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalCorrection = 0.0;
        laser_corrections_[i].sinVertCorrection = 0.0;
        laser_corrections_[i].cosVertCorrection = 1.0;
    }
}

void HDLGrabberDriver::loadVPLC32Corrections() {
    double vlpc32VerticalCorrections[] = {
            -25.0, -1.0, -1.667, -15.639, -11.31,
            0, -0.667, -8.843, -7.254, 0.333, -0.333, -6.148, -5.333,
            1.333, 0.667, -4.0, -4.667, 1.667, 1, -3.667, -3.333, 3.333, 2.333, -2.667,
            -3.0, 7.0, 4.667, -2.333, -2.0, 15.0, 10.333, -1.333};

    for (int i = 0; i < HDL_LASER_PER_FIRING; i++) {
        laser_corrections_[i].azimuthCorrection = 0.0;
        laser_corrections_[i].distanceCorrection = 0.0;
        laser_corrections_[i].horizontalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalCorrection = vlpc32VerticalCorrections[i];
        laser_corrections_[i].sinVertCorrection = std::sin(HDL_GrabberDriver_toRadians(vlpc32VerticalCorrections[i]));
        laser_corrections_[i].cosVertCorrection = std::cos(HDL_GrabberDriver_toRadians(vlpc32VerticalCorrections[i]));
    }
    for (int i = HDL_LASER_PER_FIRING; i < HDL_MAX_NUM_LASERS; i++) {
        laser_corrections_[i].azimuthCorrection = 0.0;
        laser_corrections_[i].distanceCorrection = 0.0;
        laser_corrections_[i].horizontalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalOffsetCorrection = 0.0;
        laser_corrections_[i].verticalCorrection = 0.0;
        laser_corrections_[i].sinVertCorrection = 0.0;
        laser_corrections_[i].cosVertCorrection = 1.0;
    }
}


void HDLGrabberDriver::loadScanInterpolationCorrectionData() {


    int icount = 1;
    scan_data_count_ = 0;
    scan_correction_LUT_ = NULL;

    if (sweepCorrectionFile_.empty()) {
        ROS_ERROR_STREAM("hdl_grabber_driver.cpp -- Error: sweep corrections file name is empty");
    } 
    else 
    {
        ROS_INFO_STREAM("hdl_grabber_driver.cpp -- sweep corr: " << sweepCorrectionFile_);
        scanFile_ = fopen(sweepCorrectionFile_.c_str(), "r");
        if (scanFile_ != NULL) {

            scan_correction_LUT_ = (struct HDLScanInterpolation *) malloc(
                    icount * 100 * sizeof(struct HDLScanInterpolation));

            while (feof(scanFile_) == 0) {

                if (scan_data_count_ == (icount * 100)) {
                    icount++;
                    scan_correction_LUT_ = (struct HDLScanInterpolation *) realloc(scan_correction_LUT_, icount * 100 *
                                                                                                         sizeof(struct HDLScanInterpolation));


                }
                fscanf(scanFile_, "%u\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
                       &scan_correction_LUT_[scan_data_count_].sweepcounter,
                       &scan_correction_LUT_[scan_data_count_].dt_scan,
                       &scan_correction_LUT_[scan_data_count_].ppx[0], &scan_correction_LUT_[scan_data_count_].ppx[1],
                       &scan_correction_LUT_[scan_data_count_].ppy[0], &scan_correction_LUT_[scan_data_count_].ppy[1],
                       &scan_correction_LUT_[scan_data_count_].ppyaw[0],
                       &scan_correction_LUT_[scan_data_count_].ppyaw[1],
                       &scan_correction_LUT_[scan_data_count_].pppitch[0],
                       &scan_correction_LUT_[scan_data_count_].pppitch[1],
                       &scan_correction_LUT_[scan_data_count_].pproll[0],
                       &scan_correction_LUT_[scan_data_count_].pproll[1],
                       &scan_correction_LUT_[scan_data_count_].ppz[0], &scan_correction_LUT_[scan_data_count_].ppz[1]);
                scan_data_count_++;
            }
            scan_correction_LUT_ = (struct HDLScanInterpolation *) realloc(scan_correction_LUT_, scan_data_count_ *
                                                                                                 sizeof(struct HDLScanInterpolation));
            fclose(scanFile_);
            scanFile_ = NULL;

        }
    }
}

void
HDLGrabberDriver::setTransformationParameters(double x, double y, double z, double roll, double pitch, double yaw) {

    isTransformOn_ = true;
    rpyRotVelodyne_.r = roll;
    rpyRotVelodyne_.p = pitch;
    rpyRotVelodyne_.y = yaw;
    trasVelodyne_.x = x;
    trasVelodyne_.y = y;
    trasVelodyne_.z = z;
    QuaternionFnc quaternion;
    quaternion.euler2quaternion(quatVelodyne_, rpyRotVelodyne_);

}

bool HDLGrabberDriver::getDataForScanCorrection(unsigned int sweepcounter, struct HDLScanInterpolation *params) {

    int i, j;
    char temp[50];
    params->sweepcounter = sweepcounter;

    for (j = 0; j < 2; j++) {
        params->ppx[j] = 0;
        params->ppy[j] = 0;
        params->ppz[j] = 0;
        params->ppyaw[j] = 0;
        params->pppitch[j] = 0;
        params->pproll[j] = 0;
    }

    for (i = 0; i < scan_data_count_; i++) {
        if (sweepcounter == scan_correction_LUT_[i].sweepcounter) {
            params->dt_scan = scan_correction_LUT_[i].dt_scan;
            for (j = 0; j < 2; j++) {
                params->ppx[j] = scan_correction_LUT_[i].ppx[j];
                params->ppy[j] = scan_correction_LUT_[i].ppy[j];
                params->ppz[j] = scan_correction_LUT_[i].ppz[j];
                params->ppyaw[j] = scan_correction_LUT_[i].ppyaw[j];
                params->pppitch[j] = scan_correction_LUT_[i].pppitch[j];
                params->pproll[j] = scan_correction_LUT_[i].pproll[j];
            }
            return (true);

        }
    }


    return (false);


}

bool HDLGrabberDriver::getParamsForFiringCorrection(unsigned int scancounter, float azimuth_pos,
                                                    struct HDLScanInterpolation *params,
                                                    struct HDLFiringCorrection *firing_params) {


//	printf("Azimuthal position %f\n",azimuth_pos);
    float time_to_correct = (params->dt_scan / 360.0) * (360.0 - azimuth_pos);
    float angle_to_correct = ((params->ppyaw[1] - params->ppyaw[0]) / params->dt_scan) * time_to_correct; // w*t
    float speedxy[2] = {(params->ppx[1] - params->ppx[0]) / params->dt_scan,
                        (params->ppy[1] - params->ppy[0]) / params->dt_scan};
    float speed;
    speed = sqrt(speedxy[0] * speedxy[0] + speedxy[1] * speedxy[1]);

    firing_params->yaw = -angle_to_correct;
    firing_params->x = -(speed * cos(angle_to_correct)) * time_to_correct;
    firing_params->y = -(speed * sin(angle_to_correct)) * time_to_correct;
    firing_params->pitch = 0;
    firing_params->roll = 0;
    firing_params->z = 0;

}

bool HDLGrabberDriver::doScanCorrection(pcl::PointXYZI &undistorXYZI, struct HDLFiringCorrection *firing_params) {

    QuaternionFnc quaternion;

    QuaternionFnc::euler_t rpy;
    QuaternionFnc::quat_t quat;
    QuaternionFnc::point_t point_in, point_out, point_translation;
    QuaternionFnc::point_t temp_point;
    // correct pitch inclination if exist
    /*	rpy.r=0;
        rpy.p=0;
        rpy.y=0;
        point_in.x=undistorXYZI.x;
        point_in.y=undistorXYZI.y;
        point_in.z=undistorXYZI.z;

        quaternion.euler2quaternion(quat,rpy);
        quaternion.quat2Rotation3DPoint(temp_point,point_in,quat);
        point_in.x=temp_point.x;
        point_in.y=temp_point.y;
        point_in.z=temp_point.z;
        */

    point_translation.z = 0;
    point_translation.y = firing_params->y;
    point_translation.x = firing_params->x;
    point_translation.z = firing_params->z;

    rpy.r = 0;
    rpy.p = 0;
    rpy.y = firing_params->yaw;

    point_in.x = undistorXYZI.x;
    point_in.y = undistorXYZI.y;
    point_in.z = undistorXYZI.z;

    // rotation
    quaternion.euler2quaternion(quat, rpy);
    quaternion.quat2Rotation3DPoint(point_out, point_in, quat);
    // translation
    undistorXYZI.x = static_cast<float> (point_out.x + point_translation.x);
    undistorXYZI.y = static_cast<float> (point_out.y + point_translation.y);
    undistorXYZI.z = static_cast<float> (point_out.z + point_translation.z);


    return (true);

}

/////////////////////////////////////////////////////////////////////////////
void HDLGrabberDriver::start() {


    terminate_read_packet_thread_ = false;

    if (isRunning())
        return;

    if (this->use_laser_)
        queue_consumer_thread_ = new boost::thread(boost::bind(&HDLGrabberDriver::processVelodynePackets, this));

    if (this->use_gps_)
        queue_consumer_IMU_thread_ = new boost::thread(boost::bind(&HDLGrabberDriver::processGPSIMUPackets, this));


    is_pause_acquisition_on_ = false;
    is_set_pause_on_ = false;
    // sync time stamp
    stampFile_ = NULL;
    gpsFile_ = NULL;
    pclFile_ = NULL;
    timeSyncFile_ = NULL;
    //   stampFile_=fopen("/home/carlota/0_Work/6_Data/CSIC_IROS_2018/2018_09_21/VelodyneTime.txt","w");
    //   gpsFile_=fopen("/home/carlota/0_Work/6_Data/CSIC_IROS_2018/2018_09_21/VelodyneGPS.txt","w");
    //  pclFile_=fopen("/home/carlota/0_Work/6_Data/CSIC_IROS_2018/2018_09_21/pclVelodyne.txt","w");

    //        stampFile_=fopen("VelodyneTime.txt","w");
    //         gpsFile_=fopen("VelodyneGPS.txt","w");
    //         pclFile_=fopen("pclVelodyne.txt","w");
//          timeSyncFile_ =fopen("pcapVelodyneTime.txt","w");

    scanFile_ = NULL;
    scan_correction_LUT_ = NULL;
    loadScanInterpolationCorrectionData();


    if (scan_correction_LUT_ == NULL) 
    {
        ROS_WARN_STREAM("hdl_grabber_driver.cpp -- Correction is OFF");
        is_scan_correction_on_ = false;
    } 
    else 
    {
        ROS_INFO_STREAM("hdl_grabber_driver.cpp -- Correction if ON");
        is_scan_correction_on_ = true;
    }

    if (pcap_file_name_.empty()) {


        try {
            try {
                if (this->use_laser_)
                    hdl_read_socket_ = new udp::socket(hdl_read_socket_service_,
                                                       udp::endpoint(boost::asio::ip::address_v4::any(),
                                                                     udp_listener_endpoint_.port()));
                if (this->use_gps_) {
                    hdl_read_gps_socket_ = new udp::socket(hdl_read_gps_socket_service_,
                                                           udp::endpoint(boost::asio::ip::address_v4::any(),
                                                                         udp_listener_gps_endpoint_.port()));
                }


            }
            catch (std::exception bind) {

                /* delete hdl_read_socket_;
                 delete hdl_read_gps_socket_;


                 if(this->use_laser_)
                   hdl_read_socket_ = new udp::socket (hdl_read_socket_service_, udp::endpoint(boost::asio::ip::address_v4::any(), udp_listener_endpoint_.port()));
                 if(this->use_gps_)
                   hdl_read_gps_socket_ = new udp::socket (hdl_read_gps_socket_service_, udp::endpoint(boost::asio::ip::address_v4::any(), udp_listener_gps_endpoint_.port()));*/
            }

            if (this->use_laser_)
                hdl_read_socket_service_.run();
            if (this->use_gps_)
                hdl_read_gps_socket_service_.run();

        }
        catch (std::exception &e) {
            PCL_ERROR ("[pcl::HDLGrabber::start] Unable to bind to socket! %s\n", e.what());
            return;
        }


        if (this->use_laser_)
            hdl_read_packet_thread_ = new boost::thread(boost::bind(&HDLGrabberDriver::readHDLPacketsFromSocket, this));
        if (this->use_gps_) {
            GPS_read_packet_thread_ = new boost::thread(boost::bind(&HDLGrabberDriver::readGPSPacketsFromSocket, this));
        }
    }     
    else 
    {

        std::cout << "start read packet" << std::endl;
        hdl_read_packet_thread_ = new boost::thread(boost::bind(&HDLGrabberDriver::readPacketsFromPcap, this));

    }
}

void HDLGrabberDriver::processVelodynePackets() {
    while (true) 
    {
        unsigned char *data;

        if (!hdl_data_.dequeue(data))
            return;


        unsigned char productID;
        switch (velodyne_model_) 
        {
            case VLP_32C:
                
            case VLP_16:
                productID = toPointCloudsVPL(reinterpret_cast<HDLDataPacket *> (data));
                if (productID != 0x22)
                {
                    ROS_ERROR_STREAM("WRONG LASER");
                    ros::shutdown();
                }
                else
                    ROS_INFO_STREAM_ONCE("ProductID 0x22 detected, you're connected to a VLP_16 or Puck LITE sensor");
                break;                
                

            case HDL_32E:
                productID = toPointClouds(reinterpret_cast<HDLDataPacket *> (data));
                if (productID != 0x21)
                {
                    ROS_ERROR_STREAM("WRONG LASER");
                    ros::shutdown();
                }
                else
                    ROS_INFO_STREAM_ONCE("ProductID 0x21 detected, you're connected to a HDL_32E sensor");
                break;
                
            default:
                ROS_ERROR_STREAM("NOT DEFINED \n");
                break;;
        }


        free(data);

    }

}


/////////////////////////////////////////////////////////////////////////////
unsigned char HDLGrabberDriver::toPointClouds(HDLDataPacket *dataPacket) {
    static uint32_t scanCounter = 0;
    static uint32_t sweepCounter = 0;
    static unsigned int stampcounter = 0;
    char temp[150];
    FILE *pcl_xyz = NULL;
    
    std::string xyz_fname = "pcl_xyz/pcloud_%05u.pcd";// cambiar
    // std::string xyz_fname="/home/carlota/5_Data/SHOW/pruebas_villaverde/pcl_xyz/%06u.txt";// cambiar
    if (sizeof(HDLLaserReturn) != 3)
    {   
        ROS_ERROR_STREAM("sizeof(HDLLaserReturn) != 3");
        return -1;
    }

    current_scan_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    current_scan_xyzrgb_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    current_scan_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    time_t time_;
    time(&time_);
    // time_t velodyneTime = (time_ & 0x00000000ffffffffl) << 32 | dataPacket->gpsTimestamp;
    time_t velodyneTime = dataPacket->gpsTimestamp;
    current_scan_xyz_->header.stamp = velodyneTime;
    current_scan_xyzrgb_->header.stamp = velodyneTime;
    current_scan_xyzi_->header.stamp = velodyneTime;
    current_scan_xyz_->header.seq = scanCounter;
    current_scan_xyzrgb_->header.seq = scanCounter;
    current_scan_xyzi_->header.seq = scanCounter;
    scanCounter++;
    // firecount++;

    if (firecount == 0) 
    { // cambiar !!

        if (velodyneTimef1 > dataPacket->gpsTimestamp)
            std::cout << "POSIBLE ERROR time stamp velodyne: "
                      << (double) dataPacket->gpsTimestamp - (double) velodyneTimef1 << std::endl;


        velodyneTimef1 = dataPacket->gpsTimestamp;
        // get sweep params
        if (is_scan_correction_on_) {
            std::cerr << "getDataForScanCorrection" << std::endl;
            getDataForScanCorrection(sweepCounter, &scan_correction_params_);
        }
        if (stampcounter > dataPacket->gpsTimestamp)
            std::cout << "POSIBLE ERROR time stamp velodyne: "
                      << (double) dataPacket->gpsTimestamp - (double) stampcounter << std::endl;

        stampcounter = dataPacket->gpsTimestamp;

    }


    // sybc time stamp
    if (stampFile_ != NULL)
        fprintf(stampFile_, "%d\t%ld\t%u\t%d\n", firecount, velodyneTimef1, dataPacket->gpsTimestamp, sweepCounter);

    for (int i = 0; i < HDL_FIRING_PER_PKT; ++i) {
        HDLFiringData firingData = dataPacket->firingData[i];
        int offset = (firingData.blockIdentifier == BLOCK_0_TO_31) ? 0 : 32;


        for (int j = 0; j < HDL_LASER_PER_FIRING; j++) {
            if (firingData.rotationalPosition < last_azimuth_) {
                //carlota
                if (is_set_pause_on_) {
                    setPauseEventOn();
                }

                if (current_sweep_xyzrgb_->size() > 0) {

                    velodyneTime = velodyneTimef1;
                    current_sweep_xyz_->is_dense = current_sweep_xyzrgb_->is_dense = current_sweep_xyzi_->is_dense = false;
                    current_sweep_xyz_->header.stamp = velodyneTime;
                    current_sweep_xyzrgb_->header.stamp = velodyneTime;
                    current_sweep_xyzi_->header.stamp = velodyneTime;
                    current_sweep_xyz_->header.seq = sweepCounter;
                    current_sweep_xyzrgb_->header.seq = sweepCounter;
                    current_sweep_xyzi_->header.seq = sweepCounter;
                    // carlota
                    current_snap_sweep_xyz_->is_dense = current_snap_sweep_xyzrgb_->is_dense = current_snap_sweep_xyzi_->is_dense = false;
                    current_snap_sweep_xyz_->header.stamp = velodyneTime;
                    current_snap_sweep_xyzrgb_->header.stamp = velodyneTime;
                    current_snap_sweep_xyzi_->header.stamp = velodyneTime;
                    current_snap_sweep_xyz_->header.seq = sweepCounter;
                    current_snap_sweep_xyzrgb_->header.seq = sweepCounter;
                    current_snap_sweep_xyzi_->header.seq = sweepCounter;


                    if (pclFile_ != NULL) {
                        sprintf(temp, xyz_fname.c_str(), sweepCounter);
                        pcl::io::savePCDFileASCII(temp, *current_sweep_xyzrgb_);
                        //               if((pcl_xyz=fopen(temp,"wb"))!=NULL)
                        //                 {
                        //                 	float tempv;
                        // 	  for (size_t kk = 0; kk < current_snap_sweep_xyz_->points.size (); ++kk){
                        //                         tempv =(float)current_snap_sweep_xyzi_->points[kk].x;
                        // 	  fwrite(&tempv,sizeof(float),1,pcl_xyz);
                        // 	  tempv =(float)current_snap_sweep_xyzi_->points[kk].y;
                        // 	  fwrite(&tempv,sizeof(float),1,pcl_xyz);
                        // 	  tempv =(float)current_snap_sweep_xyzi_->points[kk].z;
                        // 	  fwrite(&tempv,sizeof(float),1,pcl_xyz);
                        // 	  tempv =(float)current_snap_sweep_xyzi_->points[kk].intensity;
                        // 	  fwrite(&tempv,sizeof(float),1,pcl_xyz);
                        //   }
                        //   fclose(pcl_xyz);
                        // //  fprintf(pclFile_,"%d\t%ld\n",sweepCounter,velodyneTime);

                        //                 }
                        fprintf(pclFile_, "%d\t%ld\n", sweepCounter, velodyneTime);

                    }
                    sweepCounter++;

                    if (is_pause_acquisition_on_) {
                        fireCurrentSnapSweep(dataPacket->firingData[0].rotationalPosition,
                                             dataPacket->firingData[11].rotationalPosition, sweepCounter);

                    } else {

                        fireCurrentSweep();
                    }
                    firecount = 0;
                }// if cloud size >0

                current_sweep_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>());
                current_sweep_xyzrgb_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
                current_sweep_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>());
                // carlota
                current_snap_sweep_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>());
                current_snap_sweep_xyzrgb_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
                current_snap_sweep_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            }// if rot azimuth > last rot azimuth
            if (firecount == 0) { // cambiar !!
                if (velodyneTimef1 > dataPacket->gpsTimestamp)
                    std::cout << "POSIBLE ERROR time stamp velodyne: "
                              << (double) dataPacket->gpsTimestamp - (double) velodyneTimef1 << std::endl;

                velodyneTimef1 = dataPacket->gpsTimestamp;
                if (is_scan_correction_on_) {
                    getDataForScanCorrection(sweepCounter, &scan_correction_params_);
                    getParamsForFiringCorrection(firecount, (firingData.rotationalPosition) / 100,
                                                 &scan_correction_params_,
                                                 &firing_correction_params_); //do correction numscan*0.533msec, numscan++
                }

                if (stampcounter > dataPacket->gpsTimestamp)
                    std::cout << "stampCounter POSIBLE ERROR time stamp velodyne: "
                              << (double) dataPacket->gpsTimestamp - (double) stampcounter << std::endl;

                stampcounter = dataPacket->gpsTimestamp;
            }
            pcl::PointXYZ xyz;
            pcl::PointXYZI xyzi;
            pcl::PointXYZRGBA xyzrgb;

            computeXYZI(xyzi, firingData.rotationalPosition, firingData.laserReturns[j],
                        laser_corrections_[j + offset]);


            xyzrgb.rgba = laser_rgb_mapping_[j + offset].rgba;
            if ((boost::math::isnan)(xyzi.x) ||
                (boost::math::isnan)(xyzi.y) ||
                (boost::math::isnan)(xyzi.z)) {
                continue;
            }

            // if correction on do process on xyzi
            if (is_scan_correction_on_) {
                getParamsForFiringCorrection(firecount, (firingData.rotationalPosition) / 100, &scan_correction_params_,
                                             &firing_correction_params_); //do correction numscan*0.533msec, numscan++
                doScanCorrection(xyzi, &firing_correction_params_);

            }
            xyz.x = xyzrgb.x = xyzi.x;
            xyz.y = xyzrgb.y = xyzi.y;
            xyz.z = xyzrgb.z = xyzi.z;

            current_scan_xyz_->push_back(xyz);
            current_scan_xyzi_->push_back(xyzi);
            current_scan_xyzrgb_->push_back(xyzrgb);

            current_sweep_xyz_->push_back(xyz);
            current_sweep_xyzi_->push_back(xyzi);
            current_sweep_xyzrgb_->push_back(xyzrgb);

            //
            current_snap_sweep_xyz_->push_back(xyz);
            current_snap_sweep_xyzi_->push_back(xyzi);
            current_snap_sweep_xyzrgb_->push_back(xyzrgb);


            last_azimuth_ = firingData.rotationalPosition;
        }
    }

    firecount++;

    current_scan_xyz_->is_dense = current_scan_xyzrgb_->is_dense = current_scan_xyzi_->is_dense = true;
    fireCurrentScan(dataPacket->firingData[0].rotationalPosition, dataPacket->firingData[11].rotationalPosition);
    
    ROS_DEBUG("Return Mode: %#010x\n", dataPacket->blank1);
    ROS_DEBUG("Product ID : %#010x\n", dataPacket->blank2);
    
    return dataPacket->blank2;

}

unsigned char HDLGrabberDriver::toPointCloudsVPL(HDLDataPacket *dataPacket) 
{
    static uint32_t scanCounter = 0;
    static uint32_t sweepCounter = 0;
    static unsigned int stampcounter = 0;
    char temp[150];
    FILE *pcl_xyz = NULL;
    //std::string xyz_fname="pcl_xyz/pcloud_%05u.pcd";// cambiar
    std::string xyz_fname = "/home/carlota/5_Data/SHOW/pruebas_villaverde/pcl_xyz_vlp16/%06u.txt";// cambiar
    if (sizeof(HDLLaserReturn) != 3)
    {   
        ROS_ERROR_STREAM("sizeof(HDLLaserReturn) != 3");
        return -1;
    }    

    current_scan_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    current_scan_xyzrgb_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    current_scan_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    time_t time_;
    time(&time_);
    // time_t velodyneTime = (time_ & 0x00000000ffffffffl) << 32 | dataPacket->gpsTimestamp;
        
    time_t velodyneTime = dataPacket->gpsTimestamp;
    current_scan_xyz_->header.stamp = velodyneTime;
    current_scan_xyzrgb_->header.stamp = velodyneTime;
    current_scan_xyzi_->header.stamp = velodyneTime;
    current_scan_xyz_->header.seq = scanCounter;
    current_scan_xyzrgb_->header.seq = scanCounter;
    current_scan_xyzi_->header.seq = scanCounter;
    scanCounter++;

    // VLP_gragger
    float azimuth = 0.0f;
    float azimuth_diff = 0.0f;
    float last_azimuth_diff = 0.0f;
    float azimuth_corrected_f = 0.0f;
    int azimuth_corrected = 0;

    // rotate
    QuaternionFnc quaternion;

    QuaternionFnc::euler_t rpy;
    QuaternionFnc::quat_t quat;
    QuaternionFnc::point_t point_in, point_out;


    if (firecount == 0) { // cambiar !!
        velodyneTimef1 = dataPacket->gpsTimestamp;
        // get sweep params
        std::cout << "1. sweepCounter : " << sweepCounter << std::endl;
        if (is_scan_correction_on_) {

            getDataForScanCorrection(sweepCounter, &scan_correction_params_);

        }

        if (stampcounter > dataPacket->gpsTimestamp)
            std::cout << "POSIBLE ERROR time stamp velodyne: " << dataPacket->gpsTimestamp - stampcounter << std::endl;

        stampcounter = dataPacket->gpsTimestamp;
    }


    if (stampFile_ != NULL)
        fprintf(stampFile_, "%d\t%ld\t%u\n", firecount, velodyneTimef1, dataPacket->gpsTimestamp);

    for (int i = 0; i < HDL_FIRING_PER_PKT; ++i) //12
    {
        azimuth = static_cast<float>(dataPacket->firingData[i].rotationalPosition);
        if (i < (HDL_FIRING_PER_PKT - 1)) {
            azimuth_diff = static_cast<float>(
                    (36000 + dataPacket->firingData[i + 1].rotationalPosition -
                     dataPacket->firingData[i].rotationalPosition) %
                    36000);
            last_azimuth_diff = azimuth_diff;
        } else {
            azimuth_diff = last_azimuth_diff;
        }

        for (int j = 0; j < HDL_LASER_PER_FIRING; j++) // 32
        {
            azimuth_corrected_f =
                    azimuth + (azimuth_diff * (static_cast<float>(j) / 2.0f) *
                               (float) 2.304 / (float) 55.296);
            azimuth_corrected =
                    static_cast<int>(round(fmod(azimuth_corrected_f, 36000.0)));

            if (azimuth_corrected < last_azimuth_) {
                //carlota
                if (is_set_pause_on_) {
                    setPauseEventOn();
                }

                if (current_sweep_xyzrgb_->size() > 0) {
                    velodyneTime = velodyneTimef1;
                    current_sweep_xyz_->is_dense = current_sweep_xyzrgb_->is_dense = current_sweep_xyzi_->is_dense = false;
                    current_sweep_xyz_->header.stamp = velodyneTime;
                    current_sweep_xyzrgb_->header.stamp = velodyneTime;
                    current_sweep_xyzi_->header.stamp = velodyneTime;
                    current_sweep_xyz_->header.seq = sweepCounter;
                    current_sweep_xyzrgb_->header.seq = sweepCounter;
                    current_sweep_xyzi_->header.seq = sweepCounter;

                    // carlota
                    current_snap_sweep_xyz_->is_dense = current_snap_sweep_xyzrgb_->is_dense = current_snap_sweep_xyzi_->is_dense = false;
                    current_snap_sweep_xyz_->header.stamp = velodyneTime;
                    current_snap_sweep_xyzrgb_->header.stamp = velodyneTime;
                    current_snap_sweep_xyzi_->header.stamp = velodyneTime;
                    current_snap_sweep_xyz_->header.seq = sweepCounter;
                    current_snap_sweep_xyzrgb_->header.seq = sweepCounter;
                    current_snap_sweep_xyzi_->header.seq = sweepCounter;

                    sprintf(temp, xyz_fname.c_str(), sweepCounter);

                    // if((pcl_xyz=fopen(temp,"w"))!=NULL)
                    // {
                    // 	float tempv;
                    // 	fprintf(pcl_xyz,"%ld\n",velodyneTime);

                    // 	for (size_t kk = 0; kk < current_snap_sweep_xyz_->points.size (); ++kk){
                    // 		fprintf(pcl_xyz,"%lf\t%lf\t%lf\n",current_snap_sweep_xyzi_->points[kk].x,
                    // 		                          current_snap_sweep_xyzi_->points[kk].y,
                    // 								  current_snap_sweep_xyzi_->points[kk].z);
                    // 	}
                    // 	fclose(pcl_xyz);

                    // }
                    if (pclFile_ != NULL) {
                        sprintf(temp, xyz_fname.c_str(), sweepCounter);
                        //     pcl::io::savePCDFileASCII (temp,  *current_sweep_xyzrgb_);
                        fprintf(pclFile_, "%u\t%ld\n", sweepCounter, velodyneTime);
                    }
                    sweepCounter++;

                    if (is_pause_acquisition_on_) {
                        fireCurrentSnapSweep(dataPacket->firingData[0].rotationalPosition,
                                             dataPacket->firingData[11].rotationalPosition, sweepCounter);

                    } else {
                        fireCurrentSweep();
                    }
                    firecount = 0;
                }

                current_sweep_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>());
                current_sweep_xyzrgb_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
                current_sweep_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>());
                // carlota
                current_snap_sweep_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>());
                current_snap_sweep_xyzrgb_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
                current_snap_sweep_xyzi_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            }// if rot azimuth > last rot azimuth

            if (firecount == 0) {
                velodyneTimef1 = dataPacket->gpsTimestamp;
                if (is_scan_correction_on_) {
                    getDataForScanCorrection(sweepCounter,
                                             &scan_correction_params_); // from sweepcounter, reset numscan
                }

                if (stampcounter > dataPacket->gpsTimestamp)
                    std::cout << "POSIBLE ERROR time stamp velodyne: " << dataPacket->gpsTimestamp - stampcounter
                              << std::endl;

                stampcounter = dataPacket->gpsTimestamp;
            }
            pcl::PointXYZ xyz;
            pcl::PointXYZI xyzi;
            pcl::PointXYZRGBA xyzrgb;

            computeXYZI(xyzi, (int) (azimuth_corrected) /*firingData.rotationalPosition*/,
                        dataPacket->firingData[i].laserReturns[j], laser_corrections_[j]);

            // test translation
            if (isTransformOn_ /*&& velodyne_model_ == VLP_16*/) {

                point_in.x = xyzi.x;
                point_in.y = xyzi.y;
                point_in.z = xyzi.z;
                // rotation
                quaternion.euler2quaternion(quat, rpy);
                quaternion.quat2Rotation3DPoint(point_out, point_in, quatVelodyne_);
                /*
                     float RT_L_all[4][4];
                     RT_L_all[0][0]=-0.06045533989092144; RT_L_all[0][1]=0.9931658677176205;   RT_L_all[0][2]=-0.09983341664682803; RT_L_all[0][3]=0.1576628582783537;
                     RT_L_all[1][0]=0.9975213918651683; RT_L_all[1][1]=0.06372056252311083;   RT_L_all[1][2]=0.02984564764108106; RT_L_all[1][3]=1.318538334838201;
                     RT_L_all[2][0]=0.0360031200043886; RT_L_all[2][1]=-0.09778163995579293;   RT_L_all[2][2]=-0.9945564469840339; RT_L_all[2][3]=-0.0768054860124197;
                     RT_L_all[3][0]=0; RT_L_all[3][1]=0;   RT_L_all[3][2]=0; RT_L_all[3][3]=1;

                     xyzi.x = point_in.x*RT_L_all[0][0]+ point_in.y*RT_L_all[0][1] + point_in.z*RT_L_all[0][2] +RT_L_all[0][3];
                     xyzi.y = point_in.x*RT_L_all[1][0]+ point_in.y*RT_L_all[1][1] + point_in.z*RT_L_all[1][2] +RT_L_all[1][3];
                     xyzi.z = point_in.x*RT_L_all[2][0]+ point_in.y*RT_L_all[2][1] + point_in.z*RT_L_all[2][2] +RT_L_all[2][3];

                     */
                // translation

                xyzi.x = static_cast<float> (point_out.x ) + trasVelodyne_.x; //- (1.32);
                xyzi.y = static_cast<float> (point_out.y ) + trasVelodyne_.y; //(0.13);
                xyzi.z = static_cast<float> (point_out.z ) + trasVelodyne_.z; //(-0.1);


            }
            /*else if(isTransformOn_ &&velodyne_model_ == VLP_32C)
            {
                 point_in.x=xyzi.x;
                 point_in.y=xyzi.y;
                 point_in.z=xyzi.z;

                 float RT_R_all[4][4];
                 RT_R_all[0][0]=0.6042802429389806; RT_R_all[0][1]=0.7962070446568585;   RT_R_all[0][2]=-0.02999550020249573; RT_R_all[0][3]=0.1269419543671188;
                 RT_R_all[1][0]=0.7967069797497724; RT_R_all[1][1]=-0.6042831138806452;   RT_R_all[1][2]=0.009995333746650443; RT_R_all[1][3]=-1.348893973687469;
                 RT_R_all[2][0]=-0.01016741912199213; RT_R_all[2][1]=-0.02993760707709619;   RT_R_all[2][2]=-0.9995000566637778; RT_R_all[2][3]=-0.1173490007247007;
                 RT_R_all[3][0]=0; RT_R_all[3][1]=0;   RT_R_all[3][2]=0; RT_R_all[3][3]=1;

                 xyzi.x = point_in.x*RT_R_all[0][0]+ point_in.y*RT_R_all[0][1] + point_in.z*RT_R_all[0][2] +RT_R_all[0][3];
                 xyzi.y = point_in.x*RT_R_all[1][0]+ point_in.y*RT_R_all[1][1] + point_in.z*RT_R_all[1][2] +RT_R_all[1][3];
                 xyzi.z = point_in.x*RT_R_all[2][0]+ point_in.y*RT_R_all[2][1] + point_in.z*RT_R_all[2][2] +RT_R_all[2][3];



            }
            */
            // OJO!!
            xyz.x = xyzrgb.x = xyzi.x;
            xyz.y = xyzrgb.y = xyzi.y;
            xyz.z = xyzrgb.z = xyzi.z;
            xyzrgb.rgba = laser_rgb_mapping_[j].rgba;

            if (std::isnan(xyz.x) || std::isnan(xyz.y) || std::isnan(xyz.z)) {
                continue;
            }
            // if correction on do process on xyzi
            if (is_scan_correction_on_) {
                getParamsForFiringCorrection(firecount,
                                             ((float) azimuth_corrected/*firingData.rotationalPosition*/) / 100,
                                             &scan_correction_params_,
                                             &firing_correction_params_); //do correction numscan*0.533msec, numscan++
                doScanCorrection(xyzi, &firing_correction_params_);
            }

            current_scan_xyz_->push_back(xyz);
            current_scan_xyzi_->push_back(xyzi);
            current_scan_xyzrgb_->push_back(xyzrgb);

            current_sweep_xyz_->push_back(xyz);
            current_sweep_xyzi_->push_back(xyzi);
            current_sweep_xyzrgb_->push_back(xyzrgb);

            //
            current_snap_sweep_xyz_->push_back(xyz);
            current_snap_sweep_xyzi_->push_back(xyzi);
            current_snap_sweep_xyzrgb_->push_back(xyzrgb);

            last_azimuth_ = azimuth_corrected;
            //last_azimuth_ = firingData.rotationalPosition;
        }
    }

    firecount++;
    current_scan_xyz_->is_dense = current_scan_xyzrgb_->is_dense = current_scan_xyzi_->is_dense = true;
    fireCurrentScan(dataPacket->firingData[0].rotationalPosition, dataPacket->firingData[11].rotationalPosition);

    ROS_DEBUG("Return Mode: %#010x\n", dataPacket->blank1);
    ROS_DEBUG("Product ID : %#010x\n", dataPacket->blank2);
    
    return dataPacket->blank2;
    
}

void HDLGrabberDriver::computeXYZI(pcl::PointXYZI &point, int azimuth, HDLLaserReturn laserReturn,
                                   HDLLaserCorrection correction) {
    double cosAzimuth, sinAzimuth;

    //double distanceM = laserReturn.distance * 0.002; //HDLgrabber
    //  double distanceM = laserReturn.distance * 0.004;
//  printf("model_dist_correction_ %f\n",model_dist_correction_);
    double distanceM = (double) laserReturn.distance * model_dist_correction_;
// max=10000.0
// min_distance_threshold_ =  2.0;

    point.intensity = static_cast<float> (laserReturn.intensity);
    if (distanceM < min_distance_threshold_ || distanceM > max_distance_threshold_) {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();

        return;
    }

    if (correction.azimuthCorrection == 0) {
        cosAzimuth = cos_lookup_table_[azimuth];
        sinAzimuth = sin_lookup_table_[azimuth];
    } else {
        double azimuthInRadians = HDL_GrabberDriver_toRadians(
                (static_cast<double> (azimuth) / 100.0) - correction.azimuthCorrection);
        cosAzimuth = std::cos(azimuthInRadians);
        sinAzimuth = std::sin(azimuthInRadians);
    }

    distanceM += correction.distanceCorrection;

    double xyDistance = distanceM * correction.cosVertCorrection;

    point.x = static_cast<float> (xyDistance * sinAzimuth - correction.horizontalOffsetCorrection * cosAzimuth);
    point.y = static_cast<float> (xyDistance * cosAzimuth + correction.horizontalOffsetCorrection * sinAzimuth);
    point.z = static_cast<float> (distanceM * correction.sinVertCorrection + correction.cosVertOffsetCorrection);


    point.intensity = static_cast<float> (laserReturn.intensity);
}

/////////////////////////////////////////////////////////////////////////////
void HDLGrabberDriver::fireCurrentSweep() {
    if (sweep_xyz_signal_->num_slots() > 0)
        sweep_xyz_signal_->operator()(current_sweep_xyz_);

    if (sweep_xyzrgb_signal_->num_slots() > 0)
        sweep_xyzrgb_signal_->operator()(current_sweep_xyzrgb_);

    if (sweep_xyzi_signal_->num_slots() > 0)
        sweep_xyzi_signal_->operator()(current_sweep_xyzi_);
}

/////////////////////////////////////////////////////////////////////////////
void HDLGrabberDriver::fireCurrentScan(const unsigned short startAngle, const unsigned short endAngle) {
    const float start = static_cast<float> (startAngle) / 100.0f;
    const float end = static_cast<float> (endAngle) / 100.0f;

    if (scan_xyz_signal_->num_slots() > 0)
        scan_xyz_signal_->operator()(current_scan_xyz_, start, end);

    if (scan_xyzrgb_signal_->num_slots() > 0)
        scan_xyzrgb_signal_->operator()(current_scan_xyzrgb_, start, end);

    if (scan_xyzi_signal_->num_slots() > 0)
        scan_xyzi_signal_->operator()(current_scan_xyzi_, start, end);
}

/////////////////////////////////////////////////////////////////////////////
void HDLGrabberDriver::fireCurrentGPS() {
    if (gps_packet_signal_->num_slots() > 0)
        gps_packet_signal_->operator()(hdl_imu_data_);

}


void HDLGrabberDriver::processGPSIMUPackets() {
    while (true) {
        unsigned char *data_gps;


        if (!gps_data_.dequeue(data_gps))
            return;

        HDLIMUPacket *dataPacket = NULL;

        dataPacket = (reinterpret_cast<HDLIMUPacket *> (data_gps));
        hdl_imu_data_.reset(new HDLIMUData);

        time_t time_;
        time(&time_);
        time_t velodyneTime = (time_ & 0x00000000ffffffffl) << 32 | dataPacket->gpsTimestamp;

        // Velodyne timestamp /////////////
        hdl_imu_data_->gpsTimestamp = dataPacket->gpsTimestamp;

        // from velodyne_ros_driver gps_imu branch

        const float earth_gravity = 9.80665;

        for (int i = 0; i < 3; i++) {
            hdl_imu_data_->gyro_temp_accel[i].gyro =
                    ((float) convert12bit2int16(dataPacket->gyro_temp_accel[i].gyro)) * 0.09766 * M_PI / 180.0;
            hdl_imu_data_->gyro_temp_accel[i].temp =
                    ((float) convert12bit2int16(dataPacket->gyro_temp_accel[i].temp)) * 0.1453 + 25;
            hdl_imu_data_->gyro_temp_accel[i].accel_x =
                    ((float) convert12bit2int16(dataPacket->gyro_temp_accel[i].accel_x)) * 0.001221;
            hdl_imu_data_->gyro_temp_accel[i].accel_y =
                    ((float) convert12bit2int16(dataPacket->gyro_temp_accel[i].accel_y)) * 0.001221;

        }

        // x-axis
        hdl_imu_data_->gyro_temp_accel_xyz[0].gyro = hdl_imu_data_->gyro_temp_accel[1].gyro;
        hdl_imu_data_->gyro_temp_accel_xyz[0].temp = hdl_imu_data_->gyro_temp_accel[1].temp;
        hdl_imu_data_->gyro_temp_accel_xyz[0].accel_x = -hdl_imu_data_->gyro_temp_accel[0].accel_y * earth_gravity;
        hdl_imu_data_->gyro_temp_accel_xyz[0].accel_y = hdl_imu_data_->gyro_temp_accel[2].accel_x * earth_gravity;
        // y-axis
        hdl_imu_data_->gyro_temp_accel_xyz[1].gyro = -hdl_imu_data_->gyro_temp_accel[0].gyro;
        hdl_imu_data_->gyro_temp_accel_xyz[1].temp = hdl_imu_data_->gyro_temp_accel[0].temp;
        hdl_imu_data_->gyro_temp_accel_xyz[1].accel_x = -hdl_imu_data_->gyro_temp_accel[1].accel_y * earth_gravity;
        hdl_imu_data_->gyro_temp_accel_xyz[1].accel_y = -hdl_imu_data_->gyro_temp_accel[2].accel_y * earth_gravity;
        //  z-axis
        hdl_imu_data_->gyro_temp_accel_xyz[2].gyro = -hdl_imu_data_->gyro_temp_accel[2].gyro;
        hdl_imu_data_->gyro_temp_accel_xyz[2].temp = hdl_imu_data_->gyro_temp_accel[2].temp;
        hdl_imu_data_->gyro_temp_accel_xyz[2].accel_x = -hdl_imu_data_->gyro_temp_accel[0].accel_x * earth_gravity;
        hdl_imu_data_->gyro_temp_accel_xyz[2].accel_y = -hdl_imu_data_->gyro_temp_accel[1].accel_x * earth_gravity;
        memcpy(hdl_imu_data_->NMEA, dataPacket->NMEA, 72);
        //////////////////////////////////////////////
        /*    memcpy(hdl_imu_data_->NMEA,dataPacket->NMEA,72);

            hdl_imu_data_->Giro1 = (((0x0800) & dataPacket->Giro1) ? (-1.0)*((double)(((unsigned short)(~(dataPacket->Giro1 | 0xF800)))+1)) : (dataPacket->Giro1) );
            hdl_imu_data_->Giro1 = (double) hdl_imu_data_->Giro1  * 0.09766;
            hdl_imu_data_->Temp1 = ((double)((0x0fff) & dataPacket->Temp1))*0.1453+25;
            hdl_imu_data_->Acel1X = ((double)((0x0fff) & dataPacket->Acel1X))*0.001221;
            hdl_imu_data_->Acel1Y = ((double)((0x0fff) & dataPacket->Acel1Y))*0.001221;

            hdl_imu_data_->Giro2 = (((0x0800) & dataPacket->Giro2) ? (-1.0)*((double)(((unsigned short)(~(dataPacket->Giro2 | 0xF800)))+1)) : (dataPacket->Giro2) );
            hdl_imu_data_->Giro2 = (double) hdl_imu_data_->Giro2  * 0.09766;
            hdl_imu_data_->Temp2 = ((double)((0x0fff) & dataPacket->Temp2))*0.1453+25;
            hdl_imu_data_->Acel2X = ((double)((0x0fff) & dataPacket->Acel2X))*0.001221;
            hdl_imu_data_->Acel2Y = ((double)((0x0fff) & dataPacket->Acel2Y))*0.001221;

            hdl_imu_data_->Giro3 = (((0x0800) & dataPacket->Giro3) ? (-1.0)*((double)(((unsigned short)(~(dataPacket->Giro3 | 0xF800)))+1)) : (dataPacket->Giro3) );
            hdl_imu_data_->Giro3 = (double) hdl_imu_data_->Giro3  * 0.09766;
            hdl_imu_data_->Temp3 = ((double)((0x0fff) & dataPacket->Temp3))*0.1453+25;
            hdl_imu_data_->Acel3X = ((double)((0x0fff) & dataPacket->Acel3X))*0.001221;
            hdl_imu_data_->Acel3Y = ((double)((0x0fff) & dataPacket->Acel3Y))*0.001221;

            // sybc time stamp
            if(gpsFile_!=NULL)
                fprintf(gpsFile_,"%u\t%f\t%s\n",dataPacket->gpsTimestamp,hdl_imu_data_->Giro3,hdl_imu_data_->NMEA);
            //

              std::cout << "\n\nRecibimos un paquete del GPS/IMU " << " timeStamp " << dataPacket->gpsTimestamp << "\ncadena GPS-->" << hdl_imu_data_->NMEA << std::endl;
                            std::cout << "\n\nGiroscopo Gyro1 " << Giro1 << " Temp1 " << ((float)((0x0fff) & dataIMUPacket->Temp1))*0.1453+25 << " Acel1X " <<  ((float)((0x0fff) & dataIMUPacket->Acel1X))*0.001221 << " Acel1Y " << ((float)((0x0fff) & dataIMUPacket->Acel1Y))*0.001221 << std::endl;
                std::cout << "\n\nGiroscopo Gyro2 " << Giro2 << " Temp2 " << ((float)((0x0fff) & dataIMUPacket->Temp2))*0.1453+25 << " Acel2X " <<  ((float)((0x0fff) & dataIMUPacket->Acel2X))*0.001221 << " Acel2Y " << ((float)((0x0fff) & dataIMUPacket->Acel2Y))*0.001221 << std::endl;
                std::cout << "\n\nGiroscopo Gyro3 " << Giro3 << " Temp3 " << ((float)((0x0fff) & dataIMUPacket->Temp3))*0.1453+25 << " Acel3X " <<  ((float)((0x0fff) & dataIMUPacket->Acel3X))*0.001221 << " Acel3Y " << ((float)((0x0fff) & dataIMUPacket->Acel3Y))*0.001221 << std::endl;


        */
        fireCurrentGPS();
        free(data_gps);
    }

}

int16_t HDLGrabberDriver::convert12bit2int16(uint16_t v) {
    v = v & 0x0fff;
    int16_t r = v;
    if (r > 2047)
        r = -((~(r--)) & 0x0fff);
    return r;
}

void HDLGrabberDriver::readHDLPacketsFromSocket() {
    unsigned char data[1500];
    udp::endpoint sender_endpoint;

    while (!terminate_read_packet_thread_ && hdl_read_socket_->is_open()) {
        size_t length = hdl_read_socket_->receive_from(boost::asio::buffer(data, 1500), sender_endpoint);

        /*	std::cout << "\n\nRecibimos un paquete tamaño " << length << " dirección " << sender_endpoint.address () << " puerto " << sender_endpoint.port () << "source address filter " << source_address_filter_.to_string() << " source port filter " << source_port_filter_ << "listener port " << udp_listener_endpoint_.port() << std::endl; */

        if (isAddressUnspecified(source_address_filter_) ||
            (source_address_filter_ == sender_endpoint.address() && source_port_filter_ == sender_endpoint.port())) {

            //		 HDLDataPacket *dataPacket = reinterpret_cast<HDLDataPacket *> (data);
            //	         std::cout << "\n\nRecibimos un paquete del laser con tamaño " << length << " timeStamp " << dataPacket->gpsTimestamp << std::endl;
            enqueueHDLPacket(data, length);
        }
    }
}

void HDLGrabberDriver::readGPSPacketsFromSocket() {


    unsigned char data_gps[600];
    udp::endpoint sender_gps_endpoint;

    while (!terminate_read_packet_thread_ && hdl_read_gps_socket_->is_open()) {
        size_t length_gps = hdl_read_gps_socket_->receive_from(boost::asio::buffer(data_gps, 600), sender_gps_endpoint);

        //	std::cout << "\n\nRecibimos un paquete tamaño " << length_gps << " dirección " << sender_gps_endpoint.address () << " puerto " << sender_gps_endpoint.port () << "source address filter " << source_address_filter_.to_string() << " source port filter " << source_port_filter_ << "listener port " << udp_listener_gps_endpoint_.port() << std::endl;

        /*    if (length_gps > 0 && sender_gps_endpoint.port () == 8308)
            {
            HDLIMUPacket *dataIMUPacket = reinterpret_cast<HDLIMUPacket *> (data_gps);

                float Giro1 = (((0x0800) & dataIMUPacket->Giro1) ? (-1.0)*((float)(((unsigned short)(~(dataIMUPacket->Giro1 | 0xF800)))+1)) : (dataIMUPacket->Giro1) );
                Giro1 = (float) Giro1  * 0.09766;
                float Giro2 = (((0x0800) & dataIMUPacket->Giro2) ? (-1.0)*((float)(((unsigned short)(~(dataIMUPacket->Giro2 | 0xF800)))+1)) : (dataIMUPacket->Giro2) );
                    Giro2 = (float) Giro2  * 0.09766;
                float Giro3 = (((0x0800) & dataIMUPacket->Giro3) ? (-1.0)*((float)(((unsigned short)(~(dataIMUPacket->Giro3 | 0xF800)))+1)) : (dataIMUPacket->Giro3) );
                Giro3 = (float) Giro3  * 0.09766;


        //		std::cout << "\n\nRecibimos un paquete del GPS/IMU con tamaño " << length_gps << " timeStamp " << dataIMUPacket->gpsTimestamp << "\ncadena GPS-->" << dataIMUPacket->NMEA << std::endl;
        /*		std::cout << "\n\nGiroscopo Gyro1 " << Giro1 << " Temp1 " << ((float)((0x0fff) & dataIMUPacket->Temp1))*0.1453+25 << " Acel1X " <<  ((float)((0x0fff) & dataIMUPacket->Acel1X))*0.001221 << " Acel1Y " << ((float)((0x0fff) & dataIMUPacket->Acel1Y))*0.001221 << std::endl;
                std::cout << "\n\nGiroscopo Gyro2 " << Giro2 << " Temp2 " << ((float)((0x0fff) & dataIMUPacket->Temp2))*0.1453+25 << " Acel2X " <<  ((float)((0x0fff) & dataIMUPacket->Acel2X))*0.001221 << " Acel2Y " << ((float)((0x0fff) & dataIMUPacket->Acel2Y))*0.001221 << std::endl;
                std::cout << "\n\nGiroscopo Gyro3 " << Giro3 << " Temp3 " << ((float)((0x0fff) & dataIMUPacket->Temp3))*0.1453+25 << " Acel3X " <<  ((float)((0x0fff) & dataIMUPacket->Acel3X))*0.001221 << " Acel3Y " << ((float)((0x0fff) & dataIMUPacket->Acel3Y))*0.001221 << std::endl;



                std::cout << "\n\nRecibimos un paquete tamaño " << length_gps << std::endl;
                enqueueHDLPacket (data_gps, length_gps);
            }
            */
        if (isAddressUnspecified(source_address_filter_) ||
            (source_address_filter_ == sender_gps_endpoint.address() &&
             source_port_filter_ == sender_gps_endpoint.port())) {

            //		 HDLDataPacket *dataPacket = reinterpret_cast<HDLDataPacket *> (data);
            //	         std::cout << "\n\nRecibimos un paquete del laser con tamaño " << length << " timeStamp " << dataPacket->gpsTimestamp << std::endl;
            enqueueHDLPacket(data_gps, length_gps);
        }
    }
}

void HDLGrabberDriver::setPcapFilter(std::string ip_address, int port) // in non port is set -1
{

    std::ostringstream stringStream;

    if (port > 0) {
        stringStream << " and src port " << port << " and src host " << ip_address;
    } else {
        stringStream << "and src host " << ip_address;
    }

    pcap_ip_port_filter_.assign(stringStream.str());
}

void HDLGrabberDriver::readPacketsFromPcap() {
    struct pcap_pkthdr *header;
    const unsigned char *data;

    // carlota get timestamp from pcap header
    HDLDataPacket *packet_data;
    HDLIMUPacket *data_gps;
    unsigned long long auxTimeMS;

    //
    char errbuff[PCAP_ERRBUF_SIZE];
    std::ostringstream stringStream;
    pcap_t *pcap = pcap_open_offline(pcap_file_name_.c_str(), errbuff);

    struct bpf_program filter;

    stringStream << "udp ";

    if (!isAddressUnspecified(source_address_filter_)) {
        stringStream << " and src port " << source_port_filter_ << " and src host "
                     << source_address_filter_.to_string();

        //   stringStream << " and src port " << 2368 << " and src host " << "172.29.21.9";
        //  stringStream << " and src port " << 2368 << " and src host " << "192.168.0.201";
    }
    // PCAP_NETMASK_UNKNOWN should be 0xffffffff, but it's undefined in older PCAP versions
    // stringStream << "and src host " << "192.168.0.201";
    //  stringStream << "and src host " << "172.29.21.9";
    // stringStream << "and src host " << "192.168.1.201";
    stringStream << pcap_ip_port_filter_;


    if (pcap_compile(pcap, &filter, stringStream.str().c_str(), 0, 0xffffffff) == -1) {
        PCL_WARN ("[pcl::HDLGrabber::readPacketsFromPcap] Issue compiling filter: %s.\n", pcap_geterr(pcap));
    } else if (pcap_setfilter(pcap, &filter) == -1) {
        PCL_WARN ("[pcl::HDLGrabber::readPacketsFromPcap] Issue setting filter: %s.\n", pcap_geterr(pcap));
    }

    std::cout << stringStream.str().c_str() << std::endl;


    struct timeval lasttime;
    unsigned long long uSecDelay;

    lasttime.tv_sec = 0;

    int returnValue = pcap_next_ex(pcap, &header, &data);

    while (returnValue >= 0 && !terminate_read_packet_thread_) {

        // otherwise continue
        if (lasttime.tv_sec == 0) {
            lasttime.tv_sec = header->ts.tv_sec;
            lasttime.tv_usec = header->ts.tv_usec;
        }
        if (lasttime.tv_usec > header->ts.tv_usec) {
            lasttime.tv_usec -= 1000000;
            lasttime.tv_sec++;
        }
        uSecDelay = ((header->ts.tv_sec - lasttime.tv_sec) * 1000000) +
                    (header->ts.tv_usec - lasttime.tv_usec);

        boost::this_thread::sleep(boost::posix_time::microseconds(uSecDelay));

        lasttime.tv_sec = header->ts.tv_sec;
        lasttime.tv_usec = header->ts.tv_usec;
        /*  struct timeval captureTime;
          gettimeofday(&captureTime,0);
          double auxTimeMS = captureTime.tv_usec/1000.0 + captureTime.tv_sec*1000.0;*/
        auxTimeMS = header->ts.tv_usec + header->ts.tv_sec * 1000000;
        if ((header->len - 42) == 1206) // laser
        {
            packet_data = reinterpret_cast<HDLDataPacket *> ((unsigned char *) data + 42);

            if (timeSyncFile_ != NULL) {
                // equivale a Controlador C4 (uint64_t)(auxTimeMS*1000.0)
                fprintf(timeSyncFile_, "%d\t%llu\t%u\n", 0, (unsigned long long) (auxTimeMS),
                        packet_data->gpsTimestamp);
            }
            // write time stamp

        } else if ((header->len - 42) == 512) // gps
        {
            data_gps = reinterpret_cast<HDLIMUPacket *> ((unsigned char *) data + 42);
            // write gps_data ->gpsTimestamp;
            if (timeSyncFile_ != NULL) {
                fprintf(timeSyncFile_, "%d\t%llu\t%u\n", 1, (unsigned long long) (auxTimeMS), data_gps->gpsTimestamp);
            }

        }


        // The ETHERNET header is 42 bytes long; unnecessary
        enqueueHDLPacket(data + 42, header->len - 42);
        // add conditional lock for pause reading

        returnValue = pcap_next_ex(pcap, &header, &data);
        if (is_pause_acquisition_on_) {
            //std::cout<<"En wait for pause"<<std::endl;
            waitForPauseEventOff();
        }
    }
    //
    std::cout << "End of PCAP file" << std::endl;

}

void HDLGrabberDriver::readPacketsFromPcap(unsigned long long uSecDelay) {
    struct pcap_pkthdr *header;
    const unsigned char *data;
    char errbuff[PCAP_ERRBUF_SIZE];

    pcap_t *pcap = pcap_open_offline(pcap_file_name_.c_str(), errbuff);

    struct bpf_program filter;
    std::ostringstream string_stream;

    string_stream << "udp ";
    if (!isAddressUnspecified(source_address_filter_)) {
        string_stream << " and src port " << source_port_filter_ << " and src host "
                      << source_address_filter_.to_string();
    }

    // PCAP_NETMASK_UNKNOWN should be 0xffffffff, but it's undefined in older PCAP versions
    if (pcap_compile(pcap, &filter, string_stream.str().c_str(), 0, 0xffffffff) == -1) {
        PCL_WARN ("[pcl::HDLGrabber::readPacketsFromPcap] Issue compiling filter: %s.\n", pcap_geterr(pcap));
    } else if (pcap_setfilter(pcap, &filter) == -1) {
        PCL_WARN ("[pcl::HDLGrabber::readPacketsFromPcap] Issue setting filter: %s.\n", pcap_geterr(pcap));
    }

    struct timeval lasttime;
    unsigned long long usec_delay;

    lasttime.tv_sec = 0;

    int returnValue = pcap_next_ex(pcap, &header, &data);

    while (returnValue >= 0 && !terminate_read_packet_thread_) {
        if (lasttime.tv_sec == 0) {
            lasttime.tv_sec = header->ts.tv_sec;
            lasttime.tv_usec = header->ts.tv_usec;
        }
        if (lasttime.tv_usec > header->ts.tv_usec) {
            lasttime.tv_usec -= 1000000;
            lasttime.tv_sec++;
        }
        usec_delay = ((header->ts.tv_sec - lasttime.tv_sec) * 1000000) +
                     (header->ts.tv_usec - lasttime.tv_usec);

        boost::this_thread::sleep(boost::posix_time::microseconds(usec_delay));

        lasttime.tv_sec = header->ts.tv_sec;
        lasttime.tv_usec = header->ts.tv_usec;

        // The ETHERNET header is 42 bytes long; unnecessary
        enqueueHDLPacket(data + 42, header->len - 42);

        returnValue = pcap_next_ex(pcap, &header, &data);
    }
}

void HDLGrabberDriver::enqueueHDLPacket(const unsigned char *data, std::size_t bytesReceived) {

    if (bytesReceived == 1206 && this->use_laser_) {
        unsigned char *dup = static_cast<unsigned char *> (malloc(bytesReceived * sizeof(unsigned char)));
        memcpy(dup, data, bytesReceived * sizeof(unsigned char));
        hdl_data_.enqueue(dup);


    } else if (bytesReceived == 512 && this->use_gps_) {

        unsigned char *dup = static_cast<unsigned char *> (malloc(bytesReceived * sizeof(unsigned char)));
        memcpy(dup, data, bytesReceived * sizeof(unsigned char));
        gps_data_.enqueue(dup);

    }
}

/////////////////////////////////////////////////////////////////////////////
void HDLGrabberDriver::stop() {

    //carlota
    is_pause_acquisition_on_ = false;
    is_set_pause_on_ = false;

    terminate_read_packet_thread_ = true;
    hdl_data_.stopQueue();
    gps_data_.stopQueue();

    if (hdl_read_packet_thread_ != NULL) {
        hdl_read_packet_thread_->interrupt();
        hdl_read_packet_thread_->join();
        delete hdl_read_packet_thread_;
        hdl_read_packet_thread_ = NULL;
    }

    if (GPS_read_packet_thread_ != NULL) {
        GPS_read_packet_thread_->interrupt();
        GPS_read_packet_thread_->join();
        delete GPS_read_packet_thread_;
        GPS_read_packet_thread_ = NULL;
    }

    if (queue_consumer_thread_ != NULL) {
        queue_consumer_thread_->join();
        delete queue_consumer_thread_;
        queue_consumer_thread_ = NULL;
    }

    if (queue_consumer_IMU_thread_ != NULL) {
        queue_consumer_IMU_thread_->join();
        delete queue_consumer_IMU_thread_;
        queue_consumer_IMU_thread_ = NULL;
    }

    if (hdl_read_socket_ != NULL) {
        delete hdl_read_socket_;
        hdl_read_socket_ = NULL;
    }
    if (hdl_read_gps_socket_ != NULL) {
        delete hdl_read_gps_socket_;
        hdl_read_gps_socket_ = NULL;
    }
    // sync time stamp
    if (stampFile_ != NULL)
        fclose(stampFile_);

    stampFile_ = NULL;
    if (gpsFile_ != NULL)
        fclose(gpsFile_);

    gpsFile_ = NULL;
    if (pclFile_ != NULL)
        fclose(pclFile_);

    pclFile_ = NULL;
    if (scanFile_ != NULL)
        fclose(scanFile_);
    scanFile_ = NULL;

    if (timeSyncFile_ != NULL)
        fclose(timeSyncFile_);
    timeSyncFile_ = NULL;

    std::cout << "HDL  stop" << std::endl;

}

/////////////////////////////////////////////////////////////////////////////
bool HDLGrabberDriver::isRunning() const {
    return (!hdl_data_.isEmpty() || (hdl_read_packet_thread_ != NULL &&
                                     !hdl_read_packet_thread_->timed_join(boost::posix_time::milliseconds(10))));
}

/////////////////////////////////////////////////////////////////////////////
std::string HDLGrabberDriver::getName() const {
    return (std::string("Velodyne High Definition Laser (HDL) Grabber"));
}

/////////////////////////////////////////////////////////////////////////////
float HDLGrabberDriver::getFramesPerSecond() const {
    return (0.0f);
}

bool HDLGrabberDriver::isAddressUnspecified(const boost::asio::ip::address &ipAddress) {
#if BOOST_VERSION >= 104700
    return (ipAddress.is_unspecified());
#else
    if (ipAddress.is_v4 ())
      return (ipAddress.to_v4 ().to_ulong() == 0);

    return (false);
#endif
}

// carlota
void HDLGrabberDriver::waitForPauseEventOff() {

    boost::unique_lock<boost::mutex> lock(hdl_pause_event_mutex_);
    while (is_pause_acquisition_on_) {
        hdl_pause_event_cond_var_.wait(lock);
    }

}

void HDLGrabberDriver::requestPauseOn() {

    is_set_pause_on_ = true;
}

void HDLGrabberDriver::setPauseEventOn() {


    //std::cout<<"En set pause On"<<std::endl;

    boost::unique_lock<boost::mutex> lock(hdl_pause_event_mutex_);
    is_pause_acquisition_on_ = true;
    lock.unlock();

}

void HDLGrabberDriver::setPauseEventOff() {

    //std::cout<<"En set pause Off"<<std::endl;
    is_set_pause_on_ = false;
    boost::unique_lock<boost::mutex> lock(hdl_pause_event_mutex_);
    is_pause_acquisition_on_ = false;
    hdl_pause_event_cond_var_.notify_all();
    lock.unlock();

}

void HDLGrabberDriver::fireCurrentSnapSweep(const unsigned short startAngle, const unsigned short endAngle,
                                            unsigned int sweepCount) {


    const float start = static_cast<float> (startAngle) / 100.0f;
    const float end = static_cast<float> (endAngle) / 100.0f;

    //printf("fireCurrentSnapSweep start=%0.2f\t end=%0.2f\t size=%ld\n",start,end,current_snap_sweep_xyzi_->size());
    if (snap_sweep_xyz_signal_->num_slots() > 0)
        snap_sweep_xyz_signal_->operator()(current_snap_sweep_xyz_, start, end, sweepCount);

    if (snap_sweep_xyzrgb_signal_->num_slots() > 0)
        snap_sweep_xyzrgb_signal_->operator()(current_snap_sweep_xyzrgb_, start, end, sweepCount);

    if (snap_sweep_xyzi_signal_->num_slots() > 0)
        snap_sweep_xyzi_signal_->operator()(current_snap_sweep_xyzi_, start, end, sweepCount);

}

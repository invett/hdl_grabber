#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nmea_msgs/Sentence.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <hdl_grabber/hdl_grabber_driver.h>
#include "hdl_msgs/getPCloudSweep.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>

#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/io/grabber.h>
#include <pcl/io/impl/synchronized_queue.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <string>
#include <iostream>
#include <iomanip>


#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

namespace hdl_grabber {

class HdlGrabberNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  HdlGrabberNodelet() {
  }
  virtual ~HdlGrabberNodelet() {
    stop();
	if(odom_data_sync_!=NULL)
		delete odom_data_sync_;
	if(odom_pose_sync_!=NULL)
		delete odom_pose_sync_;
  }

  void onInit() override {
    node = getNodeHandle();
    mt_node= getMTNodeHandle();
    private_nh = getPrivateNodeHandle();
    initializeParameters();

  // use private node handle to get parameters
    config_.frame_id = private_nh.param<std::string>("frame_id", "velodyne");
    
    std::string tf_prefix = tf::getPrefixParam(private_nh);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

    // get model name, validate string, determine packet rate
    config_.model= private_nh.param<std::string>("model", "HDL32E");
    
                    // packet frequency (Hz)

    if ((config_.model == "HDL64E"))    // generates 1333312 points per second
        {                                   // 1 packet holds 384 points
        config_.emodel = HDLGrabberDriver::HDL_64E;
        
        }
    else if (config_.model == "HDL32E")
        {
        config_.emodel = HDLGrabberDriver::HDL_32E;
        }
    else if (config_.model == "VLP32C")
        {
        
        config_.emodel = HDLGrabberDriver::VLP_32C;
        }
    else if (config_.model == "VLP16")
        {
        
        config_.emodel = HDLGrabberDriver::VLP_16;
        }
    else
        {
        ROS_ERROR_STREAM("unknown Velodyne LIDAR model: " << config_.model <<" default model will be used"); 
        config_.emodel = HDLGrabberDriver::HDL_32E;    
        }
        std::string deviceName(std::string("Velodyne ") + config_.model);
        NODELET_INFO_STREAM("config_.model" <<config_.model);

    std::string pcap_file;
    pcap_file= private_nh.param<std::string>("pcap", "");
    
    int udp_port;
    private_nh.param("port", udp_port, (int) HDLGrabberDriver::HDL_DATA_PORT);

    int udp_port_gps;
    private_nh.param("port_gps", udp_port_gps, (int) HDLGrabberDriver::HDL_GPSDATA_PORT);
    
    std::string calib_file;
    private_nh.param("calib_file", calib_file, std::string(""));

    std::string ipaddress;
    private_nh.param("ipaddress", ipaddress, std::string(""));

    bool isgps;
    private_nh.param("isgps", isgps, true);
    private_nh.param("ismapping", is_mapping_on_, true);

    std::string path;
    private_nh.param("datapath",path, std::string(""));
    std::string temp;
   


   if(!pcap_file.empty())
   {
     // get sync pose from file
     temp.clear();
     std::string odomname;
     private_nh.param("syncdata", temp, std::string(""));

     if(temp.empty())
        temp.assign("temp.txt");

        odomname=path;
        odomname.append(temp);
        ROS_WARN_STREAM("odomname : " <<odomname.c_str());
       if(!loadSyncData(odomname, is_mapping_on_))
       {
         sync_data_=false;
         ROS_INFO_STREAM("sync_data_=false : " <<sync_data_);

       }
       else
       {
         sync_data_=true;
         ROS_INFO_STREAM("sync_data_=true : " <<sync_data_);
       }

        std::string sweep_corr_file;
        private_nh.param("sweep_corr_file", sweep_corr_file, std::string(""));
        temp.clear();
        if(!sweep_corr_file.empty())
        {
        temp.assign(path);
        temp.append(sweep_corr_file);
        sweep_corr_file.assign(temp);
        }
        temp.clear();
        temp.assign(path);
        temp.append(pcap_file);
        pcap_file.assign(temp);
        ROS_WARN_STREAM("pcap file" <<pcap_file.c_str());
        ROS_WARN_STREAM("sweep Filte: " <<sweep_corr_file.c_str());
        input_.reset(new HDLGrabberDriver(calib_file,pcap_file,isgps,true,sweep_corr_file,config_.emodel));
        input_->setPcapFilter(ipaddress,-1);
        ROS_INFO_STREAM("ipaddress: " <<ipaddress.c_str());

        is_live_data_=false;

    }else{
        
        input_.reset(new HDLGrabberDriver(boost::asio::ip::address::from_string(ipaddress),udp_port,udp_port_gps,calib_file, true,true,config_.emodel));
        
        is_live_data_=true;
        

    }
    
    // Register a callback function that gets complete 360 degree sweeps.

        boost::function<void (const CloudConstPtr&)> cloud_cb = boost::bind (
            &HdlGrabberNodelet::cloud_callback, this, _1);
        boost::signals2::connection cloud_connection = input_->registerCallback (cloud_cb);

        boost::function<void(const CloudConstPtr&, float, float, unsigned int)> snap_cloud_cb =
            boost::bind(&HdlGrabberNodelet::snapScanCallback, this, _1, _2, _3,_4);
        boost::signals2::connection snap_cloud_conn = input_->registerCallback (snap_cloud_cb);

        boost::function<void (const boost::shared_ptr<const HDLGrabberDriver::HDLIMUData>&)> gps_cb = boost::bind (
            &HdlGrabberNodelet::getGPSCallback, this, _1);
        boost::signals2::connection gps_connection = input_->registerCallback (gps_cb);
        // pointcloud output topic

        std::string pcloud_topic;
        private_nh.param("pcloud_topic", pcloud_topic, std::string("velodyne_points"));
        output_ = mt_node.advertise<sensor_msgs::PointCloud2> (pcloud_topic, 4);
        // advertise topics
        //snap_pointcloud_output_ = mt_node.advertise<sensor_msgs::PointCloud2> ("velodyne_points", 10);
        service_pcl_ = mt_node.advertiseService("request_pcloud_sweep", &HdlGrabberNodelet::serverPCloudSweep, this);
        gps_data_.reset(new HDLGrabberDriver::HDLIMUData);  
        // point cloud request service for solving navigation tests and cloud by cloud pcap data reading
        request_pcl_srv_ = mt_node.serviceClient<hdl_msgs::getPCloudSweep>("request_pcloud_sweep");
        // imu_pub_ = node.advertise<sensor_msgs::Imu>("/vn100/imu_data", 1);
        imu_vel_pub_ = node.advertise<sensor_msgs::Imu>("/velodyne/imu_data", 1);
        
        if(is_enable_gps_)
        {
            nav_fix_pub_ = node.advertise<sensor_msgs::NavSatFix>("/velodyne/gps", 2);
        }
        else
        {
            nmea_pub_ = node.advertise<nmea_msgs::Sentence>("/velodyne/nmea_sentence",10);
        }
        
        //msf_pose_update_pub_ = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/msf_core/pose_after_update",2);
        //msf_pose_pub_ = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/msf_core/pose",2);
        odom_pub_=node.advertise<geometry_msgs::PoseStamped> ("drive/pose_odom", 1);
  
        pose_map_pub_=node.advertise<geometry_msgs::PoseStamped> ("drive/pose_map", 1);
        
        // SET MIN MAX FILTER DISTANCE

        // set rotation & traslation
        bool isTransOn;
        private_nh.param("isTransformOn", isTransOn, false);
        double x,y,z,roll,pitch,yaw;

        if(isTransOn)
        {
            private_nh.param("tras_x", x, 0.0); 
            private_nh.param("tras_y", y, 0.0);  
            private_nh.param("tras_z", z, 0.0);   
            private_nh.param("roll", roll, 0.0); 
            private_nh.param("pitch", pitch, 0.0);  
            private_nh.param("yaw", yaw, 0.0);  
            //////////////////// SET TRANSFORM  ////////////
            // setTransformationParameters(x,y,z,roll,pitch,yaw)
            //input_->setTransformationParameters((-1.32),0.13, (-0.1), M_PI, 0, (M_PI + 0.09));
            input_->setTransformationParameters(x,y, z, roll,pitch,yaw);
            ////////////////////////////////////////////////
        }
        double minDM, maxDM;
        private_nh.param("minFilterDistance", minDM, 0.0);
        private_nh.param("maxFilterDistance", maxDM, 150.0);
        //////////////////// SET MAX MIN DISTANCE //////
        input_->setMinDistanceThreshold(minDM);
        input_->setMaxDistanceThreshold(maxDM);
        ////////////////////////////////////////////////

        

        start();

  }
    void initializeParameters()
    {
        
            is_running_ =false;
            is_mapping_on_=true;
            is_first_cloud_ =0;
            is_live_data_ =false;
            acquisition_thread_=NULL;
            snap_thread_=NULL;
            gps_thread_=NULL;
            sync_data_=false;
            odom_data_sync_=NULL;
            iodom_samples_=0;
            is_new_sweep_cloud_=false;
            is_new_cloud_=false;
            odom_data_sync_=NULL;
            odom_pose_sync_=NULL;
            imu_pose_sync_ = NULL;
            gps_data_sync_ = NULL;
            is_IMU_on_ = false;
            is_new_gps_imu_ = false;
            is_enable_gps_ = false;
    }

    bool loadSyncData(std::string fname,bool ismapping){ // ismapping or localizing{
    int icount =1,itt=0;
    iodom_samples_=0;
    int nums =1;
    FILE *odom_file=NULL;

    
    int temp;
        odom_file = fopen(fname.c_str(),"r");
        if(odom_file ==NULL){
            

            iodom_samples_ =10000;
            odom_data_sync_=(st_sync_data_ *)realloc(odom_data_sync_,(icount*iodom_samples_*sizeof(st_sync_data_)));

            odom_pose_sync_=(st_pose_data_*)malloc(iodom_samples_*sizeof(st_pose_data_));
            map_pose_sync_=(st_pose_data_*)malloc(iodom_samples_*sizeof(st_pose_data_));

            for (itt=0; itt < iodom_samples_;itt++)
            {
                odom_pose_sync_[itt].timestamp=0;
                map_pose_sync_[itt].timestamp=0;

                odom_pose_sync_[itt].x=0;
                odom_pose_sync_[itt].y=0;
                odom_pose_sync_[itt].z=0;
                odom_pose_sync_[itt].yaw=0;
                odom_pose_sync_[itt].roll = 0;
                odom_pose_sync_[itt].pitch = 0;
                odom_pose_sync_[itt].sweep_counter= 0;

                map_pose_sync_[itt].x=0;
                map_pose_sync_[itt].y=0;
                map_pose_sync_[itt].z=0;
                map_pose_sync_[itt].yaw=0;
                map_pose_sync_[itt].roll = 0;
                map_pose_sync_[itt].pitch =  0;
                map_pose_sync_[itt].sweep_counter= 0;
            }


            sync_data_=false;

        }else{
            
            odom_data_sync_=(st_sync_data_ *)malloc(icount*100*sizeof(st_sync_data_));
            

            while( nums>0 ){

                
                    if(iodom_samples_== (icount*100)){
                        icount++;
                        odom_data_sync_=(st_sync_data_ *)realloc(odom_data_sync_,(icount*100*sizeof(st_sync_data_)));


                    }
                 nums= fscanf(odom_file , "%d\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
                    &odom_data_sync_[iodom_samples_].sweep_counter,&odom_data_sync_[iodom_samples_].timestamp,
                    &odom_data_sync_[iodom_samples_].x, &odom_data_sync_[iodom_samples_].y,
                    &odom_data_sync_[iodom_samples_].yaw, &odom_data_sync_[iodom_samples_].speed,
                    &odom_data_sync_[iodom_samples_].east,&odom_data_sync_[iodom_samples_].north,
                    &odom_data_sync_[iodom_samples_].yaw_absolute,
                    &odom_data_sync_[iodom_samples_].pitch_absolute, &odom_data_sync_[iodom_samples_].roll_absolute,
                    &odom_data_sync_[iodom_samples_].z  );
                if(nums>0)
                    iodom_samples_++;
               

                
            }
            NODELET_INFO_STREAM("LoadSyncData 1");
            odom_pose_sync_=(st_pose_data_*)malloc(iodom_samples_*sizeof(st_pose_data_));
            map_pose_sync_=(st_pose_data_*)malloc(iodom_samples_*sizeof(st_pose_data_));

            for (itt=0;itt<iodom_samples_;itt++){
                odom_pose_sync_[itt].timestamp=odom_data_sync_[itt].timestamp;
                map_pose_sync_[itt].timestamp=odom_data_sync_[itt].timestamp;
                    // datoa para imu en velodyne frame DOWN roll = -pi wrt to velodyne
                    odom_pose_sync_[itt].x=odom_data_sync_[itt].x;
                    odom_pose_sync_[itt].y=odom_data_sync_[itt].y;
                    odom_pose_sync_[itt].z=odom_data_sync_[itt].z; // normalizado en la pose0
                    odom_pose_sync_[itt].yaw=odom_data_sync_[itt].yaw;
                    odom_pose_sync_[itt].roll = odom_data_sync_[itt].roll_absolute;
                    odom_pose_sync_[itt].pitch = odom_data_sync_[itt].pitch_absolute;
                    odom_pose_sync_[itt].sweep_counter= odom_data_sync_[itt].sweep_counter;
                //  odom_pose_sync_[itt].frame_id.assign("/odom");

                // odom_pose_sync_[itt].frame_id="/odom";
                // datos para la pose velodyne frame UP in velodyne frame
                    map_pose_sync_[itt].x=odom_data_sync_[itt].east;
                    map_pose_sync_[itt].y=odom_data_sync_[itt].north;
                    map_pose_sync_[itt].z=odom_data_sync_[itt].z; // normalizado en la pose0
                    map_pose_sync_[itt].yaw=odom_data_sync_[itt].yaw_absolute;
                    map_pose_sync_[itt].roll = odom_data_sync_[itt].roll_absolute;
                    map_pose_sync_[itt].pitch = odom_data_sync_[itt].pitch_absolute;
                    map_pose_sync_[itt].sweep_counter= odom_data_sync_[itt].sweep_counter;

            }
            
            sync_data_=true;
            fclose(odom_file);
            ROS_ERROR("ODOM FILE  ON");
            // get IMU data
            is_IMU_on_ = false;
            std::string imufile;
            imufile.assign(fname,0,fname.length() - 4);
            imufile.append("_IMU_data.txt");
            odom_file = NULL;
            odom_file = fopen(imufile.c_str(),"r");
            iimu_samples_ = 0;
            icount =1;
            imu_pose_sync_ =(st_imu_data_*)malloc(icount*100*sizeof(st_imu_data_));

        if(odom_file!=NULL)
        {
            int nums =1;
            while( nums>0 ){

            
                if(iimu_samples_== (icount*100)){
                icount++;
                imu_pose_sync_ =(st_imu_data_*)realloc(imu_pose_sync_,(icount*100*sizeof(st_imu_data_)));
                }
                    nums =fscanf(odom_file , "%d\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
                    &imu_pose_sync_[iimu_samples_].sweep_counter,&imu_pose_sync_[iimu_samples_].timestamp,
                    &imu_pose_sync_[iimu_samples_].accx, &imu_pose_sync_[iimu_samples_].accy,&imu_pose_sync_[iimu_samples_].accz,
                    &imu_pose_sync_[iimu_samples_].gyrox, &imu_pose_sync_[iimu_samples_].gyroy,&imu_pose_sync_[iimu_samples_].gyroz,
                    &imu_pose_sync_[iimu_samples_].yaw, &imu_pose_sync_[iimu_samples_].pitch,&imu_pose_sync_[iimu_samples_].roll);
                
                if(nums>0)
                  iimu_samples_++;
                

           
            
            
            }
            fclose(odom_file);
            imu_pose_sync_ =(st_imu_data_*)realloc(imu_pose_sync_,(iimu_samples_*sizeof(st_imu_data_)));
            is_IMU_on_ = true;
            ROS_ERROR("VelodyneHDLDriver IMU ON");

        }
        // get GPS data
        is_enable_gps_=false;
        std::string  gpsfile;
        gpsfile.assign(fname,0,fname.length() - 4);
        gpsfile.append("_GPS_data.txt");
        odom_file = NULL;
        odom_file = fopen(gpsfile.c_str(),"r");
        if(odom_file!=NULL)
        {
            gps_data_sync_ =(st_gps_data*)malloc(iodom_samples_*sizeof(st_gps_data));
                for (itt=0;itt<iodom_samples_;itt++)
                {
                    fscanf(odom_file , "%d\t%u\t%f\t%f\t%f\t%lf\t%d\n",
                    &gps_data_sync_[itt].sweep_counter,&gps_data_sync_[itt].timestamp,
                    &gps_data_sync_[itt].latitude, &gps_data_sync_[itt].longitude,&gps_data_sync_[itt].altitude,
                    &gps_data_sync_[itt].GPS_UTC_time_usec, &gps_data_sync_[itt].gps_status);
                }
                fclose(odom_file);
                is_enable_gps_ = true;
                ROS_ERROR("External GPS ON");

        }
        }

        if(odom_data_sync_!=NULL)
        delete odom_data_sync_;
        odom_data_sync_=NULL;

        return(sync_data_);
    }


    void cloud_callback (const CloudConstPtr& cloud)
    {

    boost::mutex::scoped_lock lock (cloud_mutex_);
    cloud_= cloud;
    is_new_cloud_=true;

    }
    /** start the device
     *
     *  @returns true unless end of file reached
     */
    bool start(void)
    {
        if(!is_running_){
            input_->start();

            is_running_=true;
        if(is_live_data_== false)
        {
    //    ROS_ERROR("VelodyneHDLDriver::runSyncAcquisition");
        acquisition_thread_ = new boost::thread (boost::bind (&HdlGrabberNodelet::runSyncAcquisition, this));
        }

        else
        { // if  is_live && non sync data nor buscan_imu
    //  ROS_ERROR("VelodyneHDLDriver::runPCloudAcquisition");
        acquisition_thread_ = new boost::thread (boost::bind (&HdlGrabberNodelet::runPCloudAcquisition, this));
        }


            snap_thread_ = new boost::thread (boost::bind (&HdlGrabberNodelet::snapScan, this));
            gps_thread_=new boost::thread (boost::bind (&HdlGrabberNodelet::runGPSData, this));
        }



    return true;
    }


    void runPCloudAcquisition()
    {
    ros::Time last_time =ros::Time::now();
    double tdiff=0;
    while (this->is_running_)
        {

        CloudConstPtr cloud;
        Cloud::Ptr cloud_ptr (new Cloud);
        // See if we can get a cloud
        //

        if (this->cloud_mutex_.try_lock ())
        {

            if(this->cloud_ && this->is_new_cloud_==true) {
            pcl::copyPointCloud(*(this->cloud_),*cloud_ptr);
            //this->is_new_cloud_=false;

            }

            this->cloud_mutex_.unlock ();
        }

        if (!cloud_ptr->empty() && this->is_new_cloud_==true)
        {

        //    ROS_ERROR("VelodyneHDLDriver::runSyncAcquisition 1");
            // copy
            this->is_new_cloud_=false;
            // copy
            sensor_msgs::PointCloud2 output;
            pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
            toPCLPointCloud2 (*cloud_ptr, *cloud2);
            pcl_conversions::toPCL(ros::Time::now(),cloud2->header.stamp);
            pcl_conversions::fromPCL(*cloud2, output);

            output.header.frame_id =this->config_.frame_id;// "/velodyne";
            this->output_.publish(output);
            tdiff = output.header.stamp.toSec() -last_time.toSec();
         //   ROS_WARN_STREAM("time betweenn pcloudds "<< tdiff);
            last_time = output.header.stamp;
            // write sync file
            // get las data available from socket
            // write all data + cloud_ptr->header.stamp

        }
        boost::this_thread::sleep (boost::posix_time::microseconds (100));


        }

    }


    void runSyncAcquisition()
    {
    int sync_idx=-1,ii;

    geometry_msgs::PoseStamped drive_odom_msg;
    geometry_msgs::PoseStamped drive_map_msg;
    nav_msgs::Odometry drive_nav_odom;
    char stemp[250];
    sensor_msgs::Imu msg_imu;
    sensor_msgs::NavSatFix msg_fix;
    ros::Time gps_init,imu_init, imu_now,imu_last;
    double diff_time =0;
    //msf moc
    geometry_msgs::PoseWithCovarianceStamped msf_pose;
    geometry_msgs::PoseWithCovarianceStamped msf_pose_update;
    bool is_first_pose=true;
    int id_sync_imu=-1,id_imu=0;;
    tf::Quaternion q,qmap;
    imu_last = ros::Time::now();
        while (this->is_running_)
            {

                CloudConstPtr cloud;
                Cloud::Ptr cloud_ptr (new Cloud);
                // See if we can get a cloud

                if (this->cloud_mutex_.try_lock ())
                {

            if(this->cloud_ && this->is_new_cloud_==true && this->is_new_sweep_cloud_!=true) {
                    pcl::copyPointCloud(*(this->cloud_),*cloud_ptr);
                    //this->is_new_cloud_=false;

                    }

                this->cloud_mutex_.unlock ();
                }

                gps_init = ros::Time::now();
                if (!cloud_ptr->empty() && this->is_new_cloud_==true)
                {

                    // copy
            this->is_new_cloud_=false;
            imu_init = ros::Time::now();
            // copy
            sensor_msgs::PointCloud2 output;
            pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
            toPCLPointCloud2 (*cloud_ptr, *cloud2);
            pcl_conversions::toPCL(ros::Time::now(),cloud2->header.stamp);
            pcl_conversions::fromPCL(*cloud2, output);

            output.header.frame_id = this->config_.frame_id;
            drive_odom_msg.header.stamp=output.header.stamp;
            drive_map_msg.header.stamp=output.header.stamp;    
            drive_nav_odom.header.stamp =   output.header.stamp;   

            if(this->sync_data_)
            {
            // get data from pose file
            

            for (ii=0;ii<this->iodom_samples_;ii++){

                    if(this->odom_pose_sync_[ii].sweep_counter==(unsigned int)cloud_ptr->header.seq)
                    {
                    sync_idx=ii; 
                    id_sync_imu = this->odom_pose_sync_[ii].sweep_counter;    
                    ROS_WARN_STREAM("odom OK num cloud sync == " <<cloud_ptr->header.seq);             
                    // odom  pose stampend
                    drive_odom_msg.pose.position.x=this->odom_pose_sync_[ii].x;
                    drive_odom_msg.pose.position.y=this->odom_pose_sync_[ii].y;
                    drive_odom_msg.pose.position.z=this->odom_pose_sync_[ii].z;
                    q.setRPY(this->odom_pose_sync_[ii].roll, this->odom_pose_sync_[ii].pitch, this->odom_pose_sync_[ii].yaw);
                    drive_odom_msg.pose.orientation.x= q.x() ;
                    drive_odom_msg.pose.orientation.y= q.y() ;
                    drive_odom_msg.pose.orientation.z= q.z() ;
                    drive_odom_msg.pose.orientation.w= q.w() ;
                    //drive_odom_msg.header.frame_id = this->odom_pose_sync_[ii].frame_id;
                    if(this->is_mapping_on_){
                    drive_odom_msg.header.frame_id="/odom";//"/map";
                    }else{
                    drive_odom_msg.header.frame_id="/odom";
                    }
                    //
                    // map pose stampend
                    drive_map_msg.pose.position.x=this->map_pose_sync_[ii].x;
                    drive_map_msg.pose.position.y=this->map_pose_sync_[ii].y;
                    drive_map_msg.pose.position.z=this->map_pose_sync_[ii].z;
                    qmap.setRPY(this->map_pose_sync_[ii].roll, this->map_pose_sync_[ii].pitch, this->map_pose_sync_[ii].yaw);
                    drive_map_msg.pose.orientation.x= qmap.x() ;
                    drive_map_msg.pose.orientation.y= qmap.y() ;
                    drive_map_msg.pose.orientation.z= qmap.z() ;
                    drive_map_msg.pose.orientation.w= qmap.w() ;
                    //drive_map_msg.header.frame_id = this->map_pose_sync_[ii].frame_id; //? map / odom??
                    if(this->is_mapping_on_){
                        drive_map_msg.header.frame_id="/odom";
                    }else{
                        drive_map_msg.header.frame_id="/map";
                    }
                    // if imu on

                   
                    if(this->is_enable_gps_)
                    {
                        msg_fix.header.stamp = output.header.stamp; //output.header.stamp;
                        msg_fix.header.frame_id = "base_link";
                        msg_fix.latitude = gps_data_sync_[ii].latitude;
                        msg_fix.longitude = gps_data_sync_[ii].longitude;
                        msg_fix.altitude = gps_data_sync_[ii].altitude;
                        msg_fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
                        msg_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
                        msg_fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;;
                        this->nav_fix_pub_.publish(msg_fix);
                        
                    }

                    //mfs_pose
                
                    if(is_first_pose)
                    {
                        
                        msf_pose_update.header.stamp = ros::Time::now(); 
                        msf_pose.pose.pose.position.x = this->odom_pose_sync_[ii].x;
                        msf_pose.pose.pose.position.y = this->odom_pose_sync_[ii].y;
                        msf_pose.pose.pose.position.z = this->odom_pose_sync_[ii].z;
                        msf_pose.pose.pose.orientation.x= q.x() ;
                        msf_pose.pose.pose.orientation.y= q.y() ;
                        msf_pose.pose.pose.orientation.z= q.z() ;
                        msf_pose.pose.pose.orientation.w= q.w() ; 
                        msf_pose.header.stamp =  output.header.stamp;//ros::Time::now(); 
                        msf_pose_update = msf_pose;
                        is_first_pose = false;
                    }else
                    {
                        msf_pose_update = msf_pose; // pose anterior (ii-1)
                        msf_pose_update.header.stamp = ros::Time::now();  
                        msf_pose.pose.pose.position.x = this->odom_pose_sync_[ii].x;
                        msf_pose.pose.pose.position.y = this->odom_pose_sync_[ii].y;
                        msf_pose.pose.pose.position.z = this->odom_pose_sync_[ii].z;
                        msf_pose.pose.pose.orientation.x= q.x() ;
                        msf_pose.pose.pose.orientation.y= q.y() ;
                        msf_pose.pose.pose.orientation.z= q.z() ;
                        msf_pose.pose.pose.orientation.w= q.w() ; 
                        msf_pose.header.stamp =  output.header.stamp; //::Time::now(); 
                    
                    }
                    break;

                    }
                }

                if(sync_idx >=0)
                {
    /*               if(this->is_IMU_on_)
                {
                    msg_imu.header.stamp = imu_init;
                    this->imu_pub_.publish(msg_imu);
                } */
                this->output_.publish(output);
                //this->odom_pub_.publish(drive_odom_msg);
                //this->pose_map_pub_.publish(drive_map_msg);  
                
                if(this->is_enable_gps_)
                { 
                    msg_fix.header.stamp = output.header.stamp;
                    this->nav_fix_pub_.publish(msg_fix);
                        
                }
//                 this->msf_pose_update_pub_.publish(msf_pose_update);
//                 this->msf_pose_pub_.publish(msf_pose);
                sync_idx=-1;
                }
                //this->output_.publish(output);
            }
            else
            {
                    
                this->output_.publish(output);
                
            }
        }
        
//                 // publish IMU 100 Hz
//         if(this->is_IMU_on_ && id_sync_imu>0)
//         {
//             imu_now = ros::Time::now();
//             diff_time = imu_now.toSec() -imu_last.toSec();
// 
//             
//             if((diff_time*1000)> 8)
//             {
//                 imu_last = imu_now;
//                 
//                 for (ii=id_imu;ii<this->iimu_samples_;ii++)
//                 {
//                     
//                     if(this->imu_pose_sync_[ii].sweep_counter == id_sync_imu )
//                     {
//                         //ROS_WARN_STREAM("Diff time "<< (diff_time*1000) <<"    id_sync_imu "<<id_sync_imu << "timestamp " <<this->imu_pose_sync_[ii].timestamp);
//                             msg_imu.linear_acceleration.x = (double)this->imu_pose_sync_[ii].accx;
//                             msg_imu.linear_acceleration.y = (double)this->imu_pose_sync_[ii].accy;
//                             msg_imu.linear_acceleration.z = (double)this->imu_pose_sync_[ii].accz;
//                             msg_imu.angular_velocity.x = (double)this->imu_pose_sync_[ii].gyrox;
//                             msg_imu.angular_velocity.y = (double)this->imu_pose_sync_[ii].gyroy;
//                             msg_imu.angular_velocity.z = (double)this->imu_pose_sync_[ii].gyroz;
//                             q.setRPY(this->imu_pose_sync_[ii].roll, this->imu_pose_sync_[ii].pitch, this->imu_pose_sync_[ii].yaw);
//                             msg_imu.orientation.w = q.w();
//                             msg_imu.orientation.x = q.x();
//                             msg_imu.orientation.y = q.y();
//                             msg_imu.orientation.z = q.z();
//                             msg_imu.header.stamp = ros::Time::now();
//                             msg_imu.header.frame_id = "/velodyne";
//                             this->imu_pub_.publish(msg_imu);
//                             id_imu =ii+1;
//                             break;
//                     }
//                             
//                 }
// 
//             }
//         }

        boost::this_thread::sleep (boost::posix_time::microseconds (300));

    }

    }

/** stop the device
 *
 *  @returns true unless end of file reached
 */
    bool stop(void)
    {

        if(is_running_){

            is_running_=false;
            if(gps_thread_!=NULL){
                gps_thread_->interrupt ();
                gps_thread_->join ();
                delete gps_thread_;
                gps_thread_ = NULL;
            }
            if (acquisition_thread_!= NULL)
            {
                acquisition_thread_->interrupt ();
                acquisition_thread_->join ();
                    delete acquisition_thread_;
                    acquisition_thread_ = NULL;
            }

            if (snap_thread_!= NULL)
            {
                snap_thread_->interrupt ();
                snap_thread_->join ();
                    delete snap_thread_;
                    snap_thread_ = NULL;
            }

            if(input_->isRunning ())
                input_->stop ();

        printf("STOP NODE\n");
        // if imu buscan
        // stop thread imu
        // stop thread buscan

        }
    return true;
    }


    void snapScan ()
    {
    int sync_idx=-1,ii;
    geometry_msgs::PoseStamped drive_odom_msg;
    geometry_msgs::PoseStamped drive_map_msg;
    hdl_msgs::getPCloudSweep call_srv;
    sensor_msgs::Imu msg_imu;
    nav_msgs::Odometry drive_nav_odom;
    char stemp[250];
    sensor_msgs::NavSatFix msg_fix;
    ros::Time gps_init,imu_init;
    //msf moc
    geometry_msgs::PoseWithCovarianceStamped msf_pose;
    geometry_msgs::PoseWithCovarianceStamped msf_pose_update;
    bool is_first_pose=true;
    int id_sync_imu=-1,id_imu=0, id_sync_imu_first = -1;
    tf::Quaternion q,qmap;
    ros::Time imu_now,imu_last;
    double diff_time =0, init_yaw =0,init_pitch=0, init_roll=0;
    imu_last = ros::Time::now();
    bool is_first_yaw = true;

        while (this->is_running_)
        {

            CloudConstPtr cloud;
            Cloud::Ptr cloud_ptr (new Cloud);
            // See if we can get a cloud
            gps_init = ros::Time::now();
            imu_init = ros::Time::now();
            if (this->snap_cloud_mutex_.try_lock ())
            {

                if(this->snap_cloud_ && this->is_new_sweep_cloud_) {
                    pcl::copyPointCloud(*(this->snap_cloud_),*cloud_ptr);
                    this->is_new_sweep_cloud_=false;
                }

            this->snap_cloud_mutex_.unlock ();
            }

            if (cloud_ptr->size()>0 )
            {
                        // get sync data

             if(this->sync_data_){
                // get data from pose file
                
                id_sync_imu =(int)cloud_ptr->header.seq;
                for (ii=0;ii<this->iodom_samples_;ii++){

                        if(this->odom_pose_sync_[ii].sweep_counter==(unsigned int)cloud_ptr->header.seq)
                        {
                    //	if(this->odom_pose_sync_[ii].timestamp==(unsigned int)cloud_ptr->header.stamp){// modificar para evitar drift timestamp
                        ROS_WARN_STREAM("odom OK num cloud sync == " <<cloud_ptr->header.seq);

                        sync_idx=ii;
                        id_sync_imu_first =this->odom_pose_sync_[ii].sweep_counter;
                        // odom  pose stampend
                        drive_odom_msg.pose.position.x=this->odom_pose_sync_[ii].x;
                        drive_odom_msg.pose.position.y=this->odom_pose_sync_[ii].y;
                        drive_odom_msg.pose.position.z=this->odom_pose_sync_[ii].z;
                        tf::Quaternion q= tf::createQuaternionFromRPY(this->odom_pose_sync_[ii].roll, this->odom_pose_sync_[ii].pitch, this->odom_pose_sync_[ii].yaw);
                        drive_odom_msg.pose.orientation.x= q.x() ;
                        drive_odom_msg.pose.orientation.y= q.y() ;
                        drive_odom_msg.pose.orientation.z= q.z() ;
                        drive_odom_msg.pose.orientation.w= q.w() ;
                        //drive_odom_msg.header.frame_id = this->odom_pose_sync_[ii].frame_id;
                            if(this->is_mapping_on_){
                            drive_odom_msg.header.frame_id="/odom";//"/map";
                            }else{
                            drive_odom_msg.header.frame_id="/odom";
                            }
                        //
                        // map pose stampend
                        drive_map_msg.pose.position.x=this->odom_pose_sync_[ii].x;//this->map_pose_sync_[ii].x;
                        drive_map_msg.pose.position.y=this->odom_pose_sync_[ii].y;//this->map_pose_sync_[ii].y;
                        drive_map_msg.pose.position.z=this->odom_pose_sync_[ii].z;//this->map_pose_sync_[ii].z;
                        tf::Quaternion qmap= tf::createQuaternionFromRPY(this->map_pose_sync_[ii].roll, this->map_pose_sync_[ii].pitch, this->map_pose_sync_[ii].yaw);
                        drive_map_msg.pose.orientation.x= qmap.x() ;
                        drive_map_msg.pose.orientation.y= qmap.y() ;
                        drive_map_msg.pose.orientation.z= qmap.z() ;
                        drive_map_msg.pose.orientation.w= qmap.w() ;
                        //drive_map_msg.header.frame_id = this->map_pose_sync_[ii].frame_id; //? map / odom??
                        if(this->is_mapping_on_){
                            drive_map_msg.header.frame_id="/odom";
                        }else{
                            drive_map_msg.header.frame_id="/map";
                        }

                        if(this->odom_pose_sync_[ii].timestamp >0)
                        {
                            if(this->is_enable_gps_)
                            {
                                msg_fix.header.stamp = gps_init; //output.header.stamp;
                                msg_fix.header.frame_id = "base_link";
                                msg_fix.latitude = gps_data_sync_[ii].latitude;
                                msg_fix.longitude = gps_data_sync_[ii].longitude;
                                msg_fix.altitude = gps_data_sync_[ii].altitude;
                                msg_fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
                                msg_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
                                msg_fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;;
                                this->nav_fix_pub_.publish(msg_fix);
                                
                            }
                        
                            if(is_first_pose)
                            {
                                
                                msf_pose_update.header.stamp = ros::Time::now(); 
                                msf_pose.pose.pose.position.x = this->odom_pose_sync_[ii].x + 1.09*cos(this->odom_pose_sync_[ii].yaw);
                                msf_pose.pose.pose.position.y = this->odom_pose_sync_[ii].y + 1.09*sin(this->odom_pose_sync_[ii].yaw);
                                msf_pose.pose.pose.position.z = this->odom_pose_sync_[ii].z;
                                msf_pose.pose.pose.orientation.x= q.x() ;
                                msf_pose.pose.pose.orientation.y= q.y() ;
                                msf_pose.pose.pose.orientation.z= q.z() ;
                                msf_pose.pose.pose.orientation.w= q.w() ; 
                                msf_pose.header.stamp =  ros::Time::now(); 
                                msf_pose_update = msf_pose;
                                is_first_pose = false;
                            }else
                            {
                                msf_pose_update = msf_pose; // pose anterior (ii-1)
                                msf_pose_update.header.stamp = ros::Time::now();  
                                msf_pose.pose.pose.position.x = this->odom_pose_sync_[ii].x + 1.09*cos(this->odom_pose_sync_[ii].yaw);
                                msf_pose.pose.pose.position.y = this->odom_pose_sync_[ii].y + 1.09*sin(this->odom_pose_sync_[ii].yaw);
                                msf_pose.pose.pose.position.z = this->odom_pose_sync_[ii].z;
                                msf_pose.pose.pose.orientation.x= q.x() ;
                                msf_pose.pose.pose.orientation.y= q.y() ;
                                msf_pose.pose.pose.orientation.z= q.z() ;
                                msf_pose.pose.pose.orientation.w= q.w() ; 
                                msf_pose.header.stamp =  ros::Time::now(); 
                            
                            }
                        }
                        break;

                        }
                    }

                } // this->sync_data_
                               // get IMU deskweing
                //     if(this->is_IMU_on_ && id_sync_imu>0)
                //     {

                //         Eigen::Vector3f ang_v(msg_imu.angular_velocity.x, msg_imu.angular_velocity.y, msg_imu.angular_velocity.z);
                //         ang_v *= -1;
                //         double scan_period =0.1;//= private_nh.param<double>("scan_period", 0.1);
                //         for(int i = 0; i < cloud_ptr->size(); i++) {
                //         const auto& pt = cloud_ptr->at(i);

                //         // TODO: transform IMU data into the LIDAR frame
                //         double delta_t = scan_period * static_cast<double>(i) / cloud_ptr->size();
                //         Eigen::Quaternionf delta_q(1, delta_t / 2.0 * ang_v[0], delta_t / 2.0 * ang_v[1], delta_t / 2.0 * ang_v[2]);
                //         Eigen::Vector3f pt_ = delta_q.inverse() * pt.getVector3fMap();            
                //         cloud_ptr->at(i).getVector3fMap() = pt_;
                //         }
                //         ROS_WARN("Desweing OK");
                // }

                sensor_msgs::PointCloud2 output;
                pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
                toPCLPointCloud2 (*cloud_ptr, *cloud2);
                pcl_conversions::toPCL(ros::Time::now(),cloud2->header.stamp);
                pcl_conversions::fromPCL(*cloud2, output);

                output.header.frame_id = this->config_.frame_id;
                drive_odom_msg.header.stamp=output.header.stamp;
                drive_map_msg.header.stamp=output.header.stamp;

                if(sync_idx>=0){

                    if(this->is_enable_gps_)
                    { 
                        msg_fix.header.stamp = output.header.stamp;
                        this->nav_fix_pub_.publish(msg_fix);
                            
                    }
                    msf_pose.header.stamp = output.header.stamp;
                    msf_pose_update.header.stamp = output.header.stamp;
//                     this->msf_pose_update_pub_.publish(msf_pose_update);
//                     this->msf_pose_pub_.publish(msf_pose);

                   this->odom_pub_.publish(drive_odom_msg);
                    this->pose_map_pub_.publish(drive_map_msg);
                    //this->snap_pointcloud_output_.publish(output);


                }else{

                    call_srv.request.num=1;
                    if(this->request_pcl_srv_.call(call_srv)){
                        ROS_WARN_STREAM("2 HDL driver Request PointCloud velodyne frame: " <<cloud_ptr->header.seq);
                    }else{
                        ROS_WARN("2 HDL driver Request PointCloud velodyne - failed");
                    }

                    //call for a new service
                } // if sync_idx>=0
                    //publish odometry
            } // cloud_ptr->size()>0
            
                   if(this->is_IMU_on_ && id_sync_imu>0)
                    {
                        imu_now = ros::Time::now();
                        diff_time = imu_now.toSec() -imu_last.toSec();

                        
                        if((diff_time*1000)> 8)
                        {
                            imu_last = imu_now;
                            
                            for (ii=id_imu;ii<this->iimu_samples_;ii++)
                            {
                                
                                if(this->imu_pose_sync_[ii].sweep_counter == id_sync_imu )
                                {
                                    //ROS_WARN_STREAM("Diff time "<< (diff_time*1000) <<"    id_sync_imu "<<id_sync_imu << "timestamp " <<this->imu_pose_sync_[ii].timestamp);
                                    if(is_first_yaw && id_sync_imu_first > 0)
                                    {
                                        ROS_WARN_STREAM("1 First yaw "<< init_yaw <<"    frame "<<id_sync_imu_first);
                                        is_first_yaw  = false;
                                        init_yaw  = 0;//this->imu_pose_sync_[ii].yaw;
                                        ROS_WARN_STREAM("First yaw "<< init_yaw <<"    frame "<<id_sync_imu_first);

                                    }
                                        msg_imu.linear_acceleration.x = (double)this->imu_pose_sync_[ii].accx;
                                        msg_imu.linear_acceleration.y = (double)this->imu_pose_sync_[ii].accy;
                                        msg_imu.linear_acceleration.z = (double)this->imu_pose_sync_[ii].accz;
                                        msg_imu.angular_velocity.x = (double)this->imu_pose_sync_[ii].gyrox;
                                        msg_imu.angular_velocity.y = (double)this->imu_pose_sync_[ii].gyroy;
                                        msg_imu.angular_velocity.z = (double)this->imu_pose_sync_[ii].gyroz;
                                        q.setRPY(this->imu_pose_sync_[ii].roll, this->imu_pose_sync_[ii].pitch, (this->imu_pose_sync_[ii].yaw -init_yaw));
                                        msg_imu.orientation.w = q.w();
                                        msg_imu.orientation.x = q.x();
                                        msg_imu.orientation.y = q.y();
                                        msg_imu.orientation.z = q.z();
                                        msg_imu.header.stamp = ros::Time::now();
                                        msg_imu.header.frame_id = "/velodyne";
                                        // this->imu_pub_.publish(msg_imu);
                                        id_imu =ii+1;
                                        break;
                                }
                                        
                            }

                        }
                    }
        
            // if(is_first_cloud_ < 3 && sync_idx>=0)
            // {
            //     call_srv.request.num=1;
            //     if(this->request_pcl_srv_.call(call_srv)){
            //     ROS_INFO("3 times HDL driver Request PointCloud velodyne");
            //     }else{
            //     ROS_WARN("3 HDL driver Request PointCloud velodyne - failed");
            //     }
            //     is_first_cloud_++;   
            //     boost::this_thread::sleep (boost::posix_time::microseconds (500));      
            // }
            sync_idx=-1;
            boost::this_thread::sleep (boost::posix_time::microseconds (300));

        } // while

    }



    void snapScanCallback (const CloudConstPtr& snap, float start, float end,unsigned int count){

        boost::mutex::scoped_lock lock (snap_cloud_mutex_);
        snap_cloud_= snap;
        this->is_new_sweep_cloud_=true;
    }

    bool serverPCloudSweep(hdl_msgs::getPCloudSweep::Request  &req,
        hdl_msgs::getPCloudSweep::Response &res){
        int num;

                if(req.num>0){
                    this->input_->setPauseEventOff();
                    this->input_->requestPauseOn();

                }else{
                    this->input_->setPauseEventOff();
                }
                res.status =req.num;


            return true;
    }
    void getGPSCallback(const boost::shared_ptr<const HDLGrabberDriver::HDLIMUData> & data){

        boost::mutex::scoped_lock lock (gps_mutex_);
        gps_data_= data;
        is_new_gps_imu_ = true;
    }
    void runGPSData(){

    std::string nmea_sentence;
    nmea_msgs::Sentence nmea_sentence_msg;
    sensor_msgs::Imu msg_imu;

        while (this->is_running_)
        {
            if (this->gps_mutex_.try_lock ())
            {
        
                if(this->gps_data_ && is_new_gps_imu_ == true) 
        {
            is_new_gps_imu_ = false;
            msg_imu.header.frame_id = this->config_.frame_id;
            msg_imu.header.stamp = ros::Time::now();

            msg_imu.linear_acceleration.x = (this->gps_data_->gyro_temp_accel_xyz[0].accel_x + this->gps_data_->gyro_temp_accel_xyz[0].accel_y)/2.0;
            msg_imu.linear_acceleration.y = (this->gps_data_->gyro_temp_accel_xyz[1].accel_x + this->gps_data_->gyro_temp_accel_xyz[1].accel_y)/2.0;
            msg_imu.linear_acceleration.z = (this->gps_data_->gyro_temp_accel_xyz[2].accel_x + this->gps_data_->gyro_temp_accel_xyz[2].accel_y)/2.0;

            msg_imu.angular_velocity.x = this->gps_data_->gyro_temp_accel_xyz[0].gyro;
            msg_imu.angular_velocity.y = this->gps_data_->gyro_temp_accel_xyz[1].gyro;
            msg_imu.angular_velocity.z = this->gps_data_->gyro_temp_accel_xyz[2].gyro;

            msg_imu.orientation_covariance[0] = -1;
            imu_vel_pub_.publish(msg_imu);
            nmea_sentence.assign((char*)gps_data_->NMEA);
            // get gps data with parserGPS function
            //	  ROS_ERROR_STREAM("GPS: "<<nmea_str.c_str());
            // === NMEA Sentence ===
            if(!this->is_enable_gps_)
            {  
            nmea_sentence = nmea_sentence.substr(0,nmea_sentence.length()-2);
            nmea_sentence_msg.sentence = nmea_sentence;
            nmea_sentence_msg.header.stamp = msg_imu.header.stamp;
            nmea_pub_.publish(nmea_sentence_msg);
            }


                }

            this->gps_mutex_.unlock ();
            }

        }
    }




  private:

    // configuration parameters
    struct
    {
      std::string frame_id;            ///< tf frame ID
      std::string model;               ///< device model name
      HDLGrabberDriver::VELODYNE_MODEL emodel;
    } config_;
    typedef struct
    {
      unsigned int timestamp;
      float x; // eastig
      float y; // northing
      float z; // altitud
      float speed;
      float yaw;
      float east;
      float north;
      float yaw_absolute;
      float pitch_absolute;
      float roll_absolute;
      unsigned int sweep_counter;
    }st_sync_data_;

  typedef struct
	{
	  unsigned int timestamp;
	  float x; // eastig
	  float y; // northing
    float z; // altitud
	  float yaw;
    float pitch;
    float roll;
    unsigned int sweep_counter;
	//  std::string frame_id;
	}st_pose_data_;
	

typedef struct
  {
    float accx; 
    float accy; 
    float accz; 
    float gyrox; 
    float gyroy; 
    float gyroz; 
    float yaw;
    float pitch;
    float roll;
    unsigned int sweep_counter;
    unsigned int timestamp; 
  }st_imu_data_;
  
  typedef struct
  {
    unsigned int sweep_counter;
    unsigned int timestamp;
    float latitude; 
    float longitude; 
    float altitude; 
    int gps_status; 
    double GPS_UTC_time_usec;
  }st_gps_data;

    // ROS
  ros::NodeHandle node;
  ros::NodeHandle mt_node;
  ros::NodeHandle private_nh;

  boost::shared_ptr<HDLGrabberDriver> input_;
  ros::Publisher output_;
  
  ros::ServiceServer service_pcl_;
  boost::mutex cloud_mutex_, snap_cloud_mutex_;
  CloudConstPtr cloud_,snap_cloud_;
  Cloud::Ptr cloud_ptr_ ;
  boost::shared_ptr<const HDLGrabberDriver::HDLIMUData> gps_data_;
  boost::mutex gps_mutex_;
  //hdl_grabber_driver::getPointCloudSnap pcl_srv_;
  //    ros::Publisher snap_pointcloud_output_;

  bool sync_data_;
  bool is_live_data_;

  bool is_running_;
  bool is_new_cloud_;
  bool is_new_sweep_cloud_;
  boost::thread *acquisition_thread_;
  boost::thread *snap_thread_;
  boost::thread *gps_thread_;  

  ros::ServiceClient request_pcl_srv_;
  

  // if is_live_data OFF & sync_data ON

  ros::Publisher odom_pub_;
  ros::Publisher pose_map_pub_;
  ros::Publisher pose_odom_pub_;
  st_sync_data_ *odom_data_sync_;
  st_pose_data_ *odom_pose_sync_;
  st_pose_data_ *map_pose_sync_;
  unsigned int iodom_samples_;
  unsigned int iimu_samples_;

  // ros::Publisher imu_pub_;
  st_imu_data_ *imu_pose_sync_;
  bool is_IMU_on_;
  // On-board GPS IMU 
  ros::Publisher imu_vel_pub_;
  ros::Publisher gps_vel_pub_;
  ros::Publisher nmea_pub_;
  bool is_new_gps_imu_;
  // HDL_slam prior poses
//   ros::Publisher msf_pose_update_pub_;
//   ros::Publisher msf_pose_pub_;
  ros::Subscriber sub_hdl_slam_odom_;

  // if is_live_data OFF
 
  int is_first_cloud_;
  // if is_live_data ON
  
  bool is_mapping_on_;
  bool is_sim_;
  //
  // Trimble GPS or Applanix
  ros::Publisher nav_fix_pub_;
  st_gps_data *gps_data_sync_;
  bool is_enable_gps_;
  };

}

PLUGINLIB_EXPORT_CLASS(hdl_grabber::HdlGrabberNodelet, nodelet::Nodelet)

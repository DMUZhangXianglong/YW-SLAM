#include "utility.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <Eigen/Dense>


using namespace gtsam;
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

class SensorProcess : public ParamServer
{
public:
    //rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::Subscription<inertiallabs_msgs::msg::GPSData>::SharedPtr subGPSData;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImu;
    queue<sensor_msgs::msg::Imu> qu;
    rclcpp::Subscription<inertiallabs_msgs::msg::INSData>::SharedPtr subINS_Data;
	rclcpp::Subscription<inertiallabs_msgs::msg::GPSData>::SharedPtr subGPS_Data;
    rclcpp::Subscription<inertiallabs_msgs::msg::GNSSData>::SharedPtr subGNSS_Data;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubINS_Data_Info;
    mutex gps_mutex, ins_mutex;
    deque< pair<builtin_interfaces::msg::Time, vector<double> > > gps_queue;
    deque<pair<builtin_interfaces::msg::Time, vector<int> >> gnss_queue;
    deque< pair<builtin_interfaces::msg::Time, vector<double> > > ins_queue;
    SensorProcess(const rclcpp::NodeOptions & options) :
            ParamServer("lio_sam_imuprocess", options){
        




        // subImu = create_subscription<sensor_msgs::msg::Imu>(
        //     "/Imu_data", qos_imu,
        //     std::bind(&SensorProcess::imuHandler, this, std::placeholders::_1));

        pubImu = create_publisher<sensor_msgs::msg::Imu>(
            "/imu/data", qos);

        subINS_Data = create_subscription<inertiallabs_msgs::msg::INSData>(
            "/Inertial_Labs/ins_data", 10,
            std::bind(&SensorProcess::INSDataHandler, this, std::placeholders::_1));

        subGPS_Data = create_subscription<inertiallabs_msgs::msg::GPSData>(
            "/Inertial_Labs/gps_data", 10,
            std::bind(&SensorProcess::GPSDataHandler, this, std::placeholders::_1));

        subGNSS_Data = create_subscription<inertiallabs_msgs::msg::GNSSData>(
        "/Inertial_Labs/gnss_data", 10,
        std::bind(&SensorProcess::GNSSDataHandler, this, std::placeholders::_1));

        pubINS_Data_Info= create_publisher<nav_msgs::msg::Odometry>(
            "/ins_data_info", qos);

        }
    void INSDataHandler(const inertiallabs_msgs::msg::INSData::SharedPtr msg_ins){
        std::lock_guard<std::mutex> lock(ins_mutex);
        //RCLCPP_INFO_STREAM(this->get_logger(), "INSDataHandler_in");
        inertiallabs_msgs::msg::INSData msgIn = *msg_ins;
        vector<double> ins_data{msgIn.llh.x, msgIn.llh.y, msgIn.llh.z, (360 - msgIn.ypr.x + 90) / 180.0 * M_PI, 0, 0}; // xyz yaw pitch row
        ins_queue.push_back({msgIn.header.stamp, ins_data});
        if(ins_queue.size() > 100) ins_queue.pop_front();
        //RCLCPP_INFO_STREAM(this->get_logger(), "INSDataHandler_out");

    }

    void GPSDataHandler(const inertiallabs_msgs::msg::GPSData::SharedPtr msg_gps){
        std::lock_guard<std::mutex> lock(gps_mutex);
        //RCLCPP_INFO_STREAM(this->get_logger(), "GPSDataHandler_in");
        inertiallabs_msgs::msg::GPSData msgIn = *msg_gps;

        vector<double> gps_data{msgIn.llh.x, msgIn.llh.y, msgIn.llh.z};
        gps_queue.push_back({msgIn.header.stamp, gps_data});
        if(gps_queue.size() > 100) gps_queue.pop_front();
        //RCLCPP_INFO_STREAM(this->get_logger(), "GPSDataHandler_out");

    }

    void GNSSDataHandler(const inertiallabs_msgs::msg::GNSSData::SharedPtr msg_gnss){
        std::lock_guard<std::mutex> lock1(gps_mutex);
        std::lock_guard<std::mutex> lock2(ins_mutex);
        // RCLCPP_INFO_STREAM(this->get_logger(), "GNSSDataHandler_in");
        inertiallabs_msgs::msg::GNSSData msgIn = *msg_gnss;
        vector<int> gnss_data{msgIn.gnss_info_1, msgIn.gnss_info_2};
        gnss_queue.push_back({msgIn.header.stamp, gnss_data});
        // 使用ins_data
        if(useINS == true){
            if((!gnss_queue.empty()) && (!ins_queue.empty())){
                double ins_front = stamp2Sec(ins_queue.front().first);
                double gnss_front = stamp2Sec(gnss_queue.front().first);
                // ins队列的数据比较多
                if(ins_front < gnss_front - 0.01){
                    while(!ins_queue.empty() && stamp2Sec(ins_queue.front().first) < stamp2Sec(gnss_queue.front().first)-0.01){
                        ins_queue.pop_front();
                    }
                    if(ins_queue.empty()){
                    }
                }
                else if(gnss_front < ins_front - 0.01){
                    while(!gnss_queue.empty() && (stamp2Sec(gnss_queue.front().first) < stamp2Sec(ins_queue.front().first)-0.01)){
                        gnss_queue.pop_front();
                    }
                    if(gnss_queue.empty()){
                    }
                }
                if(gnss_queue.empty()) return;
                if(ins_queue.empty() || abs(stamp2Sec(ins_queue.front().first) - stamp2Sec(gnss_queue.front().first)) > 0.001){
                    RCLCPP_WARN_STREAM(this->get_logger(), "TIME NOT SYNC!!!");
                    return;
                }
                else{
                    while((!gnss_queue.empty()) && (!ins_queue.empty())){
                        nav_msgs::msg::Odometry gnss_odom;
                        auto gnss_front = gnss_queue.front();
                        auto ins_front = ins_queue.front();
                        gnss_odom.header.stamp = gnss_front.first;

                        gnss_odom.pose.pose.position.x = ins_front.second[0];
                        gnss_odom.pose.pose.position.y = ins_front.second[1];
                        gnss_odom.pose.pose.position.z = ins_front.second[2];
                        tf2::Quaternion q;
                        q.setRPY(ins_front.second[5], ins_front.second[4], ins_front.second[3]);
                        gnss_odom.pose.pose.orientation.w = q.w();
                        gnss_odom.pose.pose.orientation.x = q.x();
                        gnss_odom.pose.pose.orientation.y = q.y();
                        gnss_odom.pose.pose.orientation.z = q.z();

                        double conv_gnss = 1.0;
                        if(gnss_front.second[0] == 4)
                        {
                            conv_gnss = 0.0001;
                        }
                        else if (gnss_front.second[0] == 5)
                        {
                            conv_gnss = 0.01;
                        }
                        else{
                            conv_gnss = 2.0;
                        }
                        
                        gnss_odom.pose.covariance[0] = conv_gnss;
                        gnss_odom.pose.covariance[7] = conv_gnss;
                        gnss_odom.pose.covariance[14] = conv_gnss;

                        gnss_odom = insConverter(gnss_odom);

                        pubINS_Data_Info->publish(gnss_odom);
                        RCLCPP_INFO_STREAM(this->get_logger(), "pub_ins_data_info");

                        gnss_queue.pop_front();
                        ins_queue.pop_front();
                    }
                }
            }

        }
        else{
            //RCLCPP_INFO_STREAM(this->get_logger(), "PLEASE SET UP THE useINS!!!");
        }

        // RCLCPP_INFO_STREAM(this->get_logger(), "GNSSDataHandler_out");

    }

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg){
        sensor_msgs::msg::Imu thisImu = *imuMsg;
        static bool first = true;
        static bool finish = false;
        static sensor_msgs::msg::Imu lastImu;
        sensor_msgs::msg::Imu newImu = thisImu;
        static sensor_msgs::msg::Imu lastnewImu = thisImu;
        static int bias = 141815675;
        if(first){
            lastImu = thisImu;
            lastnewImu.header.stamp.nanosec = bias;
            first = false;
        }
        else{
            if(!finish){
                if(thisImu.header.stamp.sec == lastImu.header.stamp.sec){
                    lastImu = thisImu;
                    lastnewImu = newImu;
                    return;
                }
                else{
                    lastImu = thisImu;
                    newImu = thisImu;
                    newImu.header.stamp.nanosec = 0;
                    
                    lastnewImu = newImu;
                    finish = true;
                }
            }
            else{
                // if(thisImu.header.stamp.sec != lastImu.header.stamp.sec){
                //     lastImu = thisImu;
                //     newImu.header.stamp.nanosec = 0;
                //     lastnewImu = newImu;
                //     return;
                // }
                long long delta = (4294967296 + (long long)thisImu.header.stamp.nanosec - (long long)lastImu.header.stamp.nanosec) % 4294967296;
                newImu.header = lastnewImu.header;
                newImu.header.stamp.nanosec = (lastnewImu.header.stamp.nanosec + delta) % 1000000000;
                newImu.header.stamp.sec += (lastnewImu.header.stamp.nanosec + delta) / 1000000000;
                // if(newImu.header.stamp.sec > lastnewImu.header.stamp.sec && newImu.header.stamp.nanosec > lastnewImu.header.stamp.nanosec){
                //     newImu.header.stamp.nanosec = 0.0;
                // }
                
                lastImu = thisImu;
                lastnewImu = newImu;
            }
            // double temp = newImu.angular_velocity.x;
            // newImu.angular_velocity.x = newImu.angular_velocity.y;
            // newImu.angular_velocity.y = -temp;
            // temp = newImu.linear_acceleration.x;
            // newImu.linear_acceleration.x = newImu.linear_acceleration.y;
            // newImu.linear_acceleration.y = -temp;
            newImu.linear_acceleration.x = 0;
            newImu.linear_acceleration.y = 0;
            newImu.linear_acceleration.z = 9.8;
            newImu.angular_velocity.x = 0;
            newImu.angular_velocity.y = 0;
            newImu.angular_velocity.z = 0;
            newImu.orientation.w = 1;
            newImu.orientation.x = 0;
            newImu.orientation.y = 0;
            newImu.orientation.z = 0;
            pubImu->publish(newImu);
        }
    }    

};


class SensorCalibration : public ParamServer
{
public:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_aft_opt;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gnss_info;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubGpsOdom;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_GNSS_Odom_Path;
    rclcpp::CallbackGroup::SharedPtr callbackGroupGNSS;
    rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;

    deque<nav_msgs::msg::Odometry> GNSSInfoQueue;
    deque<nav_msgs::msg::Odometry> GPSOdomQueue;
    deque<sensor_msgs::msg::Imu> imuQueue; // imuQueue
    mutex mtx_gnss, mtx_imu, mutex_update;
    bool newGps = false;
    Eigen::Vector3d enuxyz;
    double noise_x = 1e-2, noise_y = 1e-2, noise_z = 1e-2; //gps_noise
    double lidarOdomTime = 0;
    Eigen::Affine3f lidarOdomAffine;
    Eigen::Matrix4d LidarFrame_T_gpsFrame; // LO and GPS Trans
    vector<pair<double, vector<double>>> localPoseMap; // 优化后的激光odom对应的时间和位姿
    vector<pair<double, vector<double>>> globalPoseMap; // 优化后激光odom在gps系下对应的时间和位姿
    map<double, vector<double>> gpsPositionMap; //激光里程计对应时间戳下的gps原始数据
    vector<pair<double, vector<double>>> gpsPoseMap;

    //gtsam
    std::thread threadOpt;
    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;
    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
    bool systemInitialized = false;
    gtsam::Pose3 prevPose_;
    int key = 0;

    SensorCalibration(const rclcpp::NodeOptions & options) :
            ParamServer("lio_sam_sensor_calib", options){

        callbackGroupGNSS = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupOdom = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        auto gnssOpt = rclcpp::SubscriptionOptions();
        gnssOpt.callback_group = callbackGroupGNSS;
        auto odomOpt = rclcpp::SubscriptionOptions();
        odomOpt.callback_group = callbackGroupOdom;

        sub_odom_aft_opt = create_subscription<nav_msgs::msg::Odometry>(
            "lio_sam/mapping/odometry", qos_lidar,
            std::bind(&SensorCalibration::odometryHandler, this, std::placeholders::_1),
            odomOpt);
       
        sub_gnss_info = create_subscription<nav_msgs::msg::Odometry>(
            "/ins_data_info", qos,
            std::bind(&SensorCalibration::gnssHandler, this, std::placeholders::_1),
            gnssOpt);
        
        subImu = create_subscription<sensor_msgs::msg::Imu>(
            "/Imu_data", qos_imu,
            std::bind(&SensorCalibration::imuHandler, this, std::placeholders::_1));
        pubGpsOdom = create_publisher<nav_msgs::msg::Odometry>("/lio_sam/sensors/gpsOdom_incremental", 1);
        pub_GNSS_Odom_Path = create_publisher<nav_msgs::msg::Path>("/lio_sam/lidar_in_gps_path", 10);

        // Lidar->GPS
        LidarFrame_T_gpsFrame.setIdentity();
        //if(!CalibEnableFlag){
            LidarFrame_T_gpsFrame.block<3, 3>(0, 0) = Lidar_T_GPS_Rot;
            LidarFrame_T_gpsFrame.block<3, 1>(0, 3) = Lidar_T_GPS_Trans;
        //}
        

        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << M_PI * M_PI, M_PI * M_PI, M_PI * M_PI, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        odometryNoise = noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9).finished());
        if(CalibEnableFlag)
            threadOpt = std::thread(&SensorCalibration::gpsOptimize, this);

    }

    ////1 激光里程计数据处理/////////////
    Eigen::Affine3f odom2affine(nav_msgs::msg::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf2::Quaternion orientation(
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }
    
    // 记录激光里程计odom系下位姿、激光里程计在gps系下位姿、对应gps在odom系下位姿
    void odometryHandler(nav_msgs::msg::Odometry::SharedPtr odomptr){
        std::lock_guard<std::mutex> lock(mtx_gnss);
        //RCLCPP_INFO_STREAM(this->get_logger(), "odometryHandler_in");
        nav_msgs::msg::Odometry odom_in = *odomptr;
        lidarOdomAffine = odom2affine(odom_in);
        lidarOdomTime = stamp2Sec(odom_in.header.stamp);
        Eigen::Vector3d LIO_T(odom_in.pose.pose.position.x, odom_in.pose.pose.position.y, odom_in.pose.pose.position.z);
        Eigen::Quaterniond LIO_Q;
        LIO_Q.w() = odom_in.pose.pose.orientation.w;
        LIO_Q.x() = odom_in.pose.pose.orientation.x;
        LIO_Q.y() = odom_in.pose.pose.orientation.y;
        LIO_Q.z() = odom_in.pose.pose.orientation.z;
        
        inputLIO(lidarOdomTime, LIO_T, LIO_Q);
        
        if (GNSSInfoQueue.empty())
            return;
        while(!GNSSInfoQueue.empty()){
            nav_msgs::msg::Odometry gpsMsg = GNSSInfoQueue.front();
            double gpsTime = stamp2Sec(gpsMsg.header.stamp);
            // 0.02s sync tolerance
            if (gpsTime >= lidarOdomTime - 0.02 && gpsTime <= lidarOdomTime + 0.02) {
                // std::cout << " GPS time = " << gpsTime << std::endl;

                double gx = gpsMsg.pose.pose.position.x;
                double gy = gpsMsg.pose.pose.position.y;
                double gz = gpsMsg.pose.pose.position.z;
                double pos_accuracy = gpsMsg.pose.covariance[0];
                if (pos_accuracy <= 0)
                    pos_accuracy = 1.0;
                if (pos_accuracy > 10)
                    break;
                inputGps(lidarOdomTime, gx, gy, gz, pos_accuracy);
                GNSSInfoQueue.pop_front();
                break;
            } else if (gpsTime < lidarOdomTime - 0.02)
                GNSSInfoQueue.pop_front();
            else if (gpsTime > lidarOdomTime + 0.02)
                break;
        }
        //RCLCPP_INFO_STREAM(this->get_logger(), "odometryHandler_out");

    }    
    // 记录激光里程计输入的位姿以及转到gps系下的位姿
    void inputLIO(double time, Eigen::Vector3d odomPosition, Eigen::Quaterniond odomQuaterniond)
    {
        //RCLCPP_INFO_STREAM(this->get_logger(), "inputLIO_in");
        vector<double> localPose { odomPosition.x(), odomPosition.y(), odomPosition.z(),
            odomQuaterniond.w(), odomQuaterniond.x(), odomQuaterniond.y(), odomQuaterniond.z() };
        mutex_update.lock();
        localPoseMap.push_back({time, localPose});
        mutex_update.unlock();
        vector<double> globalPose = LidarFrame_2_GPSFrame(localPose);
       
        // Eigen::Quaterniond globalQuaterniond;
        // // 激光里程计转GPS系
        // globalQuaterniond = LidarFrame_T_gpsFrame.block<3, 3>(0, 0) * odomQuaterniond;
        // Eigen::Vector3d globalPosition = LidarFrame_T_gpsFrame.block<3, 3>(0, 0) * odomPosition + LidarFrame_T_gpsFrame.block<3, 1>(0, 3);
        // vector<double> globalPose { globalPosition.x(), globalPosition.y(), globalPosition.z(),
        //     globalQuaterniond.w(), globalQuaterniond.x(), globalQuaterniond.y(), globalQuaterniond.z() };
        mutex_update.lock();
        globalPoseMap.push_back({time, globalPose});
        mutex_update.unlock();

        // if(saveOdomFlag){
        //     std::string savePath = std::getenv("HOME") + savePCDDirectory + "lidar_pose.dat";
        //     std::ofstream file;
        //     static bool first_time = true;
        //     if(first_time){
        //         first_time = false;
        //         file.open(savePath, ios::out);
        //     }
        //     else{
        //         file.open(savePath, ios::out | ios::app);
        //     }
        //     if(file.is_open()){
        //         file.precision(20);
        //         file << time << " " << odomPosition.x() << " " << odomPosition.y() << " " << odomPosition.z() << " "
        //         << odomQuaterniond.w() << " " << odomQuaterniond.x() << " " << odomQuaterniond.y() << " " << odomQuaterniond.z() << std::endl;
        //         file.close();
        //     }
        // }
        //RCLCPP_INFO_STREAM(this->get_logger(), "inputLIO_out");
    }
    vector<double> LidarFrame_2_GPSFrame(vector<double>& localPose){
        Eigen::Quaterniond globalQuaterniond;
        Eigen::Vector3d odomPosition = Eigen::Vector3d(localPose[0], localPose[1], localPose[2]);
        Eigen::Quaterniond odomQuaterniond = Eigen::Quaterniond(localPose[3], localPose[4], localPose[5], localPose[6]);
        globalQuaterniond = LidarFrame_T_gpsFrame.block<3, 3>(0, 0) * odomQuaterniond;
        Eigen::Vector3d globalPosition = LidarFrame_T_gpsFrame.block<3, 3>(0, 0) * odomPosition + LidarFrame_T_gpsFrame.block<3, 1>(0, 3);
        vector<double> globalPose { globalPosition.x(), globalPosition.y(), globalPosition.z(),
            globalQuaterniond.w(), globalQuaterniond.x(), globalQuaterniond.y(), globalQuaterniond.z() };
        return globalPose;

    }
    // 记录激光里程计对应的gps位姿，gps优化标志位置位
    void inputGps(double time, double gx, double gy, double gz, double posAccuracy)
    {
        vector<double> tmp { gx, gy, gz, 1.0, 0.0, 0.0, 0.0, posAccuracy };
        gpsPositionMap[time] = tmp;
        newGps = true;
        
    }

    /////2 gnss和IMU数据处理////////////////////
    // 找到gnss对应时刻的imu姿态数据（其实也可以用它自带的数据），合成gps里程计（转到lidarodom系下），然后做一下噪声校准。
    
    void gnssHandler(nav_msgs::msg::Odometry::SharedPtr gnssptr){
        std::lock_guard<std::mutex> lock1(mtx_gnss);
        std::lock_guard<std::mutex> lock2(mtx_imu);
       // RCLCPP_INFO_STREAM(this->get_logger(), "gnssHandler_in");
        nav_msgs::msg::Odometry gnss_in = *gnssptr;
        GNSSInfoQueue.push_back(gnss_in);
        //double gnss_time = (double)gnss_in.header.stamp.sec + (double)gnss_in.header.stamp.nanosec * 1e-9;
        
        if(saveOdomFlag){
            double gpsTime = stamp2Sec(gnss_in.header.stamp);
            double gx = gnss_in.pose.pose.position.x;
            double gy = gnss_in.pose.pose.position.y;
            double gz = gnss_in.pose.pose.position.z;
            double pos_accuracy = gnss_in.pose.covariance[0];
            std::string savePath = std::getenv("HOME") + savePCDDirectory + "gps_pose.dat";
            std::ofstream file;
            static bool first_time = true;
            if(first_time){
                first_time = false;
                file.open(savePath, ios::out);
            }
            else{
                file.open(savePath, ios::out | ios::app);
            }
            if(file.is_open()){
                file.precision(20);
                file << gpsTime << " " << gx << " " << gy << " " << gz << " "
                << 1.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << std::endl;
                file.close();
            }
        }





        Eigen::Vector3d xyz;
        Eigen::Vector3d xyzTR;
        nav_msgs::msg::Odometry gpsOdom = gnss_in;
        Eigen::Quaterniond q_gnss;
        q_gnss.x() = gpsOdom.pose.pose.orientation.x;
        q_gnss.y() = gpsOdom.pose.pose.orientation.y;
        q_gnss.z() = gpsOdom.pose.pose.orientation.z;
        q_gnss.w() = gpsOdom.pose.pose.orientation.w;
        Eigen::Matrix3d R_gnss;
        R_gnss = q_gnss.normalized().toRotationMatrix();
        xyz << (double)gpsOdom.pose.pose.position.x, (double)gpsOdom.pose.pose.position.y, (double)gpsOdom.pose.pose.position.z;
        gpsOdom.header.frame_id = odometryFrame;

        // gpsOdom的位置是用优化后的外参从gnss系转到odom系下的，姿态是用ins姿态
        xyzTR = LidarFrame_T_gpsFrame.inverse().block<3, 3>(0, 0) * xyz + LidarFrame_T_gpsFrame.inverse().block<3, 1>(0, 3);
       
        gpsOdom.pose.pose.position.x = xyzTR.x();
        gpsOdom.pose.pose.position.y = xyzTR.y();
        gpsOdom.pose.pose.position.z = xyzTR.z();

        R_gnss = LidarFrame_T_gpsFrame.inverse().block<3, 3>(0, 0) * R_gnss;
        q_gnss = Eigen::Quaterniond(R_gnss);
        q_gnss.normalize();
        
        sensor_msgs::msg::Imu imuMsg = getTimeimu(gnss_in.header.stamp);
        Eigen::Quaterniond AHRS(imuMsg.orientation.w,
            imuMsg.orientation.x,
            imuMsg.orientation.y,
            imuMsg.orientation.z);
        if(UseIMUPose){
            gpsOdom.pose.pose.orientation.w = AHRS.w();
            gpsOdom.pose.pose.orientation.x = AHRS.x();
            gpsOdom.pose.pose.orientation.y = AHRS.y();
            gpsOdom.pose.pose.orientation.z = AHRS.z();
        }
        else{
            gpsOdom.pose.pose.orientation.w = q_gnss.w();
            gpsOdom.pose.pose.orientation.x = q_gnss.x();
            gpsOdom.pose.pose.orientation.y = q_gnss.y();
            gpsOdom.pose.pose.orientation.z = q_gnss.z();
        }
        GPSOdomQueue.push_back(gpsOdom);

        VerifyGNSSodom();
        gpsOdom.pose.covariance[0] = noise_x;
        gpsOdom.pose.covariance[7] = noise_y;
        gpsOdom.pose.covariance[14] = noise_z;

        if(systemInitialized || (!CalibEnableFlag))
            pubGpsOdom->publish(gpsOdom);

        //RCLCPP_INFO_STREAM(this->get_logger(), "gnssHandler_out");
    }

    void imuHandler(sensor_msgs::msg::Imu::SharedPtr imuMsg)
    {
        std::lock_guard<std::mutex> lock(mtx_imu);
        //RCLCPP_INFO_STREAM(this->get_logger(), "imuHandler_in");
        sensor_msgs::msg::Imu thisImu = imuConverter(*imuMsg);
        imuQueue.push_back(thisImu);
        if (imuQueue.size() > 2000) {
            imuQueue.pop_front();
        }
        //RCLCPP_INFO_STREAM(this->get_logger(), "imuHandler_out");
    }
    //插值gps时刻imu的信息
    template<typename T>
    sensor_msgs::msg::Imu getTimeimu(const T& timeStamp) // get imuMsg in specified time by interpolation
    {
        if (imuQueue.size() == 0) {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "imuQueue.size() == 0");
            sensor_msgs::msg::Imu emptyMsg;
            return emptyMsg;
        }
        // imu_deque.size() == 1
        if (imuQueue.size() == 1) {
            return imuQueue.at(0);
        }
        if (stamp2Sec(timeStamp) < stamp2Sec(imuQueue.front().header.stamp)) {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "getImuMsg(): timeStamp < imu_deque.front().header.stamp");
            return imuQueue.front();
        }
        // timeStamp > imu_deque.back().header.stamp
        if (stamp2Sec(timeStamp) > stamp2Sec(imuQueue.back().header.stamp)) {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "getImuMsg(): timeStamp > imu_deque.back().header.stamp");
            RCLCPP_DEBUG_STREAM(this->get_logger(), "gpstime: " << stamp2Sec(timeStamp));
            RCLCPP_DEBUG_STREAM(this->get_logger(), "imu_lasttime: " << stamp2Sec(imuQueue.back().header.stamp));
            RCLCPP_DEBUG_STREAM(this->get_logger(), "time_dis: " << (stamp2Sec(timeStamp) - stamp2Sec(imuQueue.back().header.stamp)));
            return imuQueue.back();
        }
        // imu_deque.front().header.stamp < timeStamp < imu_deque.back().header.stamp
        int iLow, iHigh, iMid;
        iLow = 0;
        iHigh = imuQueue.size() - 1;
        while (iHigh - iLow >= 2) {
            iMid = (iLow + iHigh) / 2;
            if (fabs(stamp2Sec(imuQueue.at(iMid).header.stamp) - stamp2Sec(timeStamp)) < 1e-15) {
                return imuQueue.at(iMid);
            }
            if (stamp2Sec(imuQueue.at(iMid).header.stamp) > stamp2Sec(timeStamp)) {
                iHigh = iMid;
            }
            if (stamp2Sec(imuQueue.at(iMid).header.stamp) < stamp2Sec(timeStamp)) {
                iLow = iMid;
            }
        }
        ////interpolation according specified time
        sensor_msgs::msg::Imu msg0, msg1, msg;
        msg0 = imuQueue.at(iLow);
        msg1 = imuQueue.at(iHigh);
        ////// std_msgs / Header header
        msg.header.stamp = timeStamp;
        msg.header.frame_id = msg0.header.frame_id;
        ////// geometry_msgs / Quaternion orientation
        Eigen::Quaterniond q0, q1, q;
        q0.coeffs() << msg0.orientation.x, msg0.orientation.y, msg0.orientation.z, msg0.orientation.w;
        q1.coeffs() << msg1.orientation.x, msg1.orientation.y, msg1.orientation.z, msg1.orientation.w;
        double t_scale = (stamp2Sec(timeStamp) - stamp2Sec(msg0.header.stamp)) / (stamp2Sec(msg1.header.stamp) - stamp2Sec(msg0.header.stamp));
        q = q0.slerp(t_scale, q1);
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        msg.orientation.w = q.w();

        msg.angular_velocity.x = msg0.angular_velocity.x + (msg1.angular_velocity.x - msg0.angular_velocity.x) * t_scale;
        msg.angular_velocity.y = msg0.angular_velocity.y + (msg1.angular_velocity.y - msg0.angular_velocity.y) * t_scale;
        msg.angular_velocity.z = msg0.angular_velocity.z + (msg1.angular_velocity.z - msg0.angular_velocity.z) * t_scale;

        msg.linear_acceleration.x = msg0.linear_acceleration.x + (msg1.linear_acceleration.x - msg0.linear_acceleration.x) * t_scale;
        msg.linear_acceleration.y = msg0.linear_acceleration.y + (msg1.linear_acceleration.y - msg0.linear_acceleration.y) * t_scale;
        msg.linear_acceleration.z = msg0.linear_acceleration.z + (msg1.linear_acceleration.z - msg0.linear_acceleration.z) * t_scale;

        return msg;
    }
    //估计gnss的累积不确定度
    void VerifyGNSSodom(){
        // 找最接近激光里程计的gps里程计
        while(!GPSOdomQueue.empty()){
            if (stamp2Sec(GPSOdomQueue.front().header.stamp) <= lidarOdomTime)
                GPSOdomQueue.pop_front();
            else
                break;
        }
        nav_msgs::msg::Odometry gpsMsg = GPSOdomQueue.front();
        Eigen::Affine3f gpsOdomAffineFront = odom2affine(GPSOdomQueue.front());
        Eigen::Affine3f gpsOdomAffineBack = odom2affine(GPSOdomQueue.back());
        Eigen::Affine3f gpsOdomAffineIncre = gpsOdomAffineFront.inverse() * gpsOdomAffineBack;
        //获取以lidarodom为基准的增量式gps里程计
        Eigen::Affine3f gpsOdomAffineLast = lidarOdomAffine * gpsOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(gpsOdomAffineLast, x, y, z, roll, pitch, yaw);

        double Fgx, Fgy, Fgz;
        Fgx = (gpsMsg.pose.pose.position.x - x) * (gpsMsg.pose.pose.position.x - x);
        Fgy = (gpsMsg.pose.pose.position.y - y) * (gpsMsg.pose.pose.position.y - y);
        Fgz = (gpsMsg.pose.pose.position.z - z) * (gpsMsg.pose.pose.position.z - z);


        //更新gps噪声，这里相当于以上一时刻lidarodom为基准，计算了到当前时刻的累积不确定度
        //covariance是每米的噪声方差，乘米数相当于计算了一下Fgx米的不确定度有多高
        //但是问题在于gps的离散的绝对测量，不是累积的测量，按道理不会有累积误差？
        if(useDynamicNoise){
            noise_x = sqrt(gpsMsg.pose.covariance[0] * Fgx);
            noise_y = sqrt(gpsMsg.pose.covariance[7] * Fgy);
            noise_z = sqrt(gpsMsg.pose.covariance[14] * Fgz);
        }
        else{
            noise_x = gpsMsg.pose.covariance[0];
            noise_y = gpsMsg.pose.covariance[7];
            noise_z = gpsMsg.pose.covariance[14];
        }
        

        if(GPSOdomQueue.size() > 2000){
            GPSOdomQueue.pop_front();
        }
    }


///////////////////////外参优化////////////////////////////////////////////////////////////////////////////////////

    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        // optParameters.factorization = gtsam::ISAM2Params::QR;

        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }
    void resetParams()
    {
        localPoseMap.clear();
        //localPoseMap.shrink_to_fit();
        globalPoseMap.clear();
        //globalPoseMap.shrink_to_fit();
        gpsPositionMap.clear();
    }

    void OptimizeHandler(){
        if(newGps == true && CalibEnableFlag){
            newGps = false;
            // 因子图初始化
            if(systemInitialized == false){
                resetOptimization();
                int PoseMap_size = globalPoseMap.size();
                // 积累一定数量的帧再初始化
                if(PoseMap_size < calib_init_size){
                    return;
                }

            }
            if(key > 500){
                systemInitialized = true;  
                return;
            } 

            // 初始化以后,如果因子图太大了，需要边缘化一下,一方面释放内存，另一方面加快优化速度
            int PoseMap_size = globalPoseMap.size();
            if(key >= 100000){
                // get updated noise before reset
                gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key - 1)));
                // reset graph
                resetOptimization();
                // add pose
                //gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
                gtsam::noiseModel::Diagonal::shared_ptr updateNoise= gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e3, 1e3, 1e3).finished()); // rad,rad,rad,m, m, m
                gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updateNoise);
                graphFactors.add(priorPose);
                // add values
                graphValues.insert(X(0), prevPose_);
                // optimize once
                optimizer.update(graphFactors, graphValues);
                graphFactors.resize(0);
                graphValues.clear();
                
                
                vector<pair<double, vector<double>>> new_localPoseMap;
                vector<pair<double, vector<double>>> new_globalPoseMap;
                map<double, vector<double>> new_gpsPositionMap;
                for(int iter = key - 1; iter < PoseMap_size; iter++){
                    new_localPoseMap.push_back(localPoseMap[iter]);
                    new_globalPoseMap.push_back(globalPoseMap[iter]);
                    double odom_time = localPoseMap[iter].first;
                    if(gpsPositionMap.find(odom_time) != gpsPositionMap.end()){
                        new_gpsPositionMap[odom_time] = gpsPositionMap[odom_time];
                    }
                }
                mutex_update.lock();
                resetParams();
                localPoseMap = new_localPoseMap;
                globalPoseMap = new_globalPoseMap;
                gpsPositionMap = new_gpsPositionMap;
                mutex_update.unlock();
                key = 1;
            }

            while(key < PoseMap_size){
                // //RCLCPP_INFO_STREAM(this->get_logger(), "key: " << key);
                // 当前处理的里程计在gps系下的位姿
                vector<double> local_pose_key = localPoseMap[key].second;
                // 当前处理的里程计在odom系下的位姿
                vector<double> global_pose_key = globalPoseMap[key].second;
                // initial pose
                // prevPose_ = gtsam::Pose3(gtsam::Rot3::Quaternion(global_pose_key[3], global_pose_key[4], global_pose_key[5], global_pose_key[6]), 
                //                          gtsam::Point3(global_pose_key[0], global_pose_key[1], global_pose_key[2]));
                // 关联gps因子
                double odom_time = localPoseMap[key].first;
                if(gpsPositionMap.find(odom_time) != gpsPositionMap.end()){
                        vector<double> gps_key = gpsPositionMap[odom_time];

                        // //RCLCPP_INFO_STREAM(this->get_logger(), "gpsfactor: " << gps_key[3] <<" " << gps_key[4] << " " << gps_key[5] << " " << gps_key[6]
                        //                                                 << " " << gps_key[0] <<" " << gps_key[1] << " " << gps_key[2]);
                        gtsam::Pose3 gpsPose = gtsam::Pose3(gtsam::Rot3::Quaternion(gps_key[3], gps_key[4], gps_key[5], gps_key[6]), 
                                                            gtsam::Point3(gps_key[0], gps_key[1], gps_key[2]));
                        if(useINSPose){
                            gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), gpsPose, correctionNoise);
                            graphFactors.add(pose_factor);
                        }
                        else{
                            RCLCPP_INFO_STREAM(this->get_logger(), "gpsnoise: " << noise_x << " " <<noise_y << " " << noise_z );
                            noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances((gtsam::Vector(3) << noise_x, noise_y, noise_z).finished());
                            gtsam::GPSFactor gps_factor(X(key), gtsam::Point3(gps_key[0], gps_key[1], gps_key[2]), gps_noise);
                            graphFactors.add(gps_factor);
                        }
                }
                                
                // 初始时刻位姿，关联一个先验因子
                if(key == 0){
                    //RCLCPP_INFO_STREAM(this->get_logger(), "priorfactor: " << global_pose_key[3] <<" " << global_pose_key[4] << " " << global_pose_key[5] << " " << global_pose_key[6] << " " << global_pose_key[0] <<" " << global_pose_key[1] << " " << global_pose_key[2]);
                    prevPose_ = gtsam::Pose3(gtsam::Rot3::Quaternion(global_pose_key[3], global_pose_key[4], global_pose_key[5], global_pose_key[6]), gtsam::Point3(global_pose_key[0], global_pose_key[1], global_pose_key[2]));                                                  
                    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
                    graphFactors.add(priorPose);

                }
                // 非初始时刻，当前帧位姿和前一帧共同关联一个between因子
                else{
                    vector<double> local_pose_key_bef = localPoseMap[key - 1].second;

                    gtsam::Pose3 pose_now = gtsam::Pose3(gtsam::Rot3::Quaternion(local_pose_key[3], local_pose_key[4], local_pose_key[5], local_pose_key[6]), 
                                                            gtsam::Point3(local_pose_key[0], local_pose_key[1], local_pose_key[2]));

                    gtsam::Pose3 pose_bef = gtsam::Pose3(gtsam::Rot3::Quaternion(local_pose_key_bef[3], local_pose_key_bef[4], local_pose_key_bef[5], local_pose_key_bef[6]), 
                                                            gtsam::Point3(local_pose_key_bef[0], local_pose_key_bef[1], local_pose_key_bef[2]));
                    graphFactors.add(BetweenFactor<Pose3>(X(key - 1), X(key), pose_bef.between(pose_now), odometryNoise));
                    prevPose_ = prevPose_ * pose_bef.between(pose_now);
                }
                // add values
                
                graphValues.insert(X(key), prevPose_);
                // optimize once
                optimizer.update(graphFactors, graphValues);
                graphFactors.resize(0);
                graphValues.clear();

                gtsam::Values result = optimizer.calculateEstimate();

                /////////////////////////////可视化优化后轨迹//////////////////////////////////////////////
                nav_msgs::msg::Path lidarPath_in_gps;
                lidarPath_in_gps.header.stamp = rclcpp::Clock().now();
                lidarPath_in_gps.header.frame_id = "map";
                for(int i = 0; i <= key; i++){
                    gtsam::Pose3 pose_key = result.at<gtsam::Pose3>(X(i));
                    gtsam::Rot3 pose_key_rot = pose_key.rotation();
                    gtsam::Point3 pose_key_trans = pose_key.translation();
                    geometry_msgs::msg::PoseStamped odompose;
                    odompose.header.stamp = rclcpp::Clock().now();
                    odompose.header.frame_id = odometryFrame;
                    odompose.pose.position.x = pose_key_trans.x();
                    odompose.pose.position.y = pose_key_trans.y();
                    odompose.pose.position.z = pose_key_trans.z();
                    // 发布ins轨迹可视化
                    lidarPath_in_gps.poses.push_back(odompose);

                }

                pub_GNSS_Odom_Path->publish(lidarPath_in_gps);
                /////////////////////////////////////////////////////////////////////////////////


                prevPose_ = result.at<gtsam::Pose3>(X(key));

                // 更新globalPoseMap
                gtsam::Rot3 opt_rot = prevPose_.rotation();
                gtsam::Point3 opt_trans = prevPose_.translation();
                tf2::Quaternion q;
                q.setRPY(opt_rot.roll(), opt_rot.pitch(), opt_rot.yaw());
                vector<double> new_globalpose{opt_trans.x(), opt_trans.y(), opt_trans.z(),q.w(), q.x(), q.y(), q.z()};
                globalPoseMap[key].second = new_globalpose;
                ++key;
            }

            //RCLCPP_INFO_STREAM(this->get_logger(), "key: " << key);
            // 更新外参
            Eigen::Matrix4d LIOFrame_T_body = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d GPSFrame_T_body = Eigen::Matrix4d::Identity();
            LIOFrame_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[key - 1].second[3], localPoseMap[key - 1].second[4],
                localPoseMap[key - 1].second[5], localPoseMap[key - 1].second[6])
                                                    .toRotationMatrix();
            LIOFrame_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[key - 1].second[0], localPoseMap[key - 1].second[1], localPoseMap[key - 1].second[2]);
            GPSFrame_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPoseMap[key - 1].second[3], globalPoseMap[key - 1].second[4],
                globalPoseMap[key - 1].second[5], globalPoseMap[key - 1].second[6])
                                                    .toRotationMatrix();
            GPSFrame_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPoseMap[key - 1].second[0], globalPoseMap[key - 1].second[1], globalPoseMap[key - 1].second[2]);
            ////RCLCPP_INFO_STREAM(this->get_logger(), "LIOFrame_T_body: ");
            ////RCLCPP_INFO_STREAM(this->get_logger(), LIOFrame_T_body);
            ////RCLCPP_INFO_STREAM(this->get_logger(), "GPSFrame_T_body: ");
            ////RCLCPP_INFO_STREAM(this->get_logger(), GPSFrame_T_body);
            LidarFrame_T_gpsFrame = GPSFrame_T_body * LIOFrame_T_body.inverse();
            if(CalibEnableFlag){
                RCLCPP_INFO_STREAM(this->get_logger(), "L_T_g: " << LidarFrame_T_gpsFrame);
            }
            // //RCLCPP_INFO_STREAM(this->get_logger(), "g_T_L: " << LidarFrame_T_gpsFrame.inverse());
            
            systemInitialized = true;  
        }  
        return;
    }

    void LoadOdomData(std::string file_name, vector<pair<double, vector<double>>>& PoseMap){
        std::string savePath = std::getenv("HOME") + savePCDDirectory + file_name;
        std::ifstream input_odom;
        input_odom.open(savePath);
        if(input_odom.is_open()){
            std::string line = "";
            while(std::getline(input_odom, line)){
                std::vector<double> input_vector;
                std::istringstream iss(line);
                double value;
                while(iss >> value){
                    input_vector.push_back(value);
                }
                double time = input_vector[0];
                if(file_name == "gps_sync_pose.dat" && testOffset){
                    Eigen::Vector3d pos_bef(input_vector[1], input_vector[2], input_vector[3]);
                    Eigen::Vector3d offset_pos(fixedOffsetX, fixedOffsetY, 0);
                    Eigen::AngleAxisd rotation(fixedOffsetRow, Eigen::Vector3d::UnitZ());
                    Eigen::Vector3d pos_aft = rotation.toRotationMatrix() * pos_bef + offset_pos;
                    input_vector[1] = pos_aft.x();
                    input_vector[2] = pos_aft.y();
                    input_vector[3] = pos_aft.z();
                }
                vector<double> localPose = { input_vector[1], input_vector[2], input_vector[3], // xyz
                                             input_vector[4], input_vector[5], input_vector[6], input_vector[7] // wxyz
                                            };
                PoseMap.push_back({time, localPose});
            }
            input_odom.close();
        }
        
    }

    void StartOptimize(){
        resetOptimization();
        key = 0;
        int PoseMap_size = gpsPoseMap.size();
        if(PoseMap_size < calib_init_size){
            RCLCPP_ERROR_STREAM(this->get_logger(), "No enough poses!");
            return;
        }

        while(key < PoseMap_size){
            vector<double> local_pose_key = localPoseMap[key].second;
            vector<double> global_pose_key = LidarFrame_2_GPSFrame(local_pose_key);
            vector<double> gps_key = gpsPoseMap[key].second;
            cout << "global_pose_key_bef: " << endl;
            for(int i = 0; i < global_pose_key.size(); i++){
                cout << global_pose_key[i] << " ";
            }
            cout << endl;

            // cout << "gps_key: " << endl;
            // for(int i = 0; i < gps_key.size(); i++){
            //     cout << gps_key[i] << " ";
            // }
            // cout << endl;

            gtsam::Pose3 gpsPose = gtsam::Pose3(gtsam::Rot3::Quaternion(gps_key[3], gps_key[4], gps_key[5], gps_key[6]), 
                                                            gtsam::Point3(gps_key[0], gps_key[1], gps_key[2]));
            // 关联gps因子
            // if(useINSPose){
            //     gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), gpsPose, correctionNoise);
            //     graphFactors.add(pose_factor);
            // }
            // else{
                // //RCLCPP_INFO_STREAM(this->get_logger(), "gpsnoise: " << noise_x << " " <<noise_y << " " << noise_z );
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances((gtsam::Vector(3) << 1e-2, 1e-2, 1e-2).finished());
                //gtsam::GPSFactor gps_factor(X(key), gtsam::Point3(gps_key[0], gps_key[1], gps_key[2]), gps_noise);
                gtsam::GPSFactor gps_factor(X(key), gtsam::Point3(gps_key[0], gps_key[1], 0.0), gps_noise);
                graphFactors.add(gps_factor);
            //}

            // 初始时刻位姿，关联一个先验因子
            if(key == 0){
                //RCLCPP_INFO_STREAM(this->get_logger(), "priorfactor: " << global_pose_key[3] <<" " << global_pose_key[4] << " " << global_pose_key[5] << " " << global_pose_key[6] << " " << global_pose_key[0] <<" " << global_pose_key[1] << " " << global_pose_key[2]);
                prevPose_ = gtsam::Pose3(gtsam::Rot3::Quaternion(global_pose_key[3], global_pose_key[4], global_pose_key[5], global_pose_key[6]), gtsam::Point3(global_pose_key[0], global_pose_key[1], global_pose_key[2])); 

                gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
                graphFactors.add(priorPose);
            }

            // 非初始时刻，当前帧位姿和前一帧共同关联一个between因子
            else{
                vector<double> local_pose_key_bef = localPoseMap[key - 1].second;
                gtsam::Pose3 pose_now = gtsam::Pose3(gtsam::Rot3::Quaternion(local_pose_key[3], local_pose_key[4], local_pose_key[5], local_pose_key[6]), 
                                                        gtsam::Point3(local_pose_key[0], local_pose_key[1], local_pose_key[2]));

                gtsam::Pose3 pose_bef = gtsam::Pose3(gtsam::Rot3::Quaternion(local_pose_key_bef[3], local_pose_key_bef[4], local_pose_key_bef[5], local_pose_key_bef[6]), 
                                                        gtsam::Point3(local_pose_key_bef[0], local_pose_key_bef[1], local_pose_key_bef[2]));
                graphFactors.add(BetweenFactor<Pose3>(X(key - 1), X(key), pose_bef.between(pose_now), odometryNoise));
                prevPose_ = prevPose_ * pose_bef.between(pose_now);
            }
            graphValues.insert(X(key), prevPose_);



            //prevPose_ = result.at<gtsam::Pose3>(X(key));
            //cout << "prevPose_: " << prevPose_ << endl;
            
            ++key;
        }

        // // 优化
        // optimizer.update(graphFactors, graphValues);
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();
        gtsam::Values result = optimizer.calculateEstimate();
        
        // optimizer.update();
        // graphFactors.resize(0);
        // graphValues.clear();
       // gtsam::Values result = optimizer.calculateEstimate();

        // 更新
        for(int i = 0; i < key; i++){
            auto keyPose = result.at<gtsam::Pose3>(X(i));
            cout << "i: " << i << " keypose: " << keyPose << endl;
            gtsam::Rot3 opt_rot = keyPose.rotation();
            gtsam::Point3 opt_trans = keyPose.translation();
            tf2::Quaternion q;
            q.setRPY(opt_rot.roll(), opt_rot.pitch(), opt_rot.yaw());
            vector<double> new_globalpose{opt_trans.x(), opt_trans.y(), opt_trans.z(),q.w(), q.x(), q.y(), q.z()};
            globalPoseMap[i].second = new_globalpose;
            cout << "i: " << i << " globalPoseMap: " <<endl;
            for(int j = 0; j < new_globalpose.size(); j++){
                cout << globalPoseMap[i].second[j] << " ";
            }
            cout << endl;
        }

        //可视化优化后轨迹/////
        nav_msgs::msg::Path lidarPath_in_gps;
        lidarPath_in_gps.header.stamp = rclcpp::Clock().now();
        lidarPath_in_gps.header.frame_id = "map";
        for(int i = 0; i < key; i++){
            gtsam::Pose3 pose_key = result.at<gtsam::Pose3>(X(i));
            gtsam::Rot3 pose_key_rot = pose_key.rotation();
            gtsam::Point3 pose_key_trans = pose_key.translation();
            geometry_msgs::msg::PoseStamped odompose;
            odompose.header.stamp = rclcpp::Clock().now();
            odompose.header.frame_id = odometryFrame;
            odompose.pose.position.x = pose_key_trans.x();
            odompose.pose.position.y = pose_key_trans.y();
            odompose.pose.position.z = pose_key_trans.z();
            lidarPath_in_gps.poses.push_back(odompose);
        }
        pub_GNSS_Odom_Path->publish(lidarPath_in_gps);




        // 更新外参
        Eigen::Matrix4d LIOFrame_T_body = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d GPSFrame_T_body = Eigen::Matrix4d::Identity();

        LIOFrame_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[0].second[3], localPoseMap[0].second[4],
            localPoseMap[0].second[5], localPoseMap[0].second[6])
                                                .toRotationMatrix();
        LIOFrame_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[0].second[0], localPoseMap[0].second[1], localPoseMap[0].second[2]);
        cout << "LIOFrame_T_body: " << LIOFrame_T_body << endl;
        GPSFrame_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPoseMap[0].second[3], globalPoseMap[0].second[4],
            globalPoseMap[0].second[5], globalPoseMap[0].second[6])
                                                .toRotationMatrix();
            cout << "globalPoseMap[key - 1]: " << endl;                                    
            for(int i = 0; i < 7; i++){
                cout << globalPoseMap[key - 1].second[i] << " ";
            }
            cout << endl;
        GPSFrame_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPoseMap[0].second[0], globalPoseMap[0].second[1], globalPoseMap[0].second[2]);
        cout << "GPSFrame_T_body.block<3, 1>(0, 3): " << GPSFrame_T_body.block<3, 1>(0, 3) << endl;
        cout << "GPSFrame_T_body: " << GPSFrame_T_body << endl;
        LidarFrame_T_gpsFrame = GPSFrame_T_body * LIOFrame_T_body.inverse();
        cout << "L_T_g: " << endl;
        cout << LidarFrame_T_gpsFrame << endl;

        vector<double> aft_pose = LidarFrame_2_GPSFrame(localPoseMap[key - 1].second);
        cout << "aft_pose: " << endl;
        for(int i = 0; i < aft_pose.size(); i++){
            cout << aft_pose[i] << " ";
        }
        cout << endl;



    }

    void SaveOdomData(std::string file_name, vector<pair<double, vector<double>>>& PoseMap){
        std::string savePath = std::getenv("HOME") + savePCDDirectory + file_name;
        std::ofstream output_odom;
        output_odom.open(savePath);
        if(output_odom.is_open()){
            for(int i = 0; i < PoseMap.size(); i++){
                output_odom << PoseMap[i].first << " " 
                << PoseMap[i].second[0] << " "
                << PoseMap[i].second[1] << " "
                << PoseMap[i].second[2] << " "
                << PoseMap[i].second[3] << " "
                << PoseMap[i].second[4] << " "
                << PoseMap[i].second[5] << " "
                << PoseMap[i].second[6] << endl;
            }
            output_odom.close();
        }

    }

    void OfflineOptimize(){
        LoadOdomData("lidar_sync_pose.dat", localPoseMap);
        LoadOdomData("lidar_sync_pose.dat", globalPoseMap);
        LoadOdomData("gps_sync_pose.dat", gpsPoseMap);
        // for(int i = 0; i < localPoseMap.size(); i++){
        //     cout << "no" << i << " lidar_pose: " << localPoseMap[i].first << " " << localPoseMap[i].second[0] << endl;
        // }
        // for(int i = 0; i < gpsPoseMap.size(); i++){
        //     cout << "No" << i << " gps_pose: " << gpsPoseMap[i].first << " " << gpsPoseMap[i].second[0] << endl;
        // }
        for(int iter = 0; iter < 100; iter++){
            StartOptimize();
            for(int i = 0; i < globalPoseMap.size(); i++){
                globalPoseMap[i].second = LidarFrame_2_GPSFrame(localPoseMap[i].second);
            }
        }
        
        SaveOdomData("lidar_aft_pose.dat", globalPoseMap);
        if(testOffset){
            SaveOdomData("gps_aft_pose.dat", gpsPoseMap);
        }
        return;
    }

    // 外参优化线程
    void gpsOptimize()
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Calib Optimize Process Started.\033[0m");

        if(offlineCalib){
            OfflineOptimize();
        }
        else{
            rclcpp::Rate rate(1);
            while(rclcpp::ok()){
                rate.sleep();
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "start_calib");
                OptimizeHandler();
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "end_calib");
            }
        }
        return;
    }
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(false);
    rclcpp::executors::MultiThreadedExecutor e;

    auto sensor_process = std::make_shared<SensorProcess>(options);
    auto sensor_calib = std::make_shared<SensorCalibration>(options);
    e.add_node(sensor_process);
    e.add_node(sensor_calib);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Sensor Process Started.\033[0m");

    e.spin();

    rclcpp::shutdown();
    return 0;
}

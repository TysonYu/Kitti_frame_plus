//
//  Created by Tyson YU on 3/29/2019
//

#ifndef DATA_LOADER_H
#define DATA_LOADER_H

#include "common_include.h"

class PointCloudLoader
{
public:
    typedef std::shared_ptr<PointCloudLoader> Ptr;
    string data_dir_;
    int cloud_id_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_;

public:
    //  functions
    PointCloudLoader();
    PointCloudLoader(string & data_dir): data_dir_(data_dir){}
    void readKittiPclBinData(int cloud_id_);

};

class ImageLoader
{
public:
    typedef std::shared_ptr<ImageLoader> Ptr;
    string data_dir_;
    int image_id_;
    cv::Mat image_;
    cv::Mat ground_truth_;
public:
    //  function
    ImageLoader();
    ImageLoader(string &data_dir): data_dir_(data_dir){}
    void readKittiImage(int image_id_);
};

class Calibration
{
public:
    typedef std::shared_ptr<Calibration> Ptr;
    string data_dir_;
    Eigen::Matrix4f rt1_;//激光雷达到相机cam0的RT矩阵
    Eigen::Matrix4f rt2_;//cam0-to-cam2;
    Eigen::Matrix4f Rt_;//激光雷达到Cam2
    Eigen::Matrix4f intrisic_;//相机内参
    int imu_id_;
    std::vector<std::string> groundtruth_;
    Calibration(string &data_dir): data_dir_(data_dir)
    {
    rt1_ << 7.027555e-03,-9.999753e-01,2.599616e-05,-7.137748e-03,   
            -2.254837e-03,-4.184312e-05,-9.999975e-01,-7.482656e-02,  
            9.999728e-01,7.027479e-03,-2.255075e-03,-3.336324e-01, 
            0,0,0,1;  
    rt2_ << 9.999280e-01, 8.085985e-03, -8.866797e-03, 0,
            -8.123205e-03, 9.999583e-01,-4.169750e-03, 0,
            8.832711e-03, 4.241477e-03, 9.999520e-01, 0,
            0, 0, 0, 1;
    Rt_ = rt2_*rt1_;
    intrisic_ << 7.070912e+02, 0.000000e+00, 6.018873e+02, 4.688783e+01, 
                0.000000e+00, 7.070912e+02, 1.831104e+02, 1.178601e-01, 
                0.000000e+00, 0.000000e+00, 1.000000e+00, 6.203223e-03,
                0,0,0,1;
    // rt1_ << 7.967514e-03,-9.999679e-01,-8.462264e-04,-1.377769e-02,   
    //         -2.771053e-03,8.241710e-04,-9.999958e-01,-5.542117e-02,  
    //         9.999644e-01,7.969825e-03,-2.764397e-03,-2.918589e-01, 
    //         0,0,0,1;
    // rt2_ << 9.999454e-01, 7.259129e-03, -7.519551e-03, 0,
    //         -7.292213e-03, 9.999638e-01,-4.381729e-03, 0,
    //         7.487471e-03, 4.436324e-03, 9.999621e-01, 0,
    //         0, 0, 0, 1;
    // Rt_ = rt2_*rt1_;
    // intrisic_ << 7.188560e+02, 0.000000e+00, 6.071928e+02, 4.538225e+01, 
    //             0.000000e+00, 7.188560e+02, 1.852157e+02, -1.130887e-01, 
    //             0.000000e+00, 0.000000e+00, 1.000000e+00, 3.779761e-03,
    //             0,0,0,1;
    }
    Eigen::Matrix4f Position_;//GroundTruth position
    void ReadGroundTruth(int imu_id_);
};




#endif //DATA_LOADER_H
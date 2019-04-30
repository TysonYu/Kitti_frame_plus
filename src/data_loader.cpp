//
//  Created by Tyson YU on 3/29/2019
//

#include "data_loader.h"


void PointCloudLoader::readKittiPclBinData(int cloud_id_)
{
    std::stringstream ss;
    ss << std::setw(10) << std::setfill('0') << cloud_id_ << ".bin";
    std::string in_file = data_dir_ + "/velodyne_points/data/" + ss.str();
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good())
    {
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i=0; input.good() && !input.eof(); i++) {
        pcl::PointXYZ point_xyz;
        float temp;
        input.read((char *) &point_xyz.x, 3*sizeof(float));
        input.read((char *) &temp, sizeof(float));
        cloud->points.push_back(point_xyz);
    }
    input.close();
    raw_cloud_ = cloud;
}

void ImageLoader::readKittiImage(int image_id_)
{
    std::stringstream ss;
    ss << std::setw(10) << std::setfill('0') << image_id_ << ".png";
    std::string in_file = data_dir_ + "/image_02/data/" + ss.str();
    image_ = cv::imread(in_file, cv::IMREAD_COLOR);

    // std::string ground_truth_path = "/media/icey/TysonYu/dataset/data_depth_annotated/train/2011_09_26_drive_0001_sync/proj_depth/groundtruth/image_02/" + ss.str();
    // ground_truth_ = cv::imread(ground_truth_path, CV_LOAD_IMAGE_UNCHANGED);
}

void Calibration::ReadGroundTruth(int imu_id_)
{
    std::string in_file = data_dir_ + "/05.txt";
    cout << "starting read ground truth " << in_file << endl;
    groundtruth_.clear();
    int count = 0;
    bool flag = false;
    ifstream fin(in_file);
    string line;
    while(getline(fin,line))
    {
        istringstream sin(line);
        string field;
        if(count == imu_id_)
        {
            for(int i = 0; i < 12; i++)
            {
                getline(sin, field, ' ');
                groundtruth_.push_back(field);
            }
            flag = true;
            break;
        }
        count ++;
    }
    if(flag == true)
        std::cout << "Ground Truth successfully found!" << std::endl;
    // for(int i = 0; i < groundtruth_.size();i++)
    //     cout << groundtruth_.at(i) << "    ";
    Position_ << atof(groundtruth_.at(0).c_str()), atof(groundtruth_.at(1).c_str()), atof(groundtruth_.at(2).c_str()), atof(groundtruth_.at(3).c_str()),
                 atof(groundtruth_.at(4).c_str()), atof(groundtruth_.at(5).c_str()), atof(groundtruth_.at(6).c_str()), atof(groundtruth_.at(7).c_str()),
                 atof(groundtruth_.at(8).c_str()), atof(groundtruth_.at(9).c_str()), atof(groundtruth_.at(10).c_str()), atof(groundtruth_.at(11).c_str()),
                 0,0,0,1;    
    // std::cout << Position_ << std::endl;
}
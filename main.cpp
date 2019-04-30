//
//  create by tiezheng yu on 2019-4-16
//


#include "common_include.h"
#include "data_loader.h"
#include "mrf.h"

int main(int argc, char **argv)
{
    string data_file = "/media/icey/TysonYu/dataset/2011_09_30/2011_09_30_drive_0018_sync";
    PointCloudLoader::Ptr point_cloud_loader (new PointCloudLoader(data_file));
    ImageLoader::Ptr image_loader (new ImageLoader(data_file));
    Calibration::Ptr calibration (new Calibration(data_file));

    pcl::visualization::PCLVisualizer viewer("result");//pcl viewer
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    for(int i = 0; i < 2700; i = i +10)
    {
        cout << "========== fram number :" << i << "=============================" << endl;
        boost::timer timer;
        //  input data
        point_cloud_loader->readKittiPclBinData(i);
        image_loader->readKittiImage(i);
        calibration->ReadGroundTruth(i);
        //  Assignment
        MRF::Ptr mrf (new MRF);
        mrf->calibration_ = calibration;
        mrf->raw_cloud_ = point_cloud_loader->raw_cloud_;
        mrf->raw_image_ = image_loader->image_;
        mrf->MRFProcess();
        // for()
        cout<<"total cost time: "<<timer.elapsed() <<endl;

        for(int j = 0; j < mrf->result_cloud_->points.size(); j++)
        {
            result_cloud->points.emplace_back(mrf->result_cloud_->points[j]);
        }

        // cv::imshow("image", mrf->raw_image_);
        // cv::waitKey(0);
        // cv::destroyWindow("image");
        viewer.addPointCloud(result_cloud,to_string(i));
        viewer.setBackgroundColor(0,0,0);
        viewer.addCoordinateSystem();
        viewer.spin();
        viewer.removeCoordinateSystem();
        viewer.removeAllPointClouds(); 
    }
    return 0;
}

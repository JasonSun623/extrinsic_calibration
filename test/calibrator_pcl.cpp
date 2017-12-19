// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>

#include "extrinsic_calibration/calibrator.h"
#include "extrinsic_calibration/cvutils.h"

using namespace extrinsic_calibration;
using namespace std;

#define KINECT_COLOR_HEIGHT 540
#define KINECT_COLOR_WIDTH 960

const int N_REQUIRED_SAMPLES = 1;

typedef pcl::PointXYZRGB PointT;

template <typename PointT>
std::vector<int> filterCloudNan(pcl::PointCloud<PointT> &cloud, pcl::PointCloud<PointT> &out_cloud)
{
    int cloud_sz = cloud.points.size();
    auto &out_pts = out_cloud.points;
    out_pts.clear();
    out_pts.reserve(cloud_sz);
    std::vector<int> good_indices;
    for (unsigned int i = 0; i < cloud_sz; ++i)
    {
        const PointT &pt = cloud.points[i];
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z))
        {
            out_pts.push_back(pt);
            good_indices.push_back(i);
        }
    }
    return good_indices;
}

void calib_test(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr out_cloud)
{
    // calibration
    Calibrator calibrator;
    calibrator.nRequiredSamples = N_REQUIRED_SAMPLES;

    // init marker poses
    MarkerPose m_pose;
    m_pose.markerId = 0; // we are using calibration board marker 0 -> 0.gif
    calibrator.markerPoses.push_back(m_pose);

    std::vector<float> m_vBounds{-0.5, -0.5, -0.5, 0.5, 0.5, 0.5};
    // point3f buffer
    vector<Point3f> pCameraCoordinates;
    vector<PointRGB> color;
    pCameraCoordinates.reserve(KINECT_COLOR_HEIGHT * KINECT_COLOR_WIDTH);
    color.reserve(KINECT_COLOR_HEIGHT * KINECT_COLOR_WIDTH);

    // pcl viewer
    for (int i = 0; i < cloud->points.size(); ++i)
    {
        const PointT &cloud_pt = cloud->points[i];
        pCameraCoordinates.push_back(Point3f(cloud_pt.x, cloud_pt.y, cloud_pt.z));
        color.push_back(PointRGB(cloud_pt.r, cloud_pt.g, cloud_pt.b));
    }

    bool res = calibrator.calibrate(color, pCameraCoordinates, KINECT_COLOR_WIDTH, KINECT_COLOR_HEIGHT);
    if (res)
    {
        LOGF(DBUG, "calibrate success.");
    }
    else
    {
        LOGF(WARN, "calibrate FAIL.");
        return;
    }

    std::vector<int> good_indices = filterCloudNan(*cloud, *out_cloud);
#pragma omp parallel for
    for (unsigned int i = 0; i < out_cloud->points.size(); i++)
    {
        PointT &pt = out_cloud->points[i];
        pt.x += calibrator.worldT[0];
        pt.y += calibrator.worldT[1];
        pt.z += calibrator.worldT[2];

        pt = CvUtils::RotatePoint<PointT>(pt, calibrator.worldR);
        PointT &pt_ori = cloud->points[good_indices[i]];
        pt.r = pt_ori.r;
        pt.g = pt_ori.g;
        pt.b = pt_ori.b;
    }

    // pcl::toPCLPointCloud2(*out_cloud, cloud2_pcl);
    // pcl_conversions::moveFromPCL(cloud2_pcl, final_pc2_msg_);

    // final_pc2_msg_.header.seq = 0;
    // final_pc2_msg_.header.stamp = ros::Time::now();
    // final_pc2_msg_.header.frame_id = cloud_msg->header.frame_id;
    // final_cloud_pub_.publish(final_pc2_msg_);

}

int main(int argc, char **argv)
{
    std::string pcd_file_1 = "/home/vincent/catkin_ws2/src/extrinsic_calibration/data/1511494760.pcd";
    std::string pcd_file_2 = "/home/vincent/catkin_ws2/src/extrinsic_calibration/data/1511494760.pcd";

    if (argc > 1)
    {
        pcd_file_1 = argv[1];
    }
    if (argc > 2)
    {
        pcd_file_2 = argv[2];
    }

    //
    pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr out_cloud1(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr out_cloud2(new pcl::PointCloud<PointT>);
    if(pcl::io::loadPCDFile(pcd_file_1, *cloud1) == -1)
    {
        std::cerr << "Couldn't read pcd file! " << pcd_file_1 << "\n";
        return -1;
    }
    if(pcl::io::loadPCDFile(pcd_file_2, *cloud2) == -1)
    {
        std::cerr << "Couldn't read pcd file! " << pcd_file_2 << "\n";
        return -1;
    }
    calib_test(cloud1, out_cloud1);

    calib_test(cloud2, out_cloud2);

    pcl::PointCloud<PointT>::Ptr merged_cloud(new pcl::PointCloud<PointT>);
    *merged_cloud += *out_cloud1;
    *merged_cloud += *out_cloud2;



    return 0;
}

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>


#include "extrinsic_calibration/calibrator.h"
#include "extrinsic_calibration/cvutils.h"

using namespace extrinsic_calibration;
using namespace std;

#define KINECT_COLOR_HEIGHT 424
#define KINECT_COLOR_WIDTH 512

#define PRINT(a) std::cout << #a << ": " << a << std::endl;

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

bool find_pattern_and_transform(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr out_cloud)
{
    // calibration
    Calibrator calibrator;
    calibrator.nRequiredSamples = 1;

    // init marker poses
    MarkerPose m_pose;
    m_pose.markerId = 0; // we are using calibration board marker 0 -> 0.gif
    calibrator.markerPoses.push_back(m_pose);

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

    bool res = calibrator.calibrate(color, pCameraCoordinates, KINECT_COLOR_WIDTH, KINECT_COLOR_HEIGHT, true);
    if (res)
    {
        LOGF(DBUG, "calibrate success.");
    }
    else
    {
        LOGF(WARN, "calibrate FAIL. Could not detect the full pattern in the cloud");
        return false;
    }

    const auto& T = calibrator.worldT;
    const auto& R = calibrator.worldR;

    std::vector<int> good_indices = filterCloudNan(*cloud, *out_cloud);
#pragma omp parallel for
    for (unsigned int i = 0; i < out_cloud->points.size(); i++)
    {
        PointT& pt = out_cloud->points[i];
        pt.x += T[0];
        pt.y += T[1];
        pt.z += T[2];

        pt = CvUtils::RotatePoint<PointT>(pt, R);

        const PointT &pt_ori = cloud->points[good_indices[i]];

        pt.r = pt_ori.r;
        pt.g = pt_ori.g;
        pt.b = pt_ori.b;
    }

    return true;
}

int main(int argc, char **argv)
{
    std::string pcd_file_1;// = "../data/1.pcd";
    std::string pcd_file_2;// = "../data/2.pcd";

    if (argc != 3)
    {
        printf("Usage: %s pcd_file_1 pcd_file_2\n", argv[0]);
        return -1;
    }

    pcd_file_1 = argv[1];
    pcd_file_2 = argv[2];

    //
    pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr out_cloud1(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr out_cloud2(new pcl::PointCloud<PointT>);

    const size_t CLOUD_SIZE = KINECT_COLOR_HEIGHT * KINECT_COLOR_WIDTH;
    if(pcl::io::loadPCDFile(pcd_file_1, *cloud1) == -1)
    {
        std::cerr << "Couldn't read pcd file! " << pcd_file_1 << "\n";
        return -1;
    } else {
        if (cloud1->size() != CLOUD_SIZE)
        {
            printf("%s cloud size of %d does not match [%d x %d]\n", pcd_file_1.c_str(), cloud1->size(), KINECT_COLOR_HEIGHT, KINECT_COLOR_WIDTH);
            return -1;
        }
    }
    if(pcl::io::loadPCDFile(pcd_file_2, *cloud2) == -1)
    {
        std::cerr << "Couldn't read pcd file! " << pcd_file_2 << "\n";
        return -1;
    } else {
        if (cloud2->size() != CLOUD_SIZE)
        {
            printf("%s cloud size of %d does not match [%d x %d]\n", pcd_file_2.c_str(), cloud2->size(), KINECT_COLOR_HEIGHT, KINECT_COLOR_WIDTH);
            return -1;
        }
    }


    pcl::PointCloud<PointT>::Ptr merged_cloud(new pcl::PointCloud<PointT>);
    *merged_cloud += *cloud1;
    *merged_cloud += *cloud2;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Merged cloud"));
    viewer->addCoordinateSystem();
    viewer->addPointCloud (merged_cloud, "mergedcloud");
    std::cout << "Showing initial clouds" << std::endl;
    viewer->spin();
    
    if (!find_pattern_and_transform(cloud1, out_cloud1))
    {
        return -1;
    }
    if (!find_pattern_and_transform(cloud2, out_cloud2))
    {
        return -1;
    }

    merged_cloud->clear();
    *merged_cloud += *out_cloud1;
    *merged_cloud += *out_cloud2;

    viewer->removeAllPointClouds();
    viewer->addPointCloud (merged_cloud, "mergedcloud");
    std::cout << "Showing final aligned clouds" << std::endl;
    viewer->spin();

    // std::string save_file = "../data/v1.pcd";
    // pcl::io::savePCDFileBinary(save_file, *out_cloud1);
    // printf("Saved to %s\n", save_file.c_str());

    // save_file = "../data/v2.pcd";
    // pcl::io::savePCDFileBinary(save_file, *out_cloud2);
    // printf("Saved to %s\n", save_file.c_str());


    return 0;
}

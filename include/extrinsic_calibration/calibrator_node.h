#ifndef CAMERA_EXTRINSIC_CALIBRATOR_H_
#define CAMERA_EXTRINSIC_CALIBRATOR_H_

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>

// OpenCV
#include <opencv/cxcore.h>
#include <opencv/cv.h>

// local includes
#include <extrinsic_calibration/utils.h>
#include <extrinsic_calibration/cvutils.h>
#include <extrinsic_calibration/marker_detector.h>
#include <extrinsic_calibration/calibrator.h>
#include <extrinsic_calibration/laser_marker.h>

/*
 * FIX ME:
 * Following functions need to be implemented in CameraExtrinsicCalibrator class
 * 1. findRigidTransform(findRigidTransform(const ScanPointSet& cluster, const cv::Mat& img)
 *    Finds the rigid body transformation between the lidar and camera frames.
 */

namespace extrinsic_calibration
{

// constants
static const int COLOR_IMAGE_WIDTH  = 960;
static const int COLOR_IMAGE_HEIGHT = 540;

static const std::string CAMERA_COLOR_TOPIC = "/kinect2/qhd/image_color";
static const std::string CAMERA_DEPTH_TOPIC = "/kinect2/qhd/points";
static const std::string FINAL_CLOUD_TOPIC  = "/kinect2/calibrated/cloud";

/**
 * @brief      Removes the pointcloud data which have NaN values.
 *
 * @param      cloud      The input cloud
 * @param      out_cloud  The output cloud
 * @return     The indices of valid points in the pointcloud data.
 */
template <typename PointT>
std::vector<int> filterCloudNan(pcl::PointCloud<PointT>& cloud, pcl::PointCloud<PointT>& out_cloud)
{
  int cloud_sz = cloud.points.size();
  auto& out_pts = out_cloud.points;
  out_pts.clear(); 
  out_pts.reserve(cloud_sz);
  std::vector<int> good_indices;
  for (unsigned int i=0; i < cloud_sz; ++i)
  {
    const PointT& pt = cloud.points[i];
    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z))
    {
      out_pts.push_back(pt);
      good_indices.push_back(i);
    }
  }
  return good_indices;
}


typedef pcl::PointXYZRGB PointT;

/**
 * @brief      Class for RGBD-camera extrinsic-calibration.
 */
class CameraExtrinsicCalibrator
{
public:
  CameraExtrinsicCalibrator():calibrator_(new Calibrator())
  {
    // ROS node parameters
    std::string cloud_topic;
    ROS_INFO("pcloud_node launched");
    // Parameters
    nh_.param("cloud_topic", cloud_topic, std::string(CAMERA_DEPTH_TOPIC));
    // Subscribers
    cloud_sub_ = nh_.subscribe(cloud_topic, 10, &CameraExtrinsicCalibrator::cloudCallback, this);
    // Publishers
    final_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(FINAL_CLOUD_TOPIC, 1);
    markers_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 20);
    image_transport::ImageTransport image_transport(nh_);
    image_pub_ = image_transport.advertise("camera/image/trial",1);
    initCalibration();
    initDataBuffer();
  }
  ~CameraExtrinsicCalibrator()
  {}

  /**
   * @brief      Callback for the pointcloud data (depth-registered).
   * @param[in]  cloud_msg  The cloud message.
   */
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    pcl::fromROSMsg(*cloud_msg, *pcloud_);
    const int cloud_sz = pcloud_->size();
    if (!calibrator_->bCalibrated)
    {
      if (cloud_sz != pCameraCoordinates_.size()) {
        pCameraCoordinates_.resize(cloud_sz);
        color_.resize(cloud_sz);
      }
      for (int i=0; i<cloud_sz; ++i) {
        const PointT& cloud_pt = pcloud_->points[i];
        pCameraCoordinates_[i] = Point3f(cloud_pt.x,cloud_pt.y,cloud_pt.z);
        color_[i] = PointRGB(cloud_pt.r,cloud_pt.g,cloud_pt.b);
      }
      bool result = calibrator_->calibrate(color_, pCameraCoordinates_, COLOR_IMAGE_WIDTH, COLOR_IMAGE_HEIGHT);
      if (result) 
      {
        LOGF(DBUG, "calibration successful");
      }
      else return;
    }
    std::vector<int> good_indices = filterCloudNan(*pcloud_, *filtered_cloud_);
    
    #pragma omp parallel for
    for (unsigned int i = 0; i < filtered_cloud_->points.size(); i++)
    {
      PointT& pt = filtered_cloud_->points[i];
      pt.x += calibrator_->worldT[0];
      pt.y += calibrator_->worldT[1];
      pt.z += calibrator_->worldT[2];

      pt = CvUtils::RotatePoint<PointT>(pt, calibrator_->worldR);
      PointT& pt_ori = pcloud_->points[good_indices[i]];
      pt.r = pt_ori.r;
      pt.g = pt_ori.g;
      pt.b = pt_ori.b;
    }
    pcl::toPCLPointCloud2(*filtered_cloud_, cloud2_pcl);
    pcl_conversions::moveFromPCL(cloud2_pcl, final_pc2_msg_);

    final_pc2_msg_.header.seq = 0;
    final_pc2_msg_.header.stamp = ros::Time::now();
    final_pc2_msg_.header.frame_id = cloud_msg->header.frame_id;
    final_cloud_pub_.publish(final_pc2_msg_);
  }

  void setCalibrationNSamples(unsigned int n_samples)
  {
    calib_n_samples_ = n_samples;
    calibrator_->nRequiredSamples = calib_n_samples_;
  }
  
  unsigned int getCalibrationNSamples() const
  {
    return calib_n_samples_;
  }

  void setCloudBounds(std::vector<float> bounds)
  {
    int size = bounds.size() > cloud_bounds_.size() ? cloud_bounds_.size() : bounds.size();
    for (int i=0; i<size; ++i) {
      cloud_bounds_[i] = bounds[i];
    }
  }

  std::vector<float> getCloudBounds() const
  {
    return cloud_bounds_;
  }

  std::vector<float> cloud_bounds_ {-0.5, -0.5, -0.5, 0.5, 0.5, 0.5};

private:
  std::unique_ptr<Calibrator> calibrator_;
  unsigned int calib_n_samples_ = 10;

  // cloud data
  pcl::PointCloud<PointT>::Ptr pcloud_;
  pcl::PointCloud<PointT>::Ptr filtered_cloud_;
  pcl::PCLPointCloud2 cloud2_pcl;

  // ROS params
  sensor_msgs::PointCloud2 final_pc2_msg_;
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher final_cloud_pub_;
  ros::Publisher markers_pub_;
  image_transport::Publisher image_pub_;

  // buffer data for calibration
  std::vector<Point3f> pCameraCoordinates_;
  std::vector<PointRGB> color_;

  void initCalibration()
  {
    calibrator_->nRequiredSamples = calib_n_samples_;
    // init marker poses
    MarkerPose m_pose;
    m_pose.markerId = 0;   // we are using calibration board marker 0 -> 0.gif
    calibrator_->markerPoses.push_back(m_pose);
  }

  void initDataBuffer()
  {
    const int total_buffer_sz = COLOR_IMAGE_HEIGHT * COLOR_IMAGE_WIDTH;
    pCameraCoordinates_.resize(total_buffer_sz);
    color_.resize(total_buffer_sz);
    pcloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    filtered_cloud_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
  }

  /* Note: Following functions need to be implemented. */
  /**
   * @brief      Finds the rigid body transformation between a laser cluster (line) and camera's image.
   *
   * @param      cluster  The cluster
   * @param[in]  img      The image
   */
  void findRigidTransform(const ScanPointSet& cluster, const cv::Mat& img);

};

}

#endif
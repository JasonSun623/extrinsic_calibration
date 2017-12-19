/**
 * Adopted from A. Leigh, J. Pineau, N. Olmedo and H. Zhang, Person Tracking and Following 
 * with 2D Laser Scanners, International Conference on Robotics and Automation (ICRA),
 * Seattle, Washington, USA, 2015.
 * https://github.com/angusleigh/leg_tracker
 */

/* FIX ME: Additional functions need to implemented in LaserProcessor class
 * 1. ScanPointSet* findCalibrationLine()
 *    Finds a single cluster of laser scan-points which shows the calibration plane.
 *    Using this single cluster as the reference for calibration.  
 */

#ifndef LASER_MARKER_H_
#define LASER_MARKER_H_

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

// std-library utilities
#include <math.h>
#include <list>
#include <set>

static const float EUCLIDEAN_THRESHOLD = 0.25; 
static const int MIN_POINTS_PER_CLUSTER = 6;

namespace extrinsic_calibration
{

/**
 * @brief      Class for denoting a scan-point in laser-scan data.
 */
class ScanPoint
{
public:
  int index;
  float range;
  float intensity;
  float x;
  float y;
  /**
   * @brief      Extracts a ScanPoint from a laser-scan message.
   * @param[in]  idx   The index of the scan point.
   * @param[in]  scan  The laser-scan message.
   *
   * @return     Extracted ScanPoint.
   */
  static ScanPoint* extract(int idx, const sensor_msgs::LaserScan& scan);
};

/**
 * @brief      Struct for initializing an ordered set of ScanPoint(s).
 */
struct CompareScanPoint
{
  /**
   * @brief      A comparator to have an initialization of ordered set of ScanPoint(s).
   */
  inline bool operator() (const ScanPoint* a, const ScanPoint* b)
  {
    return (a->index < b->index);
  }
};

/**
 * @brief      Class for denoting a set of scan-points.
 */
class ScanPointSet : public std::set<ScanPoint*, CompareScanPoint>
{
public:
  ~ScanPointSet()
  {
    clear();
  }
  void clear()
  {
    for (auto it = begin(); it != end(); ++it)
      delete *it;
    std::set<ScanPoint*, CompareScanPoint>::clear();
  }
};

/**
 * @brief      Class for processing laser-scans. 
 */
class LaserProcessor
{
public:
  explicit LaserProcessor(): clusters_(), scan_()
  {}
  LaserProcessor(const sensor_msgs::LaserScan& scan_msg): 
    clusters_(),
    scan_(scan_msg)   
  {
    ScanPointSet* cluster = new ScanPointSet;
    for (int i=0; i < scan_.ranges.size(); ++i)
    {
      ScanPoint* s = ScanPoint::extract(i, scan_);
      if (s!=NULL) cluster->insert(s);
    }
    clusters_->push_back(cluster);
  }

  ~LaserProcessor()
  {
    for (auto it = clusters_->begin(); it != clusters_->end(); ++it)
    {
      delete (*it);
    }
  }
  /**
   * @brief      Filters (removes) the clusters which have less than \p numPoints scan-points.
   * @param[in]  numPoints  The minimum number of points for a cluster to have.
   */
  void passThrough(uint32_t numPoints);

  /**
   * @brief      Forms clusters of scan-points from a scan message, bounded by a distance threshold.
   * @param[in]  distThresh  The distance threshold for clustering nearby scan-points.
   */
  void formClusters(float distThresh);

  std::list<ScanPointSet*>& getClusters() { return *clusters_; }

private:
  std::unique_ptr<std::list<ScanPointSet*>> clusters_;
  sensor_msgs::LaserScan scan_;

  /* Note: Following functions need to be implemented. */

  /**
   * @brief      Finds the line of scan-points corresponding to the calibration plane.
   * @return     A ScanPointSet denoting the cluster of that one single line representing the calibration plane.
   */
  ScanPointSet* findCalibrationLine();
  /**
   * @brief      Publishes cluster corners in form of RViz markers.
   */
  void publishClusterCorners();
};

/**
 * @brief      Class for laser-marker ROS node.
 */
class LaserMarker
{
public:
  LaserMarker() : processor()
  {
    std::string scan_topic;
    nh_.param("scan_topic", scan_topic, std::string("scan"));
    nh_.param("fixed_frame", fixed_frame, std::string("map"));
    // subscibers and publishers
    laser_sub_ =  nh_.subscribe(scan_topic, 10, &LaserMarker::laserCallback, this);
    markers_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 20);
  }
  ~LaserMarker()
  {}
  /**
   * @brief      Callback for laser scan message.
   * @param[in]  scan_msg  The laser scan message.
   */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber laser_sub_;
  ros::Publisher markers_pub_;
  std::unique_ptr<LaserProcessor> processor;
  std::string fixed_frame;
};

} 

#endif
#include <extrinsic_calibration/laser_marker.h>

using namespace extrinsic_calibration;

ScanPoint* ScanPoint::extract(int idx, const sensor_msgs::LaserScan& scan)
{
  ScanPoint* sp = new ScanPoint();
  sp->index = idx;
  sp->intensity = scan.intensities[idx];
  sp->range = scan.ranges[idx];
  sp->x = cos(scan.angle_min + idx*scan.angle_increment) * sp->range;
  sp->y = sin(scan.angle_min + idx*scan.angle_increment) * sp->range;
  if (sp->range > scan.range_min && sp->range < scan.range_max) 
  {
    return sp;
  }
  else 
  {
    delete sp;
    return NULL;
  }
}

/**
 * @brief      Removes the clusters which have less than \p numPoints scan-points.
 */
void LaserProcessor::passThrough(uint32_t numPoints)
{
  for (auto it = clusters_->begin(); it != clusters_->end(); ++it)
  {
    if ((*it)->size() < numPoints)
    {
      delete (*it);
      clusters_->erase(it);
    }
  }
}

/**
 * @brief      Cluster scan-points by their Euclidean-distance from other scan-points.
 */
void LaserProcessor::formClusters(float distThresh)
{
  std::list<ScanPointSet*> tmpClusters;
  std::list<ScanPointSet*>::iterator clItr = clusters_->begin();
  // Iterate through all clusters of scan-points
  while (clItr != clusters_->end())
  {
    while ((*clItr)->size() > 0) 
    {
      // Gather the scan-points which are within distThresh distance from rest of the scan-points.
      ScanPointSet::iterator firstScanPtSet = (*clItr)->begin();
      std::list<ScanPoint*> scanPtList;
      scanPtList.push_back(*firstScanPtSet);
      (*clItr)->erase(firstScanPtSet);
      std::list<ScanPoint*>::iterator spItr = scanPtList.begin();  // scan-points iterator within a cluster
      while (spItr != scanPtList.end())
      {
        int expand = (int)(asin(distThresh / (*spItr)->range) / scan_.angle_increment);
        ScanPointSet::iterator otherPtsItr = (*clItr)->begin();
        // Loop through all other scan-points within a cluster
        while (otherPtsItr != (*clItr)->end() && (*otherPtsItr)->index < (*spItr)->index + expand)
        {
          if (sqrt(pow((*spItr)->x - (*otherPtsItr)->x, 2.0f) + pow((*spItr)->y - (*otherPtsItr)->y, 2.0f)) < distThresh)
          {
            scanPtList.push_back(*otherPtsItr);  // add the nearby scan-point to list of scan-points
            (*clItr)->erase(otherPtsItr++);
          }
          else 
          {
            ++otherPtsItr;
          }
        }
        ++spItr;
      }

      // Form new clusters
      ScanPointSet* cluster = new ScanPointSet;
      for (spItr = scanPtList.begin(); spItr != scanPtList.end(); ++spItr)
        cluster->insert(*spItr);

      // Push new cluster to list of clusters.
      tmpClusters.push_back(cluster);
    }
    delete (*clItr);
    clusters_->erase(clItr++);
  }
  clusters_->insert(clusters_->begin(), tmpClusters.begin(), tmpClusters.end());
}

void LaserMarker::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  processor = std::unique_ptr<LaserProcessor>(new LaserProcessor(*scan_msg));
  // LaserProcessor processor(*scan_msg);
  processor->formClusters(EUCLIDEAN_THRESHOLD);
  processor->passThrough(MIN_POINTS_PER_CLUSTER);

  /**
   * Note:
   * Add code here to compute the transformation between laser scan-points
   * and the RGB(D) image. 
   * If required, filter for one laser-scan line, corresponding to the calibration plane,
   * out of the clusters_ obtained above.
   */
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_marker");
  ROS_INFO("laser_marker node initialized");
  LaserMarker marker;
  ros::spin();
  return 0;
}


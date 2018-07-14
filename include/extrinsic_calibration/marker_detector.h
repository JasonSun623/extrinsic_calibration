/**
 * Adopted from:
 * Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
 * Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015
 * https://github.com/MarekKowalski/LiveScan3D
 */

#ifndef MARKER_DETECTOR_H_
#define MARKER_DETECTOR_H_

// PCL
#include <pcl/point_cloud.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>

// OpenCV
// #include <opencv/cxcore.h>
// #include <opencv/cv.h>
#include <opencv2/opencv.hpp>

// local includes
#include <extrinsic_calibration/utils.h>

#include "utils.h"

namespace extrinsic_calibration
{

struct MarkerInfo
{
  int id;
  std::vector<Point2f> corners;
  std::vector<Point3f> points;

  MarkerInfo()
  {
    id = -1;
  }

  MarkerInfo(int id, const std::vector<Point2f>& corners, const std::vector<Point3f>& points)
  {
    this->id = id;
    this->corners = corners;
    this->points = points;
  }
};


struct MarkerPose
{
  int markerId = 0;
  float R[3][3];
  float t[3];
  std::vector<float> r {0.0f,0.0f,0.0f};

  MarkerPose()
  {
    for (int i = 0; i < 3; ++i) {
      t[i] = 0;
      for (int j = 0; j < 3; ++j) {
        if (i == j) R[i][j] = 1;
        else R[i][j] = 0;
      }
    }
    updateRotationMatrix();
  }

  inline void setOrientation(float X, float Y, float Z)
  {
    r[0] = X;
    r[1] = Y;
    r[2] = Z;
    updateRotationMatrix();
  }

  inline void getOrientation(float& X, float& Y, float& Z)
  {
    X = r[0];
    Y = r[1];
    Z = r[2];
  }

  inline void updateRotationMatrix()
  {
    float radX = r[0] * (float)PI / 180.0f;
    float radY = r[1] * (float)PI / 180.0f;
    float radZ = r[2] * (float)PI / 180.0f;

    float c1 = (float)cos(radZ);
    float c2 = (float)cos(radY);
    float c3 = (float)cos(radX);
    float s1 = (float)sin(radZ);
    float s2 = (float)sin(radY);
    float s3 = (float)sin(radX);

    //Z Y X rotation
    R[0][0] = c1 * c2;  // at x=0 and y=0, cos(x) * cos(y) = 1
    R[0][1] = c1 * s2 * s3 - c3 * s1;
    R[0][2] = s1 * s3 + c1 * c3 * s2;
    R[1][0] = c2 * s1;
    R[1][1] = c1 * c3 + s1 * s2 * s3;  // 1 + 0 = 1
    R[1][2] = c3 * s1 * s2 - c1 * s3;
    R[2][0] = -s2;
    R[2][1] = c2 * s3;
    R[2][2] = c2 * c3;    // 1 * 1 = 1

    // 1 0 0
    // 0 1 0
    // 0 0 1
  }
};



class MarkerDetector
{
public:
  MarkerDetector();
  ~MarkerDetector()
  {}
  
  bool bDraw = false;
  bool findMarkers(const std::vector<PointRGB>& img, int height, int width, MarkerInfo &marker);
  inline void setBinaryThreshold(int thresh)
  {
    nThreshold = thresh;
  }
private:
  //image processing parameters
  int nMarkerCorners;
  std::vector<cv::Point2f> vPts;
  int nMinSize;
  int nMaxSize;
  int nThreshold;
  double dApproxPolyCoef;
  double dMarkerFrame;

  bool findMarkers(cv::Mat& img, MarkerInfo& marker);
  void getMarkerPoints(std::vector<Point3f>& pts);
  void getMarkerPointsForWarp(std::vector<cv::Point2f>& pts);
  int getCode(cv::Mat& img, std::vector<cv::Point2f> points, const std::vector<cv::Point2f>& corners);
  bool getCorners(std::vector<cv::Point2f>& corners);
  void cornersSubPix(std::vector<cv::Point2f>& corners, const std::vector<cv::Point>& contours, bool order);
  cv::Point2f getIntersection(const cv::Vec4f& lin1, const cv::Vec4f& lin2);
  double getMarkerArea(const MarkerInfo& marker);

};

}

#endif
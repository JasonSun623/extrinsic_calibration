/**
 * Adopted from:
 * Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
 * Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015
 * https://github.com/MarekKowalski/LiveScan3D
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include <extrinsic_calibration/marker_detector.h>
#include <extrinsic_calibration/utils.h>
#include <extrinsic_calibration/cvutils.h>
#include <opencv2/opencv.hpp>

namespace extrinsic_calibration
{

/**
 * @brief      Class for carrying out extrinsic-calibration of an RGBD camera.
 */
class Calibrator
{
public:
  int iUsedMarkerId = 0;

  std::vector<float> worldT;
  std::vector<std::vector<float>> worldR;
  std::vector<float> cameraT;
  std::vector<std::vector<float>> cameraR;
  std::vector<MarkerPose> markerPoses;
  bool bCalibrated = false;
  int nRequiredSamples = 10;

  // constructor, destructor
  Calibrator();
  ~Calibrator();

  bool calibrate(const std::vector<PointRGB>& pBuffer,
    const std::vector<Point3f>& pCameraCoordinates,
    int cColorWidth, 
    int cColorHeight, bool visualize=false);

private:
  std::unique_ptr<MarkerDetector> pDetector;
  int nSampleCounter;
  std::vector<std::vector<Point3f>> marker3DSamples;

  void procrustes(const MarkerInfo &marker,
    const std::vector<Point3f> &markerInWorld,
    std::vector<float> &markerT,
    std::vector<std::vector<float>> &markerR);
  
  template <typename PointXYZ>
  bool getMarkerCorners3D(std::vector<Point3f> &marker3D, 
    const MarkerInfo &marker, 
    const std::vector<PointXYZ> &pCameraCoordinates, 
    int cColorWidth, 
    int cColorHeight);
};

}

#endif

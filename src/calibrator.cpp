/**
 * Adopted from:
 * Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
 * Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015
 * https://github.com/MarekKowalski/LiveScan3D
 */

#include <extrinsic_calibration/calibrator.h>

namespace extrinsic_calibration
{

using std::vector;

Calibrator::Calibrator():pDetector(new MarkerDetector())
{
  iUsedMarkerId = 0;
  bCalibrated = false;
  nSampleCounter = 0;
  nRequiredSamples = 10;
  worldT = vector<float>(3, 0.0f);
  for (int i = 0; i < 3; i++)
  {
    worldR.push_back(vector<float>(3, 0.0f));
    worldR[i][i] = 1.0f;
  }
}

Calibrator::~Calibrator()
{}

/**
 * @brief      Finds the transformation matrix between the calibration plane (having markers)
 *             and the camera. 
 *             
 * @param[in]  pBuffer             The pixel values (RGB) of the (depth-registered) image.
 * @param[in]  pCameraCoordinates  The xyz information of the pixels in the depth-registered image.
 * @param[in]  cColorWidth         The width of the image.
 * @param[in]  cColorHeight        The height of the image.
 *
 * @return     Boolean true if calibration is successful, false if not.
 */
bool Calibrator::calibrate(const std::vector<PointRGB>& pBuffer,
  const std::vector<Point3f>& pCameraCoordinates,
  int cColorWidth, 
  int cColorHeight, bool visualize)
{
  LOGF(INFO, "enter calibrate function.");
  if (pCameraCoordinates.size() == 0)
  {
    LOGF(INFO, "Size of coordinates is empty!");
    return false;
  }
  MarkerInfo marker;
  pDetector->bDraw = visualize;
  bool res = pDetector->findMarkers(pBuffer, cColorHeight, cColorWidth, marker);  // this only changes marker.corners. you can view the markers with bDraw=True

  if (!res)
  {
    LOGF(ERROR, "getMarker failed");
    return false;
  }
  int indexInPoses = -1;
  // PRINT(markerPoses.size())
  for(unsigned int j = 0; j < markerPoses.size(); j++)
  { 
    if(marker.id == markerPoses[j].markerId)
    {
      indexInPoses = j;
      break;
    }
  }
  if (indexInPoses == -1)
  {
    for (unsigned int j = 0; j < markerPoses.size(); j++)
    {
      LOGF(ERROR, "markerPoses marker.id " + std::to_string(marker.id) + " markerPoses[" + std::to_string(markerPoses[j].markerId) + "].markerId.");
    }
    LOGF(ERROR, "markerPoses " + std::to_string(markerPoses.size()) + " failed");
    return false;
  }

  const MarkerPose& markerPose = markerPoses[indexInPoses];
  iUsedMarkerId = markerPose.markerId;
  std::vector<Point3f> marker3D(marker.corners.size());
  bool success = getMarkerCorners3D(marker3D, marker, pCameraCoordinates, cColorWidth, cColorHeight);
  if(!success)
  {
    LOGF(ERROR, "getMarkerCorners3D failed");
    return false;
  }

  marker3DSamples.push_back(marker3D);
  nSampleCounter++;
  if (nSampleCounter < nRequiredSamples)
  {
    LOGF(WARNING, "nSampleCounter " + std::to_string(nSampleCounter) + " less than nRequiredSamples " + std::to_string(nRequiredSamples));
    return false;
  }

  // averaging
  for(size_t i = 0; i < marker3D.size(); i++)
  {
    marker3D[i] = Point3f();
    for(int j = 0; j < nRequiredSamples; j++)
    {
      marker3D[i].x += marker3DSamples[j][i].x / (float)nRequiredSamples;
      marker3D[i].y += marker3DSamples[j][i].y / (float)nRequiredSamples;
      marker3D[i].z += marker3DSamples[j][i].z / (float)nRequiredSamples;
    }
    // printf("marker3D %.3f,%.3f,%.3f\n", marker3D[i].x,marker3D[i].y,marker3D[i].z);
  }

  procrustes(marker, marker3D, worldT, worldR);
  std::vector<std::vector<float>> Rcopy = worldR;
  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      worldR[i][j] = 0;
      for(int k = 0; k < 3; k++)
      {
        worldR[i][j] += markerPose.R[i][k] * Rcopy[k][j];
      }
    }
    printf("worldR[%d] %.2f,%.2f,%.2f\n", i, worldR[i][0],worldR[i][1],worldR[i][2]);
  }
  std::vector<float> translationIncr(3);
  translationIncr[0] = markerPose.t[0];
  translationIncr[1] = markerPose.t[1];
  translationIncr[2] = markerPose.t[2];
  translationIncr = CvUtils::InverseRotatePoint(translationIncr, worldR);

  worldT[0] += translationIncr[0];
  worldT[1] += translationIncr[1];
  worldT[2] += translationIncr[2];

  printf("worldT %.2f,%.2f,%.2f\n", worldT[0], worldT[1], worldT[2]);
  bCalibrated = true;
  marker3DSamples.clear();
  nSampleCounter = 0;
  LOGF(INFO, "exit calibrate function.");
  return true;
}

template <typename PointXYZ>
bool Calibrator::getMarkerCorners3D(std::vector<Point3f> &marker3D, 
  const MarkerInfo &marker, 
  const std::vector<PointXYZ> &pCameraCoordinates, 
  int cColorWidth, 
  int cColorHeight)
{
  for(unsigned int i = 0; i < marker.corners.size(); i++)
  {
    int minX = static_cast<int>(marker.corners[i].x);
    int maxX = minX + 1;
    int minY = static_cast<int>(marker.corners[i].y);
    int maxY = minY + 1;

    float dx = marker.corners[i].x - minX;
    float dy = marker.corners[i].y - minY;

    const PointXYZ& pointMin = pCameraCoordinates[minX + minY * cColorWidth];
    const PointXYZ& pointXMaxYMin = pCameraCoordinates[maxX + minY * cColorWidth];
    const PointXYZ& pointXMinYMax = pCameraCoordinates[minX + maxY * cColorWidth];
    const PointXYZ& pointMax = pCameraCoordinates[maxX + maxY * cColorWidth];

    // PRINT(minX + minY * cColorWidth)
    // PRINT(maxX + minY * cColorWidth)
    // PRINT(minX + maxY * cColorWidth)
    // PRINT(maxX + maxY * cColorWidth)

    // printf("pointMin %.3f,%.3f,%.3f\n", pointMin.x,pointMin.y,pointMin.z);
    // printf("pointXMaxYMin %.3f,%.3f,%.3f\n", pointXMaxYMin.x,pointXMaxYMin.y,pointXMaxYMin.z);
    // printf("pointXMinYMax %.3f,%.3f,%.3f\n", pointXMinYMax.x,pointXMinYMax.y,pointXMinYMax.z);
    // printf("pointMax %.3f,%.3f,%.3f\n", pointMax.x,pointMax.y,pointMax.z);
    // return false;

    if(pointMin.z < 0 || pointXMaxYMin.z < 0 || pointXMinYMax.z < 0 || pointMax.z < 0)
      return false;

    marker3D[i].x = (1 - dx) * (1 - dy) * pointMin.x + dx * (1 - dy) * pointXMaxYMin.x + (1 - dx) * dy * pointXMinYMax.x + dx * dy * pointMax.x;
    marker3D[i].y = (1 - dx) * (1 - dy) * pointMin.y + dx * (1 - dy) * pointXMaxYMin.y + (1 - dx) * dy * pointXMinYMax.y + dx * dy * pointMax.y;
    marker3D[i].z = (1 - dx) * (1 - dy) * pointMin.z + dx * (1 - dy) * pointXMaxYMin.z + (1 - dx) * dy * pointXMinYMax.z + dx * dy * pointMax.z;

    // printf("marker corners: %.3f, %.3f\n", marker.corners[i].x, marker.corners[i].y);
    // printf("marker3D %.3f,%.3f,%.3f\n", marker3D[i].x,marker3D[i].y,marker3D[i].z);
  }
  // cv::waitKey(0);
  return true;
}

/**
 * @brief      Finds the homographic transformation of markers between the
 *             image-frame and world-frame.
 *
 * @param[in]  marker          The marker in image frame
 * @param[in]  markerInWorld   The marker in world frame
 * @param      worldToMarkerT  The world-frame to image-frame translation
 * @param      worldToMarkerR  The world-frame to image-frame rotation
 */
void Calibrator::procrustes(const MarkerInfo &marker, 
  const vector<Point3f> &markerInWorld,
  vector<float> &worldToMarkerT, 
  vector<vector<float>> &worldToMarkerR)
{
  int nVertices = marker.points.size();
  Point3f markerCenterInWorld;
  Point3f markerCenter;

  for(int i = 0; i < nVertices; i++)
  {
    markerCenterInWorld.x += markerInWorld[i].x;
    markerCenterInWorld.y += markerInWorld[i].y;
    markerCenterInWorld.z += markerInWorld[i].z;

    markerCenter.x += marker.points[i].x;
    markerCenter.y += marker.points[i].y;
    markerCenter.z += marker.points[i].z;
  }
    
  markerCenterInWorld.x /= nVertices;
  markerCenterInWorld.y /= nVertices;
  markerCenterInWorld.z /= nVertices;
  markerCenter.x /= nVertices;
  markerCenter.y /= nVertices;
  markerCenter.z /= nVertices;

  // PRINT(markerCenterInWorld.x)
  // PRINT(markerCenterInWorld.y)
  // PRINT(markerCenterInWorld.z)
  // PRINT(markerCenter.x)
  // PRINT(markerCenter.y)
  // PRINT(markerCenter.z)

  worldToMarkerT.resize(3);
  worldToMarkerT[0] = -markerCenterInWorld.x;
  worldToMarkerT[1] = -markerCenterInWorld.y;
  worldToMarkerT[2] = -markerCenterInWorld.z;

  vector<Point3f> markerInWorldTranslated(nVertices);
  vector<Point3f> markerTranslated(nVertices);

  for(int i = 0; i < nVertices; i++)
  {
    markerInWorldTranslated[i].x = markerInWorld[i].x + worldToMarkerT[0];
    markerInWorldTranslated[i].y = markerInWorld[i].y + worldToMarkerT[1];
    markerInWorldTranslated[i].z = markerInWorld[i].z + worldToMarkerT[2];

    markerTranslated[i].x = marker.points[i].x - markerCenter.x;
    markerTranslated[i].y = marker.points[i].y - markerCenter.y;
    markerTranslated[i].z = marker.points[i].z - markerCenter.z;
  }

  cv::Mat A(nVertices, 3, CV_64F);
  cv::Mat B(nVertices, 3, CV_64F);
  for(int i = 0; i < nVertices; i++)
  {
    A.at<double>(i, 0) = markerTranslated[i].x;
    A.at<double>(i, 1) = markerTranslated[i].y;
    A.at<double>(i, 2) = markerTranslated[i].z;
    B.at<double>(i, 0) = markerInWorldTranslated[i].x;
    B.at<double>(i, 1) = markerInWorldTranslated[i].y;
    B.at<double>(i, 2) = markerInWorldTranslated[i].z;
  }

  cv::Mat M = A.t() * B;
  cv::SVD svd;
  svd(M);
  cv::Mat R = svd.u * svd.vt;
  double det = cv::determinant(R);

  if(det < 0)
  {
    cv::Mat temp = cv::Mat::eye(3, 3, CV_64F);
    temp.at<double>(2, 2) = -1;
    R = svd.u * temp * svd.vt;
  }
  worldToMarkerR.resize(3);
  for(int i = 0; i < 3; i++)
  {
    worldToMarkerR[i].resize(3);
    for(int j = 0; j < 3; j++)
    {
      worldToMarkerR[i][j] = static_cast<float>(R.at<double>(i, j));
    }
    // printf("worldToMarkerR[%d] %.2f,%.2f,%.2f\n", i, worldToMarkerR[i][0],worldToMarkerR[i][1],worldToMarkerR[i][2]);
  }
}

}
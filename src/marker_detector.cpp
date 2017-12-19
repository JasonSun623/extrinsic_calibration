/**
 * Adopted from:
 * Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
 * Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015
 * https://github.com/MarekKowalski/LiveScan3D
 */

#include <extrinsic_calibration/marker_detector.h>

namespace extrinsic_calibration
{

MarkerDetector::MarkerDetector()
{
  // image processing parameters
  nMinSize = 100;
  nMaxSize = 1000000000;
  nThreshold = 120;
  dApproxPolyCoef = 0.12;
  dMarkerFrame = 0.4;
  nMarkerCorners = 5;
  bDraw = false;
  getMarkerPointsForWarp(vPts);
}

bool MarkerDetector::findMarkers(const std::vector<PointRGB>& img, int height, int width, MarkerInfo &marker)
{
  cv::Mat cvImg(height, width, CV_8UC3);
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      cvImg.at<cv::Vec3b>(i, j)[0] = img[j + width * i].b;
      cvImg.at<cv::Vec3b>(i, j)[1] = img[j + width * i].g;
      cvImg.at<cv::Vec3b>(i, j)[2] = img[j + width * i].r;
    }
  }
  bool res = findMarkers(cvImg, marker);
  return res;
}

/**
 * @brief      Finds markers in a given image.
 * @details    Contours are extracted from the image, the corners are then found 
 *             as markers.
 *
 * @param      img     The image
 * @param      marker  The marker
 *
 * @return     Boolean true if markers are found, false if not.
 */
bool MarkerDetector::findMarkers(cv::Mat& img, MarkerInfo& marker)
{

  std::vector<MarkerInfo> markers;
  cv::Mat img2, img3;

  cv::imshow("image", img);
  cv::waitKey(0);
  PRINT("cvtColor")

  cv::cvtColor(img, img2, CV_BGR2GRAY);

  PRINT("findMakers 1")

  cv::threshold(img2, img2, nThreshold, 255, CV_THRESH_BINARY);
  img2.copyTo(img3);

  PRINT("findMakers 2")

  std::vector<std::vector<cv::Point>> contours; 
  cv::findContours(img3, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

  PRINT("findMakers 3")

  for (unsigned int i = 0; i < contours.size(); i++)
  {
    double area = cv::contourArea(contours[i]);
    if (area < nMinSize || area > nMaxSize) continue;

    std::vector<cv::Point> corners;
    cv::approxPolyDP(contours[i], corners, sqrt(area)*dApproxPolyCoef, true);

    std::vector<cv::Point2f> cornersFloat;
    for (unsigned int j = 0; j < corners.size(); j++)
    {
      cornersFloat.push_back(cv::Point2f((float)corners[j].x, (float)corners[j].y));
    }

    if (!cv::isContourConvex(corners) && corners.size() == nMarkerCorners && getCorners(cornersFloat))
    { 
      bool order = true;
      int code = getCode(img2, vPts, cornersFloat);
      if (code < 0)
      {
        std::reverse(cornersFloat.begin() + 1, cornersFloat.end());
        code = getCode(img2, vPts, cornersFloat);
        if (code < 0) continue;
        order = false;
      }

      float test = cornersFloat[0].x - cornersFloat[1].x;
      cornersSubPix(cornersFloat, contours[i], order);

      std::vector<Point2f> cornersFloat2(nMarkerCorners);
      std::vector<Point3f> points3D;     

      for (int i = 0; i < nMarkerCorners; i++)
      {
        cornersFloat2[i] = Point2f(cornersFloat[i].x, cornersFloat[i].y);
      }
      getMarkerPoints(points3D);
      markers.push_back(MarkerInfo(code, cornersFloat2, points3D));
      if (bDraw)
      {
        for (unsigned int j = 0; j < corners.size(); j++)
        {
          cv::circle(img, cornersFloat[j], 2, cv::Scalar(0, 50 * j, 0), 1);
          cv::line(img, cornersFloat[j], cornersFloat[(j + 1) % cornersFloat.size()], cv::Scalar(0, 0, 255), 2);
        }
      }
    }
  }
  if (markers.size() > 0)
  {
    double maxArea = 0;
    int maxInd = 0;
    for (unsigned int i = 0; i < markers.size(); i++)
    {
      double marker_area = getMarkerArea(markers[i]);
      if (marker_area > maxArea)
      {
        maxInd = i;
        maxArea = marker_area;
      }
    }
    marker = markers[maxInd];
    if (bDraw)
    {
      for (int j = 0; j < nMarkerCorners; j++)
      {
        cv::Point2f pt1 = cv::Point2f(marker.corners[j].x, marker.corners[j].y);
        cv::Point2f pt2 = cv::Point2f(marker.corners[(j + 1) % nMarkerCorners].x, marker.corners[(j + 1) % nMarkerCorners].y);
        cv::line(img, pt1, pt2, cv::Scalar(0, 255, 0), 2);
      }
      cv::imshow("aa",img);
    }
    return true;
  }
  return false;
}

void MarkerDetector::getMarkerPoints(std::vector<Point3f>& pts)
{
  pts.push_back(Point3f(0.0f, -1.0f, 0.0f));
  pts.push_back(Point3f(-1.0f, -1.6667f, 0.0f));
  pts.push_back(Point3f(-1.0f, 1.0f, 0.0f));
  pts.push_back(Point3f(1.0f, 1.0f, 0.0f));
  pts.push_back(Point3f(1.0f, -1.6667f, 0.0f));
}

void MarkerDetector::getMarkerPointsForWarp(std::vector<cv::Point2f>& pts)
{
  pts.push_back(cv::Point2f(0, 1));
  pts.push_back(cv::Point2f(-1, 1.6667f));
  pts.push_back(cv::Point2f(-1, -1));
  pts.push_back(cv::Point2f(1, -1));
  pts.push_back(cv::Point2f(1, 1.6667f));
}

bool MarkerDetector::getCorners(std::vector<cv::Point2f>& corners)
{
  std::vector<int> hull;
  cv::convexHull(corners, hull);
  if (hull.size() != corners.size() - 1) return false;
  int index = -1;
  for (unsigned int i = 0; i < corners.size(); i++)
  {
    bool found = false;
    for (unsigned int j = 0; j < hull.size(); j++)
    {
      if (hull[j] == i)
      {
        found = true;
        break;
      }
    }
    if (!found)
    {
      index = i;
      break;
    }
  }
  std::vector<cv::Point2f> corners2;
  for (unsigned int i = 0; i < corners.size(); i++)
  {
    corners2.push_back(corners[(index + i)%corners.size()]);
  }
  corners = corners2;
  return true;
}

void MarkerDetector::cornersSubPix(std::vector<cv::Point2f>& corners, const std::vector<cv::Point>& contour, bool order)
{
  std::vector<int> indices(corners.size());
  for (unsigned int i = 0; i < corners.size(); i++)
  {
    for (unsigned int j = 0; j < contour.size(); j++)
    {
      if (corners[i].x == contour[j].x && corners[i].y == contour[j].y)
      {
        indices[i] = j;
        break;
      }
    }
  }
  std::vector<std::vector<cv::Point>> pts(corners.size());
  for (unsigned int i = 0; i < corners.size(); i++)
  {
    int index1, index2;
    if (order)
    {
      index1 = indices[i];
      index2 = indices[(i + 1) % corners.size()];
    }
    else
    {
      index1 = indices[(i + 1) % corners.size()];
      index2 = indices[i];
    }
    if (index1 < index2)
    {
      pts[i].resize(index2 - index1);
      std::copy(contour.begin() + index1, contour.begin() + index2, pts[i].begin());
    }
    else
    {
      pts[i].resize(index2 + contour.size() - index1);
      std::copy(contour.begin() + index1, contour.end(), pts[i].begin());
      std::copy(contour.begin(), contour.begin() + index2, pts[i].end() - index2);
    }
  }

  std::vector<cv::Vec4f> lines(corners.size());
  for (unsigned int i = 0; i < corners.size(); i++)
  {
    cv::fitLine(pts[i], lines[i], CV_DIST_L2, 0, 0.01, 0.01);
  }

  const int corners_sz = corners.size(); 
  corners.clear();
  for (unsigned int i = corners_sz - 1; i < 2 * corners_sz - 1; i++)
  {
    corners.push_back(getIntersection(lines[(i + 1) % corners_sz], lines[i % corners_sz]));
  }
}

cv::Point2f MarkerDetector::getIntersection(const cv::Vec4f& lin1, const cv::Vec4f& lin2)
{
  float c1 = lin2[2] - lin1[2];
  float c2 = lin2[3] - lin1[3];
  float a1 = lin1[0];
  float a2 = lin1[1];
  float b1 = -lin2[0];
  float b2 = -lin2[1];

  cv::Mat A(2, 2, CV_32F);
  cv::Mat b(2, 1, CV_32F);
  cv::Mat dst(2, 1, CV_32F);

  A.at<float>(0, 0) = a1;
  A.at<float>(0, 1) = b1;
  A.at<float>(1, 0) = a2;
  A.at<float>(1, 1) = b2;
  b.at<float>(0, 0) = c1;
  b.at<float>(1, 0) = c2;

  cv::solve(A, b, dst);

  cv::Point2f res(dst.at<float>(0, 0) * lin1[0] + lin1[2], dst.at<float>(0, 0) * lin1[1] + lin1[3]);
  return res;
}

int MarkerDetector::getCode(cv::Mat& img, std::vector<cv::Point2f> points, const std::vector<cv::Point2f>& corners)
{
  cv::Mat H, img2;
  int minX = 0, minY = 0;
  double markerInterior = 2 - 2 * dMarkerFrame;

  for (unsigned int i = 0; i < points.size(); i++)
  {
    points[i].x = static_cast<float>((points[i].x - dMarkerFrame + 1) * 50);
    points[i].y = static_cast<float>((points[i].y - dMarkerFrame + 1) * 50);
  }
  H = cv::findHomography(corners, points);
  
  const int warp_p_side = 50 * markerInterior;
  cv::warpPerspective(img, img2, H, cv::Size(warp_p_side, warp_p_side));

  int xdiff = img2.cols / 3;
  int ydiff = img2.rows / 3;
  int tot = xdiff * ydiff;
  int vals[9];

  cv::Mat integral;
  cv::integral(img2, integral);
  
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      int temp;
      temp = integral.at<int>((i + 1) * xdiff, (j + 1) * ydiff);
      temp += integral.at<int>(i * xdiff, j * ydiff);
      temp -= integral.at<int>((i + 1) * xdiff, j * ydiff);
      temp -= integral.at<int>(i * xdiff, (j + 1) * ydiff);

      temp = temp / tot;

      if (temp < 128)
        vals[j + i * 3] = 0;
      else
        vals[j + i * 3] = 1;
    }
  }

  int ones = 0;
  int code = 0;
  for (int i = 0; i < 4; i++)
  {
    if (vals[i] == vals[i + 4]) return -1;
    else if (vals[i] == 1)
    {
      code += static_cast<int>(pow(2, (double)(3 - i)));
      ones++;
    }
  } 
  if (ones / 2 == (float)ones / 2.0) {
    if (vals[8] == 0) return -1;
  }
  if (ones / 2 != ones / 2.0) { 
    if (vals[8] == 1) return -1;
  }
  return code;
}

double MarkerDetector::getMarkerArea(const MarkerInfo& marker)
{
  cv::Mat hull;
  std::vector<cv::Point2f> cvCorners(nMarkerCorners);
  for (int i = 0; i < nMarkerCorners; i++)
  {
    cvCorners[i] = cv::Point2f(marker.corners[i].x, marker.corners[i].y);
  }
  cv::convexHull(cvCorners, hull);
  return cv::contourArea(hull);
}

}
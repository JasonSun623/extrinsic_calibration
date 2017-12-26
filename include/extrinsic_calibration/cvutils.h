/**
 * Adopted from:
 * Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
 * Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015
 * https://github.com/MarekKowalski/LiveScan3D
 */

#ifndef CVUTILS_H_
#define CVUTILS_H_

#include <extrinsic_calibration/utils.h>
#include <vector>

namespace extrinsic_calibration
{

class CvUtils
{
public:
  CvUtils()
  {}
  ~CvUtils()
  {}

  static std::vector<float> InverseRotatePoint(const std::vector<float> &point, const std::vector<std::vector<float>> &R)
  {
    std::vector<float> res(3);
    res[0] = point[0] * R[0][0] + point[1] * R[1][0] + point[2] * R[2][0];
    res[1] = point[0] * R[0][1] + point[1] * R[1][1] + point[2] * R[2][1];
    res[2] = point[0] * R[0][2] + point[1] * R[1][2] + point[2] * R[2][2];
    return res;
  }

  template <typename PointT>
  static PointT RotatePoint(const PointT &point, const std::vector<std::vector<float>> &R)
  {
    PointT res;
    res.x = point.x * R[0][0] + point.y * R[0][1] + point.z * R[0][2];
    res.y = point.x * R[1][0] + point.y * R[1][1] + point.z * R[1][2];
    res.z = point.x * R[2][0] + point.y * R[2][1] + point.z * R[2][2];
    return res;
  }

  static std::vector<float> RotatePoint(const std::vector<float> &point, const std::vector<std::vector<float>> &R)
  {
    std::vector<float> res(3);
    res[0] = point[0] * R[0][0] + point[1] * R[0][1] + point[2] * R[0][2];
    res[1] = point[0] * R[1][0] + point[1] * R[1][1] + point[2] * R[1][2];
    res[2] = point[0] * R[2][0] + point[1] * R[2][1] + point[2] * R[2][2];
    return res;
  }

  static std::vector<std::vector<float>> RotateMatrix(const std::vector<std::vector<float>> &mat, const std::vector<std::vector<float>> &R)
  {
    std::vector<std::vector<float>> res(3, std::vector<float>(3, 0.0f));
    if ((mat.size() != 3) || (mat[0].size() != 3) || (mat[1].size() != 3) || (mat[2].size() != 3))
    {
      return res;
    }
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        res[i][j] = mat[i][0] * R[0][j] + mat[i][1] * R[1][j] + mat[i][2] * R[2][j];
      }
    }
    return res;
  }

  static std::vector<float> TransformPoint(const std::vector<float> &mat, const std::vector<float> &T)
  {
    return CvUtils::TransPoint(mat, T);
  }

  static std::vector<float> TransPoint(const std::vector<float> &mat, const std::vector<float> &T)
  {
    std::vector<float> res(3, 0.0f);
    if (mat.size() != 3)
    {
      return res;
    }
    for (int i = 0; i < 3; i++)
    {
      res[i] = mat[i] + T[i];
    }
    return res;
  }
};

}

#endif
#include <extrinsic_calibration/calibrator_node.h>

using extrinsic_calibration::CameraExtrinsicCalibrator;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcloud_node");
  std::unique_ptr<CameraExtrinsicCalibrator> exc(new CameraExtrinsicCalibrator());
  std::vector<float> cloud_bounds {-0.5, -0.5, -0.5, 0.5, 0.5, 0.5};  // min x,y,z and max x,y,z
  exc->setCloudBounds(cloud_bounds);
  exc->setCalibrationNSamples(10);
  ros::spin();

  return 0;
}

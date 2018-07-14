#### extrinsic_calibration

---

This package is for converting the poses of different RGBD cameras into world space. 
One-shot calibration at its best ;)

### PREREQUISITES
- OpenCV
- PCL

### HOW TO
This only works with RGBD cameras (must be intrisically calibrated first!). All the cameras have to see the full calibration board. 

1) Print out the calibration board in "etc/calib_marker.gif". Recommended size for the board is A3 if the cameras are far away, A4 otherwise

2) Place the calibration board in view of all the cameras. Make sure that you can see the full calibration board in each camera pointcloud. Note that this will not work if there are holes along the edges of the calibration board pointcloud. ALSO, make sure that lights are not shining strongly on the calibration pattern, since the program uses a threshold to separate light-colored areas from dark ones.

3) Capture the pointclouds and save them.

4) Get the dimensions (height, width) of your pointcloud. Then edit the code 'test/calibration_pcl.cpp'
'#define KINECT_COLOR_HEIGHT 540' -> change 540 to your pointcloud height dimension
'#define KINECT_COLOR_WIDTH 960'  -> change 960 to your pointcloud width dimension

5) Build the executable 'test/calibration_pcl.cpp'

6) Run 'calibration_pcl pcdfile1 pcdfile2' (this example only works with 2 pcd files, but you can easily add more in the code).

Outputs WorldR and WorldT points for each pcd file. This is the transformation matrix between the camera and world space.

### Using with ROS
1) 

### Credits
This code is a ported version of https://github.com/MarekKowalski/LiveScan3D

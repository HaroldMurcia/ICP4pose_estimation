
# ICP for pose estimation

Repository for position estimation using ICP algorithms.
To run this repository you need to download the Kitti databases from the following site http://www.cvlibs.net/datasets/kitti/eval_odometry.php. Here you can download the sequences in .bin format, to be able to use this repository you need to convert these files to rosbags following the steps of the following repository: https://github.com/AbnerCSZ/lidar2rosbag_KITTI.


## Software Requirements
- [Python 3.7](https://www.python.org/downloads/3.7)
- [numpy](https://www.numpy.org/)
- [sklearn.neighbors](https://scikit-learn.org/stable/modules/neighbors.html)

## let's start 
 cd ICP-kitti
 
 python3 RosIcp1kitti.py

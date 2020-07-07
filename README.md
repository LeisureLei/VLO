A feature extraction method based on point cloud cvrvature  
![cvrvature point](https://github.com/LeisureLei/VLO/blob/master/cvrvature%20point.png)
# BUILD ENVIRONMENT
Ubuntu 16.04 + PCL 1.7 + OpenCV 3.3.1 + Eigen 3.2.5 + ROS Kinetic
# BUILD
cd VLO/  
catkin_make -j8  
source devel/setup.sh  
# Run
roslaunch vlo vlo.launch
rosbag play 0095.bag  

# KITTI Raw Data 0095 Baidu Yun download link
https://pan.baidu.com/s/1yHvsHfk61irttAexl8R-qw  
tr2f

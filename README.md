A feature extraction method based on point cloud cvrvature
# BUILD ENVIRONMENT
Ubuntu 16.04 + PCL 1.7 + OpenCV 3.3.1 + Eigen 3.2.5 + ROS Kinetic
# BUILD
cd VLO/  
catkin_make -j8  
source devel/setup.sh  
# Run
roslaunch vlo vlo.launch
rosbag play 0095.bag  

# KITTI Raw Data 0095 download link

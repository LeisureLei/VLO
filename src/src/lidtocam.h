#pragma once
#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <algorithm>
#include <map>
#include <unistd.h>
#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>
#include <list>
#include <array>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <chrono>


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


using namespace cv;
using namespace std;

VideoWriter outputVideo;
unsigned int currentFrame = 0;

int N =5;

//3x3 rectifying rotation to make image planes co-planar, R_rect_0X:3x3
Eigen::Matrix<double,4,4> R_rect_00;

//3x4 projection matrix after rectification, P_rect_02:3x4						
Eigen::Matrix<double,3,4>  P_rect_00;

//Transform from velo to cam0, T:4x4
Eigen::Matrix<double,4,4> T_velToCam;

//Imu to velo
Eigen::Matrix4d T_imuToVel;


Eigen::Matrix3d K;  //相机内参

Mat K_Mat;

bool initial_Flag = false; 
bool addPoint = false;
int num = 0;


Mat	image ;
Mat image_gray;  //当前帧灰度图像
Mat image_gray_last;  //上一帧灰度图像


vector<Point2f> keypoints; //当前帧特征点
vector<Point2f> keypointsLast;//上一帧特征点
vector<Point2f> curvaturePoints; //每一帧的曲率投影点
vector<Point3d> pnp3dPoints;
vector<Eigen::Matrix3d> PoseR;
vector<Eigen::Vector3d> Poset;
vector<Eigen::Matrix4d> PoseAll;

vector<Eigen::Vector4d> lidarPointClouds;

bool judgeFAST(Point P_uv,Mat image_gray)
{	
	int x = P_uv.x;
	int y = P_uv.y;
	int gray_16[16];
	if(x >= 3 && x <= 1245 && y >= 3 && y <= 378){
		int gray = int(image_gray.ptr<uchar>(y)[x]); //读取坐标点(x,y)处灰度值
		
		float T_big = gray*1.2; 
		float T_small = gray*0.8;  //判断阈值
		
		//(x,y)周围16个点的灰度值
		gray_16[0] = int(image_gray.ptr<uchar>(y-3)[x]);
		gray_16[1] = int(image_gray.ptr<uchar>(y-3)[x+1]);
		gray_16[2] = int(image_gray.ptr<uchar>(y-2)[x+2]);
		gray_16[3] = int(image_gray.ptr<uchar>(y-1)[x+3]);
		gray_16[4] =int(image_gray.ptr<uchar>(y)[x+3]);
		gray_16[5] = int(image_gray.ptr<uchar>(y+1)[x+3]);
		gray_16[6] = int(image_gray.ptr<uchar>(y+2)[x+2]);
		gray_16[7] = int(image_gray.ptr<uchar>(y+3)[x+1]);
		gray_16[8] = int(image_gray.ptr<uchar>(y+3)[x]);
		gray_16[9] = int(image_gray.ptr<uchar>(y+3)[x-1]);
		gray_16[10] = int(image_gray.ptr<uchar>(y+2)[x-2]);
		gray_16[11] = int(image_gray.ptr<uchar>(y+1)[x-3]);
		gray_16[12] = int(image_gray.ptr<uchar>(y)[x-3]);
		gray_16[13] = int(image_gray.ptr<uchar>(y-1)[x-3]);
		gray_16[14] = int(image_gray.ptr<uchar>(y-2)[x-2]);
		gray_16[15] = int(image_gray.ptr<uchar>(y-3)[x-1]);
		
		//FAST-12
		if(gray_16[0] >= T_big && gray_16[4] >= T_big && gray_16[8] >= T_big){
			for(int j=0;j<16;j++){		
				if( gray_16[j]>=T_big && gray_16[j-15>=0?j-15:j+1]>=T_big && gray_16[j-14>=0?j-14:j+2]>=T_big && gray_16[j-13>=0?j-13:j+3]>=T_big 
				&& gray_16[j-12>=0?j-12:j+4]>=T_big && gray_16[j-11>=0?j-11:j+5]>=T_big && gray_16[j-10>=0?j-10:j+6]>=T_big && gray_16[j-9>=0?j-9:j+7]>=T_big 
				&& gray_16[j-8>=0?j-8:j+8]>=T_big && gray_16[j-7>=0?j-7:j+9]>=T_big && gray_16[j-6>=0?j-6:j+10]>=T_big && gray_16[j-5>=0?j-5:j+11]>=T_big ){
					return true;
				}
				else{
					return false;
				}
			}
		}
		else if(gray_16[0] >= T_big && gray_16[4] >= T_big && gray_16[12] >= T_big){
			for(int j=0;j<16;j++){		
				if( gray_16[j]>=T_big && gray_16[j-15>=0?j-15:j+1]>=T_big && gray_16[j-14>=0?j-14:j+2]>=T_big && gray_16[j-13>=0?j-13:j+3]>=T_big 
				&& gray_16[j-12>=0?j-12:j+4]>=T_big && gray_16[j-11>=0?j-11:j+5]>=T_big && gray_16[j-10>=0?j-10:j+6]>=T_big && gray_16[j-9>=0?j-9:j+7]>=T_big 
				&& gray_16[j-8>=0?j-8:j+8]>=T_big && gray_16[j-7>=0?j-7:j+9]>=T_big && gray_16[j-6>=0?j-6:j+10]>=T_big && gray_16[j-5>=0?j-5:j+11]>=T_big ){
					return true;
				}
				else{
					return false;
				}
			}
		}
		else if(gray_16[0] >= T_big && gray_16[8] >= T_big && gray_16[12] >= T_big){
			for(int j=0;j<16;j++){		
				if( gray_16[j]>=T_big && gray_16[j-15>=0?j-15:j+1]>=T_big && gray_16[j-14>=0?j-14:j+2]>=T_big && gray_16[j-13>=0?j-13:j+3]>=T_big 
				&& gray_16[j-12>=0?j-12:j+4]>=T_big && gray_16[j-11>=0?j-11:j+5]>=T_big && gray_16[j-10>=0?j-10:j+6]>=T_big && gray_16[j-9>=0?j-9:j+7]>=T_big 
				&& gray_16[j-8>=0?j-8:j+8]>=T_big && gray_16[j-7>=0?j-7:j+9]>=T_big && gray_16[j-6>=0?j-6:j+10]>=T_big && gray_16[j-5>=0?j-5:j+11]>=T_big ){
					return true;
				}
				else{
					return false;
				}
			}
		}
		else if(gray_16[4] >= T_big && gray_16[8] >= T_big && gray_16[12] >= T_big){
			for(int j=0;j<16;j++){		
				if( gray_16[j]>=T_big && gray_16[j-15>=0?j-15:j+1]>=T_big && gray_16[j-14>=0?j-14:j+2]>=T_big && gray_16[j-13>=0?j-13:j+3]>=T_big 
				&& gray_16[j-12>=0?j-12:j+4]>=T_big && gray_16[j-11>=0?j-11:j+5]>=T_big && gray_16[j-10>=0?j-10:j+6]>=T_big && gray_16[j-9>=0?j-9:j+7]>=T_big 
				&& gray_16[j-8>=0?j-8:j+8]>=T_big && gray_16[j-7>=0?j-7:j+9]>=T_big && gray_16[j-6>=0?j-6:j+10]>=T_big && gray_16[j-5>=0?j-5:j+11]>=T_big ){
					return true;
				}
				else{
					return false;
				}
			}
		}
		else if(gray_16[0] <= T_small && gray_16[4] <= T_small && gray_16[8] <= T_small){
			for(int j=0;j<16;j++){		
				if( gray_16[j]<=T_small && gray_16[j-15>=0?j-15:j+1]<=T_small && gray_16[j-14>=0?j-14:j+2]<=T_small && gray_16[j-13>=0?j-13:j+3]<=T_small 
				&& gray_16[j-12>=0?j-12:j+4]<=T_small && gray_16[j-11>=0?j-11:j+5]<=T_small && gray_16[j-10>=0?j-10:j+6]<=T_small && gray_16[j-9>=0?j-9:j+7]<=T_small 
				&& gray_16[j-8>=0?j-8:j+8]<=T_small && gray_16[j-7>=0?j-7:j+9]<=T_small && gray_16[j-6>=0?j-6:j+10]<=T_small && gray_16[j-5>=0?j-5:j+11]<=T_small ){
					return true;
				}
				else{
					return false;
				}
			}
		}
		else if(gray_16[0] <= T_small && gray_16[4] <= T_small && gray_16[12] <= T_small){
			for(int j=0;j<16;j++){		
				if( gray_16[j]<=T_small && gray_16[j-15>=0?j-15:j+1]<=T_small && gray_16[j-14>=0?j-14:j+2]<=T_small && gray_16[j-13>=0?j-13:j+3]<=T_small 
				&& gray_16[j-12>=0?j-12:j+4]<=T_small && gray_16[j-11>=0?j-11:j+5]<=T_small && gray_16[j-10>=0?j-10:j+6]<=T_small && gray_16[j-9>=0?j-9:j+7]<=T_small 
				&& gray_16[j-8>=0?j-8:j+8]<=T_small && gray_16[j-7>=0?j-7:j+9]<=T_small && gray_16[j-6>=0?j-6:j+10]<=T_small && gray_16[j-5>=0?j-5:j+11]<=T_small ){
					return true;
				}
				else{
					return false;
				}
			}
		}
		else if(gray_16[0] <= T_small && gray_16[8] <= T_small && gray_16[12] <= T_small){
			for(int j=0;j<16;j++){		
				if( gray_16[j]<=T_small && gray_16[j-15>=0?j-15:j+1]<=T_small && gray_16[j-14>=0?j-14:j+2]<=T_small && gray_16[j-13>=0?j-13:j+3]<=T_small 
				&& gray_16[j-12>=0?j-12:j+4]<=T_small && gray_16[j-11>=0?j-11:j+5]<=T_small && gray_16[j-10>=0?j-10:j+6]<=T_small && gray_16[j-9>=0?j-9:j+7]<=T_small 
				&& gray_16[j-8>=0?j-8:j+8]<=T_small && gray_16[j-7>=0?j-7:j+9]<=T_small && gray_16[j-6>=0?j-6:j+10]<=T_small && gray_16[j-5>=0?j-5:j+11]<=T_small ){
					return true;
				}
				else{
					return false;
				}
			}
		}
		else if(gray_16[4] <= T_small && gray_16[8] <= T_small && gray_16[12] <= T_small){
			for(int j=0;j<16;j++){		
				if( gray_16[j]<=T_small && gray_16[j-15>=0?j-15:j+1]<=T_small && gray_16[j-14>=0?j-14:j+2]<=T_small && gray_16[j-13>=0?j-13:j+3]<=T_small 
				&& gray_16[j-12>=0?j-12:j+4]<=T_small && gray_16[j-11>=0?j-11:j+5]<=T_small && gray_16[j-10>=0?j-10:j+6]<=T_small && gray_16[j-9>=0?j-9:j+7]<=T_small 
				&& gray_16[j-8>=0?j-8:j+8]<=T_small && gray_16[j-7>=0?j-7:j+9]<=T_small && gray_16[j-6>=0?j-6:j+10]<=T_small && gray_16[j-5>=0?j-5:j+11]<=T_small ){
					return true;
				}
				else{
					return false;
				}
			}
		}
		else{
			return false;
		}
		
        /*
		//FAST-9
		for(int j=0;j<16;j++){		
			if( gray_16[j]>=T_big && gray_16[j-15>=0?j-15:j+1]>=T_big && gray_16[j-14>=0?j-14:j+2]>=T_big && gray_16[j-13>=0?j-13:j+3]>=T_big 
			&& gray_16[j-12>=0?j-12:j+4]>=T_big && gray_16[j-11>=0?j-11:j+5]>=T_big && gray_16[j-10>=0?j-10:j+6]>=T_big && gray_16[j-9>=0?j-9:j+7]>=T_big 
			&& gray_16[j-8>=0?j-8:j+8]>=T_big ){
				return true;
			}
			else if(gray_16[j]<=T_small && gray_16[j-15>=0?j-15:j+1]<=T_small && gray_16[j-14>=0?j-14:j+2]<=T_small && gray_16[j-13>=0?j-13:j+3]<=T_small 
				&& gray_16[j-12>=0?j-12:j+4]<=T_small && gray_16[j-11>=0?j-11:j+5]<=T_small && gray_16[j-10>=0?j-10:j+6]<=T_small && gray_16[j-9>=0?j-9:j+7]<=T_small 
				&& gray_16[j-8>=0?j-8:j+8]<=T_small){
				return true;
			}
			else{
				return false;
			}
		}
        */		
	}
	else{
		return false;
	}
}

bool judgeweakFAST(Point P_uv,const Mat& image_gray)
{
	int x = P_uv.x;
	int y = P_uv.y;
	int gray_4[8];
	float T_up = int(image_gray.ptr<uchar>(y)[x])*0.6;
	float T_down =  int(image_gray.ptr<uchar>(y)[x])*1.4;
	if(x >= 3 && x <= 1245 && y >= 3 && y <= 378)
	{
		gray_4[0] = int(image_gray.ptr<uchar>(y-3)[x]);
		gray_4[1] = int(image_gray.ptr<uchar>(y)[x+3]);
		gray_4[2] = int(image_gray.ptr<uchar>(y+3)[x]);
		gray_4[3] = int(image_gray.ptr<uchar>(y)[x-3]);





		if(gray_4[0]>=T_up && gray_4[1]>=T_up && gray_4[2]>=T_up && gray_4[3]>=T_up)
		{
			return true;
		}
		else if(gray_4[0]<=T_down && gray_4[1]<=T_down && gray_4[2]<=T_down && gray_4[3]<=T_down)
		{
			return true;
		}
		else
		{
			return false;
		}
		
	}

}
void reduceVector(vector<Point2f> &v, vector<unsigned char> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++){
        if (status[i]){
            v[j++] = v[i];
		}
	}
    v.resize(j);
}

float pixelsquaredis(Point2f p, Point2f q)
{
	float pixeldis = (p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y);
	return pixeldis;
}

void reduceVector(vector<Point3d> &v, vector<unsigned char> status)
{
	int j = 0;
	for(size_t i = 0;i<v.size();i++){
		if(status[i]){
			v[j++] = v[i];
		}
	}
	v.resize(j);
}

float distance(const Point2f &p , const Point2f &q){

	float dx =p.x - q.x;
	float dy = p.y - q.y;
	return sqrt(dx*dx+dy*dy);
}

void rejectWithF(vector<Point2f> &cur_keypoint, vector<Point2f> &prev_keypoint){
	vector<uchar> status;
	findFundamentalMat(cur_keypoint, prev_keypoint, FM_RANSAC, 1.00, 0.99,status);
	reduceVector(cur_keypoint,status);
	reduceVector(prev_keypoint,status);
}

void loadMatrix()
{
				
	K<< 7.215377e+02, 0.000000e+00, 6.095593e+02,
		0.000000e+00, 7.215377e+02 ,1.728540e+02 ,
		0.000000e+00, 0.000000e+00, 1.000000e+00 ;

	
			 
	T_imuToVel << 9.999976e-01 ,7.553071e-04, -2.035826e-03,-8.086759e-01,
 			-7.854027e-04,9.998898e-01 ,-1.482298e-02,3.195559e-01,
 			 2.024406e-03 ,1.482454e-02 ,9.998881e-01,-7.997231e-01,
			  0,0,0,1;
	K_Mat = (Mat_<double>(3,3)<< 7.215377e+02, 0.000000e+00, 6.095593e+02,
								0.000000e+00, 7.215377e+02 ,1.728540e+02 ,
								0.000000e+00, 0.000000e+00, 1.000000e+00  );

}

Eigen::Vector3d transformProject(const pcl::PointXYZ& lidarpoint)
{	
	T_velToCam << 0.00944039,-0.999948,0.00391788,-0.07,
				 0.00523786,-0.00386855,  -0.999979, 0.05,
   					0.999942,0.00946071 ,0.00520107,-0.29,
          			0 ,         0  ,        0  ,      1;
	R_rect_00 << 9.999239e-01, 9.837760e-03, -7.445048e-03, 0,
				-9.869795e-03 ,	9.999421e-01 ,-4.278459e-03 ,0,
				7.402527e-03, 4.351614e-03 ,9.999631e-01,0,
				0,0,0,1;
	P_rect_00 << 7.215377e+02, 0.000000e+00, 6.095593e+02, 0.000000e+00 ,
				0.000000e+00, 7.215377e+02 ,1.728540e+02 ,0.000000e+00 ,
				0.000000e+00, 0.000000e+00, 1.000000e+00 ,0.000000e+00;
	
	Eigen::Vector3d P_lidar = Eigen::Vector3d(lidarpoint.x, lidarpoint.y, lidarpoint.z );
	Eigen::Vector3d z_P_uv = P_rect_00*R_rect_00*T_velToCam*Eigen::Vector4d(P_lidar[0],P_lidar[1],P_lidar[2],1);

	return z_P_uv;
}

//求图像点的归一化坐标
vector<Point2f> pixelToCam(const vector<Point2f>& point)
{
	vector<Point2f> cam_normalization;
	for(size_t i = 0;i< point.size();i++){
		Eigen::Vector3d pixel(point[i].x,point[i].y,1);
		Eigen::Vector3d cam(K.inverse()*pixel);
		Point2f tmp_point;
		tmp_point.x = cam[0];
		tmp_point.y = cam[1];

		cam_normalization.push_back(tmp_point);
	}
	return cam_normalization;
}



bool inImage(const Point2f& point)
{
	if(point.x >= 0 && point.x<= 1242 && point.y >= 0 && point.y <= 375){
		return true;
	}
	else{
		return false;
	}
}



//双向光流
void opticalTrack(vector<Point2f>& keypointsLast, vector<Point2f>& keypoints)
{
    vector<Point2f> next_keypoints; 
    vector<Point2f> prev_keypoints;	

	for(auto kp:keypointsLast)
	{
		prev_keypoints.push_back(kp);
	}

	vector<unsigned char> status;
	vector<float> error;
	
	calcOpticalFlowPyrLK( image_gray_last ,  image_gray , prev_keypoints , next_keypoints , status ,error ,Size(21, 21), 3);

	vector<unsigned char> back_status;
	vector<Point2f> back_keypointsl;

	calcOpticalFlowPyrLK(image_gray,image_gray_last,next_keypoints, back_keypointsl,back_status,error,Size(21, 21), 3);

	for(size_t i = 0;i<status.size();i++){
		if( status[i] && back_status[i] && inImage(next_keypoints[i]) && distance(prev_keypoints[i],back_keypointsl[i])<0.5 ){
			status[i] = 1;
		}
		else{
			status[i] = 0;
		}
	}

	reduceVector(keypointsLast,status);
	reduceVector(next_keypoints,status);

	for(auto kp:next_keypoints){
		keypoints.push_back(kp);
	}
}

void rejectWithDis()
{
	//去除距离过大的点
	vector<Point2f> next_keypoints;
	vector<Point2f> prev_keypoints;

	for(auto kp:keypoints){
		next_keypoints.push_back(kp);
	}
	for(auto kp:keypointsLast){
		prev_keypoints.push_back(kp);
	}

	auto iterkey = keypoints.begin();
	auto iterKeyLast = keypointsLast.begin();

	for(size_t i=0;i < next_keypoints.size();i++){
		if(distance(next_keypoints[i],prev_keypoints[i])>40){

			iterkey = keypoints.erase(iterkey);
			iterKeyLast = keypointsLast.erase(iterKeyLast);
			i++;
			continue;
		}
		iterkey++;
		iterKeyLast++;

	}
}

//画出两帧之间光流
void visualization()
{
	Mat image_show = image_gray.clone();
	vector<Point2f> p1;
	vector<Point2f> p2;
	for(auto kp:keypoints){
		p1.push_back(kp);
	}
	for(auto kp:keypointsLast){
		p2.push_back(kp);
	}

	RNG rng(0);

	int pointNum = 0;
	for(int i=0;i<keypoints.size();i++)
	{
		Scalar color = Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
		circle(image_show,p1[i],4,Scalar(0,0,255),1);
		line(image_show,p2[i],p1[i],color,2);
	}
	// imwrite("/home/leibing/VLO/src/opti_image/"+to_string(currentFrame)+".jpg",image_show);
	outputVideo.write(image_show);
	// currentFrame++;
	//cout<<"pointNum :"<<pointNum<<endl<<endl;
	imshow("Optical flow points",image_show);
	waitKey(1);
}

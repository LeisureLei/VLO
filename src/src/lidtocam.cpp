#include "lidtocam.h"
#include <mutex>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <queue>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

queue<sensor_msgs::ImageConstPtr> ImgMsgQueue;
queue<sensor_msgs::PointCloud2::ConstPtr> LidarPtrQueue;



mutex m_buf;
bool initial = false;

void pointCloudSharpCallback(const sensor_msgs::PointCloud2::ConstPtr& PointCloudConerMsg)
{	
	m_buf.lock();
	LidarPtrQueue.push(PointCloudConerMsg);
	m_buf.unlock();
}

void imageCallback(const sensor_msgs::Image::ConstPtr& ImageMsg)
{
	m_buf.lock();
	// cout<<"image time:"<<to_string(ImageMsg->header.stamp.toSec())<<endl;
	ImgMsgQueue.push(ImageMsg);
	// cout<<"Img size:"<<ImgMsgQueue.size()<<endl;
	m_buf.unlock();
}


int main( int argc, char** argv)
{	
	ros::init(argc, argv, "lidtocam");   
	ros::NodeHandle nh;
	
	ros::Subscriber pointCloudCb = nh.subscribe("/laser_cloud_sharp", 100 ,pointCloudSharpCallback );
	ros::Subscriber imageCb = nh.subscribe("/cam00/image_raw", 100, imageCallback ); 
	int n=0;
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		
		ros::spinOnce();  //callback
		
		if( !LidarPtrQueue.empty() && !ImgMsgQueue.empty() )
		{			
			sensor_msgs::ImageConstPtr ImageMsg = ImgMsgQueue.front();
			cv_bridge::CvImageConstPtr cv_ptr;
			try
    		{
        		cv_ptr = cv_bridge::toCvCopy(ImageMsg, sensor_msgs::image_encodings::BGR8); // Caution the type here.
    		}
   			catch (cv_bridge::Exception& ex)
    		{
        		ROS_ERROR("cv_bridge exception in rgbcallback: %s", ex.what());
        		exit(-1);
			}

			image_gray = cv_ptr->image.clone(); //opencv image clone
			
			double lidarcorner_time = LidarPtrQueue.front()->header.stamp.toSec();
			double img_time = ImageMsg->header.stamp.toSec();

		
			if( lidarcorner_time-img_time<0.020 && lidarcorner_time-img_time>-0.020 )
			{
				// cout<<"time synced! time offset is: "<<lidarcorner_time-img_time<<endl;
				pcl::PointCloud<pcl::PointXYZ> cornerlaserCloud;
    			pcl::fromROSMsg(*LidarPtrQueue.front(), cornerlaserCloud);
				curvaturePoints.clear();
				for(int i=0;i<cornerlaserCloud.points.size();++i)
				{
					
					Eigen::Vector3d z_P_uv = transformProject(cornerlaserCloud.points[i]);
					if(z_P_uv[0]>0 && z_P_uv[1]>0 && z_P_uv[2]>0 && z_P_uv[0]/z_P_uv[2]>10 && z_P_uv[1]/z_P_uv[2]>10 && z_P_uv[0]/z_P_uv[2] < image_gray.cols - 10 && z_P_uv[1]/z_P_uv[2]< image_gray.rows - 10 )
					{
						Point2f P_uv = Point2f(z_P_uv[0]/z_P_uv[2],z_P_uv[1]/z_P_uv[2]);
						//去除异常点
						if(P_uv.x>(image_gray.cols/2-30) && P_uv.x<(image_gray.cols/2+30) && P_uv.y>(image_gray.rows/2) && P_uv.y<(image_gray.rows-5))
						{
							continue;
						}
						curvaturePoints.push_back(P_uv);
						// circle(image_gray,P_uv,3,Scalar(0,0,255),1);
					}
					else
					{
						continue;
					}		
				}


				if(!initial)
				{
					// outputVideo.open("~/VLO/src/better_opti_demo.avi",CV_FOURCC('M','J','P','G'),30,Size(image_gray.cols,image_gray.rows),true);
					
					for(auto kp:curvaturePoints)
					{
						if(judgeFAST(kp,image_gray))
						{
							keypointsLast.push_back(kp);  //第一帧的全部曲率投影点当做初始特征点
							continue;
						}
						Point2f around_cv_point[N][N];
						for(int i=0;i<N;i++)
						{
							int symbol = 0;
							int mid = (N-1)/2;
							for(int j=0;j<N;j++)
							{
								if(i==mid && j == mid){
									continue;
								}
								around_cv_point[i][j].x = kp.x+i-mid;
								around_cv_point[i][j].y = kp.y+j-mid;
								if( judgeFAST( around_cv_point[i][j] , image_gray) )
								{
									keypointsLast.push_back( around_cv_point[i][j] );
									symbol = 1;
									break;
								}
							}	
							if(symbol == 1){
								break;
							}
						}	
						
					}
					image_gray_last = image_gray;
					initial = true;
				}
				else if(keypointsLast.size()>50)
				{
					opticalTrack(keypointsLast,keypoints);//双向光流去除外点
					cout<<"keypointslast size:"<<keypointsLast.size()<<endl;
					// cout<<"keypoints size:"<<keypointsLast.size()<<endl;
					visualization();
					//里程计部分
					//TO DO

					//每次结束，keypointsLast存放上一帧特征点，keypoints清零
					keypointsLast.clear();
					for(auto kp:keypoints)
					{
						keypointsLast.push_back(kp);
					}
					keypoints.clear();
					image_gray_last = image_gray;
				}
				else
				{
					for(auto kp:curvaturePoints)
					{
						if(judgeweakFAST(kp,image_gray_last))
						{
							keypointsLast.push_back(kp);
							continue;
						}
						Point2f around_cv_point[N][N];
						for(int i=0;i<N;i++)
						{
							int symbol = 0;
							int mid = (N-1)/2;
							for(int j=0;j<N;j++)
							{
								if(i==mid && j == mid){
									continue;
								}
								around_cv_point[i][j].x = kp.x+i-mid;
								around_cv_point[i][j].y = kp.y+j-mid;
								if( judgeFAST( around_cv_point[i][j] , image_gray) )
								{
									keypointsLast.push_back( around_cv_point[i][j] );
									symbol = 1;
									break;
								}
							}	
							if(symbol == 1){
								break;
							}
						}
					}
					opticalTrack(keypointsLast,keypoints);
					visualization();
					keypointsLast.clear();
					for(auto kp:keypoints)
					{
						keypointsLast.push_back(kp);
					}
					//里程计部分
					//TO TO
					
					keypoints.clear();
					image_gray_last = image_gray;
					cout<<"have less than 30 points to track,add "<< curvaturePoints.size()<<" points"<<endl;
				}

				m_buf.lock();
				LidarPtrQueue.pop();
				ImgMsgQueue.pop();
				m_buf.unlock();
			}
			else if(lidarcorner_time-img_time>0.020)
			{
				m_buf.lock();
				ImgMsgQueue.pop();
				m_buf.unlock();
			}
			else if(lidarcorner_time-img_time<-0.020)
			{
				m_buf.lock();
				LidarPtrQueue.pop();
				m_buf.unlock();
			}
		}
		loop_rate.sleep();
	}

	
	// outputVideo.release();
	return 0;
}

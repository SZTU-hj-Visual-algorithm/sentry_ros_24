#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <iostream>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

class SubscribeAndPublish
{
public:
	SubscribeAndPublish(){
		pub = node.advertise<livox_ros_driver::CustomMsg>("/livox/lidar_merge", 10);
		sub_front = new message_filters::Subscriber<livox_ros_driver2::CustomMsg>(node, "livox/lidar_192_168_1_3", 200000);
		sub_back = new message_filters::Subscriber<livox_ros_driver2::CustomMsg>(node,  "livox/lidar_192_168_1_140", 200000);
		
		typedef message_filters::sync_policies::ApproximateTime<livox_ros_driver2::CustomMsg,livox_ros_driver2::CustomMsg> syncPolicy;
		message_filters::Synchronizer<syncPolicy> sync(syncPolicy(9), *sub_front, *sub_back);
		sync.registerCallback(boost::bind(&SubscribeAndPublish::callBack, this,  _1, _2));
		
		ros::spin();
	}

	void callBack(const livox_ros_driver2::CustomMsgConstPtr& lidar_front, const livox_ros_driver2::CustomMsgConstPtr& lidar_back){
			PointCloudXYZI pointCloud_front;
			PointCloudXYZI pointCloud_back;
			PointCloudXYZI pointCloud_back_out;
			PointCloudXYZI finalPointCloud;
			convert2PointCloud2(lidar_front, pointCloud_front);
			convert2PointCloud2(lidar_back, pointCloud_back);
			// ROS_INFO("Start merging point cloud");
			Eigen::Matrix4f transform_back2front = Eigen::Matrix4f::Identity(); 
				transform_back2front(0, 0) = -1; 
				transform_back2front(0, 1) = 0;
				transform_back2front(0, 2) = 0;
				transform_back2front(0, 3) = 0;

				transform_back2front(1, 0) =  0; 
				transform_back2front(1, 1) = -1;
				transform_back2front(1, 2) = 0;
				transform_back2front(1, 3) = 0;

				transform_back2front(2, 0) = 0;
				transform_back2front(2, 1) = 0;
				transform_back2front(2, 2) = 1;
				transform_back2front(2, 3) = -0.03;

				transform_back2front(4, 4) = 1;
				pcl::transformPointCloud(pointCloud_back, pointCloud_back_out, transform_back2front);

				finalPointCloud = pointCloud_front + pointCloud_back_out ;

				livox_ros_driver2::CustomMsg finalMsg;
				finalMsg.header = lidar_front->header;
				finalMsg.timebase = lidar_front->timebase;
				finalMsg.point_num = finalPointCloud.size();
				finalMsg.lidar_id = lidar_front->lidar_id;
				
				for(unsigned int i = 0; i < finalMsg.point_num; i++)
				{
					livox_ros_driver2::CustomPoint p;
					p.x = finalPointCloud[i].x;
					p.y = finalPointCloud[i].y;
					p.z = finalPointCloud[i].z;
					p.reflectivity = finalPointCloud[i].intensity;
					p.offset_time = finalPointCloud[i].curvature * float(1000000);
					finalMsg.points.push_back(p);
				}
				
				pub.publish(finalMsg);

	}

	void convert2PointCloud2(const livox_ros_driver2::CustomMsgConstPtr& lidarMsg, PointCloudXYZI& pclPointCloud ){
		for(unsigned int i = 0; i < lidarMsg->point_num; i++)
		{
			PointType point;
			point.x = lidarMsg->points[i].x;
			point.y = lidarMsg->points[i].y;
			point.z = lidarMsg->points[i].z;
			point.intensity = lidarMsg->points[i].reflectivity;
			point.curvature = lidarMsg->points[i].offset_time / float(1000000); 
			pclPointCloud.push_back(point);
		}
	}

	


private:
	ros::NodeHandle node;
	ros::Publisher pub;
	message_filters::Subscriber<livox_ros_driver2::CustomMsg>* sub_front;
	message_filters::Subscriber<livox_ros_driver2::CustomMsg>* sub_back;
};

int main(int argc, char* argv[]){
	ros::init(argc, argv, "pointCloudMerge");
	
	SubscribeAndPublish sap;

	return 0;
}
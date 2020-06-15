// Node build by Luis Yoichi Morales 2016

#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Core>



class Transform
{
public:
  Transform();

  std::string out_frame;
  std::string advertise_cloud;
  std::string subscribe_cloud;
  
  std::string out_frame_default;
  std::string advertise_cloud_default;
  std::string subscribe_cloud_default;


private:

  tf::TransformListener tf_listener;

  ros::NodeHandle nh ;
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);  

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_PointCloud_;

  sensor_msgs::PointCloud2::Ptr in_cloud  ;
  sensor_msgs::PointCloud2::Ptr  out_cloud ;

  
  boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > pcl_transformed_cloud_;


  ros::Subscriber velodyne_sub ;
  ros::Publisher velodyne_pub; 
  
  };


Transform::Transform():
  nh("~"),
  in_cloud(new sensor_msgs::PointCloud2),
  out_cloud(new sensor_msgs::PointCloud2),
  pcl_PointCloud_(new pcl::PointCloud<pcl::PointXYZI>),
  tf_listener()
{
  
  out_frame_default = "/velodyne";
  advertise_cloud_default = "velodyne_cloud";
  subscribe_cloud_default = "velodyne_points";



  
  //nh.param("in_frame", in_frame, in_frame_default);
  nh.param("out_frame",  out_frame,  out_frame_default);
  nh.param("advertise_cloud",  advertise_cloud,  advertise_cloud_default);
  nh.param("subscribe_cloud",  subscribe_cloud,  subscribe_cloud_default);

  velodyne_sub = nh.subscribe(subscribe_cloud, 10, &Transform::cloudCallback, this);  	
  velodyne_pub = nh.advertise<sensor_msgs::PointCloud2>( advertise_cloud, 100);
}



void Transform::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
  *in_cloud = *msg;
  
  
  boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > pcl_transformed_cloud_;
  
  pcl::fromROSMsg( *in_cloud, *pcl_PointCloud_ );
	
	// Check the size to filter erroneously empty point cloud
	if(pcl_PointCloud_->points.size() < 1)
        return;

	Eigen::Affine3d eigen_transform;

	if ( out_frame.compare( in_cloud->header.frame_id) != 0){
	  
	  //std::cerr << "out_frame:" << out_frame << "  in_frame:"<< in_frame << "  advertise:" <<advertise_cloud << "\n" ; 
	  tf::StampedTransform transform;
	  
	  try{
	    tf_listener.waitForTransform( out_frame, in_cloud->header.frame_id, in_cloud->header.stamp, ros::Duration(0.5));
	    tf_listener.lookupTransform    ( out_frame, in_cloud->header.frame_id, in_cloud->header.stamp, transform);
	  }	
	  catch (tf::TransformException ex){
	    ROS_ERROR("%s",ex.what());
	    return;
	  }
	  
	  // Put in the Eigen::Affine3d 
	  tf::transformTFToEigen (transform, eigen_transform);
	  // Apply the transform
	  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_tmp (new pcl::PointCloud<pcl::PointXYZI>);
	  pcl::transformPointCloud( *pcl_PointCloud_, *pcl_tmp, eigen_transform);
	  pcl_PointCloud_ = pcl_tmp;

	}

	// Convert pcl::pointCloud to sensor_msgs::PointCloud2
	pcl::toROSMsg( *pcl_PointCloud_, *out_cloud );
//SAMUEL - use ros::Time::now() -------------------------------------
	out_cloud->header.stamp = ros::Time::now();
//	out_cloud->header.stamp = in_cloud->header.stamp;
//-------------------------------------------------------------------
	out_cloud->header.frame_id = out_frame;
	//transformed_cloud_pub_.publish(*transformed_cloud_);
	velodyne_pub.publish( *out_cloud );
}



int main(int argc, char **argv){
  ros::init(argc, argv, "pointcloud_transform");	
  Transform node;
  
  //std::cerr << "out_frame:" << node.out_frame << "  in_frame:"<< node.in_frame << "  advertise:" <<node.advertise_cloud << " subscribe:" << node.subscribe_cloud << "\n" ;
  
  ros::spin();

  
  
  return 0;
}

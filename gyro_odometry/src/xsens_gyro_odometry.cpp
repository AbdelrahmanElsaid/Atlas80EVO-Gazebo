//Written by Luis Yoichi Morales July 2016
//Program to compute odometry using the velocity information from wheels and angular velocity from IMU
#include "ros/ros.h"
#include <stdlib.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "sensor_msgs/Imu.h"


#include <message_filters/synchronizer.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>



class Gyrodometry
{
public:
    Gyrodometry();
    void run();

private:    


  double t , p_t;

  ros::NodeHandle nh;

  ros::Publisher odom_pub;
  bool is_initialized ;
//---------- SAMUEL - flag for enabling publish tf transform ----------
  bool publish_tf;
//---------------------------------------------------------------------
  
  //tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom_msg;
  
  ros::Time prev_time2;

  double d_time ;
  double x  ;  //m
  double y  ;  //m
  double th ; //rad
  double cv , cw;  // current velocities

  int prevCount;
  double diff;
  double wx, wy, wz;

  std::string child_frame_id, parent_frame_id;
  std::string advertise_topic;
  std::string subscribe_odom_topic;
  std::string subscribe_imu_topic;
  sensor_msgs::Imu imu_data ;
  void imu_Callback( const sensor_msgs::ImuConstPtr &msg);
  void odom_Callback( const nav_msgs::OdometryConstPtr &odom_data);
  ros::Subscriber imu_subscriber ;
  ros::Subscriber odom_subscriber;
};

Gyrodometry::Gyrodometry():
  nh("~")
{
  advertise_topic      = "/xsens_gyro_odometry";
  subscribe_odom_topic = "/odom";
  subscribe_imu_topic  = "/mti/sensor/imu";
    
  odom_pub = nh.advertise<nav_msgs::Odometry>( advertise_topic, 1);
  //sensor_msgs::ImuConstPtr &imu_data ;
    
  is_initialized = false ;

  parent_frame_id = "map";
  child_frame_id = "odometry_frame";
  
  x  = 0;  //m
  y  = 0;  //m
  th = 0; //rad
  diff = 0 ;
  
  nh.param("parent_frame_id", parent_frame_id, parent_frame_id );
  nh.param("child_frame_id", child_frame_id, child_frame_id);
//---------- SAMUEL - flag for enabling publish tf transform ----------
  nh.param("publish_tf", publish_tf, true);
//---------------------------------------------------------------------
  
  nh.param("x_m"   , x,  x  );
  nh.param("y_m"   , y,  y  );
  nh.param("th_rad", th, th );


  // Create the odometry transform data to send to tf
  odom_trans.header.frame_id = parent_frame_id;
  odom_trans.child_frame_id = child_frame_id;

  t = p_t = 0;
  
}

//void Gyrodometry::Callback(const nav_msgs::OdometryConstPtr &odom_data, const sensor_msgs::ImuConstPtr &imu_data) { 
//void Gyrodometry::Callback( const sensor_msgs::ImuConstPtr &imu_data, const nav_msgs::OdometryConstPtr &odom_data) {

void Gyrodometry::imu_Callback( const sensor_msgs::ImuConstPtr &msg) {
  
  double d_time = msg->header.stamp.toSec() - p_t;
  p_t = msg->header.stamp.toSec() ;

//SAMUEL - modified - lower theta sensitivity by checking the angular_velocity.z -------
//  if( d_time < 0.1)
//    th = th + (  msg->angular_velocity.z * d_time );
//  cw = msg->angular_velocity.z ;
  if ( d_time < 0.1 && fabs(msg->angular_velocity.z) > 0.025 )
    th = th + (  msg->angular_velocity.z * d_time );
  cw = msg->angular_velocity.z ;

//--------------------------------------------------------------------------------------

  //std::cerr << "\ndtime " << d_time << " " << th ;
  //std::cerr << "\ncw " << my_var << " diff " << my_var << " t " << d_time << " th " << th * 180/M_PI;
  
}

void Gyrodometry::odom_Callback( const nav_msgs::OdometryConstPtr &odom_data) {
  
  tf::Quaternion tmp_;
  tfScalar yaw, pitch, roll;
  //std::cerr << "\n In callback!!! " ;

  if ( !is_initialized ) {

    prev_time2 = odom_data->header.stamp;
    //std::cerr << " Initializing time stamp \n" ;
    is_initialized = true ;
  }

  else{
    cv = odom_data->twist.twist.linear.x ; //14Nov2019


    //std::cerr << "\n th " << th*180/M_PI << " x " << x << " y " << y << "  v "<< cv << " t " << d_time;
    
    d_time = odom_data->header.stamp.toSec() - prev_time2.toSec() ;
    
    x  += ( cv * cos(th) * d_time ); 
    y  += ( cv * sin(th) * d_time );


    

    
    //Create a quaternion from theta
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
  
  // Update the odometry transform data to send to tf
//SAMUEL - use ros::Time::now() --------------------------------
  odom_trans.header.stamp = odom_data->header.stamp;
//  odom_trans.header.stamp = ros::Time::now(); //current_time;
//--------------------------------------------------------------
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0;
  odom_trans.transform.rotation = odom_quat;
  
  //odom_broadcaster.sendTransform(odom_trans);

  
    tf::Transform transform;    
    tf::Quaternion q;
    transform.setOrigin( tf::Vector3( x, y, 0) );
    q.setRPY( 0, 0, th );
    transform.setRotation(q);
    static tf::TransformBroadcaster br;

//SAMUEL - flag for enabling publish tf transform -------------------------------------
    if (publish_tf) {
      br.sendTransform(tf::StampedTransform(transform,  odom_trans.header.stamp,  parent_frame_id, child_frame_id ));
//      br.sendTransform(tf::StampedTransform(transform,  ros::Time::now(),  parent_frame_id, child_frame_id ));
    }
//-------------------------------------------------------------------------------------

    // Update the odometry message on odom
//SAMUEL - use ros::Time::now() -------------------------------
    odom_msg.header.stamp = odom_data->header.stamp;
//    odom_msg.header.stamp = ros::Time::now(); //current_time;
//SAMUEL - use flexible frame_ids -----------------------------
    odom_msg.header.frame_id = parent_frame_id;
    odom_msg.child_frame_id = child_frame_id;
//-------------------------------------------------------------
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation = odom_quat;
    
    odom_msg.twist.twist.linear.x = cv;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = cw;
    
    // Publish to odom
    odom_pub.publish(odom_msg);
    
    
    //prevCount = msg->wheel;
    prev_time2 = odom_data->header.stamp;
    
               
  }
}




void Gyrodometry::run(){

  imu_subscriber = nh.subscribe( subscribe_imu_topic, 1, &Gyrodometry::imu_Callback, this);
  odom_subscriber = nh.subscribe( subscribe_odom_topic, 1, &Gyrodometry::odom_Callback, this);

  /*
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub (nh,  subscribe_odom_topic, 100);
  message_filters::Subscriber<sensor_msgs::Imu>    imu_sub (nh,  subscribe_imu_topic , 100);
  
  //typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu>MySyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, nav_msgs::Odometry>MySyncPolicy;
  
  //message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, imu_sub  );  
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub, odom_sub  );  
  sync.registerCallback( boost::bind( &Gyrodometry::Callback,        this, _1, _2 )  );
  */  

  //std::cerr << "\n Before spin! "  << subscribe_odom_topic << " " << subscribe_imu_topic << "\n\n";

  ros::spin();
}


int main(int argc, char** argv){
  ros::init(argc, argv, "xsens_gyro_odometry");    
  Gyrodometry node;

  

  node.run();

  return 0;
}



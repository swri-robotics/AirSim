#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  double th = 0.0;
  ros::Time last = ros::Time(0);
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");
  std::string vehicle_frame;
  pnh.param<std::string>("vehicle_frame", vehicle_frame, "base_link");
  ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("imu_in", 10, [&](const sensor_msgs::ImuConstPtr& msg)
  {
    if (last == ros::Time(0))
    {
        last = ros::Time::now();
    }
    th -= (ros::Time::now()-last).toSec()*msg->angular_velocity.z;
    last = ros::Time::now();
  });

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  double x = 0.0;
  double y = 0.0;
  ros::Time last_time = ros::Time::now();
  tf2_ros::TransformBroadcaster odom_broadcaster;
  ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("odom_in", 10, [&](const nav_msgs::OdometryConstPtr& msg)
  {
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = msg->header.stamp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = vehicle_frame;

    double dt = (msg->header.stamp - last_time).toSec();
    double vx = msg->twist.twist.linear.x;
    double vy = 0.0;
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    last_time = msg->header.stamp;
    x += delta_x;
    y += delta_y;
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = msg->pose.pose.position.z;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

    nav_msgs::Odometry odom = *msg;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = odom_trans.transform.rotation;
    odom_pub.publish(odom);

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
  });

  while(n.ok()){

    ros::spinOnce();
  }
}

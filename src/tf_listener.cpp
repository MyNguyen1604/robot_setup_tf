//Create a node that will use that transform to take a point in the "base_laser" frame and transform it to a point in the "base_link" frame

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h> // A TransformListener object automatically subscribes to the transform message topic over ROS and manages all transform data coming in over the wire.

void transformPoint(const tf::TransformListener& listener){
  //create a point in the base_laser frame to transform to the base_link frame
  geometry_msgs::PointStamped laser_point;//gan laser_point la PointStamped msgs
  laser_point.header.frame_id = "base_laser";
  
  laser_point.header.stamp = ros::Time();
  
  laser_point.point.x = 1.0;
  laser_point.point.y = 0.2;
  laser_point.point.z = 0.0;
  // Here, we'll create our point as a geometry_msgs::PointStamped. The "Stamped" on the end of the message name just means that it includes a header, allowing us to associate both a timestamp and a frame_id with the message. We'll set the stamp field of the laser_point message to be ros::Time() which is a special time value that allows us to ask the TransformListener for the latest available transform
  //Dong bo ve thoi gian nen de dang cap nhat duoc gia tri chuyen doi moi nhat
  
 try{
   geometry_msgs::PointStamped base_point;
   listener.transformPoint("base_link", laser_point, base_point);
   //transformPoint() with three arguments: the name of the frame we want to transform the point to ("base_link" in our case), the point we're transforming, and storage for the transformed point
   ROS_INFO("base_laser : (%.2f, %.2f, %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f", laser_point.point.x, laser_point.point.y, laser_point.point.z, base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
 }
 catch(tf::TransformException& ex){
   ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what()); //Viec chuyen doi bi tu choi
 }
}//This function will serve as a callback for the ros::Timer created in the main() of our program and will fire every second. 

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;
  
  tf::TransformListener listener(ros::Duration(10));
  //Transform a point once every second
  
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
  ros::spin();
}

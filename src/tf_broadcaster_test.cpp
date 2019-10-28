///Create a node that publishes the base_laser to base_link transform over ROS


// #include <ros/ros.h>
// #include <tf/transform_broadcaster.h> //The tf package provides an implementation of a tf::TransformBroadcaster to help make the task of publishing transforms easier. To use the TransformBroadcaster, we need to include the tf/transform_broadcaster.h header file. 

// int main(int argc, char** argv){
//    ros::init(argc, argv, "robot_tf_publisher_test");
//    ros::NodeHandle n;
   
//    ros::Rate r(100);
//    tf::TransformBroadcaster broadcaster;
   
//    while(n.ok()){
//      broadcaster.sendTransform(
//        tf::StampedTransform(
//          tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
//          ros::Time::now(), "base_link", "camera_depth_frame"));//create a TransformBroadcaster object
         
//          // Sending a transform with a TransformBroadcaster requires five arguments
//          ///First, we pass in the rotation transform, which is specified by a btQuaternion for any rotation that needs to occur between the two coordinate frames
//          ///Second, a btVector3 for any translation that we'd like to apply
//          ///Third, we need to give the transform being published a timestamp
//          ///Fourth, we need to pass the name of the parent node of the link we're creating
//          ///Fifth, we need to pass the name of the child node of the link we're creating
//      r.sleep();
//    }
//  }
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "mobile_robot_tf_publisher");
  ros::NodeHandle nh;

  // ros::Rate r(11);
  ros::Rate r(10);//40);

  tf::TransformBroadcaster base_to_laser_broadcaster;

  while(nh.ok()){
    base_to_laser_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.19, 0.0, 0.395)),
        ros::Time::now(),"base_link", "base_laser"));
    base_to_laser_broadcaster.sendTransform( 
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
        ros::Time::now(),"base_laser", "camera_link"));
    r.sleep();
  }
}

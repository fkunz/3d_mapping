#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ethzasl_icp_mapper/CorrectPose.h>

#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace correct_world_odom {

class CorrectPose {
 public:
  CorrectPose(tf::TransformListener &tf);
  ~CorrectPose() {}

  void updateTf(tf::TransformBroadcaster &tf_broadcaster);

 private:
  void poseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);

  ros::Subscriber initialpose_sub_;
  tf::StampedTransform initialpose_stamped_tf_;
  tf::TransformListener &tf_;
  };

  CorrectPose::CorrectPose(tf::TransformListener &tf) : tf_(tf) {
    ros::NodeHandle nh("");

    initialpose_sub_ = nh.subscribe("/initialpose", 5, &CorrectPose::poseCB, this);

    initialpose_stamped_tf_.child_frame_id_ = "odom";
    initialpose_stamped_tf_.frame_id_ = "odom_corrected";
    initialpose_stamped_tf_.setIdentity();
    initialpose_stamped_tf_.stamp_ = ros::Time::now();
  }

  void CorrectPose::updateTf(tf::TransformBroadcaster &tf_broadcaster) {
    initialpose_stamped_tf_.stamp_ = ros::Time::now();
    tf_broadcaster.sendTransform(initialpose_stamped_tf_);
  }

  void CorrectPose::poseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose) {
    ros::Time valid_tf_time = pose->header.stamp;
    try {
        tf_.getLatestCommonTime("world", "base_link", valid_tf_time, NULL);

    } catch (tf::TransformException ex) {
        ROS_FATAL("Failed to get latest common time. %s",ex.what());
        return;
    }

    geometry_msgs::PoseStamped initialpose, initialpose_in_odom;
    initialpose.header = pose->header;
    initialpose.pose = pose->pose.pose;
    try {
        tf_.transformPose("odom", initialpose, initialpose_in_odom);
    } catch (tf::TransformException ex) {
        ROS_FATAL("Failed to transform initialpose into odom frame. %s",ex.what());
        return;
    }

    tf::Stamped<tf::Pose> initialpose_tf_pose;
    tf::poseStampedMsgToTF(initialpose_in_odom, initialpose_tf_pose);
    initialpose_stamped_tf_.setData(initialpose_tf_pose.inverse());
    return;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "correct_icp_mapper_pose");

  tf::TransformListener tf(ros::Duration(10.0f));
  tf::TransformBroadcaster tf_broadcaster;

  correct_world_odom::CorrectPose correct_pose_node(tf);

  ros::Rate rate(100.0f);
  while (ros::ok()) {
      ros::spinOnce();
      correct_pose_node.updateTf(tf_broadcaster);
      rate.sleep();
  }
  exit(0);
}

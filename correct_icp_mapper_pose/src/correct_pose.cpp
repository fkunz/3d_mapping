#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ethzasl_icp_mapper/CorrectPose.h>

#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace correct_icp_pose {

  class CorrectPose
  {
  private:
    ros::ServiceClient client_;
    ros::Subscriber initialpose_sub_;
    tf::TransformListener &tf_;
	
  public:

    void poseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);

    CorrectPose(tf::TransformListener &tf);
  };

  CorrectPose::CorrectPose(tf::TransformListener &tf) :
      tf_(tf) {
      ros::NodeHandle nh("");

      initialpose_sub_ = nh.subscribe("/initialpose", 5, &CorrectPose::poseCB, this);

      client_ = nh.serviceClient<ethzasl_icp_mapper::CorrectPose>("/mapper/correct_pose");
  }

  void CorrectPose::poseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose) {

    ros::Time valid_tf_time = pose->header.stamp;
    try {
        tf_.getLatestCommonTime("world", "base_link", valid_tf_time, NULL);

    } catch (tf::TransformException ex) {
        ROS_FATAL("Failed to get latest common time. %s",ex.what());
        return;
    }

    geometry_msgs::PoseStamped initialpose, initialpose_in_world;
    initialpose.header = pose->header;
    initialpose.pose = pose->pose.pose;
    try {
        tf_.transformPose("world", initialpose, initialpose_in_world);
    } catch (tf::TransformException ex) {
        ROS_FATAL("Failed to transform initialpose into /world frame. %s",ex.what());
        return;
    }

    tf::StampedTransform bl_odom_stamped_tf;
    try {
        tf_.lookupTransform("base_link", "odom", valid_tf_time, bl_odom_stamped_tf);
    } catch (tf::TransformException ex) {
        ROS_FATAL("Failed to lookup base_link->odom transform. %s",ex.what());
        return;
    }

    tf::Stamped<tf::Pose> initialpose_tf_pose;
    tf::poseStampedMsgToTF(initialpose_in_world, initialpose_tf_pose);
    tf::Transform initialpose_tf(initialpose_tf_pose);
    tf::Transform bl_odom_tf(bl_odom_stamped_tf);

    tf::Transform correctpose_tf = initialpose_tf * bl_odom_tf;

    geometry_msgs::PoseStamped correctpose_tf_as_pose;
    tf::Stamped<tf::Pose> correctpose_tf_pose(correctpose_tf, valid_tf_time, "world");
    tf::poseStampedTFToMsg(correctpose_tf_pose, correctpose_tf_as_pose);

    ethzasl_icp_mapper::CorrectPoseRequest req;
    req.odom.header = correctpose_tf_as_pose.header;
    req.odom.pose.pose = correctpose_tf_as_pose.pose;
    req.odom.pose.covariance = pose->pose.covariance;
    req.odom.child_frame_id = "odom";
    ethzasl_icp_mapper::CorrectPoseResponse res;
    client_.call(req, res);
    return;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "correct_icp_mapper_pose");

  tf::TransformListener tf(ros::Duration(10));

  correct_icp_pose::CorrectPose correct_pose_node(tf);

  ros::spin();
  exit(0);
}

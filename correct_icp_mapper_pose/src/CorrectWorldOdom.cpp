//=================================================================================================
// Copyright (c) 2015, Florian Kunz, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace correct_world_odom {

class CorrectPose {
 public:
  CorrectPose(tf::TransformListener &tf);
  ~CorrectPose() {}
  void updateTf(tf::TransformBroadcaster &tf_broadcaster);

 private:
  void poseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);

  tf::TransformListener &tf_;
  std::string odom_corrected_frame_, odom_frame_;
  ros::Subscriber initialpose_sub_;
  tf::StampedTransform initialpose_stamped_tf_;
  };

  CorrectPose::CorrectPose(tf::TransformListener &tf) : tf_(tf) {
    ros::NodeHandle private_nh("~");
    private_nh.param("odom_corrected_frame", odom_corrected_frame_, std::string("odom_corrected"));
    private_nh.param("odom_frame", odom_frame_, std::string("odom"));

    initialpose_sub_ = private_nh.subscribe("/initialpose", 5, &CorrectPose::poseCB, this);

    initialpose_stamped_tf_.child_frame_id_ = odom_frame_;
    initialpose_stamped_tf_.frame_id_ = odom_corrected_frame_;
    initialpose_stamped_tf_.setIdentity();
    initialpose_stamped_tf_.stamp_ = ros::Time::now();
  }

  void CorrectPose::updateTf(tf::TransformBroadcaster &tf_broadcaster) {
    initialpose_stamped_tf_.stamp_ = ros::Time::now();
    tf_broadcaster.sendTransform(initialpose_stamped_tf_);
  }

  void CorrectPose::poseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose) {

    geometry_msgs::PoseStamped initialpose, initialpose_in_odom;
    initialpose.header = pose->header;
    initialpose.pose = pose->pose.pose;
    try {
        tf_.transformPose(odom_frame_, initialpose, initialpose_in_odom);
    } catch (tf::TransformException ex) {
        ROS_FATAL("Failed to transform initialpose into %s frame. %s",
                  odom_frame_.c_str(), ex.what());
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

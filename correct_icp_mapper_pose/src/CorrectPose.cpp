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
#include <ethzasl_icp_mapper/CorrectPose.h>

namespace correct_icp_pose {

class CorrectPose
{
public:
    CorrectPose(tf::TransformListener &tf);
    void poseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);
private:
    bool getTransform(const std::string &source_frame, const std::string &target_frame,
                      const ros::Time &valid_tf_time, tf::Transform &transform);
    bool getTransformForPose(const geometry_msgs::PoseWithCovarianceStamped &pose,
                             const std::string &source_frame, const std::string &target_frame,
                             const ros::Time &valid_tf_time, tf::Transform &transform);
    tf::TransformListener &tf_;
    std::string icp_map_frame_, icp_odom_frame_, robot_footprint_frame_;
    ros::Subscriber initialpose_sub_;
    ros::ServiceClient client_;
};

CorrectPose::CorrectPose(tf::TransformListener &tf) : tf_(tf)
{
    ros::NodeHandle private_nh("~");
    private_nh.param("icp_map_frame", icp_map_frame_, std::string("map"));
    private_nh.param("icp_odom_frame", icp_odom_frame_, std::string("odom"));
    private_nh.param("robot_footprint_frame", robot_footprint_frame_, std::string("base_link"));

    initialpose_sub_ = private_nh.subscribe("/initialpose", 5, &CorrectPose::poseCB, this);

    client_ = private_nh.serviceClient<ethzasl_icp_mapper::CorrectPose>("/mapper/correct_pose");
}

void CorrectPose::poseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose)
{
    ros::Time valid_tf_time = pose->header.stamp;
    try {
        tf_.getLatestCommonTime(icp_map_frame_, robot_footprint_frame_, valid_tf_time, NULL);
    } catch (tf::TransformException ex) {
        ROS_FATAL("Failed to get latest common time. %s",ex.what());
        return;
    }

    tf::Transform bl_odom_tf, initialpose_tf;
    getTransform(robot_footprint_frame_, icp_odom_frame_, valid_tf_time, bl_odom_tf);
    getTransformForPose(*pose, icp_map_frame_, icp_map_frame_, valid_tf_time, initialpose_tf);

    tf::Transform correctpose_tf = initialpose_tf * bl_odom_tf;

    geometry_msgs::PoseStamped correctpose_tf_as_pose;
    tf::Stamped<tf::Pose> correctpose_tf_pose(correctpose_tf, valid_tf_time, icp_map_frame_);
    tf::poseStampedTFToMsg(correctpose_tf_pose, correctpose_tf_as_pose);

    ethzasl_icp_mapper::CorrectPoseRequest req;
    req.odom.header = correctpose_tf_as_pose.header;
    req.odom.pose.pose = correctpose_tf_as_pose.pose;
    // TODO(fkunz): Covariance needs to be transformed into the icp_map_frame.
    if (pose->header.frame_id == req.odom.header.frame_id)
    {
        req.odom.pose.covariance = pose->pose.covariance;
    } else {
        ROS_WARN("pose covariance must be transformed from %s into %s frame. "
                 "This is not implemented jet. Send pose in %s and you will be fine.",
                 pose->header.frame_id.c_str(), req.odom.header.frame_id.c_str(),
                 req.odom.header.frame_id.c_str());
    }
    req.odom.child_frame_id = icp_odom_frame_;
    ethzasl_icp_mapper::CorrectPoseResponse res;
    client_.call(req, res);
    return;
}

bool CorrectPose::getTransform(const std::string &source_frame, const std::string &target_frame,
                               const ros::Time &valid_tf_time, tf::Transform &transform)
{
    tf::StampedTransform stamped_tf;
    try {
        tf_.lookupTransform(target_frame, source_frame, valid_tf_time, stamped_tf);
    } catch (tf::TransformException ex) {
        ROS_FATAL("Failed to lookup %s->%s transform. %s",
                  source_frame.c_str(), target_frame.c_str(), ex.what());
        return false;
    }
    transform = tf::Transform(stamped_tf);
    return true;
}

bool CorrectPose::getTransformForPose(const geometry_msgs::PoseWithCovarianceStamped &pose,
                                      const std::string &source_frame, const std::string &target_frame,
                                      const ros::Time &valid_tf_time, tf::Transform &transform)
{
    // Transform initialpose into world coordinates.
    geometry_msgs::PoseStamped pose_stamped, pose_stamped_in_world;
    pose_stamped.header = pose.header;
    pose_stamped.pose = pose.pose.pose;
    try {
        tf_.transformPose(target_frame, valid_tf_time, pose_stamped, source_frame, pose_stamped_in_world);
    } catch (tf::TransformException ex) {
        ROS_FATAL("Failed to transform initialpose into %s frame. %s", target_frame.c_str(), ex.what());
        return false;
    }

    tf::Stamped<tf::Pose> tf_pose_in_world;
    tf::poseStampedMsgToTF(pose_stamped_in_world, tf_pose_in_world);
    transform = tf::Transform(tf_pose_in_world);
    return true;
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

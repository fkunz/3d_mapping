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

#ifndef POINTCLOUD_ASSEMBLER_TRIGGER_PC_ASSEMBLER_TRIGGER_H_
#define POINTCLOUD_ASSEMBLER_TRIGGER_PC_ASSEMBLER_TRIGGER_H_

#include <pointcloud_assembler_trigger/AssemblerTriggerConfig.h>

#include <deque>
#include <iterator>
#include <set>
#include <boost/circular_buffer.hpp>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <laser_assembler/AssembleScans2.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace pointcloud_assembler_trigger {

class PcAssemblerTrigger {
 public:
  PcAssemblerTrigger(ros::Rate *const rate,
                             tf::TransformListener *const tf_listener,
                             ros::NodeHandle private_nh);
  ~PcAssemblerTrigger();

  void update();

 private:
  typedef AssemblerTriggerConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  typedef bool (PcAssemblerTrigger::*approxTime)(ros::Time *);

  void configCB(Config &config, uint32_t level);
  void jointStatesCB(const sensor_msgs::JointStateConstPtr &joint_states_ptr);
  void scanCloudCB(const sensor_msgs::PointCloud2ConstPtr &scan_cloud_ptr);
  bool callAssembler(ros::Time start_time, ros::Time end_time);
  bool callAssembler(ros::Time end_time);
  ros::Time calculateTimeBetweenStamps(const ros::Time &first, const ros::Time &second);
  void getTransforms();
  bool approxMaxAngleTime(ros::Time *peak_time);
  bool approxMinAngleTime(ros::Time *even_time);
  void waitForInitialScan();

  ros::Rate *const update_rate_;
  tf::TransformListener *const tf_listener_;
  ros::NodeHandle nh_;
  ros::Subscriber joint_states_sub_, laser_cloud_sub_;
  ros::Publisher command_publisher_, horizontal_cw_publisher_, horizontal_ccw_publisher_, pointcloud2_publisher_, pointcloud2_cw_publisher_, pointcloud2_ccw_publisher_;
  ros::ServiceClient assemble_pc2_client_;
  boost::shared_ptr<ReconfigureServer> reconfigs_server_;
  double laser_assembler_duration;
  uint min_elements_to_compare_, decision_threshold_, min_point_count_, joint_states_buffer_size_;
  std::string base_frame_, moving_frame_;
  ros::Time last_assemble_call_;
  boost::circular_buffer<sensor_msgs::JointState> last_laser_joint_states_;
  std::deque<ros::Time> last_scan_timestamps_;
  std::map<ros::Time, double> stamped_angles_;
  std::deque<double> last_roll_angles_;
  bool assemble_full_swipe_, check_sensor_pos_, initialized_, positive_roll_;
  ros::Time init_scan_start_time_, init_scan_end_time_;
  std::vector<approxTime> init_sequence_;
};

}

#endif  // POINTCLOUD_ASSEMBLER_TRIGGER_PC_ASSEMBLER_TRIGGER_H_

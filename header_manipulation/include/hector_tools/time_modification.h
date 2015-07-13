#ifndef TIME_MODIFICATION_H
#define TIME_MODIFICATION_H

#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <header_manipulation/TimeModificationConfig.h>
#include <laser_assembler/AssembleScans2.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <topic_tools/shape_shifter.h>
#include <tf/tf.h>

class TimeModification {
 private:
  typedef header_manipulation::TimeModificationConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

  ros::Publisher generic_pub_;
  ros::Subscriber generic_sub_;
  boost::mutex mutex_;
  bool output_advertised_;
  ros::NodeHandle private_nh_;
  boost::shared_ptr<ReconfigureServer> reconf_server_;
  unsigned int seq_counter_;
  ros::Duration* const time_offset_;
  ros::Rate* const update_rate_;

  void configCB(Config &config, uint32_t level);
  void inputCB(const topic_tools::ShapeShifter::ConstPtr &input);

 public:
  TimeModification(ros::Duration *const time_offset, ros::Rate *const rate);
  ~TimeModification();

  ros::Rate * const getUpdateRate() const;
  void update();
};

#endif //TIME_MODIFICATION_H

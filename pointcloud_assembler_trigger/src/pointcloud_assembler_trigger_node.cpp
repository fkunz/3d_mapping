#include <pointcloud_assembler_trigger/PointcloudAssemblerTriggerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <laser_assembler/AssembleScans2.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>

class PointcloudAssemblerTrigger {

private:

  typedef pointcloud_assembler_trigger::PointcloudAssemblerTriggerConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

  ros::NodeHandle nh_;

  ros::Publisher command_publisher_, pointcloud2_publisher_;
  ros::ServiceClient assemble_pointcloud2_client_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  double laser_assembler_duration;
  ros::Rate* const update_rate_;
  ros::Time last_assemble_call_;

  //Callback functions
  void configCB(Config &config, uint32_t level);

  //Helper functions
  void callAssembler();


public:

  PointcloudAssemblerTrigger(ros::NodeHandle private_nh, ros::Rate* const rate);
    ~PointcloudAssemblerTrigger();

    ros::Rate * const getUpdateRate() const;
    void update();
};

PointcloudAssemblerTrigger::PointcloudAssemblerTrigger(ros::NodeHandle private_nh, ros::Rate* const rate) : update_rate_(rate)
{
  laser_assembler_duration = 0.5;

  pointcloud2_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("aggregated", 10);
  assemble_pointcloud2_client_ = nh_.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");

  reconfigure_server_.reset(new ReconfigureServer(private_nh));
  reconfigure_server_->setCallback(boost::bind(&PointcloudAssemblerTrigger::configCB, this, _1, _2));

  last_assemble_call_ = ros::Time::now();
}

PointcloudAssemblerTrigger::~PointcloudAssemblerTrigger()
{
    delete update_rate_;
}


void PointcloudAssemblerTrigger::configCB(Config &config, uint32_t level)
{
  laser_assembler_duration = config.laser_assembler_duration;
  *update_rate_ = ros::Rate(config.update_rate);
}


void PointcloudAssemblerTrigger::callAssembler()
{
  laser_assembler::AssembleScans2 assemble_srv;
  assemble_srv.request.begin = last_assemble_call_;
  last_assemble_call_ = ros::Time::now();
  assemble_srv.request.end = last_assemble_call_;
  if(assemble_pointcloud2_client_.call(assemble_srv)) {
      pointcloud2_publisher_.publish(assemble_srv.response.cloud);
      ROS_DEBUG("[hector_nutating_scanner] laser_assambler call succeeded.");
    }
  else {
      ROS_WARN_ONCE("[hector_nutating_scanner] laser_assambler call failed.");
      ROS_DEBUG("[hector_nutating_scanner] laser_assambler call failed.");
    }
}

ros::Rate* const PointcloudAssemblerTrigger::getUpdateRate() const
{
    return update_rate_;
}


void PointcloudAssemblerTrigger::update()
{
    ros::Time current_time = ros::Time::now();

    if ((current_time - last_assemble_call_) > ros::Duration(laser_assembler_duration)) {
        callAssembler();
    }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_assembler_trigger");

  ros::NodeHandle private_nh("~");
  ros::Rate* const rate = new ros::Rate(60);
  PointcloudAssemblerTrigger pointcloud_assembler_trigger(private_nh, rate);
    while (ros::ok()) {
    ros::spinOnce();
    pointcloud_assembler_trigger.update();
    rate->sleep();
    }

  return 0;
}

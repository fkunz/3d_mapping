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

#include <pointcloud_assembler_trigger/pc_assembler_trigger.h>

namespace pointcloud_assembler_trigger {

PcAssemblerTrigger::PcAssemblerTrigger(ros::Rate *const rate, tf::TransformListener *const tf_listener, ros::NodeHandle& private_nh)
    : update_rate_(rate), tf_listener_(tf_listener)
{
    // Get parameter values
    int tmp_int;
    private_nh.param("min_elements_to_compare", tmp_int, 4);
    if (tmp_int > 0)
    {
        min_elements_to_compare_ = tmp_int;
    } else
    {
        ROS_ERROR("Was given negative or null value for min_elements_to_compare.");
    }
    private_nh.param("decision_threshold", tmp_int, 3);
    if (tmp_int > 0)
    {
        decision_threshold_ = tmp_int;
    } else
    {
        ROS_ERROR("Was given negative or null value for decision_threshold.");
    }
    private_nh.param("min_point_count", tmp_int, 3);
    if (tmp_int > 0)
    {
        min_point_count_ = tmp_int;
    } else
    {
        ROS_ERROR("Was given negative or null value for min_point_count.");
    }
    private_nh.param("joint_states_buffer_size", tmp_int, 100);
    if (tmp_int > 0)
    {
        joint_states_buffer_size_ = tmp_int;
    } else
    {
        ROS_ERROR("Was given negative or null value for joint_states_buffer_size.");
    }

    private_nh.param("base_frame", base_frame_, std::string("base_link"));
    private_nh.param("sensor_frame", moving_frame_, std::string("laser1_frame"));

    private_nh.param("assemble_full_swipe", assemble_full_swipe_, false);
    private_nh.param("check_sensor_pos", check_sensor_pos_, true);

    private_nh.param("init_topic", init_topic_, std::string("/point_map"));
    private_nh.param("init_assembler_duration", init_assembler_duration_, 3.0);

    init_sub_ = nh_.subscribe(init_topic_, 1, &PcAssemblerTrigger::initTopicCB, this);
    joint_states_sub_ = nh_.subscribe("joint_states", 10, &PcAssemblerTrigger::jointStatesCB, this);
    laser_cloud_sub_ = private_nh.subscribe("modified", 10, &PcAssemblerTrigger::scanCloudCB, this);
    syscommand_sub_ = nh_.subscribe("syscommand", 10, &PcAssemblerTrigger::syscommandCB, this);

    pointcloud2_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("assembled", 10);
    pointcloud2_cw_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("cw_assembled", 10);
    pointcloud2_ccw_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("ccw_assembled", 10);
    assemble_pc2_client_ = nh_.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");

    reconfigs_server_.reset(new ReconfigureServer(private_nh));
    reconfigs_server_->setCallback(boost::bind(&PcAssemblerTrigger::configCB, this, _1, _2));

    // Initialize buffers and flags
    last_laser_joint_states_ = boost::circular_buffer<sensor_msgs::JointState>(joint_states_buffer_size_);

    resetInitialState();
}

PcAssemblerTrigger::~PcAssemblerTrigger()
{
}

void PcAssemblerTrigger::update()
{
    ros::Time start_time = ros::Time::now();
    if (last_assemble_call_.isZero())
    {
        last_assemble_call_ = start_time;
        return;
    }

    if (last_assemble_call_ > start_time)
    {
        ROS_WARN("Jump back in time detected. Reset to inital state!");
        resetInitialState();
    }

    getTransforms();

    // Check if buffer is filled.
    if (check_sensor_pos_ && stamped_angles_.size() > min_elements_to_compare_)
    {
        if (!initialized_)
        {
            waitForInitialScan();
            return;
        }

        ros::Time peak_time = start_time;
        if (approxMaxAngleTime(&peak_time))
        {
            if (!callAssembler(peak_time))
            {
                ROS_WARN("Assembler call failed or returned not enough points. Actuator just changed direction at max angle so the partial pointcloud must be dropped.");
                last_assemble_call_ = peak_time;
            }
            positive_roll_ = !positive_roll_;
            return;
        }

        ros::Time even_time = start_time;
        if (!assemble_full_swipe_ && approxMinAngleTime(&even_time))
        {
            callAssembler(even_time);
            return;
        }
    }

    ros::Time current_time = start_time;
    ros::Duration assembler_duration(laser_assembler_duration_);
    if (!initialized_)
        assembler_duration.fromSec(init_assembler_duration_);

    if ((current_time - last_assemble_call_) > ros::Duration(assembler_duration))
    {
        ROS_DEBUG("Calling assembler at current_time: %f as last_assemble_call was %f. laser_assembler_duration is: %f", current_time.toSec(), last_assemble_call_.toSec(), laser_assembler_duration_);
        callAssembler(current_time);
        return;
    }
}

void PcAssemblerTrigger::configCB(Config &config, uint32_t level)
{
    ROS_INFO("Assembler Trigger in config callback.");
    laser_assembler_duration_ = config.laser_assembler_duration;
    *update_rate_ = ros::Rate(config.update_rate);
}

void PcAssemblerTrigger::initTopicCB(const topic_tools::ShapeShifter::ConstPtr &init_msg_ptr)
{
    ROS_INFO("Received first message on init topic, leaving init state.");
    initialized_ = true;
    init_sub_.shutdown();
}

void PcAssemblerTrigger::jointStatesCB(const sensor_msgs::JointStateConstPtr &joint_states_ptr)
{
    sensor_msgs::JointState laser_joint_states;
    for (size_t i = 0; i < joint_states_ptr->name.size(); ++i)
    {
        if (strncmp(joint_states_ptr->name[i].c_str(), "laser1/x", 8) == 0)
        {
            laser_joint_states.header = joint_states_ptr->header;
            laser_joint_states.effort.push_back(joint_states_ptr->effort[i]);
            laser_joint_states.name.push_back(joint_states_ptr->name[i]);
            laser_joint_states.position.push_back(joint_states_ptr->position[i]);
            laser_joint_states.velocity.push_back(joint_states_ptr->velocity[i]);
            continue;
        }
        if (strncmp(joint_states_ptr->name[i].c_str(), "laser1/y", 8) == 0)
        {
            laser_joint_states.header = joint_states_ptr->header;
            laser_joint_states.effort.push_back(joint_states_ptr->effort[i]);
            laser_joint_states.name.push_back(joint_states_ptr->name[i]);
            laser_joint_states.position.push_back(joint_states_ptr->position[i]);
            laser_joint_states.velocity.push_back(joint_states_ptr->velocity[i]);
        }
    }
    if (laser_joint_states.name.size() > 0)
    {
        last_laser_joint_states_.push_back(laser_joint_states);
    }
}

void PcAssemblerTrigger::scanCloudCB(const sensor_msgs::PointCloud2ConstPtr &scan_cloud_ptr)
{
    last_scan_timestamps_.push_front(scan_cloud_ptr->header.stamp);
}

void PcAssemblerTrigger::syscommandCB(const std_msgs::StringConstPtr &syscommand_ptr)
{
    if (syscommand_ptr->data == "reset")
    {
        resetInitialState();
    }
}

bool PcAssemblerTrigger::callAssembler(ros::Time start_time, ros::Time end_time)
{
    ROS_DEBUG("Assembler Trigger calling assembler.");
    laser_assembler::AssembleScans2 assemble_srv;
    assemble_srv.request.begin = start_time;
    assemble_srv.request.end = end_time;
    if (assemble_pc2_client_.call(assemble_srv))
    {
        if (assemble_srv.response.cloud.width < min_point_count_)
        {
            ROS_INFO("Assembler call returned %i points. last_assemble_call stayes: %f will not change to %f", static_cast<int>(assemble_srv.response.cloud.width), last_assemble_call_.toSec(), end_time.toSec());
            return false;
        }
        ROS_DEBUG("Assembler call returned %i points. In time frame: %f - %f", static_cast<int>(assemble_srv.response.cloud.width), start_time.toSec(), end_time.toSec());
        pointcloud2_publisher_.publish(assemble_srv.response.cloud);
        if (positive_roll_)
        {
            assemble_srv.response.cloud.header.frame_id = "cw_" + assemble_srv.response.cloud.header.frame_id ;
            pointcloud2_cw_publisher_.publish(assemble_srv.response.cloud);
        } else
        {
            assemble_srv.response.cloud.header.frame_id = "ccw_" + assemble_srv.response.cloud.header.frame_id;
            pointcloud2_ccw_publisher_.publish(assemble_srv.response.cloud);
        }
        last_assemble_call_ = end_time;
        return true;
    } else
    {
        ROS_WARN_ONCE("laser_assembler call failed.");
        ROS_DEBUG("laser_assembler call failed.");
        return false;
    }
}

bool PcAssemblerTrigger::callAssembler(ros::Time end_time)
{
    if (last_assemble_call_ > end_time)
    {
        ROS_WARN("Current assemble end time is further in the past than last assemble call end time. Skipping call.");
        last_assemble_call_ = end_time;
        return false;
    }
    return callAssembler(last_assemble_call_, end_time);
}

ros::Time PcAssemblerTrigger::calculateTimeBetweenStamps(const ros::Time &first, const ros::Time &second)
{
    return ros::Time(first.toSec() / 2 + second.toSec() / 2);
}

void PcAssemblerTrigger::getTransforms()
{
    while (!last_scan_timestamps_.empty())
    {
        // Get transfrom for laser roll servo and calculate roll angle.
        tf::StampedTransform transform;
        try
        {
            tf_listener_->lookupTransform(base_frame_, moving_frame_, last_scan_timestamps_.back(), transform);
        }
        catch (tf::TransformException ex)
        {
//            ROS_ERROR_THROTTLE(1.0, "%s", ex.what());
//            ROS_DEBUG("lookupTransform failed.%s", ex.what());
            if (last_scan_timestamps_.size() > 2*min_elements_to_compare_)
            {
                last_scan_timestamps_.pop_back();
                continue;
            }
            else
            {
                break;
            }
        }
        double laser_roll = transform.getRotation().getAngle() * transform.getRotation().getAxis().getX();
        stamped_angles_[last_scan_timestamps_.back()] = laser_roll;
        last_scan_timestamps_.pop_back();
    }
    //  ROS_DEBUG("stamped_angles has size %i", static_cast<int>(stamped_angles_.size()));
}

bool PcAssemblerTrigger::approxMaxAngleTime(ros::Time *peak_time)
{
    // Iterate over past angles.
    uint outliers = 0;
    std::map<ros::Time, double>::iterator it = stamped_angles_.begin();
    std::map<ros::Time, double>::iterator next_it = it;
    std::map<ros::Time, double>::iterator before_peak_it = it;
    for (++next_it ; next_it != stamped_angles_.end(); it = next_it++)
    {
        if (!positive_roll_ && (it->second < next_it->second) ||
                positive_roll_ && (it->second > next_it->second))
        {
            ++outliers;
        } else
        {
            // For strict checking set outliers back to 0.
            outliers = 0;
            before_peak_it = it;
        }
        if (outliers >= decision_threshold_)
        {
            if (positive_roll_)
            {
                ROS_DEBUG("changing direction from pos to neg at %f; %f", it->first.toSec(), it->second);
            } else
            {
                ROS_DEBUG("changing direction from neg to pos at %f; %f", it->first.toSec(), it->second);
            }
            // TODO(fkunz): decide which time to return. before_peak_it holds the last iterator before an/the first outlier was detected.
            *peak_time = calculateTimeBetweenStamps((before_peak_it)->first, (--before_peak_it)->first);
            stamped_angles_.erase(stamped_angles_.begin(), next_it);
            ROS_DEBUG("stamped_angles after erase size %i", static_cast<int>(stamped_angles_.size()));
            return true;
        }
    }
    return false;
}

bool PcAssemblerTrigger::approxMinAngleTime(ros::Time *even_time)
{
    // Iterate over past angles.
    std::map<ros::Time, double>::iterator it = stamped_angles_.begin();
    std::map<ros::Time, double>::iterator next_it = it;
    for (++next_it ; next_it != stamped_angles_.end(); it = next_it++)
    {
        if ((it->second * next_it->second) < 0)
        {
            if (it->second > next_it->second)
            {
                ROS_DEBUG("changing sign from pos to neg at %f; %f", it->first.toSec(), it->second);
            } else
            {
                ROS_DEBUG("changing sign from neg to pos at %f; %f", it->first.toSec(), it->second);
            }
            *even_time = calculateTimeBetweenStamps(it->first, next_it->first);
            stamped_angles_.erase(stamped_angles_.begin(), next_it);
            ROS_DEBUG("stamped_angles after erase size %i", static_cast<int>(stamped_angles_.size()));
            return true;
        }
    }
    return false;
}

void PcAssemblerTrigger::waitForInitialScan()
{
    ros::Time current_time = ros::Time::now();
    ros::Time *time_ptr = &current_time;

    switch (init_sequence_.size())
    {
    case 0 :
    {
        if ((ros::Time(1,0) < init_scan_start_time_) && (init_scan_start_time_ < init_scan_end_time_))
        {
            ROS_INFO("Calling Assembler for initial scan. %f - %f", init_scan_start_time_.toSec(), init_scan_end_time_.toSec());
            callAssembler(init_scan_start_time_, init_scan_end_time_);
        } else
        {
            ROS_WARN("Init squence failed.");
        }
        initialized_ = true;
        return;
    }
    case 1 :
        time_ptr = &init_scan_end_time_;
        break;
    case 2 :
        time_ptr = &init_scan_start_time_;
        break;
    }
    if ((this->*init_sequence_.back())(time_ptr))
    {
        init_sequence_.pop_back();
        ROS_INFO("Check in init_squence succeeded. Leaving %i checks.", static_cast<int>(init_sequence_.size()));
    }
}

void PcAssemblerTrigger::resetInitialState()
{
    initialized_ = false;
    init_sub_ = nh_.subscribe(init_topic_, 1, &PcAssemblerTrigger::initTopicCB, this);

    // Setting up inital scan sequence. Wait for a maximum peak angle,
    // followed by a minimum then determin the next two maximum peak times
    // to form a scan from side to side.
    init_sequence_.push_back(&PcAssemblerTrigger::approxMaxAngleTime);
    init_sequence_.push_back(&PcAssemblerTrigger::approxMaxAngleTime);
    init_sequence_.push_back(&PcAssemblerTrigger::approxMinAngleTime);
//    init_sequence_.push_back(&PcAssemblerTrigger::approxMaxAngleTime);
    init_scan_start_time_ = ros::Time(0,0);

    last_assemble_call_ = ros::Time::now();
    positive_roll_ = true;
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trigger");

    ros::NodeHandle private_nh("~");
    ros::Rate *const rate = new ros::Rate(20);
    tf::TransformListener *const tf_listener = new tf::TransformListener(ros::Duration(10));

    pointcloud_assembler_trigger::PcAssemblerTrigger assembler_trigger(rate, tf_listener, private_nh);

    while (ros::ok())
    {
        ros::spinOnce();
        assembler_trigger.update();
        rate->sleep();
    }

    delete rate;
    delete tf_listener;

    return 0;
}

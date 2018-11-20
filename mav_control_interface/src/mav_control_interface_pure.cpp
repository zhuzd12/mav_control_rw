/*
 * Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or pureied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <Eigen/Geometry>
#include <std_msgs/String.h>
#include <tf_conversions/tf_eigen.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "mav_control_interface_pure.h"
#include "parameters.h"

namespace mav_control_interface {

MavControlInterfacePure::MavControlInterfacePure(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                                                 std::shared_ptr<PositionControllerInterface> controller)
    : nh_(nh),
      private_nh_(private_nh),
      controller_(controller)
{
  ros::NodeHandle interface_nh(private_nh, "control_interface");

  private_nh_.param<std::string>("reference_frame", reference_frame_id_, "odom");
  command_publisher_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>(
      mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1);
  
  state_info_publisher_ = nh_.advertise<std_msgs::String>("state_info", 1, true);
  predicted_state_publisher_ = nh_.advertise<visualization_msgs::Marker>( "predicted_state", 0 );
  current_reference_publisher_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "command/current_reference", 1);
  
//   std::string mav_name;
//   private_nh_.getParam("mav_name", mav_name);
//   ROS_ERROR("mav name: %s", mav_name.c_str());
  if (private_nh_.getParam("enemy_mav_name", enemy_mav_name_))
    ROS_INFO("enemy mav name: %s", enemy_mav_name_.c_str());
  else
    ROS_ERROR("enemy mav name in mav_control_interface_pure is not loaded from ros parameter server!");

  std::string obstacle_odometry_topic_name = "/" + enemy_mav_name_ + "/ground_truth/" + mav_msgs::default_topics::ODOMETRY;
  // ROS_ERROR("enemy mav topic name: %s", obstacle_odometry_topic_name.c_str());
  obstacle_odometry_subscriber_ = nh_.subscribe(obstacle_odometry_topic_name, 1,
                                                &MavControlInterfacePure::ObstacleOdometryCallback, this,
                                                ros::TransportHints().tcpNoDelay());
  
  std::string obstacle_prediction_topic_name = "/" + enemy_mav_name_ + "/KF_prediction_observer/prediction";
  ROS_INFO("enemy mav prediction topic name: %s", obstacle_prediction_topic_name.c_str());
  obstacle_prediction_subscriber_ = nh_.subscribe(obstacle_prediction_topic_name, 1, 
                                                &MavControlInterfacePure::ObstaclePredictionCallback, this,
                                                ros::TransportHints().tcpNoDelay());

  command_trajectory_subscriber_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_POSE, 1,
                                                 &MavControlInterfacePure::CommandPoseCallback, this);

  command_trajectory_array_subscriber_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &MavControlInterfacePure::CommandTrajectoryCallback, this);

  odometry_subscriber_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                       &MavControlInterfacePure::OdometryCallback, this,
                                       ros::TransportHints().tcpNoDelay());

  takeoff_server_ = nh.advertiseService("takeoff", &MavControlInterfacePure::TakeoffCallback, this);
  
  receive_prediction_ = false;
  receive_reference_ = false;
  ROS_INFO_STREAM("Created pure control interface for controller " << controller->getName());
  // state_machine_->start();
}

MavControlInterfacePure::~MavControlInterfacePure()
{
  // state_machine_->stop();
}

void MavControlInterfacePure::CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  mav_msgs::EigenTrajectoryPoint reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*msg, &reference);
  current_reference_queue_.clear();
  // mav_msgs::EigenTrajectoryPointDeque references;
  current_reference_queue_.push_back(reference);
  controller_->setReference(reference);
  receive_reference_ = true;
}

void MavControlInterfacePure::CommandTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
  int array_size = msg->points.size();
  if (array_size == 0)
    return;

  mav_msgs::EigenTrajectoryPointDeque references;
  mav_msgs::eigenTrajectoryPointDequeFromMsg(*msg, &references);
  current_reference_queue_ = references;
  if (current_reference_queue_.size() == 1)
  {
      controller_->setReference(current_reference_queue_.at(0));
  }
  else
  {
      controller_->setReferenceArray(current_reference_queue_);
  }
  receive_reference_ = true;
}

void MavControlInterfacePure::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
  ROS_INFO_ONCE("Control interface got first odometry message.");
  // mav_msgs::EigenOdometry odometry;
  mav_msgs::eigenOdometryFromMsg(*odometry_msg, &current_state_);
  // Stamp odometry upon reception to be robust against timestamps "in the future".
  current_state_.timestamp_ns = ros::Time::now().toNSec();
  controller_->setOdometry(current_state_);
  // compute cmd
  if (!receive_reference_ || !receive_prediction_) // todo: add receive_prediction and prediction time gap(ros::Time::now() - t_prediction < 0.1)
  {
      // ROS_INFO("reference not received!");
      return;
  }
  mav_msgs::EigenRollPitchYawrateThrust command;
  controller_->calculateRollPitchYawrateThrustCommand(&command);
  // publish cmd & current reference & predicted state
  publishAttitudeCommand(command);
  // publishPredictedState();

}

void MavControlInterfacePure::ObstacleOdometryCallback(const nav_msgs::OdometryConstPtr& obstacle_odometry_msg)
{
  ROS_INFO_ONCE("Control interface got first obstacle odometry message.");
  mav_msgs::EigenOdometry obstacle_odometry;
  mav_msgs::eigenOdometryFromMsg(*obstacle_odometry_msg, &obstacle_odometry);
  // Stamp odometry upon reception to be robust against timestamps "in the future".
  obstacle_odometry.timestamp_ns = ros::Time::now().toNSec();
  // ROS_ERROR("enemy mav x: %f, y: %f, z: %f",obstacle_odometry.position_W(0), obstacle_odometry.position_W(1), obstacle_odometry.position_W(2));
}

void MavControlInterfacePure::ObstaclePredictionCallback(const mav_disturbance_observer::PredictionArrayPtr msg)
{
  ROS_INFO_ONCE("Control interface got first obstacle prediction message.");
  // t_prediction = ros::Time::now();
  // ROS_ERROR("enemy mav x: %f, y: %f, z: %f",obstacle_odometry.position_W(0), obstacle_odometry.position_W(1), obstacle_odometry.position_W(2));
  // state_machine_->process_event(state_machine::ObstacleOdometryUpdate(obstacle_odometry));
  // todo: controller_->setPrediction();
  controller_->setPrediction(msg);
  receive_prediction_ = true;
}

bool MavControlInterfacePure::TakeoffCallback(std_srvs::Empty::Request& request,
                                              std_srvs::Empty::Response& response)
{
  ROS_INFO("Take off event sent");
  current_reference_queue_.clear();

  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_point.time_from_start_ns = 0;
  trajectory_point.position_W = current_state_.position_W;
  trajectory_point.position_W.z() += 1.0;
  trajectory_point.setFromYaw(mav_msgs::yawFromQuaternion(current_state_.orientation_W_B));
  current_reference_queue_.push_back(trajectory_point);

  ROS_INFO_STREAM("final take off position: " << trajectory_point.position_W.transpose());
  controller_->setReference(trajectory_point);
  return true;
}

void MavControlInterfacePure::publishAttitudeCommand(const mav_msgs::EigenRollPitchYawrateThrust& command)
{
    mav_msgs::RollPitchYawrateThrustPtr msg(new mav_msgs::RollPitchYawrateThrust);
    mav_msgs::EigenRollPitchYawrateThrust tmp_command = command;
    tmp_command.thrust.x() = 0;
    tmp_command.thrust.y() = 0;
    tmp_command.thrust.z() = std::max(0.0, command.thrust.z());
    msg->header.stamp = ros::Time::now();  // TODO(acmarkus): get from msg
    mav_msgs::msgRollPitchYawrateThrustFromEigen(command, msg.get());
    command_publisher_.publish(msg);
}

void MavControlInterfacePure::publishStateInfo(const std::string& info)
{
    if (state_info_publisher_.getNumSubscribers() > 0) {
        std_msgs::StringPtr msg(new std_msgs::String);
        msg->data = info;
        state_info_publisher_.publish(msg);
    }
}

void MavControlInterfacePure::publishPredictedState()
{
    if (predicted_state_publisher_.getNumSubscribers() > 0) {
        mav_msgs::EigenTrajectoryPointDeque predicted_state;
        controller_->getPredictedState(&predicted_state);
        visualization_msgs::Marker marker_queue;
        marker_queue.header.frame_id = reference_frame_id_;
        marker_queue.header.stamp = ros::Time();
        marker_queue.type = visualization_msgs::Marker::LINE_STRIP;
        marker_queue.scale.x = 0.05;
        marker_queue.color.a = 1.0;
        marker_queue.color.r = 1.0;

        // marker_heading.type = visualization_msgs::Marker::ARROW;
        for (size_t i = 0; i < predicted_state.size(); i++) {
            geometry_msgs::Point p;
            p.x = predicted_state.at(i).position_W(0);
            p.y = predicted_state.at(i).position_W(1);
            p.z = predicted_state.at(i).position_W(2);
            marker_queue.points.push_back(p);
        }
        predicted_state_publisher_.publish(marker_queue);
    }
}

void MavControlInterfacePure::PublishCurrentReference()
{
  ros::Time time_now = ros::Time::now();
  mav_msgs::EigenTrajectoryPoint current_reference;
  controller_->getCurrentReference(&current_reference);

  tf::Quaternion q;
  tf::Vector3 p;
  tf::vectorEigenToTF(current_reference.position_W, p);
  tf::quaternionEigenToTF(current_reference.orientation_W_B, q);

  tf::Transform transform;
  transform.setOrigin(p);
  transform.setRotation(q);

  transform_broadcaster_.sendTransform(
      tf::StampedTransform(transform, time_now, reference_frame_id_, nh_.getNamespace() + "/current_reference"));

  if (current_reference_publisher_.getNumSubscribers() > 0) {
    trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(current_reference, msg.get());
    msg->header.stamp = time_now;
    msg->header.frame_id = reference_frame_id_;
    current_reference_publisher_.publish(msg);
  }
}

}  // end namespace mav_control_interface


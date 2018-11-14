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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or Pureied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MAV_CONTROL_INTERFACE_PURE_H
#define MAV_CONTROL_INTERFACE_PURE_H

#include <deque>
#include <string>
#include <ros/ros.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_control_interface/deadzone.h>
#include <mav_control_interface/position_controller_interface.h>
#include <mav_control_interface/rc_interface.h>

#include <mav_disturbance_observer/KF_prediction_observer.h>

#include "state_machine.h"

namespace mav_control_interface {

class MavControlInterfacePure
{
 public:
  MavControlInterfacePure(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                          std::shared_ptr<PositionControllerInterface> controller);

  virtual ~MavControlInterfacePure();

 private:

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::shared_ptr<PositionControllerInterface> controller_;

  ros::Subscriber odometry_subscriber_;
  ros::Subscriber command_trajectory_subscriber_;
  ros::Subscriber command_trajectory_array_subscriber_;
  ros::Subscriber obstacle_odometry_subscriber_;
  ros::Subscriber obstacle_prediction_subscriber_;

  ros::Publisher command_publisher_;
  ros::Publisher current_reference_publisher_;
  ros::Publisher predicted_state_publisher_;
  ros::Publisher state_info_publisher_;

  ros::ServiceServer takeoff_server_; 

  std::string reference_frame_id_;
  std::string enemy_mav_name_;
  bool receive_reference;
  bool receive_prediction;
  // bool receive_odometry;
  // ros::Time t_prediction;
  mav_msgs::EigenOdometry current_state_;
  mav_msgs::EigenTrajectoryPointDeque current_reference_queue_;
  tf::TransformBroadcaster transform_broadcaster_;

  void CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void CommandTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void ObstacleOdometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void ObstaclePredictionCallback(const mav_disturbance_observer::PredictionArrayPtr msg);
  bool TakeoffCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response);

  void publishAttitudeCommand(const mav_msgs::EigenRollPitchYawrateThrust& command);
  void publishStateInfo(const std::string& info);
  void publishPredictedState();
  void PublishCurrentReference();
};

} /* namespace mav_control_interface */

#endif /* LOW_LEVEL_FLIGHT_MANAGER_H_ */

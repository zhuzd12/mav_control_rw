/*
 Copyright (c) 2018, Zhengda zhu, GNC, Tsinghua, China
 
 You can contact the author at <zhuzd12@gmail.com>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef KFPredictionObserver_H_
#define KFPredictionObserver_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

//dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <mav_disturbance_observer/KFPredictionObserverConfig.h>
#include <mav_disturbance_observer/PredictionPoint.h>
#include <mav_disturbance_observer/PredictionArray.h>

namespace mav_control {
class KFPredictionObserver
{
 public:

  KFPredictionObserver(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  void reset(const Eigen::Vector3d& initial_position, const Eigen::Vector3d& initial_velocity);

  //Getters
  Eigen::Vector3d getEstimatedPosition() const
  {
    if (initialized_)
      return state_.segment(0, 3);
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d getEstimatedVelocity() const
  {
    if (initialized_)
      return state_.segment(3, 3);
    else
      return Eigen::Vector3d::Zero();
  }

  void getEstimatedState(Eigen::VectorXd* estimated_state) const;
  // void updatePrediction();

  //Feeding
  void feedPositionMeasurement(const Eigen::Vector3d& position);
  void feedVelocityMeasurement(const Eigen::Vector3d& velocity);

  bool updateEstimator();

  virtual ~KFPredictionObserver();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  static constexpr int kStateSize = 6;
  static constexpr int kMeasurementSize = 6;
  static constexpr double kGravity = 9.8066;
  static constexpr double Ts = 0.1;
  static constexpr int TN = 20;

  typedef Eigen::Matrix<double, kStateSize, 1> StateVector;

  ros::NodeHandle nh_, private_nh_, observer_nh_;
  bool initialized_;
  Eigen::Matrix<double, kStateSize, 1> state_;  // [pos, vel]
  Eigen::Matrix<double, kStateSize, 1> predicted_state_;
  Eigen::Matrix<double, kStateSize, 1> old_state_;
  Eigen::Matrix<double, kStateSize, 1> new_state_;
  Eigen::Matrix<double, kMeasurementSize, 1> measurements_;  // [pos, vel]
  Eigen::Vector4d roll_pitch_yaw_thrust_cmd_;
  Eigen::Matrix<double, kStateSize, 1> process_noise_covariance_; // We express it as diag() later.
  Eigen::Matrix<double, kStateSize, kStateSize> state_covariance_;
  Eigen::Matrix<double, kStateSize, 1> initial_state_covariance_; // P0
  Eigen::Matrix<double, kMeasurementSize, 1> measurement_covariance_; // We express it as diag() later.

  Eigen::SparseMatrix<double> F_; // System dynamics matrix.
//  Eigen::Matrix<double, kStateSize, kStateSize> F_; // System dynamics matrix.
  Eigen::Matrix<double, kStateSize, kMeasurementSize> K_; // Kalman gain matrix.
  Eigen::SparseMatrix<double> H_; // Measurement matrix.
//  Eigen::Matrix<double, kMeasurementSize, kStateSize> H_; // Measurement matrix.

  double sampling_time_;

  ros::ServiceServer service_;
  ros::Publisher observer_state_pub_;

  void initialize();
  void systemDynamics(double dt);
  void propagate(double dt);

  dynamic_reconfigure::Server<mav_disturbance_observer::KFPredictionObserverConfig> dyn_config_server_;

  void DynConfigCallback(mav_disturbance_observer::KFPredictionObserverConfig &config, uint32_t level);

  void loadROSParams();

};
}
#endif /* SRC_KFPredictionObserver_H_ */

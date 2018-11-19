#include <mav_disturbance_observer/KF_prediction_observer.h>

namespace mav_prediction {
    constexpr int KFPredictionObserver::kStateSize;
    constexpr int KFPredictionObserver::kMeasurementSize;
    constexpr double KFPredictionObserver::kGravity;

    KFPredictionObserver::KFPredictionObserver(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      observer_nh_(private_nh, "KF_prediction"),
      initialized_(false),
      F_(kStateSize, kStateSize),
      H_(kMeasurementSize, kStateSize),
      dyn_config_server_(ros::NodeHandle(private_nh, "KF_prediction"))
    {
        state_covariance_.setZero();
        process_noise_covariance_.setZero();
        measurement_covariance_.setZero();
        initialize();
    }

    void KFPredictionObserver::initialize()
    {

        ROS_INFO("start initializing mav_prediction_observer:KF");

        odometry_subscriber_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                       &KFPredictionObserver::OdometryCallback, this,
                                       ros::TransportHints().tcpNoDelay());

        observer_state_pub_ = private_nh_.advertise<mav_disturbance_observer::PredictionArray>(
            "prediction", 10);

        dynamic_reconfigure::Server<mav_disturbance_observer::KFPredictionObserverConfig>::CallbackType f;
        f = boost::bind(&KFPredictionObserver::DynConfigCallback, this, _1, _2);
        dyn_config_server_.setCallback(f);

        loadROSParams();

        state_.setZero();
        predicted_state_.setZero();
        received_first_odometry_ = false;
        initialized_ = true;

        ROS_INFO("Kalman Filter for Prediction Initialized!");

    }

    void KFPredictionObserver::loadROSParams()
    {
         double P0_position, P0_velocity;
        if (!observer_nh_.getParam("P0_position", P0_position)) {
            ROS_ERROR("P0_position in KF is not loaded from ros parameter server");
            abort();
        }

        if (!observer_nh_.getParam("P0_velocity", P0_velocity)) {
            ROS_ERROR("P0_velocity in KF is not loaded from ros parameter server");
            abort();
        }

        if (!private_nh_.getParam("sampling_time", sampling_time_)) {
            ROS_ERROR("sampling_time in KF is not loaded from ros parameter server");
            abort();
        }

        Eigen::MatrixXd F_continous_time(kStateSize, kStateSize);
        F_continous_time.setZero();
        F_continous_time.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3);

        F_ = (sampling_time_ * F_continous_time).exp().sparseView();

        // First 9x9 (=measurement size) block is identity, rest is zero.
        H_.reserve(kMeasurementSize);
        for (int i = 0; i < kMeasurementSize; ++i) {
            H_.insert(i, i) = 1.0;
        }

        for (int i = 0; i < 3; i++) {
            initial_state_covariance_(i) = P0_position;
            initial_state_covariance_(i + 3) = P0_velocity;
        }
        state_covariance_ = initial_state_covariance_.asDiagonal();
        ROS_INFO_STREAM("state_covariance_: \n" << state_covariance_);
        
        F_.makeCompressed();
    }

    void KFPredictionObserver::DynConfigCallback(mav_disturbance_observer::KFPredictionObserverConfig &config, uint32_t level)
    {
        for (size_t i = 0; i < 3; i++) {
            process_noise_covariance_(i) = config.q_position;
            process_noise_covariance_(i + 3) = config.q_velocity;

            measurement_covariance_(i) = config.r_position;
            measurement_covariance_(i + 3) = config.r_velocity;
        }

        ROS_INFO("mav_prediction_observer:KF dynamic config is called successfully");
    }

    void KFPredictionObserver::feedPositionMeasurement(const Eigen::Vector3d& position)
    {
        this->measurements_(0) = position(0);
        this->measurements_(1) = position(1);
        this->measurements_(2) = position(2);
        // ROS_ERROR("feed position measurement: %f, %f, %f", position(0), position(1), position(2));
    }

    void KFPredictionObserver::feedVelocityMeasurement(const Eigen::Vector3d& velocity)
    {
        this->measurements_(3) = velocity(0);
        this->measurements_(4) = velocity(1);
        this->measurements_(5) = velocity(2);
        // ROS_ERROR("feed velocity measurement: %f, %f, %f", velocity(0), velocity(1), velocity(2));
    }

    void KFPredictionObserver::reset(const Eigen::Vector3d& initial_position,
                                  const Eigen::Vector3d& initial_velocity)
    {

        state_covariance_ = initial_state_covariance_.asDiagonal();

        state_.setZero();

        state_.segment(0, 3) = initial_position;
        state_.segment(3, 3) = initial_velocity;
        // ROS_ERROR("prediction reset position : %f, %f, %f", initial_position(0), initial_position(1), initial_position(2));
    }

    bool KFPredictionObserver::updateEstimator()
    {
        if (initialized_ == false)
            return false;

        ROS_INFO_ONCE("KF is updated for first time.");
        static ros::Time t_previous = ros::Time::now();
        static bool do_once = true;
        double dt;

        if (do_once) {
            dt = 0.01;
            do_once = false;
        } else {
            ros::Time t0 = ros::Time::now();
            dt = (t0 - t_previous).toSec();
            t_previous = t0;
        }

        //check that dt is not so different from 0.01
        if (dt > 0.015) {
            dt = 0.015;
        }

        if (dt < 0.005) {
            dt = 0.005;
        }

        state_covariance_ = F_ * state_covariance_ * F_.transpose();
        state_covariance_.diagonal() += process_noise_covariance_;
        //predict state
        systemDynamics(dt);
        // ROS_ERROR("prediction initial state : %f, %f, %f", state_(0), state_(1), state_(2));

        Eigen::Matrix<double, kMeasurementSize, kMeasurementSize> tmp = H_ * state_covariance_
            * H_.transpose() + measurement_covariance_.asDiagonal().toDenseMatrix();
        
        K_ = state_covariance_ * H_.transpose() * tmp.inverse();

        //Update with measurements
        state_ = predicted_state_ + K_ * (measurements_ - H_ * state_);
        
        //Update covariance
        state_covariance_ = (Eigen::Matrix<double, kStateSize, kStateSize>::Identity() - K_ * H_)
            * state_covariance_;

        //Limits on estimated_disturbances
        if (state_.allFinite() == false) {
            ROS_ERROR("The estimated state in KF Prediction Observer has a non-finite element");
            return false;
        }

        if (observer_state_pub_.getNumSubscribers() > 0) {
            // ROS_ERROR("begin to generate prediction msg");
            mav_disturbance_observer::PredictionArrayPtr msg(new mav_disturbance_observer::PredictionArray);
            msg->header.stamp = ros::Time::now();
            // Eigen::Matrix<double, kStateSize, 1> new_state, old_state;
            old_state_ = state_;

            Eigen::Matrix<double, kStateSize, kStateSize> cur_state_covariance;
            cur_state_covariance = state_covariance_;

            for (int i = 0; i < TN+1; i++) {
                mav_disturbance_observer::PredictionPoint point_msg;
                propagate(Ts);
                cur_state_covariance = F_ * cur_state_covariance * F_.transpose();
                for (int j = 0; j < 3; j++)
                {
                    point_msg.position[j] = old_state_(j);
                    point_msg.velocity[j] = old_state_(j + 3);
                    // point_msg.pos_var[j] = cur_state_covariance(j, j);
                }

                //Eigen::Matrix<double, kStateSize, kStateSize> D = cur_state_covariance.pseudoEigenvalueMatrix();
                Eigen::EigenSolver<Eigen::Matrix<double, kStateSize, kStateSize>> es(cur_state_covariance, false);
                point_msg.pos_var = std::sqrt(es.eigenvalues().real().maxCoeff());
                msg->points.push_back(point_msg);
                old_state_ = new_state_;
            }
            observer_state_pub_.publish(msg);
        }
        return true;
    }

    void KFPredictionObserver::systemDynamics(double dt)
    {
        Eigen::Vector3d old_position = state_.segment(0, 3);
        Eigen::Vector3d old_velocity = state_.segment(3, 3);

        // const Eigen::Vector3d acceleration = rotation_matrix_ * thrust + Eigen::Vector3d(0, 0, -kGravity)
        //     + this->drag_coefficients_matrix_ * old_velocity + old_external_forces;
        Eigen::Vector3d acceleration;
        acceleration.setZero();
        const Eigen::Vector3d new_velocity = old_velocity + acceleration * dt;

        const Eigen::Vector3d new_position = old_position + old_velocity * dt
            + 0.5 * acceleration * dt * dt;

        //Update the state vector

        predicted_state_.segment(0, 3) = new_position;
        predicted_state_.segment(3, 3) = new_velocity;

    }

    void KFPredictionObserver::propagate(double dt)
    {
        Eigen::Vector3d old_position = old_state_.segment(0, 3);
        Eigen::Vector3d old_velocity = old_state_.segment(3, 3);

        // const Eigen::Vector3d acceleration = rotation_matrix_ * thrust + Eigen::Vector3d(0, 0, -kGravity)
        //     + this->drag_coefficients_matrix_ * old_velocity + old_external_forces;
        Eigen::Vector3d acceleration;
        acceleration.setZero();
        const Eigen::Vector3d new_velocity = old_velocity + acceleration * dt;

        const Eigen::Vector3d new_position = old_position + old_velocity * dt
            + 0.5 * acceleration * dt * dt;

        //Update the state vector

        new_state_.segment(0, 3) = new_position;
        new_state_.segment(3, 3) = new_velocity;
    }

    void KFPredictionObserver::getEstimatedState(Eigen::VectorXd* estimated_state) const
    {
        assert(estimated_state);
        assert(initialized_);

        estimated_state->resize(kStateSize);
        *estimated_state = this->state_;
    }

    KFPredictionObserver::~KFPredictionObserver()
    {
    }

    void KFPredictionObserver::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
    {
         static mav_msgs::EigenOdometry odometry;
         mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry);
         static mav_msgs::EigenOdometry previous_odometry = odometry_;
        if (!received_first_odometry_)
        {
            reset(odometry.position_W, odometry.getVelocityWorld());
            received_first_odometry_ = true;
            return;
        }
        if (odometry.position_W.allFinite() == false) {
            odometry_.position_W = previous_odometry.position_W;
            ROS_WARN("Odometry.position has a non finite element");
        } else {
            odometry_.position_W = odometry.position_W;
            previous_odometry.position_W = odometry.position_W;
        }

        if (odometry.velocity_B.allFinite() == false) {
            odometry_.velocity_B = previous_odometry.velocity_B;
            ROS_WARN("Odometry.velocity has a non finite element");
        } else {
            odometry_.velocity_B = odometry.velocity_B;
            previous_odometry.velocity_B = odometry.velocity_B;
        }

        if (odometry.angular_velocity_B.allFinite() == false) {
            odometry_.angular_velocity_B = previous_odometry.angular_velocity_B;
            ROS_WARN("Odometry.angular_velocity has a non finite element");
        } else {
            odometry_.angular_velocity_B = odometry.angular_velocity_B;
            previous_odometry.angular_velocity_B = odometry.angular_velocity_B;
        }

        odometry_.orientation_W_B = odometry.orientation_W_B;
        odometry_.timestamp_ns = odometry.timestamp_ns;
        previous_odometry.orientation_W_B = odometry.orientation_W_B;
        
        feedPositionMeasurement(odometry_.position_W);
        feedVelocityMeasurement(odometry_.getVelocityWorld());
        bool prediction_update_successful = updateEstimator();
        if (!prediction_update_successful) {
            ROS_ERROR("prediction update failed");
            reset(odometry_.position_W, odometry_.getVelocityWorld());
        }
        // ROS_ERROR("prediction published !");
    }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PredictionNode");

  ros::NodeHandle nh, private_nh("~");

  mav_prediction::KFPredictionObserver prediction_observer_(nh, private_nh);

  ros::spin();

  return 0;
}
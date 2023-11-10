// -*- mode: c++ -*-

#pragma once
#include <beetle/model/beetle_robot_model.h>
#include <gimbalrotor/control/gimbalrotor_controller.h>
#include <beetle/sensor/imu.h>

namespace aerial_robot_control
{
  class BeetleController: public GimbalrotorController
  {
  public:
    BeetleController();
    ~BeetleController(){
      wrench_estimate_thread_.interrupt();
      wrench_estimate_thread_.join();
    }

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate
                    ) override;
  private:
    boost::shared_ptr<BeetleRobotModel> beetle_robot_model_;
    
    ros::Publisher estimate_external_wrench_pub_;
    ros::Publisher external_wrench_compensation_pub_;

    /* external wrench */
    boost::thread wrench_estimate_thread_;
    Eigen::VectorXd init_sum_momentum_;
    Eigen::VectorXd est_external_wrench_;
    Eigen::MatrixXd momentum_observer_matrix_;
    Eigen::VectorXd integrate_term_;
    double prev_est_wrench_timestamp_;

    /*low-pass filter*/
    IirFilter lpf_est_external_wrench_;
    double sample_freq_;
    double cutoff_freq_;
    bool lpf_init_flag_;
    Eigen::VectorXd filterd_est_external_wrench_;

    /* external wrench compensation */
    Eigen::VectorXd external_wrench_upper_limit_;
    Eigen::VectorXd external_wrench_lower_limit_;

    int pre_module_state_;
    double ErrI_X_;
    double ErrI_Y_;
    double ErrI_Z_;
    double ErrI_Yaw_;
    
    void controlCore() override;
    void externalWrenchEstimate();

  protected:
    void rosParamInit() override; 
  };
};

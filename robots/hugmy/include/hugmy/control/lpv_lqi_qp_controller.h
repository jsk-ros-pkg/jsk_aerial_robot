#pragma once
#include <aerial_robot_control/control/under_actuated_lqi_controller.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>


namespace aerial_robot_control {

class LpvLqiQpController : public UnderActuatedLQIController {
public:
  LpvLqiQpController();

  void initialize(ros::NodeHandle nh,
                  ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                  double ctrl_loop_rate) override;

protected:
  bool optimalGain() override;
  void controlCore() override;
  void activate() override;

  void thetaCallback(const std_msgs::Float32ConstPtr& msg);
  void zPerchingAltitudeCallback(const std_msgs::Float32ConstPtr& msg);
  void descentTriggerCallback(const std_msgs::EmptyConstPtr& msg);
  void applyThetaToModel(float theta);

  // QP
  Eigen::VectorXd solveAllocationQP(const Eigen::MatrixXd& Q, const Eigen::Vector4d& b);
  double getSchedTheta() const;
  // void loadLpvTable();
  void loadOneAxisTable(ros::NodeHandle& nh, const std::string& key,
                        std::vector<std::vector<Eigen::Vector3d>>& out);
  // void applyInterpolatedGains(double th);
  void loadQpParams();
  double zPerchingControl();

  Eigen::MatrixXd q_mat_;
  std::vector<double> theta_meas_;
  ros::Publisher joint_reflect_theta_pub_;
  ros::Subscriber theta_sub_;
  ros::Subscriber z_perching_altitude_sub_;
  bool use_theta_allocation_{true};   // θ反映
  double z_perching_altitude_{0.0};

  ros::Time started_;
  double gravity_scale_{0.0};

  double h_ = 0.0;
  ros::Subscriber descent_trigger_sub_;
  bool arm_theta_enabled_ = false;     // 降下フェーズのみ true
  double vz_descend_thresh_ = 0.05;    // [m/s]
  double descend_margin_ = 0.05;       // [m]
  float theta_cmd_hold_ = 0.0f; 

  // LPV
  // std::vector<double> theta_grid_;
  // std::vector<std::vector<Eigen::Vector3d>> table_roll_, table_pitch_, table_yaw_, table_z_;

  // QP
  Eigen::Vector4d w_;
  double rho_{0.2}, lambda_{0.01};
  int max_iter_{20};
  Eigen::VectorXd f_prev_, f_min_, f_max_;

  bool xmlToDouble(XmlRpc::XmlRpcValue& v, double& out) {
    switch (v.getType()) {
      case XmlRpc::XmlRpcValue::TypeDouble:
        out = static_cast<double>(v); return true;
      case XmlRpc::XmlRpcValue::TypeInt:
        out = static_cast<int>(v);    return true;
      case XmlRpc::XmlRpcValue::TypeBoolean:
        out = static_cast<bool>(v) ? 1.0 : 0.0; return true;
      default:
        return false;
    }
  }
};

} // namespace aerial_robot_control
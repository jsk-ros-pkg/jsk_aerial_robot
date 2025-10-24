#pragma once
#include <aerial_robot_control/control/under_actuated_lqi_controller.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

namespace aerial_robot_control
{

/**
 * @brief LQIを継承し、Fzをソフト制約・回転等式制約で配分し、
 *        ヌル空間で f_ref への近さと上限/非負制約を満たす thrust を求める
 */
class LQINullController : public UnderActuatedLQIController
{
public:
  LQINullController();
  virtual ~LQINullController() {}

  void initialize(ros::NodeHandle nh,
                  ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                  double ctrl_loop_rate) override;

  void controlCore() override;
  void sendCmd() override;

protected:
  Eigen::MatrixXd buildQ();  // [Fz; Tx; Ty; Tz] with mass/inertia scaling
  Eigen::VectorXd allocSoftFz(const Eigen::MatrixXd& Qeq,       // 3xN
                              const Eigen::VectorXd& beq,       // = 0
                              const Eigen::RowVectorXd& qz,     // 1xN
                              double az_des,
                              const Eigen::VectorXd& f_ref,
                              const Eigen::VectorXd& Wf_diag,
                              const Eigen::VectorXd& f_max,
                              double w_fz,
                              double eps_reg);


  static inline void clip(Eigen::VectorXd& f, const Eigen::VectorXd& fmax)
    {
        for (int i = 0; i < f.size(); ++i)
        {
            if (f(i) < 0.0)     f(i) = 0.0;
            if (f(i) > fmax(i)) f(i) = fmax(i);
        }
    }

  static inline Eigen::MatrixXd nullspace(const Eigen::MatrixXd& A)
  {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
    const auto S = svd.singularValues();
    const double eps = std::numeric_limits<double>::epsilon();
    const double tol = std::max(A.rows(), A.cols()) * eps * S(0);
    const int rank = (S.array() > tol).count();
    return svd.matrixV().rightCols(A.cols() - rank);
  }

  void zeroDragTorque(std::vector<float>& thrust);

  // ====== ROS param ======
  void rosParamInitSoft();
  double w_fz_ = 1.0;
  double eps_reg_ = 1e-6;
  double az_des_bias_ = 0.0;       // Fz目標のバイアス

  // ====== 参照値/制約 ======
  Eigen::VectorXd f_ref_;    // 望ましい推力（各モータ）
  Eigen::VectorXd Wf_diag_;  // f_ref に寄せる重み（各モータ）
  Eigen::VectorXd f_max_;    // 各モータの上限

  // ====== デバッグ出力 ======
  ros::Publisher nullspace_dim_pub_;
  ros::Publisher svd_singvals_pub_;
  ros::Publisher thrust_pub_;

  // 最新の Q（SVDデバッグ用）
  Eigen::MatrixXd last_Q_;
};

} // namespace aerial_robot_control

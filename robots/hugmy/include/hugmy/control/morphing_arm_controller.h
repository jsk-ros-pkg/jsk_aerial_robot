#pragma once

#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <cmath>
#include <array>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>
#include <aerial_robot_model/model/aerial_robot_model.h>


namespace aerial_robot_control
{
    class MorphingController: public PoseLinearController
    {
    public:
        MorphingController();
        virtual ~MorphingController() = default;
        void initialize(ros::NodeHandle nh,
                        ros::NodeHandle nhp,
                        boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                        boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                        boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                        double ctrl_loop_rate) override;

        void controlCore() override;
        void sendCmd() override;
        void reset() override;

    protected:
    /* ---------- publishers ---------- */
    ros::Publisher rpy_gain_pub_;
    ros::Publisher flight_cmd_pub_;
    ros::Publisher torque_allocation_matrix_inv_pub_;
    ros::Publisher debug_joint_state_pub_;
    ros::Publisher joint_state_pub_;
    ros::Publisher thrust_pub_;

    ros::Publisher nullspace_dim_pub_;
    ros::Publisher svd_singvals_pub_;
    ros::Timer ns_timer_;
    Eigen::MatrixXd last_Q_;
      
    /* ---------- subscribers (new) ---------- */
    ros::Subscriber theta_sub_;       // /arm/theta : 各ロータ傾きθ[rad]
    ros::Subscriber thrust_ref_sub_;  // /arm/thrust_ref : 推力の参照[N]
    ros::Subscriber joint_cmd_sub_;
    // void jointCmdCb(const sensor_msgs::JointState::ConstPtr &msg);

    ros::Subscriber js_sub = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1,
                                                                    [this](const sensor_msgs::JointState::ConstPtr& msg){
                                                                      KDL::JntArray q = robot_model_->getJointPositions();
                                                                      const auto& idx = robot_model_->getJointIndexMap();
                                                                      for(size_t k=0;k<msg->name.size();++k){
                                                                        auto it = idx.find(msg->name[k]);
                                                                        if(it != idx.end() && k < msg->position.size()){
                                                                          q(it->second) = msg->position[k];
                                                                        }
                                                                      }
                                                                      robot_model_->updateRobotModel(q);
                                                                    });

    /* ---------- parameters ---------- */
    std::vector<double> target_base_thrust_;  // モータごとのベーススラスト
    double target_pitch_{0.0};                // 目標ピッチ角 [rad]
    double target_roll_{0.0};
    double candidate_yaw_term_{0.0};
    bool hovering_approximate_{false};        // 小角近似モード
    bool use_theta_allocation_{true};
    double torque_allocation_matrix_inv_pub_interval_{0.05};
    double torque_allocation_matrix_inv_pub_stamp_{0.0};
    double az_des_bias_{0.0};  // 必要ならZ加速度にバイアス

    /* ---------- state for allocation ---------- */
    std::vector<double> theta_meas_;   // 各ロータの傾き θ[rad]
    Eigen::VectorXd f_ref_;            // 推力参照 [N]
    Eigen::VectorXd f_max_;            // 推力上限 [N]
    Eigen::VectorXd Wf_diag_;          // 参照追従の重み diag

    /* ---------- cached matrices ---------- */
    Eigen::MatrixXd q_mat_;
    Eigen::MatrixXd q_mat_inv_;

    /* ---------- msg cache ---------- */
    spinal::RollPitchYawTerms pid_msg_;

    virtual void rosParamInit();
    virtual void setAttitudeGains();
    virtual void sendFourAxisCommand();
    void sendTorqueAllocationMatrixInv();
    void applyThetaToModel(float theta);
    void thetaCallback(const std_msgs::Float32ConstPtr& msg);
    void thrustRefCallback(const std_msgs::Float32ConstPtr& msg);
    void publishCurrentJointStatus();
    void zeroDragTorque(std::vector<double>& thrust);

    void nullspaceTimerCb(const ros::TimerEvent&);
      Eigen::VectorXd allocSoftFz(const Eigen::MatrixXd& Qeq,          // 3xN : [Tx;Ty;Tz]
                                  const Eigen::Vector3d& beq,          // = 0
                                  const Eigen::RowVectorXd& qz,
                                  double az_des,
                                  const Eigen::VectorXd& f_ref,
                                  const Eigen::VectorXd& Wf_diag,
                                  const Eigen::VectorXd& f_max,
                                  double w_fz,
                                  double eps_reg);

    Eigen::MatrixXd buildQWithTheta();
    static inline Eigen::Vector3d rotateArm(const Eigen::Vector3d& v, double theta)
    {
        Eigen::AngleAxisd R(theta, Eigen::Vector3d::UnitY());
        return (R * v).normalized();
    }


    static inline void computeNullspaceSVD(const Eigen::MatrixXd& Q,
                                           Eigen::MatrixXd& N,               // (out) ヌル空間の列基底
                                           int& rank,                        // (out) 数値ランク
                                           double& tol,                      // (out) しきい値
                                           Eigen::VectorXd* singvals_out=nullptr) // (opt) 特異値を呼び出し側に返す
      {
        // 完全SVD（U, S, V）
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Q, Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Eigen::VectorXd& S = svd.singularValues(); // 降順
        if (singvals_out) *singvals_out = S;
        
        // 数値ランクの閾値（一般的な推奨式）
        const double eps = std::numeric_limits<double>::epsilon();
        tol = std::max(Q.rows(), Q.cols()) * eps * S(0); // S(0)=最大特異値
        
        // ランク＝しきい値より大きい特異値の数
        rank = 0;
        for (int i = 0; i < S.size(); ++i) if (S(i) > tol) ++rank;
        
        const int nullity = Q.cols() - rank; // ヌル空間次元
        if (nullity > 0) {
          // V の「小さい特異値に対応する列」がヌル空間基底
          // Sは降順なので、右端の列群がヌル空間
          const Eigen::MatrixXd V = svd.matrixV();
          N = V.rightCols(nullity); // Q * N ≈ 0
        } else {
          N.resize(Q.cols(), 0); // 空
        }
      }
      
    static inline Eigen::MatrixXd nullspace(const Eigen::MatrixXd& Q, double tol = 1e-7)
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Q, Eigen::ComputeFullU | Eigen::ComputeFullV);
        const auto& S = svd.singularValues();
        const auto& V = svd.matrixV();
        int m = Q.cols(), r = 0;
        for (int i = 0; i < S.size(); ++i) if (S(i) > tol) r++;
        int k = m - r;
        if (k <= 0) return Eigen::MatrixXd::Zero(m, 0);
        return V.rightCols(k);
    }

    static inline void clip(Eigen::VectorXd& f, const Eigen::VectorXd& fmax)
    {
        for (int i = 0; i < f.size(); ++i)
        {
            if (f(i) < 0.0)     f(i) = 0.0;
            if (f(i) > fmax(i)) f(i) = fmax(i);
        }
    }
    };
}

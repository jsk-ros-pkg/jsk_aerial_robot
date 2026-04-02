#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

#include "mpa_acados_controller/narx_acados_solver.h"
#include <cmath>
#include <string>
#include <vector>

class MpaAcadosNode {
public:
  MpaAcadosNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  : nh_(nh), pnh_(pnh)
  {
    // params
    pnh_.param<std::string>("joint_topic", joint_topic_, "/kinikun1/joint_states");
    pnh_.param<std::string>("joint_name", joint_name_, "arm1_joint");
    pnh_.param<std::string>("target_topic", target_topic_, "/target_angle");
    pnh_.param<std::string>("pub_topic_p12", pub_topic_p12_, "/p1p2_cmd");
    pnh_.param<std::string>("pub_topic_counts", pub_topic_counts_, "/mpa_cmd");

    pnh_.param<double>("pmax", pmax_, 0.7);
    pnh_.param<double>("raw_counts_per_MPa", raw_counts_per_MPa_, 4096.0/0.9);
    pnh_.param<double>("dz0", dz0_, 0.0);
    pnh_.param<double>("ctrl_rate_hz", ctrl_rate_hz_, 100.0);

    joint_sub_ = nh_.subscribe(joint_topic_, 1, &MpaAcadosNode::jointCb, this);
    target_sub_ = nh_.subscribe(target_topic_, 1, &MpaAcadosNode::targetCb, this);

    pub_p12_ = nh_.advertise<geometry_msgs::Vector3>(pub_topic_p12_, 10);
    pub_counts_ = nh_.advertise<geometry_msgs::Vector3>(pub_topic_counts_, 10);

    // acados solver 作成
    solver_.reset(new mpa_acados_controller::NarxAcadosSolver());
    if (!solver_->init(pmax_)) {
      ROS_ERROR("acados solver init failed: narx_disc_acados_create() returned nonzero");
    } else {
      ROS_INFO("acados solver init OK (pmax=%.3f)", pmax_);
    }

    timer_ = nh_.createTimer(ros::Duration(1.0/ctrl_rate_hz_), &MpaAcadosNode::onTimer, this);
  }

private:
  void jointCb(const sensor_msgs::JointState::ConstPtr &msg) {
    for (size_t i=0; i<msg->name.size(); ++i) {
      if (msg->name[i] == joint_name_) {
        current_theta_ = msg->position[i];  // rad
        has_theta_ = true;
        break;
      }
    }
  }

  void targetCb(const std_msgs::Float32::ConstPtr &msg) {
    // python版と同じく deg input を想定
    target_theta_ = msg->data * M_PI / 180.0;
    has_target_ = true;
  }

  // x0 を(雑でも)作る: いまのθ, ps, pd を最新スライスにして、残りはコピー
  std::vector<double> buildX0() {
    int nx = solver_->nx();
    // 1スライス6変数構成を知っている前提 (theta, ps, pd, dps, dpd, dz)
    const int S = 6;
    int L = nx / S;

    std::vector<double> x0(nx, 0.0);
    // 最新スライス
    double dps = 0.0;
    double dpd = 0.0;
    double dz  = 0.0;   // 今回は静的高さをそのまま

    x0[0] = current_theta_;  // theta
    x0[1] = ps_cmd_;         // ps
    x0[2] = pd_cmd_;         // pd
    x0[3] = dps;
    x0[4] = dpd;
    x0[5] = dz;

    // 残りのスライスも同じ値で埋める
    for (int j=1; j<L; ++j) {
      x0[j*S + 0] = current_theta_;
      x0[j*S + 1] = ps_cmd_;
      x0[j*S + 2] = pd_cmd_;
      x0[j*S + 3] = 0.0;
      x0[j*S + 4] = 0.0;
      x0[j*S + 5] = dz;
    }
    return x0;
  }

  void onTimer(const ros::TimerEvent &) {
    if (!solver_->isOk()) return;
    if (!has_theta_ || !has_target_) return;

    // 1) x0 とパラメータを acados に入れる
    auto x0 = buildX0();
    solver_->setX0(x0);
    solver_->setParams(target_theta_, dz0_);

    // 2) 解く
    int status = solver_->solve();
    if (status) {
      ROS_WARN_THROTTLE(1.0, "acados solve failed: %d", status);
      return;
    }

    // 3) u0 を取り出して ps, pd に変換 (python版と同じ)
    double a=0.0, b=0.0;
    if (!solver_->getU0(a, b)) return;

    // clamp 0..1
    if (a < 0.0) a = 0.0; if (a > 1.0) a = 1.0;
    if (b < 0.0) b = 0.0; if (b > 1.0) b = 1.0;

    double ps = pmax_ * (a + b);
    double pd = pmax_ * (a - b);

    // ボックス可行化 (p1,p2 >=0, <=pmax)
    double p1 = 0.5*(ps + pd);
    double p2 = 0.5*(ps - pd);
    p1 = std::min(std::max(p1, 0.0), pmax_);
    p2 = std::min(std::max(p2, 0.0), pmax_);

    // 内部状態も更新しておく
    ps_cmd_ = p1 + p2;
    pd_cmd_ = p1 - p2;

    // 4) publish
    geometry_msgs::Vector3 v;
    v.x = p1;
    v.y = p2;
    v.z = 0.0;
    pub_p12_.publish(v);

    geometry_msgs::Vector3 v2;
    v2.x = std::round(p1 * raw_counts_per_MPa_);
    v2.y = std::round(p2 * raw_counts_per_MPa_);
    v2.z = 0.0;
    pub_counts_.publish(v2);
  }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber joint_sub_;
  ros::Subscriber target_sub_;
  ros::Publisher  pub_p12_;
  ros::Publisher  pub_counts_;
  ros::Timer timer_;

  std::unique_ptr<mpa_acados_controller::NarxAcadosSolver> solver_;

  std::string joint_topic_;
  std::string joint_name_;
  std::string target_topic_;
  std::string pub_topic_p12_;
  std::string pub_topic_counts_;

  double pmax_ = 0.7;
  double raw_counts_per_MPa_ = 4096.0/0.9;
  double dz0_ = 0.0;
  double ctrl_rate_hz_ = 100.0;

  bool has_theta_ = false;
  bool has_target_ = false;
  double current_theta_ = 0.0;
  double target_theta_ = 0.0;

  // 内部で保持してる現在のΣ,Δ
  double ps_cmd_ = 0.3;   // Σ
  double pd_cmd_ = 0.0;   // Δ
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpa_acados_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  MpaAcadosNode node(nh, pnh);
  ros::spin();
  return 0;
}

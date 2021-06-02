#include<hydrus/torsion_mode_calculator.h>

TorsionModeCalculator::TorsionModeCalculator(ros::NodeHandle nh, ros::NodeHandle nhp)
:nh_(nh), nhp_(nhp), tfListener_(tfBuffer_)
{
  // ros param init
  // urdf model of hydrus
  std::string control_model_urdf;
  nhp_.getParam("robot_description_control", control_model_urdf);
  nhp_.param<std::string>("robot_ns", robot_ns_, "hydrus");
  nhp_.param<int>("rotor_num", rotor_num_, 6);
  nhp_.param<int>("torsion_num", torsion_num_, rotor_num_-1);
  nhp_.param<double>("torsion_constant", torsion_constant_, 1.0);
  nhp_.param<double>("eigen_eps", eigen_eps_, 1e-5);
  nhp_.param<int>("mode_num", mode_num_, rotor_num_-4);

  torsions_.resize(torsion_num_);
  torsions_d_.resize(torsion_num_);
  joints_.resize(torsion_num_);
  joints_d_.resize(torsion_num_);

  // kdl chain dyn param
  KDL::Tree kdl_tree;
  urdf::Model urdf_model;
  if (!urdf_model.initParamWithNodeHandle("robot_description_control", nhp_)) {
    ROS_ERROR("Failed to parse urdf robot model");
  }
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
    ROS_ERROR("Failed to construct kdl tree");
  }
  kdl_tree.getChain("root", "link"+std::to_string(rotor_num_), kdl_chain_);
  KDL::Vector grav(0.0,0.0,-9.81);
  dyn_param_ = new KDL::ChainDynParam(kdl_chain_, grav);

  jnt_q_.resize(torsion_num_*2);
  torsion_dof_update_order_.resize(torsion_num_);
  joint_dof_update_order_.resize(torsion_num_);
  for (int i = 0; i < torsion_num_; i++)
  {
    torsion_dof_update_order_.at(i) = 2*i;
    joint_dof_update_order_.at(i) = 2*i+1;
  }

  // ros subscribers
  torsion_joint_sub_ = nh.subscribe<sensor_msgs::JointState>("link_torsion/joint_states", 1, &TorsionModeCalculator::torsionJointCallback, this);
  joint_sub_ = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, &TorsionModeCalculator::jointsCallback, this);

  // ros publishers
  eigen_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("torsion_eigens", 1);
  mode_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("torsion_mode_matrix", 1);
  K_mode_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("K_mode", 1);

  // dynamic reconfigure
  reconf_server_ = new dynamic_reconfigure::Server<hydrus::torsion_modeConfig>(nhp_);
  reconf_func_ = boost::bind(&TorsionModeCalculator::cfgCallback, this, _1, _2);
  reconf_server_->setCallback(reconf_func_);
}

TorsionModeCalculator::~TorsionModeCalculator() 
{
  delete dyn_param_;
}

void TorsionModeCalculator::cfgCallback(hydrus::torsion_modeConfig &config, uint32_t level) 
{
  if(config.torsion_mode_flag)
  {
    printf("Torsion Mode Param:");
    switch(level)
    {
      case TORSION_CONSTANT:
        torsion_constant_ = config.torsion_constant;
        printf("change parameter of torsion constant: %f\n", torsion_constant_);
        break;
      default :
        printf("\n");
        break;
    }
  }
}

void TorsionModeCalculator::torsionJointCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  if (torsion_num_<=0) return;
  copy(msg->position.begin(), msg->position.end(), torsions_.begin());
  copy(msg->velocity.begin(), msg->velocity.end(), torsions_d_.begin()); 
}

void TorsionModeCalculator::jointsCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  if (torsion_num_<=0) return;
  copy(msg->position.begin(), msg->position.end(), joints_.begin());
  copy(msg->velocity.begin(), msg->velocity.end(), joints_d_.begin());
}

void TorsionModeCalculator::calculate()
{
  for (int i = 0; i < torsion_num_; i++)
  {
    jnt_q_(torsion_dof_update_order_.at(i)) = torsions_.at(i);
    jnt_q_(joint_dof_update_order_.at(i)) = joints_.at(i);
  }
  KDL::JntSpaceInertiaMatrix H(jnt_q_.rows());
  dyn_param_->JntToMass(jnt_q_, H);
  Eigen::MatrixXd H_inv = H.data.inverse();

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(torsion_num_, torsion_num_);
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(jnt_q_.rows(), jnt_q_.rows());
  Eigen::MatrixXd H_torsion_inv = Eigen::MatrixXd::Zero(torsion_num_, torsion_num_);

  for (int i = 0; i < torsion_num_; ++i) {
    K(torsion_dof_update_order_.at(i), torsion_dof_update_order_.at(i))  =-1;
  }
  K = torsion_constant_ * K;
  K = H_inv * K;
  for (int i = 0; i < torsion_num_; ++i) {
    for (int j = 0; j < torsion_num_; ++j) {
      A(i, j) = K(torsion_dof_update_order_.at(i), torsion_dof_update_order_.at(j));
      H_torsion_inv(i,j) = H_inv(torsion_dof_update_order_.at(i), torsion_dof_update_order_.at(j));
    }
  }
  ROS_DEBUG_STREAM("A: " << std::endl << A);

  // vibration mode regression
  Eigen::EigenSolver<Eigen::MatrixXd> es(A);
  ROS_DEBUG_STREAM("A eigen values: "<<std::endl << es.eigenvalues().real().transpose());
  ROS_DEBUG_STREAM("A eigen vectors: "<<std::endl << es.eigenvectors().real());

  Eigen::MatrixXd P = es.eigenvectors().real();
  Eigen::MatrixXd P_inv = P.inverse();
  Eigen::MatrixXd A_full_diag = P_inv * A * P;
  ROS_DEBUG_STREAM("A full diag: "<<std::endl << A_full_diag);

  // select largest modes
  Eigen::MatrixXd A_selected = Eigen::MatrixXd::Zero(mode_num_, mode_num_);
  Eigen::MatrixXd torsion_mode_selected  = Eigen::MatrixXd::Zero(mode_num_, torsion_num_);
  std::vector<std::pair<double, int>> sorted_mode_eigen_idx_pair;
  for (int i = 0; i < es.eigenvalues().size(); ++i) {
    double eig_v = es.eigenvalues()(i).real();
    if (eig_v < 0 && eig_v < -eigen_eps_) {
      sorted_mode_eigen_idx_pair.push_back(std::make_pair(eig_v, i));
    }
  }
  std::sort(sorted_mode_eigen_idx_pair.begin(), sorted_mode_eigen_idx_pair.end(), std::greater<std::pair<double, int>>());

  for (int i = 0; i < mode_num_; ++i) {
    for (int j = 0; j < mode_num_; ++j) {
      double val = A_full_diag(sorted_mode_eigen_idx_pair[i].second, sorted_mode_eigen_idx_pair[j].second);
      if (abs(val)>eigen_eps_) A_selected(i,j) = val;
    }
  }
  for (int i = 0; i < mode_num_; ++i) {
    for (int j = 0; j < torsion_num_; ++j) {
      torsion_mode_selected(i, j) = P(j, sorted_mode_eigen_idx_pair[i].second);
    }
  }
  ROS_DEBUG_STREAM("A mode selected: "<<std::endl << A_selected);

  Eigen::MatrixXd J_torsion = Eigen::MatrixXd::Zero(torsion_num_, rotor_num_);
  for (int i = 0; i < torsion_num_; ++i) {
    for (int j = 0; j < rotor_num_; ++j) {
      geometry_msgs::TransformStamped transformStamped;
      try{
        transformStamped = tfBuffer_.lookupTransform(robot_ns_+"/thrust"+std::to_string(j+1), robot_ns_+"/link"+std::to_string(i+2),
            ros::Time(0));
        tf2::Quaternion quat_tf;
        tf2::Vector3 trans_tf;
        tf2::fromMsg(transformStamped.transform.rotation, quat_tf);
        tf2::fromMsg(transformStamped.transform.translation, trans_tf);
        double moment_arm_length = tf2::tf2Cross(trans_tf, tf2::quatRotate(quat_tf, tf2::Vector3(0,0,1)) ).getX();
        double moment_arm_length_sgn = -1;
        moment_arm_length_sgn = i>j ? -1 : 1;
        J_torsion(i,j) = moment_arm_length_sgn * moment_arm_length;
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1, "%s",ex.what());
        return;
      }
    }
  }
  Eigen::MatrixXd torsion_B_matrix(mode_num_, rotor_num_);
  torsion_B_matrix = torsion_mode_selected * (H_torsion_inv * J_torsion);

  // publish results
  K_mode_pub_.publish(msg_utils::EigenMatrix2Float32MultiArray(torsion_B_matrix.transpose()));

  std::vector<double> torsion_eigens;
  for (int i = 0; i < mode_num_; ++i) {
    torsion_eigens.push_back(sorted_mode_eigen_idx_pair[i].first);
  }
  eigen_pub_.publish(msg_utils::Vector2Float32MultiArray(torsion_eigens));

  mode_pub_.publish(msg_utils::EigenMatrix2Float32MultiArray(torsion_mode_selected));
}

int main (int argc, char* argv[])
{
  ros::init(argc, argv, "torsion_mode_calculator");
  ros::NodeHandle nh;
  ros::NodeHandle nhp = ros::NodeHandle("~");

  TorsionModeCalculator mode_calculator(nh, nhp);

  int rate;
  nhp.param<int>("rate", rate, 10);
  ros::Rate r(rate);
  while (ros::ok())
  {
    mode_calculator.calculate();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

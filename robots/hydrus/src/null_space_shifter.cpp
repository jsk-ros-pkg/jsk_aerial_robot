#include<hydrus/null_space_shifter.h>

// for nlopt
namespace
{
  double maximizeTorsionDistanceFactor(const std::vector<double> &x, std::vector<double> &grad, void *data) 
  {
    NullSpaceShifter* shifter_ptr = reinterpret_cast<NullSpaceShifter*>(data);
    double torsion_factor = 1.0;

    int nlopt_tmp_index = shifter_ptr->getNloptTmpIndex();
    Eigen::MatrixXd K_gain = shifter_ptr->getKGain();
    Eigen::MatrixXd K_mode = shifter_ptr->getKMode();
    Eigen::MatrixXd B_eom_kernel = shifter_ptr->getBEomKernel();
    std::vector<double> torsion_eigens = shifter_ptr->getTorsionEigens();
    double nlopt_position_penalty = shifter_ptr->getNloptPositionPenalty();
    double nlopt_alpha = shifter_ptr->getNloptAlpha();

    Eigen::VectorXd gain = K_gain.col(nlopt_tmp_index);
    int motor_num = K_gain.rows();
    gain.normalize();

    for (int i = 0; i < B_eom_kernel.cols(); ++i) {
      gain = gain + x[i]*B_eom_kernel.col(i);
    }

    for (int i = 0; i < K_mode.cols(); ++i) {
      double K_corr = gain.dot( K_mode.col(i) ) / gain.norm() / K_mode.col(i).norm();
      torsion_factor -= nlopt_alpha * K_corr*K_corr * sqrt(torsion_eigens[0]/torsion_eigens[i]);
    }

    for (int i = 0; i < motor_num; ++i) {
      torsion_factor -= nlopt_position_penalty * std::pow( i-motor_num*0.5, 2 ) * std::abs(gain(i));
    }

    return torsion_factor;
  }
}

NullSpaceShifter::NullSpaceShifter(ros::NodeHandle nh, ros::NodeHandle nhp)
:nh_(nh), nhp_(nhp)
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle gain_shift_nh(control_nh, "gain_shift");

  gain_shift_nh.param<bool>("nlopt_use_global", is_nlopt_use_global_, false);
  gain_shift_nh.param<double>("nlopt_xtol_rel", nlopt_xtol_rel_, 1e-4);
  gain_shift_nh.param<int>("null_space_shift_max_eval", null_space_shift_max_eval_, 1000);
  gain_shift_nh.param<double>("null_space_shift_thresh", null_space_shift_thresh_, 0.7);
  gain_shift_nh.param<double>("null_space_shift_limit_ratio", null_space_shift_limit_ratio_, 0.7);
  gain_shift_nh.param<double>("null_space_shift_mix_limit", null_space_shift_mix_limit_, 0.7);
  gain_shift_nh.param<double>("nlopt_position_penalty", nlopt_position_penalty_, 0.01);
  gain_shift_nh.param<double>("nlopt_alpha", nlopt_alpha_, 0.1);

  // subscriber
  K_gain_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("K_gain_for_shift", 1, &NullSpaceShifter::kGainCallback, this);
  K_mode_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("K_mode", 1, &NullSpaceShifter::kModeCallback, this);
  B_eom_kernel_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("B_eom_kernel", 1, &NullSpaceShifter::bEOMKernelCallback, this);
  torsion_eigens_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("torsion_eigens", 1, &NullSpaceShifter::torsionEigensCallback, this);
  // publisher
  kernel_mix_ratio_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("kernel_mix_ratio", 1);
  K_gain_shifted_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("K_gain_null_space_shifted", 1);
  // dynamic reconfigure
  reconf_server_ = new dynamic_reconfigure::Server<hydrus::torsionNullSpaceShifterConfig>(nhp_);
  reconf_func_ = boost::bind(&NullSpaceShifter::cfgCallback, this, _1, _2);
  reconf_server_->setCallback(reconf_func_);

  K_gain_.resize(0,0);
  K_mode_.resize(0,0);
  B_eom_kernel_.resize(0,0);
  torsion_eigens_.resize(0);
}

NullSpaceShifter::~NullSpaceShifter()
{
}

void NullSpaceShifter::cfgCallback(hydrus::torsionNullSpaceShifterConfig& config, uint32_t level) {
  printf("Torsion NullSpaceShifter Param:");
  switch(level)
  {
    case 0:
      null_space_shift_thresh_ = config.null_space_shift_thresh;
      printf("change parameter of shift thresh: %f\n", null_space_shift_thresh_);
      break;
    case 1:
      null_space_shift_limit_ratio_ = config.null_space_shift_limit_ratio;
      printf("change parameter of shift limit ratio: %f\n", null_space_shift_limit_ratio_);
      break;
    case 2:
      null_space_shift_mix_limit_ = config.null_space_shift_mix_limit;
      printf("change parameter of mix limit: %f\n", null_space_shift_mix_limit_);
      break;
    case 3:
      nlopt_position_penalty_ = config.nlopt_position_penalty;
      printf("change parameter of position penalty: %f\n", nlopt_position_penalty_);
      break;
    case 4:
      nlopt_alpha_ = config.nlopt_alpha;
      printf("change parameter of alpha: %f\n", nlopt_alpha_);
      break;
    default :
      printf("\n");
      break;
  }
}

void NullSpaceShifter::kGainCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
  K_gain_ = msg_utils::Float32MultiArray2EigenMatrix(msg);
}

void NullSpaceShifter::kModeCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
  K_mode_ = msg_utils::Float32MultiArray2EigenMatrix(msg);
}

void NullSpaceShifter::bEOMKernelCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
  B_eom_kernel_ = msg_utils::Float32MultiArray2EigenMatrix(msg);
}

void NullSpaceShifter::torsionEigensCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
  torsion_eigens_ = msg_utils::Float32MultiArray2Vector(msg);
}

void NullSpaceShifter::calculate() {

  if (K_gain_.cols()>0 && B_eom_kernel_.cols()>0 && K_mode_.cols()>0 && K_mode_.cols()==torsion_eigens_.size()) {
    Eigen::MatrixXd kernel_mix_ratio(K_gain_.cols(), B_eom_kernel_.cols());
    Eigen::MatrixXd kernel_mix_ratio_for_publish = Eigen::MatrixXd::Zero(K_gain_.cols(), B_eom_kernel_.cols());
    Eigen::MatrixXd K_gain_shifted = K_gain_;

    for (int i = 0; i < K_gain_.cols(); i++) {
      std::vector<double> x(B_eom_kernel_.cols(), 0.0);
      double bound = std::abs(null_space_shift_limit_ratio_);

      if (prev_results_.size() == K_gain_.cols() && prev_results_[i].size() == x.size()) {
        for (int j = 0; j < x.size(); ++j) {
          double initial_val = prev_results_[i][j];
          if (std::abs(initial_val) < bound) {
            x[j] = initial_val;
          }
        }
      }

      try{
        auto nlopt_algorithm = nlopt::GN_DIRECT;
        if (is_nlopt_use_global_) {
          nlopt_algorithm = nlopt::GN_DIRECT;
        } else {
          nlopt_algorithm = nlopt::LN_COBYLA;//nlopt::LN_PRAXIS
        }
        nlopt::opt gain_opt(nlopt::GN_DIRECT, B_eom_kernel_.cols());
        gain_opt.set_max_objective(maximizeTorsionDistanceFactor, this);
        gain_opt.set_lower_bounds(-bound);
        gain_opt.set_upper_bounds( bound);
        gain_opt.set_xtol_rel(nlopt_xtol_rel_);
        gain_opt.set_maxeval(null_space_shift_max_eval_);
        double max_f = 0;
        nlopt_tmp_index_ = i;
        nlopt::result result = gain_opt.optimize(x, max_f);
      } catch (std::exception &e) {
        ROS_WARN_STREAM("nlopt failed: " << e.what());
      }

      ROS_DEBUG("nlopt result: ");
      for (int j = 0; j < x.size(); ++j) {
        ROS_DEBUG_STREAM("" << x[j]);
        kernel_mix_ratio(i,j) = x[j];
      }

      // mix
      double original_torsion_factor = 1.0;
      for (int j = 0; j < K_mode_.cols(); ++j) {
        double K_corr = K_gain_.col(i).dot( K_mode_.col(j) ) / K_gain_.col(i).norm() / K_mode_.col(j).norm();
        original_torsion_factor -= nlopt_alpha_ * K_corr*K_corr * sqrt(torsion_eigens_[0]/torsion_eigens_[j]);
      }
      ROS_DEBUG_STREAM("factor " << i << " : " << original_torsion_factor);

      if (original_torsion_factor < null_space_shift_thresh_) {
        double limit_factor = 1.0;
        double max_ratio = std::max(std::abs(*std::max_element(x.begin(), x.end())), std::abs(*std::min_element(x.begin(), x.end())));
        double max_gain = K_gain_.col(i).cwiseAbs().maxCoeff();
        if (max_ratio > null_space_shift_mix_limit_) {
          limit_factor = null_space_shift_mix_limit_ / max_ratio;
        }
        double norm_org = K_gain_.col(i).norm();

        for (int j = 0; j < B_eom_kernel_.cols(); ++j) {
          K_gain_shifted.col(i) += x[j] * B_eom_kernel_.col(j) * norm_org * limit_factor;
        }
        K_gain_shifted.col(i) = K_gain_shifted.col(i) * norm_org/K_gain_shifted.col(i).norm();

        kernel_mix_ratio_for_publish.row(i) = kernel_mix_ratio.row(i) *norm_org*limit_factor;
      }
    }

    prev_results_.resize(K_gain_.cols());
    for (int i = 0; i < K_gain_.cols(); ++i) {
      prev_results_[i].resize(B_eom_kernel_.cols());
      for (int j = 0; j < B_eom_kernel_.cols(); ++j) {
        prev_results_[i][j] = kernel_mix_ratio(i,j);
      }
    }

    kernel_mix_ratio_pub_.publish(msg_utils::EigenMatrix2Float32MultiArray(kernel_mix_ratio_for_publish));
    K_gain_shifted_pub_.publish(msg_utils::EigenMatrix2Float32MultiArray(K_gain_shifted));
  }
}

int main (int argc, char* argv[]) {
  ros::init(argc, argv, "null_space_shifter");
  ros::NodeHandle nh;
  ros::NodeHandle nhp = ros::NodeHandle("~");


  NullSpaceShifter null_space_shifter(nh, nhp);

  int rate;
  nhp.param<int>("rate", rate, 1);
  ros::Rate r(rate);
  while (ros::ok())
  {
    ros::spinOnce();
    null_space_shifter.calculate();
    r.sleep();
  }

  return 0;
}


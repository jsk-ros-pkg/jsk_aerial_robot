#ifndef NULL_SPACE_SHIFTER
#define NULL_SPACE_SHIFTER

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include <hydrus/torsionNullSpaceShifterConfig.h>
#include <hydrus/util/msg_utils.h>

#include <Eigen/Dense>

#include <nlopt.hpp>

#include <vector>
#include <algorithm>
#include <boost/bind.hpp>

class NullSpaceShifter
{
  public:
    NullSpaceShifter(ros::NodeHandle nh, ros::NodeHandle nhp);
    ~NullSpaceShifter();

    void calculate();

    // for nlopt
    inline int getNloptTmpIndex() const {return nlopt_tmp_index_; }
    inline const Eigen::MatrixXd& getKGain() const {return K_gain_;}
    inline const Eigen::MatrixXd& getKMode() const {return K_mode_;}
    inline const Eigen::MatrixXd& getBEomKernel() const {return B_eom_kernel_;}
    inline const std::vector<double>& getTorsionEigens() const {return torsion_eigens_;}
    inline double getNloptPositionPenalty() const {return nlopt_position_penalty_; }
    inline double getNloptAlpha() const {return nlopt_alpha_; }
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::Subscriber K_gain_sub_;
    ros::Subscriber K_mode_sub_;
    ros::Subscriber B_eom_kernel_sub_;
    ros::Subscriber torsion_eigens_sub_;

    ros::Publisher kernel_mix_ratio_pub_;
    ros::Publisher K_gain_shifted_pub_;

    dynamic_reconfigure::Server<hydrus::torsionNullSpaceShifterConfig>::CallbackType reconf_func_;
    dynamic_reconfigure::Server<hydrus::torsionNullSpaceShifterConfig>* reconf_server_;
    void cfgCallback(hydrus::torsionNullSpaceShifterConfig& config, uint32_t level);

    int nlopt_tmp_index_;
    Eigen::MatrixXd K_gain_;
    Eigen::MatrixXd K_mode_;
    Eigen::MatrixXd B_eom_kernel_;
    std::vector<double> torsion_eigens_;
    double nlopt_position_penalty_;
    double nlopt_alpha_;

    void kGainCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
    void kModeCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
    void bEOMKernelCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
    void torsionEigensCallback(const std_msgs::Float32MultiArrayConstPtr& msg);

    std::vector<std::vector<double>> prev_results_;

    bool is_nlopt_use_global_;
    double null_space_shift_thresh_;
    double null_space_shift_limit_ratio_;
    double null_space_shift_mix_limit_;
    int null_space_shift_max_eval_;
    double nlopt_xtol_rel_;

};

#endif /* ifndef NULL_SPACE_SHIFTER */

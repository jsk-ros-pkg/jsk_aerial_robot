#ifndef TRANSFORM_CONTROL_H
#define TRANSFORM_CONTROL_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_eigen.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>

#include <string>
//* for eigen cumputation 
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include <hydra_transform_control/HydraParam.h>
#include <std_msgs/UInt16.h>

#include <std_msgs/Float32.h>

class TransformController{
 public:
  TransformController(ros::NodeHandle nh, ros::NodeHandle nh_private, bool callback_flag = true);
  ~TransformController();

  void cogComputation(std::vector<tf::StampedTransform> transforms);
  void principalInertiaComputation(std::vector<tf::StampedTransform> transforms, bool continuous_flag = true);

  double getLinkLength();
  double getPropellerDiameter(); 
  int getLinkNum();

  void getLinksOriginFromCog(std::vector<Eigen::Matrix<double, 3, 1> > &links_origin);

  std::vector<Eigen::Matrix<double, 3, 1> > links_origin_from_cog_; //temporary

 private:

  ros::NodeHandle nh_,nh_private_;
  ros::Publisher transform_control_pub_;
  ros::Publisher principal_axis_pub_;
  ros::Publisher cog_rotate_pub_; //for position control => to mocap

  tf::TransformListener tf_;
  double control_rate_;
  double tf_pub_rate_;
  ros::Timer  control_timer_;
  ros::Timer  tf_pub_timer_;

  ros::Time system_tf_time_;

  double link_length_;
  double propeller_diameter_;
  int link_num_;
  double link_mass_;
  double link_i_xx_;
  double link_i_yy_;
  double link_i_zz_;
  std::string root_link_name_;
  std::vector<std::string> links_name_;

  double multilink_i_xx_,multilink_i_yy_, multilink_i_zz_;

  bool debug_log_;

  double rotate_angle_;

  Eigen::Matrix<double, 3, 1> cog_;

  Eigen::Matrix<double, 3, 3> link_inertia_, link_principal_inertia_;
  Eigen::Matrix<double, 3, 3> links_inertia_, links_principal_inertia_, init_links_principal_inertia_;
  //Eigen::Matrix<double, 2, 2> links_xy_inertia_, links_xy_principal_inertia_;
  Eigen::Matrix<double, 3, 3> rotate_matrix_;
  Eigen::Matrix<double, 3, 3> cog_matrix_;


  void controlFunc(const ros::TimerEvent & e);  
  void tfPubFunc(const ros::TimerEvent & e);
  void visualization();
  int sgn(double value){ return  (value / fabs(value));}

  //Q computation for the transform distribution
  std::vector<int> propeller_direction_;
  double x_distribution_scale_;
  double y_distribution_scale_;
  std::vector<double> q_vector_scale_; //for the Q matrix scaling (becuase of the gain setting of multiwii)
  std::vector<int> propeller_order_;
  Eigen::Matrix4d Q_;

  void propellerOrder();
  bool qCompute();
    
  void cogCoordPublish();
  void initParam();


  void param2contoller();

  //MocapData* mocapData_;

};


#endif

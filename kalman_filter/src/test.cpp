#include <kalman_filter/kf_pos_vel_acc.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  KalmanFilterPosVelAcc kf_test(nh, nh_private, std::string("test"));

  kf_test.setInitState(0, 0);
  kf_test.setInputFlag();
  Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1); 
  temp(0,0) = 0;
  //kf_test.KalmanFilter<2, 1, 1>::prediction(temp);
  //kf_test.correction(temp);

  ros::spin ();
  //delete kf_test;
  return 0;
}


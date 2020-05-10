#include <hydrus_xi/hydrus_xi_fully_actuated_robot_model.h>


HydrusXiFullyActuatedRobotModel::HydrusXiFullyActuatedRobotModel(bool init_with_rosparam, bool verbose, double wrench_margin_f_min_thre, double wrench_margin_t_min_thre, double epsilon) :
  RobotModel(init_with_rosparam, verbose, wrench_margin_f_min_thre, wrench_margin_t_min_thre, epsilon)
{
}

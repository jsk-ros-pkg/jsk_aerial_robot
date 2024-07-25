/*
1. 関節角度を探索. [-90, 90]^2
2. 姿勢変化を考慮. [0, 90]^2
3. レンチを探索. [0, 0, [0, mg], 0, 0, 0], [0, 0, 0, [tau_min, tau_max], [tau_min, tau_max], 0]
*/

#include <delta/design/delta_joint_load_calc.h>

DeltaJointLoadCalc::DeltaJointLoadCalc(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh),
  nhp_(nhp_)
{
  transformable_robot_model_ = boost::make_shared<RollingRobotModel>();
  transformable_robot_model_->updateRobotModel();

  rotor_num_ = transformable_robot_model_->getRotorNum();
  rotor_tilt_.resize(rotor_num_);
  target_thrust_.resize(rotor_num_);
  target_gimbal_angles_.resize(rotor_num_);

  auto robot_model_xml = transformable_robot_model_->getRobotModelXml("robot_description");
  for(int i = 0; i < rotor_num_; i++)
    {
      std::string rotor_tilt_name = std::string("rotor_tilt") + std::to_string(i + 1);
      TiXmlElement* rotor_tilt_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement(rotor_tilt_name);
      rotor_tilt_attr->Attribute("value", &rotor_tilt_.at(i));
      std::cout << "rotor tilt" << i + 1 << " "  << rotor_tilt_.at(i) << std::endl;
    }

  getParam<bool>(nh_, "verbose", verbose_, false);
  getParam<int>(nh_, "wrench_mode", wrench_mode_, 0);

  getParam<double>(nhp_, "max_thrust", max_thrust_, 0.0);

  getParam<double>(nhp_, "joint_angle_min", joint_angle_min_, 0.0);
  getParam<double>(nhp_, "joint_angle_max", joint_angle_max_, 0.0);
  getParam<int>(nhp_, "joint_angle_cnt_max", joint_angle_cnt_max_, 0);

  getParam<double>(nhp_, "pose_roll_max_abs", pose_roll_max_abs_, 0.0);
  getParam<double>(nhp_, "pose_pitch_max_abs", pose_pitch_max_abs_, 0.0);
  getParam<int>(nhp_, "pose_roll_cnt_max", pose_roll_cnt_max_, 0);
  getParam<int>(nhp_, "pose_pitch_cnt_max", pose_pitch_cnt_max_, 0);

  getParam<int>(nhp_, "wrench_z_cnt_max", wrench_z_cnt_max_, 0);
  getParam<double>(nhp_, "wrench_z_min_acc", wrench_z_min_acc_, 0.0);
  getParam<double>(nhp_, "wrench_z_max_acc", wrench_z_max_acc_, 0.0);

  getParam<double>(nhp_, "wrench_roll_max_abs", wrench_roll_max_abs_, 0.0);
  getParam<double>(nhp_, "wrench_pitch_max_abs", wrench_pitch_max_abs_, 0.0);
  getParam<int>(nhp_, "wrench_torque_cnt_max", wrench_torque_cnt_max_, 0);

  std::string file_name;
  if(wrench_mode_ == 0) file_name = std::string("delta_joint_load_z_") + std::to_string((int)ros::Time::now().toSec()) + std::string(".txt");
  else if(wrench_mode_ == 1) file_name = std::string("delta_joint_load_rp_") + std::to_string((int)ros::Time::now().toSec()) + std::string(".txt");
  ofs_.open(file_name, std::ios::out);
}

void DeltaJointLoadCalc::jointAngleSearch()
{
  if(wrench_mode_ != 0 && wrench_mode_ != 1)
    {
      ROS_ERROR_STREAM("set correct wrench mode.");
      return;
    }

  search_cnt_ = 0;

  const auto joint_index_map = transformable_robot_model_->getJointIndexMap();
  joint_positions_ = transformable_robot_model_->getJointPositions();
  joint_positions_(joint_index_map.find("joint1")->second) = 2.08;
  joint_positions_(joint_index_map.find("joint2")->second) = 2.08;

  transformable_robot_model_->setCogDesireOrientation(0.0, 0.0, 0.0);
  transformable_robot_model_->updateRobotModel(joint_positions_);
  transformable_robot_model_->setControlFrame("cog");
  if(verbose_) std::cout << transformable_robot_model_->getFullWrenchAllocationMatrixFromControlFrame() << std::endl;

  /*    setCogDesireOrientation()
     -> updateRobotmodel(update joint and CoG based info)
     -> setTargetFrame("cog") (hold real cog frame)
     -> getFullWrenchAllocationMatrixFromControlFrame("cog") */

  start_time_ = ros::Time::now().toSec();
  if(wrench_mode_ == 0)
    estimated_max_count_ = joint_angle_cnt_max_ * joint_angle_cnt_max_ * pose_roll_cnt_max_ * pose_pitch_cnt_max_ * wrench_z_cnt_max_;
  else if(wrench_mode_ == 1)
    estimated_max_count_ = joint_angle_cnt_max_ * joint_angle_cnt_max_ * pose_roll_cnt_max_ * pose_pitch_cnt_max_ * wrench_torque_cnt_max_ * wrench_torque_cnt_max_;

  int joint_num = transformable_robot_model_->getJointNum();

  for(int joint1_cnt = 0; joint1_cnt < joint_angle_cnt_max_; joint1_cnt++)
    {
      for(int joint2_cnt = 0; joint2_cnt < joint_angle_cnt_max_; joint2_cnt++)
        {
          for(int pose_roll_cnt = 0; pose_roll_cnt < pose_roll_cnt_max_; pose_roll_cnt++)
            {
              for(int pose_pitch_cnt = 0; pose_pitch_cnt < pose_pitch_cnt_max_; pose_pitch_cnt++)
                {
                  // setCogDesireOrientation() -> FK (update joint and CoG based info) -> setTargetFrame("cog") (hold real cog frame) -> getFullWrenchAllocationMatrixFromControlFrame("cog")

                  double joint1 = joint_angle_min_ + (joint_angle_max_ - joint_angle_min_) * (double)joint1_cnt / (double)(joint_angle_cnt_max_ - 1.0);
                  double joint2 = joint_angle_min_ + (joint_angle_max_ - joint_angle_min_) * (double)joint2_cnt / (double)(joint_angle_cnt_max_ - 1.0);
                  double pose_roll = -abs(pose_roll_max_abs_)   + 2.0 * abs(pose_roll_max_abs_)  * (double)pose_roll_cnt  / (double)(pose_roll_cnt_max_ - 1.0);
                  double pose_pitch = -abs(pose_pitch_max_abs_) + 2.0 * abs(pose_pitch_max_abs_) * (double)pose_pitch_cnt / (double)(pose_pitch_cnt_max_ - 1.0);

                  transformable_robot_model_->setCogDesireOrientation(pose_roll, pose_pitch, 0.0);

                  joint_positions_(joint_index_map.find("joint1")->second) = joint1;
                  joint_positions_(joint_index_map.find("joint2")->second) = joint2;
                  transformable_robot_model_->updateRobotModel(joint_positions_);

                  transformable_robot_model_->setControlFrame("cog");

                  full_q_mat_inv_ = aerial_robot_model::pseudoinverse(transformable_robot_model_->getFullWrenchAllocationMatrixFromControlFrame("cog"));

                  // wrench search
                  desired_wrench_ = Eigen::VectorXd::Zero(6);
                  if(wrench_mode_ == 0)
                    {
                      for(int wrench_z_cnt = 0; wrench_z_cnt < wrench_z_cnt_max_; wrench_z_cnt++)
                        {
                          desired_wrench_(2) = transformable_robot_model_->getMass() * (wrench_z_min_acc_ + (wrench_z_max_acc_ - wrench_z_min_acc_) * ((double)wrench_z_cnt / ((double)wrench_z_cnt_max_ - 1)));

                          wrenchAllocation();

                          if(verbose_)
                            {
                              std::cout << "joint1: " << joint1 << " joint2: " << joint2 << " pose_roll: " << pose_roll << " pose_pitch: " << pose_pitch << std::endl;
                              std::cout << "desired wrench: " << desired_wrench_.transpose() << std::endl;
                              std::cout << "thrust: " << target_thrust_.transpose() << std::endl;
                              std::cout << "gimbal: " << target_gimbal_angles_.transpose() << std::endl;
                              std::cout << "joint torque: " << joint_torque_.transpose() << std::endl;
                              std::cout << std::endl;
                            }
                        }
                    }
                  else if(wrench_mode_ == 1)
                    {
                      for(int wrench_roll_cnt = 0; wrench_roll_cnt < wrench_torque_cnt_max_; wrench_roll_cnt++)
                        {
                          for(int wrench_pitch_cnt = 0; wrench_pitch_cnt < wrench_torque_cnt_max_; wrench_pitch_cnt++)
                            {
                              desired_wrench_(3) = -wrench_roll_max_abs_  + 2.0 * wrench_roll_max_abs_  * wrench_roll_cnt  / ((double)wrench_torque_cnt_max_ - 1.0);
                              desired_wrench_(4) = -wrench_pitch_max_abs_ + 2.0 * wrench_pitch_max_abs_ * wrench_pitch_cnt / ((double)wrench_torque_cnt_max_ - 1.0);

                              wrenchAllocation();

                              if(verbose_)
                                {
                                  std::cout << "joint1: " << joint1 << " joint2: " << joint2 << " pose_roll: " << pose_roll << " pose_pitch: " << pose_pitch << std::endl;
                                  std::cout << "desired wrench: " << desired_wrench_.transpose() << std::endl;
                                  std::cout << "thrust: " << target_thrust_.transpose() << std::endl;
                                  std::cout << "gimbal: " << target_gimbal_angles_.transpose() << std::endl;
                                  std::cout << "joint torque: " << joint_torque_.transpose() << std::endl;
                                  std::cout << std::endl;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

  ofs_.close();
  ROS_WARN_STREAM("finish.");
  ROS_WARN_STREAM("search count: " << search_cnt_);
  ROS_WARN_STREAM("search time: " << ros::Time::now().toSec() - start_time_);
}

void DeltaJointLoadCalc::wrenchAllocation()
{
  const auto joint_index_map = transformable_robot_model_->getJointIndexMap();

  target_vectoring_force_ = full_q_mat_inv_ * desired_wrench_;

  for(int rotor_i = 0; rotor_i < rotor_num_; rotor_i++)
    {
      Eigen::VectorXd full_lambda_all_i = target_vectoring_force_.segment(2 * rotor_i, 2);
      target_thrust_(rotor_i) = full_lambda_all_i.norm() / fabs(cos(rotor_tilt_.at(rotor_i)));
      target_gimbal_angles_(rotor_i) = atan2(-full_lambda_all_i(0), full_lambda_all_i(1));
      joint_positions_(joint_index_map.find("gimbal" + std::to_string(rotor_i + 1))->second) = target_gimbal_angles_(rotor_i);
    }

  transformable_robot_model_->updateRobotModel(joint_positions_);
  computeJointTorque();

  bool ok = true;
  for(int rotor_i; rotor_i < rotor_num_; rotor_i++)
    {
      if(!(0 < target_thrust_(rotor_i) && target_thrust_(rotor_i) < max_thrust_))
        {
          ok = false;
        }
    }

  if(ok)
    ofs_ << joint_torque_.transpose() << std::endl;

  search_cnt_++;
  ROS_INFO_STREAM_THROTTLE(10.0, "time: " << ros::Time::now().toSec() - start_time_ << ". count: " << search_cnt_ << " / " << estimated_max_count_ << ". remained time is estimated: " << (ros::Time::now().toSec() - start_time_) * (1.0/ ((double)search_cnt_ / estimated_max_count_) - 1.0));
}

void DeltaJointLoadCalc::computeJointTorque()
{
  // eq.(1) of https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8962166

  const auto& sigma = transformable_robot_model_->getRotorDirection();
  const auto& joint_positions = transformable_robot_model_->getJointPositions();
  const auto& inertia_map = transformable_robot_model_->getInertiaMap();
  const int joint_num = transformable_robot_model_->getJointNum();
  const int rotor_num = transformable_robot_model_->getRotorNum();
  const double m_f_rate = transformable_robot_model_->getMFRate();
  const auto gravity = transformable_robot_model_->getGravity();
  const auto& static_thrust =  transformable_robot_model_->getStaticThrust();
  const auto& thrust_wrench_units = transformable_robot_model_->getThrustWrenchUnits();

  transformable_robot_model_->calcBasicKinematicsJacobian(); // update thrust_coord_jacobians_

  joint_torque_ = Eigen::VectorXd::Zero(joint_num);

  // update coord jacobians for cog point and convert to joint torque
  int seg_index = 0;
  std::vector<Eigen::MatrixXd> cog_coord_jacobians(inertia_map.size());
  for(const auto& inertia : inertia_map)
    {
      cog_coord_jacobians.at(seg_index) = transformable_robot_model_->getJacobian(joint_positions, inertia.first, inertia.second.getCOG());
      joint_torque_ -= cog_coord_jacobians.at(seg_index).rightCols(joint_num).transpose() * inertia.second.getMass() * (-gravity);
      seg_index ++;
    }
  // transformable_robot_model_->setCOGCoordJacobians(cog_coord_jacobians);

  // thrust
  for(int i = 0; i < rotor_num; i++)
    {
      const auto thrust_coord_jacobians = transformable_robot_model_->getThrustCoordJacobians();
      Eigen::VectorXd wrench = thrust_wrench_units.at(i) * target_thrust_(i);
      joint_torque_ -= thrust_coord_jacobians.at(i).rightCols(joint_num).transpose() * wrench;
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "delta_joint_load_calc");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~"); // node handle with private namespace

  DeltaJointLoadCalc* delta_joint_load_calc = new DeltaJointLoadCalc(nh, nhp);
  delta_joint_load_calc->jointAngleSearch();
  ros::spin();
  delete delta_joint_load_calc;
  return 0;
}

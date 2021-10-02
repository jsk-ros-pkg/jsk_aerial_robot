#include <dragon/control/full_vectoring_control.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

namespace
{
  int cnt = 0;
  double ave_cnt = 0;
  double thrustSumFunc(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
  {
    aerial_robot_model::RobotModel *robot_model = reinterpret_cast<aerial_robot_model::RobotModel*>(ptr);

    int rotor_num = robot_model->getRotorNum();

    // func: sum(f_i^2)
    double force_sq_sum = 0;
    for(int i = 0; i < rotor_num; i++) force_sq_sum += (x.at(i) * x.at(i)); 

    // grad: [2 f_1, 2 f_2, .... 2 f_i]
    if(grad.size() > 0)
      {
        grad = std::vector<double>(x.size(), 0);
        for(int i = 0; i < rotor_num; i++)
          grad.at(i) = 2 * x.at(i); // lambda
      }

    cnt++;

    return  force_sq_sum;
  }

  void wrenchAllocationEqCons(unsigned m, double *result, unsigned n, const double* x, double* grad, void* ptr)
  {
    /* x:
       0 ~rotor_num: thrust_force -> lambda
       no locked gimbal: rotor_num ~ 3 * rotor_num: (gimbal_roll, gimbal_pitch) * rotor_num
    */

    DragonFullVectoringController *controller = reinterpret_cast<DragonFullVectoringController*>(ptr);
    const auto roll_locked_gimbal = controller->getRollLockedGimbal();
    const auto gimbal_nominal_angles = controller->getGimbalNominalAngles();

    auto robot_model = controller->getRobotModelForControl();
    const int rotor_num = robot_model->getRotorNum();
    const auto& joint_index_map = robot_model->getJointIndexMap();

    int gimbal_lock_num = std::accumulate(roll_locked_gimbal.begin(), roll_locked_gimbal.end(), 0); // update
    assert(n == 3 * rotor_num - gimbal_lock_num);
    assert(m == 6);

    // update the robot model with the new gimbal angles
    KDL::JntArray gimbal_processed_joint = robot_model->getJointPositions();
    int col = 0;
    for(int i = 0; i < rotor_num; i++)
      {
        std::string s = std::to_string(i + 1);
        if(roll_locked_gimbal.at(i) == 0)
          {
            gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = x[rotor_num + col];
            gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = x[rotor_num + col + 1];
            col += 2;
          }
        else
          {
            // roll is locked
            gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = gimbal_nominal_angles.at(2 * i);
            gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = x[rotor_num + col];
            col +=1;
          }
      }
    robot_model->updateRobotModel(gimbal_processed_joint);

    // get updated kinematics
    auto cog = robot_model->getCog<KDL::Frame>();
    const auto seg_tf_map = robot_model->getSegmentsTf();
    std::vector<Eigen::Vector3d> p = robot_model->getRotorsOriginFromCog<Eigen::Vector3d>();
    const std::vector<Eigen::Vector3d> u = robot_model->getRotorsNormalFromCog<Eigen::Vector3d>();
    const auto& sigma = robot_model->getRotorDirection();
    const double m_f_rate = robot_model->getMFRate();
    const auto& inertia_map = robot_model->getInertiaMap();
    const auto& joint_segment_map = robot_model->getJointSegmentMap();
    const auto& segment_map = robot_model->getTree().getSegments();

    Eigen::Vector3d p_gimal_roll_pitch = kdlToEigen(seg_tf_map.at(std::string("gimbal1_roll_module")).Inverse() * seg_tf_map.at(std::string("gimbal1_pitch_module")).p);

    //get Q: WrenchAllocationMatrix
    Eigen::MatrixXd Q(6, rotor_num);
    Eigen::VectorXd lambda = Eigen::VectorXd::Zero(rotor_num); // thrust force
    for (unsigned int i = 0; i < rotor_num; ++i) {
      Q.block(0, i, 3, 1) = u.at(i);
      Q.block(3, i, 3, 1) = p.at(i).cross(u.at(i)) + m_f_rate * sigma.at(i + 1) * u.at(i);
      lambda(i) = x[i];
    }

    Eigen::VectorXd target_wrench_acc_cog = controller->getTargetWrenchAccCog();
    Eigen::VectorXd target_wrench_cog = Eigen::VectorXd::Zero(6);
    target_wrench_cog.head(3) = robot_model->getMass() * target_wrench_acc_cog.head(3);
    target_wrench_cog.tail(3) = robot_model->getInertia<Eigen::Matrix3d>() * target_wrench_acc_cog.tail(3);
    //target_wrench_cog.tail(3) = controller->getNominalInertia() * target_wrench_acc_cog.tail(3); // approximately assume the inertial is constant regardless of change in gimbal angles

    Eigen::VectorXd wrench_diff = Q * lambda - target_wrench_cog;
    //ROS_INFO_STREAM("Q * lambda: " << (Q * lambda).transpose() << "; wrench_diff: " << wrench_diff.transpose() << "; lambda: " << lambda.transpose() << "; target wrench: " << target_wrench_cog.transpose() <<  "\n Q: \n" << Q);
    for(int i = 0; i < m; i++) result[i] = wrench_diff(i);

    if(grad == NULL) return;

#if 1 // analytical solution
    // g(theta, phi, lambda) = Q(theta, phi) lambda - target_wrench_cog = 0
    col = 0;

    auto deInertiaCoGOffset =  [](KDL::Vector p, KDL::Vector d_p)
      {
        // cog offset for inertia m[[y^2 + z^2, -xy, -xz], [-xy, x^2 + z^2, -yz], [-xz, -yz, x^2 + y^2]]
        // derivative: m[[2y d_y + 2z dz, -x dy - y dx, - x dz - z dx],[-x dy - y dx, 2x dx + 2z dz, -y dz - z dy],[-x dz - z dx, -y dz - z dy, 2x dx + 2y dy]]

        Eigen::Matrix3d mat;
        mat << 2 * (p.y() * d_p.y() + p.z() * d_p.z()), -(p.x() * d_p.y() + p.y() * d_p.x()), -(p.x() * d_p.z() + p.z() * d_p.x()),
        -(p.x() * d_p.y() + p.y() * d_p.x()), 2 * (p.x() * d_p.x() + p.z() * d_p.z()), -(p.y() * d_p.z() + p.z() * d_p.y()),
        -(p.x() * d_p.z() + p.z() * d_p.x()), -(p.y() * d_p.z() + p.z() * d_p.y()), 2 * (p.x() * d_p.x() + p.y() * d_p.y());

        return mat;
      };

    for(int j = 0; j < rotor_num; j++)
      {
        for(int i = 0; i < 6; i++)
          {
            // lambda:  partial(g) \ partial(lambda) = Q
            grad[i * n + j] = Q(i,j);
          }

        // gimbal angles
        // Q_j = lambda_j * [u_j, v_j]^T

        double lambda_j = x[j];
        double phi_j, theta_j;

        if(roll_locked_gimbal.at(j) == 0)
          {
            phi_j = x[rotor_num + col];
            theta_j = x[rotor_num + col + 1];
          }
        else
          {
            phi_j = gimbal_nominal_angles.at(2 * j);
            theta_j = x[rotor_num + col];
          }

        Eigen::Vector3d b_z(0, 0, 1);

        // partial(u_j) / partial(phi_j) = R^{cog}_{L_j} partial(R^{L_j}_{g_roll_j}(phi_j)) R^{g_roll_j}_{g_pitch_j}(theta_j) b_z
        Eigen::Matrix3d R_L_j = kdlToEigen((cog.Inverse() * seg_tf_map.at(std::string("link") + std::to_string(j + 1))).M);
        Eigen::Matrix3d partial_R_phi_j;
        partial_R_phi_j << 0, 0, 0,  0, -sin(phi_j), -cos(phi_j), 0, cos(phi_j), -sin(phi_j);
        Eigen::Matrix3d R_theta_j = kdlToEigen(KDL::Rotation::RotY(theta_j));
        Eigen::Vector3d partial_u_j_phi_j = R_L_j * partial_R_phi_j * R_theta_j * b_z;

        // partial(u_j) / partial(theta_j) = R^{cog}_{{g_roll_j}(phi_j) partial(R^{g_roll_j}_{g_pitch_j}(theta_j)) b_z
        Eigen::Matrix3d R_g_roll_j = R_L_j * kdlToEigen(KDL::Rotation::RotX(phi_j));
        Eigen::Matrix3d partial_R_theta_j;
        partial_R_theta_j << -sin(theta_j), 0, cos(theta_j),  0, 0, 0,  -cos(theta_j), 0, -sin(theta_j);
        Eigen::Vector3d partial_u_j_theta_j = R_g_roll_j * partial_R_theta_j * b_z;


        // v = p x u + k u
        // partial(v_i) / partial(phi_j) = partial(p_i) / partial(phi_j) x u_i + p_i x partial(u_i) / partial(phi_j) + k partial(u_i) / partial(phi_j)
        // sum_i lambda_i (partial(v_i) / partial(phi_j)) = sum_i lambda_i (partial(p_i) / partial(phi_j) x u_i) + lambda_j (p_j x partial(u_j) / partial(phi_j) + k partial(u_j) / partial(phi_j))

        // partial(p_j) / partial(phi_j) => the change of CoG
        std::string joint_name = std::string("gimbal") + std::to_string(j + 1) + std::string("_roll");
        std::string joint_child_segment_name = joint_name + std::string("_module");
        KDL::Segment joint_child_segment = GetTreeElementSegment(segment_map.at(joint_child_segment_name));
        KDL::Vector a = seg_tf_map.at(std::string("link") + std::to_string(j + 1)).M * joint_child_segment.getJoint().JointAxis();
        KDL::Vector r = seg_tf_map.at(joint_child_segment_name).p;

        KDL::RigidBodyInertia inertia = KDL::RigidBodyInertia::Zero();
        for (const auto& seg: joint_segment_map.at(joint_name))
          {
            if (seg.find("thrust") == std::string::npos)
              {
                KDL::Frame f = seg_tf_map.at(seg);
                inertia = inertia + f * inertia_map.at(seg);
              }
          }
        KDL::Vector c = inertia.getCOG();
        double m = inertia.getMass();
        KDL::Vector partial_cog_phi_j = a * (c - r) * m / robot_model->getMass();

        Eigen::Vector3d sum_partial_v_p_k_phi_j(0,0,0);
        for(int k = 0; k < rotor_num; k++)
          sum_partial_v_p_k_phi_j -= x[k] * kdlToEigen(cog.M.Inverse() * partial_cog_phi_j).cross(u.at(k)); // circle 1 of memo

        // partial(p_j) / partial(phi_j) also has an additional term partial_v_p_j_phi_j =  R^{cog}_{L_j} partial(R^{L_j}_{g_roll_j}(phi_j)) p^{g_roll_j}_{g_pitch_j}
        Eigen::Vector3d partial_v_p_j_phi_j =  lambda_j * (R_L_j * partial_R_phi_j * p_gimal_roll_pitch).cross(u.at(j)); // circle 2 in memo
        Eigen::Vector3d partial_v_u_j_phi_j = lambda_j * (p.at(j).cross(partial_u_j_phi_j) +  sigma.at(j + 1) * m_f_rate * partial_u_j_phi_j); // circle 3 + 4 in memo

        // derivative of inertial for cog torque
        Eigen::Matrix3d R_g_roll_j_root = kdlToEigen(seg_tf_map.at(joint_child_segment_name).M);
        Eigen::Matrix3d local_inertia = R_g_roll_j_root.transpose() * kdlToEigen(inertia.RefPoint(c).getRotationalInertia()) * R_g_roll_j_root;
        Eigen::Matrix3d partial_cog_inertia_phi_j = R_L_j * partial_R_phi_j * local_inertia * R_g_roll_j.transpose() +  R_g_roll_j * local_inertia * partial_R_phi_j.transpose() * R_L_j.transpose()  + kdlToEigen(cog.M).transpose() * (m * deInertiaCoGOffset(c, a * (c-r)) - robot_model->getMass() * deInertiaCoGOffset(cog.p, partial_cog_phi_j)) * kdlToEigen(cog.M);
        Eigen::Vector3d partial_cog_torque_phi_j = partial_cog_inertia_phi_j * target_wrench_acc_cog.tail(3);

        // partial(v_i) / partial(theta_j) = partial(p_i) x u_i / partial(theta_j) + p_i x partial(u_i) / partial(theta_j) + k partial(u_i) / partial(theta_j)
        // sum_i lambda_i (partial(v_i) / partial(theta_j)) = sum_i lambda_i (partial(p_i) x u_i / partial(theta_j)) + lambda_j (p_j x partial(u_j) / partial(theta_j) + k partial(u_j) / partial(theta_j))

        // partial(p_j) / partial(theta_j) => the change of CoG
        joint_name = std::string("gimbal") + std::to_string(j + 1) + std::string("_pitch");
        joint_child_segment_name = joint_name + std::string("_module");
        joint_child_segment = GetTreeElementSegment(segment_map.at(joint_child_segment_name));
        a = seg_tf_map.at(std::string("gimbal") + std::to_string(j + 1) + std::string("_roll_module")).M * joint_child_segment.getJoint().JointAxis();
        r = seg_tf_map.at(joint_child_segment_name).p;

        inertia = KDL::RigidBodyInertia::Zero();
        for (const auto& seg: joint_segment_map.at(joint_name))
          {
            if (seg.find("thrust") == std::string::npos)
              {
                KDL::Frame f = seg_tf_map.at(seg);
                inertia = inertia + f * inertia_map.at(seg);
              }
          }
        c = inertia.getCOG();
        m = inertia.getMass();
        KDL::Vector partial_cog_theta_j = a * (c - r) * m / robot_model->getMass();

        Eigen::Vector3d sum_partial_v_p_k_theta_j(0,0,0);
        for(int k = 0; k < rotor_num; k++)
          sum_partial_v_p_k_theta_j -= x[k] * kdlToEigen(cog.M.Inverse() * partial_cog_theta_j).cross(u.at(k));

        Eigen::Vector3d partial_v_u_j_theta_j = lambda_j * (p.at(j).cross(partial_u_j_theta_j) + sigma.at(j + 1) * m_f_rate * partial_u_j_theta_j);

        Eigen::Matrix3d R_g_pitch_j_root = kdlToEigen(seg_tf_map.at(joint_child_segment_name).M);
        Eigen::Matrix3d R_g_pitch_j = kdlToEigen(cog.M.Inverse()) * R_g_pitch_j_root;
        local_inertia = R_g_pitch_j_root.transpose() * kdlToEigen(inertia.RefPoint(c).getRotationalInertia()) * R_g_pitch_j_root;
        Eigen::Matrix3d partial_cog_inertia_theta_j = R_g_roll_j * partial_R_theta_j * local_inertia * R_g_pitch_j.transpose() +  R_g_pitch_j * local_inertia * partial_R_theta_j.transpose() * R_g_roll_j.transpose() + kdlToEigen(cog.M).transpose() * (m * deInertiaCoGOffset(c, a * (c-r)) - robot_model->getMass() * deInertiaCoGOffset(cog.p, partial_cog_theta_j)) * kdlToEigen(cog.M);
        Eigen::Vector3d partial_cog_torque_theta_j = partial_cog_inertia_theta_j * target_wrench_acc_cog.tail(3);

        if(roll_locked_gimbal.at(j) == 0)
          {
            grad[rotor_num + col] = lambda_j * partial_u_j_phi_j(0);
            grad[rotor_num + col + 1] = lambda_j * partial_u_j_theta_j(0);
            grad[n + rotor_num + col] = lambda_j * partial_u_j_phi_j(1);
            grad[n + rotor_num + col + 1] = lambda_j * partial_u_j_theta_j(1);
            grad[2 * n + rotor_num + col] = lambda_j * partial_u_j_phi_j(2);
            grad[2 * n + rotor_num + col + 1] = lambda_j * partial_u_j_theta_j(2);

            grad[3 * n + rotor_num + col] = sum_partial_v_p_k_phi_j(0) + partial_v_u_j_phi_j(0) + partial_v_p_j_phi_j(0) - partial_cog_torque_phi_j(0);
            grad[3 * n + rotor_num + col + 1] = sum_partial_v_p_k_theta_j(0) + partial_v_u_j_theta_j(0) - partial_cog_torque_theta_j(0);
            grad[4 * n + rotor_num + col] = sum_partial_v_p_k_phi_j(1) + partial_v_u_j_phi_j(1) + partial_v_p_j_phi_j(1) - partial_cog_torque_phi_j(1);
            grad[4 * n + rotor_num + col + 1] = sum_partial_v_p_k_theta_j(1) + partial_v_u_j_theta_j(1) - partial_cog_torque_theta_j(1);
            grad[5 * n + rotor_num + col] = sum_partial_v_p_k_phi_j(2) + partial_v_u_j_phi_j(2) + partial_v_p_j_phi_j(2) - partial_cog_torque_phi_j(2);
            grad[5 * n + rotor_num + col + 1] = sum_partial_v_p_k_theta_j(2) + partial_v_u_j_theta_j(2) - partial_cog_torque_theta_j(2);

            col += 2;
          }
        else
          {
            // roll is locked
            grad[rotor_num + col] = lambda_j * partial_u_j_theta_j(0);
            grad[n + rotor_num + col] = lambda_j * partial_u_j_theta_j(1);
            grad[2 * n + rotor_num + col] = lambda_j * partial_u_j_theta_j(2);
            grad[3 * n + rotor_num + col] = sum_partial_v_p_k_theta_j(0) + partial_v_u_j_theta_j(0) - partial_cog_torque_theta_j(0);
            grad[4 * n + rotor_num + col] = sum_partial_v_p_k_theta_j(1) + partial_v_u_j_theta_j(1) - partial_cog_torque_theta_j(1);
            grad[5 * n + rotor_num + col] = sum_partial_v_p_k_theta_j(2) + partial_v_u_j_theta_j(2) - partial_cog_torque_theta_j(2);

            col += 1;
          }
      }

    // Eigen::MatrixXd grad_matrix = Eigen::MatrixXd::Zero(m, n);
    // // bug of Eigen::Map
    // for(int i = 0; i < m; i++)
    //   {
    //     for(int j = 0; j <  n; j++)
    //       {
    //         grad_matrix(i,j) = grad[i * n + j];
    //       }
    //   }
    // ROS_INFO_STREAM("cons grad matrix: \n" << grad_matrix);

#endif

#if 0 // numerical
    /* Numerical solution */
    double delta = 0.000001;
    Eigen::MatrixXd grad_dash = Eigen::MatrixXd::Zero(m,n);
    KDL::JntArray nominal_gimbal_processed_joint = robot_model->getJointPositions();
    grad_dash.leftCols(rotor_num) = Q;
    col = 0;
    for(int j = 0; j < rotor_num; j++)
      {
        Eigen::MatrixXd Q_dash(6, rotor_num);
        std::string s = std::to_string(j + 1);

        if(roll_locked_gimbal.at(j) == 0)
          {
            gimbal_processed_joint = nominal_gimbal_processed_joint;
            gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = x[rotor_num + col] + delta;
            robot_model->updateRobotModel(gimbal_processed_joint);
            std::vector<Eigen::Vector3d> p_dash = robot_model->getRotorsOriginFromCog<Eigen::Vector3d>();
            std::vector<Eigen::Vector3d> u_dash = robot_model->getRotorsNormalFromCog<Eigen::Vector3d>();

            for (unsigned int i = 0; i < rotor_num; ++i)
              {
                Q_dash.block(0, i, 3, 1) = u_dash.at(i);
                Q_dash.block(3, i, 3, 1) = p_dash.at(i).cross(u_dash.at(i)) + m_f_rate * sigma.at(i + 1) * u_dash.at(i);
              }
            grad_dash.col(rotor_num + col) = (Q_dash - Q) * lambda / delta;
            Eigen::Vector3d target_torque_cog_bash = robot_model->getInertia<Eigen::Matrix3d>() * target_wrench_acc_cog.tail(3);
            grad_dash.block(3, rotor_num + col, 3, 1) -= (target_torque_cog_bash - target_wrench_cog.tail(3)) / delta;

            gimbal_processed_joint = nominal_gimbal_processed_joint;
            gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = x[rotor_num + col + 1] + delta;
            robot_model->updateRobotModel(gimbal_processed_joint);
            p_dash = robot_model->getRotorsOriginFromCog<Eigen::Vector3d>();
            u_dash = robot_model->getRotorsNormalFromCog<Eigen::Vector3d>();
            for (unsigned int i = 0; i < rotor_num; ++i)
              {
                Q_dash.block(0, i, 3, 1) = u_dash.at(i);
                Q_dash.block(3, i, 3, 1) = p_dash.at(i).cross(u_dash.at(i)) + m_f_rate * sigma.at(i + 1) * u_dash.at(i);
              }
            grad_dash.col(rotor_num + col + 1) = (Q_dash - Q) * lambda / delta;
            target_torque_cog_bash = robot_model->getInertia<Eigen::Matrix3d>() * target_wrench_acc_cog.tail(3);
            grad_dash.block(3, rotor_num + col + 1, 3, 1) -= (target_torque_cog_bash - target_wrench_cog.tail(3)) / delta;

            col += 2;
          }
        else
          {
            gimbal_processed_joint = nominal_gimbal_processed_joint;
            gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = x[rotor_num + col] + delta;
            robot_model->updateRobotModel(gimbal_processed_joint);
            std::vector<Eigen::Vector3d> p_dash = robot_model->getRotorsOriginFromCog<Eigen::Vector3d>();
            std::vector<Eigen::Vector3d> u_dash = robot_model->getRotorsNormalFromCog<Eigen::Vector3d>();
            for (unsigned int i = 0; i < rotor_num; ++i)
              {
                Q_dash.block(0, i, 3, 1) = u_dash.at(i);
                Q_dash.block(3, i, 3, 1) = p_dash.at(i).cross(u_dash.at(i)) + m_f_rate * sigma.at(i + 1) * u_dash.at(i);
              }
            grad_dash.col(rotor_num + col) = (Q_dash - Q) * lambda / delta;
            Eigen::Vector3d target_torque_cog_bash = robot_model->getInertia<Eigen::Matrix3d>() * target_wrench_acc_cog.tail(3);
            grad_dash.block(3, rotor_num + col, 3, 1) -= (target_torque_cog_bash - target_wrench_cog.tail(3)) / delta;

            col += 1;
          }
      }

    for(int i = 0; i < m; i++)
      {
        for(int j = 0; j < n; j++)
          grad[i * n + j] = grad_dash(i,j);
      }

    //ROS_INFO_STREAM("grad_dash: \n" << grad_dash);
#endif
  }
};

DragonFullVectoringController::DragonFullVectoringController():
  PoseLinearController()
{
}

void DragonFullVectoringController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  rosParamInit();

  dragon_robot_model_ = boost::dynamic_pointer_cast<Dragon::FullVectoringRobotModel>(robot_model);
  robot_model_for_control_ = boost::make_shared<aerial_robot_model::RobotModel>();

  /* initialize the gimbal target angles */
  target_base_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_ * 2, 0);

  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
  estimate_external_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("estimated_external_wrench", 1);
  rotor_interfere_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("rotor_interfere_wrench", 1);
  interfrence_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("interference_markers", 1);

  add_external_wrench_sub_ = nh_.subscribe(std::string("apply_external_wrench"), 1, &DragonFullVectoringController::addExternalWrenchCallback, this);
  clear_external_wrench_service_ = nh_.advertiseService(std::string("clear_external_wrench"), &DragonFullVectoringController::clearExternalWrenchCallback, this);

  rotor_interfere_comp_wrench_ = Eigen::VectorXd::Zero(6); // reset
  est_external_wrench_ = Eigen::VectorXd::Zero(6);
  init_sum_momentum_ = Eigen::VectorXd::Zero(6);
  integrate_term_ = Eigen::VectorXd::Zero(6);
  prev_est_wrench_timestamp_ = 0;
  fz_bias_ = 0;
  tx_bias_ = 0;
  ty_bias_ = 0;

  prev_thrust_force_gimbal_angles_.resize(0);

  wrench_estimate_thread_ = boost::thread([this]()
                                          {
                                            ros::NodeHandle control_nh(nh_, "controller");
                                            double update_rate;
                                            control_nh.param ("wrench_estimate_update_rate", update_rate, 100.0);

                                            ros::Rate loop_rate(update_rate);
                                            while(ros::ok())
                                              {
                                                externalWrenchEstimate();
                                                loop_rate.sleep();
                                              }
                                          });
}

void DragonFullVectoringController::reset()
{
  PoseLinearController::reset();

  prev_thrust_force_gimbal_angles_.resize(0);
}

void DragonFullVectoringController::rotorInterfereCompensation()
{
  //rotor interference compensation based on previous robot model
  overlap_positions_.clear();
  overlap_weights_.clear();
  overlap_segments_.clear();
  overlap_rotors_.clear();

  if(navigator_->getForceLandingFlag())
    {
      rotor_interfere_comp_wrench_ = Eigen::VectorXd::Zero(6);
      return;
    }

  const auto& seg_tf_map = robot_model_for_control_->getSegmentsTf();

  if(seg_tf_map.size() == 0) return;

  const auto u = robot_model_for_control_->getRotorsNormalFromCog<Eigen::Vector3d>();
  double link_length = (seg_tf_map.at(std::string("inter_joint1")).p - seg_tf_map.at(std::string("link1")).p).Norm();
  KDL::Frame cog_inv = robot_model_for_control_->getCog<KDL::Frame>().Inverse();

  auto rotorInterfere = [this, &seg_tf_map, &cog_inv](int i, Eigen::Vector3d p_rotor, Eigen::Vector3d u_rotor, double link_length, std::string rotor_name)
    {
      for(int j = 0; j < motor_num_; ++j)
        {
          bool overlap_inter = false;
          std::string s = std::to_string(j + 1);

          KDL::Frame f_link  = cog_inv * seg_tf_map.at(std::string("link") + s);
          Eigen::Vector3d u_link = aerial_robot_model::kdlToEigen(f_link.M * KDL::Vector(1, 0, 0));
          Eigen::Vector3d p_link = aerial_robot_model::kdlToEigen(f_link.p);
          Eigen::Vector3d p_inter;
          double dist_inter = overlap_dist_inter_joint_thresh_;
          if(j == motor_num_ - 1)
            p_inter = p_link + u_link * link_length;
          else
            p_inter = aerial_robot_model::kdlToEigen(((cog_inv * seg_tf_map.at(std::string("inter_joint") + s)).p + (cog_inv * seg_tf_map.at(std::string("link") + std::to_string(j + 2))).p)/2);

          if(p_inter.z() > p_rotor.z() && p_link.z() > p_rotor.z()) continue; // never overlap

          // case1: inter joint
          if(p_inter.z() <= p_rotor.z())
            {
              dist_inter = (p_rotor + (p_inter - p_rotor).dot(u_rotor) * u_rotor - p_inter).norm();
              if(dist_inter < overlap_dist_inter_joint_thresh_)
                {
                  overlap_inter = true;
                  /// ROS_INFO_STREAM("rotor" << i + 1 << "_" << rotor_name << " " << p_rotor.transpose() <<  ", interfere with inter_joint" << j+1 << ": " << p_inter.transpose());
                }
            }

          if(j == i) // self overlap never
            {
              if(overlap_inter)
                {
                  overlap_positions_.push_back(p_inter);
                  overlap_weights_.push_back(1);
                  overlap_segments_.push_back(std::string("inter_joint") + s);
                  overlap_rotors_.push_back(std::string("rotor") + std::to_string(i + 1) + std::string("_") + rotor_name);
                }
              continue;
            }

          // case2: rotor
          Eigen::Vector3d p_rotor_l = aerial_robot_model::kdlToEigen((cog_inv * seg_tf_map.at(std::string("edf") + s + std::string("_left"))).p);
          Eigen::Vector3d p_rotor_r = aerial_robot_model::kdlToEigen((cog_inv * seg_tf_map.at(std::string("edf") + s + std::string("_right"))).p);
          bool overlap_rotor = false;
          double linear_rotor_weight = 0;

          if(p_rotor.z() > p_rotor_l.z() && p_rotor.z() > p_rotor_r.z())
            {
              Eigen::Vector3d p_projected_rotor_l = p_rotor + (p_rotor_l.z() - p_rotor.z()) / u_rotor.z() * u_rotor;
              Eigen::Vector3d p_projected_rotor_r = p_rotor + (p_rotor_r.z() - p_rotor.z()) / u_rotor.z() * u_rotor;
              double dist_rotor_l = (p_projected_rotor_l - p_rotor_l).norm();
              double dist_rotor_r = (p_projected_rotor_r - p_rotor_r).norm();

              if(dist_rotor_l < overlap_dist_rotor_relax_thresh_ || dist_rotor_r < overlap_dist_rotor_relax_thresh_)
                {
                  // ROS_INFO_STREAM("p_projected_rotor_l: " << p_projected_rotor_l.transpose() << "; p_projected_rotor_r: " << p_projected_rotor_r.transpose());
                  // ROS_INFO_STREAM("p_rotor_l: " << p_rotor_l.transpose() << "; p_rotor_r: " << p_rotor_r.transpose());
                  // ROS_INFO_STREAM("dist_rotor_l: " << dist_rotor_l << "; dist_rotor_r: " << dist_rotor_r);
                  double rotor_l_weight = 1;
                  double rotor_r_weight = 1;


                  double relax_range = overlap_dist_rotor_relax_thresh_ - overlap_dist_rotor_thresh_;
                  if(dist_rotor_l > overlap_dist_rotor_thresh_)
                    {
                      if(dist_rotor_l > overlap_dist_rotor_relax_thresh_) rotor_l_weight = 0;
                      else
                        {
                          if(overlap_dist_rotor_relax_thresh_ > overlap_dist_rotor_thresh_)
                            {
                              double diff = (overlap_dist_rotor_relax_thresh_ - dist_rotor_l) / relax_range;
                              rotor_l_weight = diff * diff ;
                            }
                        }
                    }
                  if(dist_rotor_r > overlap_dist_rotor_thresh_)
                    {
                      if(dist_rotor_r > overlap_dist_rotor_relax_thresh_) rotor_r_weight = 0;
                      else
                        {
                          if(overlap_dist_rotor_relax_thresh_ > overlap_dist_rotor_thresh_)
                            {
                              double diff = (overlap_dist_rotor_relax_thresh_ - dist_rotor_r) / relax_range;
                              rotor_r_weight = diff * diff ;
                            }
                        }
                    }

                  if(dist_rotor_l > overlap_dist_rotor_relax_thresh_)
                    {
                      linear_rotor_weight = (overlap_dist_rotor_relax_thresh_ - dist_rotor_r) / overlap_dist_rotor_relax_thresh_;
                      overlap_positions_.push_back(p_rotor_r);
                      overlap_weights_.push_back(rotor_r_weight);
                      overlap_segments_.push_back(std::string("rotor") + s + std::string("_right"));
                      overlap_rotors_.push_back(std::string("rotor") + std::to_string(i + 1) + std::string("_") + rotor_name);


                      /// ROS_INFO_STREAM("rotor" << i + 1 << "_" << rotor_name << " " << p_rotor.transpose() << ", interfere with rotor" << j+1 << "right: " << overlap_positions_.back().transpose());
                    }
                  else if(dist_rotor_r > overlap_dist_rotor_relax_thresh_)
                    {
                      linear_rotor_weight = (overlap_dist_rotor_relax_thresh_ - dist_rotor_l) / overlap_dist_rotor_relax_thresh_;
                      overlap_positions_.push_back(p_rotor_l);
                      overlap_weights_.push_back(rotor_l_weight);

                      overlap_segments_.push_back(std::string("rotor") + s + std::string("_left"));
                      overlap_rotors_.push_back(std::string("rotor") + std::to_string(i + 1) + std::string("_") + rotor_name);

                      /// ROS_INFO_STREAM("rotor" << i + 1 << "_" << rotor_name << " " << p_rotor.transpose() << ", interfere with rotor" << j+1 << "left: " << overlap_positions_.back().transpose());
                    }
                  else
                    {
                      linear_rotor_weight = (overlap_dist_rotor_relax_thresh_ - (dist_rotor_l + dist_rotor_r) / 2) / overlap_dist_rotor_relax_thresh_;
                      overlap_positions_.push_back((rotor_l_weight * p_rotor_l + rotor_r_weight * p_rotor_r) / (rotor_l_weight + rotor_r_weight));
                      overlap_weights_.push_back((rotor_l_weight + rotor_r_weight) / 2);

                      overlap_segments_.push_back(std::string("rotor") + s + std::string("_left&right"));
                      overlap_rotors_.push_back(std::string("rotor") + std::to_string(i + 1) + std::string("_") + rotor_name);

                      /// ROS_INFO_STREAM("rotor" << i + 1 << "_" << rotor_name << " " << p_rotor.transpose() << ", interfere with rotor" << j+1 << "left&right: " << overlap_positions_.back().transpose());
                    }

                  overlap_rotor = true;
                }
            }

          // case3: link
          Eigen::Matrix2d A; A << u_link.dot(u_link), -u_link.dot(u_rotor), u_link.dot(u_rotor), - u_rotor.dot(u_rotor);
          Eigen::Vector2d diff(-u_link.dot(p_link - p_rotor), -u_rotor.dot(p_link - p_rotor));
          Eigen::Vector2d t = A.inverse() * diff;
          double dist_link = 1e6;
          bool overlap_link = false;

          Eigen::Vector3d p_link_overlap = p_link + u_link * t(0);
          dist_link = (p_link_overlap - (p_rotor + u_rotor * t(1))).norm();

          if(t(1) < 0)
            {

              if(dist_link < overlap_dist_link_relax_thresh_)
                {
                  overlap_link = true;

                  if(t(0) < 0)
                    {
                      if(j == 0 && t(0) > overlap_dist_link_relax_thresh_) overlap_link = true; // relax for the end of the link
                      else overlap_link = false;
                    }
                  if(t(0) > link_length)
                    {
                      if(j == motor_num_ - 1 && t(0) < link_length + overlap_dist_link_relax_thresh_) overlap_link = true; // relax for the end of the link
                      else overlap_link = false;
                    }
                }

              if(overlap_link)
                {
                  double linear_link_weight = (overlap_dist_link_relax_thresh_ - dist_link) / overlap_dist_link_relax_thresh_;

                  double weight = 1;
                  if(dist_link > overlap_dist_link_thresh_)
                    {
                      double diff = (overlap_dist_link_relax_thresh_ - dist_link) / (overlap_dist_link_relax_thresh_ - overlap_dist_link_thresh_);
                      weight = diff * diff ;
                    }

                  if(overlap_rotor)
                    {
                      Eigen::Vector3d p_rotor_overlap = overlap_positions_.back();
                      double weight_rotor_overlap = overlap_weights_.back();
                      overlap_positions_.back() = (linear_rotor_weight * p_rotor_overlap + linear_link_weight * p_link_overlap) / (linear_rotor_weight + linear_link_weight);
                      overlap_weights_.back() = (weight_rotor_overlap + weight) / 2;

                      overlap_segments_.back() = overlap_segments_.back() + std::string("&link");

                      /// ROS_INFO_STREAM("  rotor" << i + 1 << "_" << rotor_name << " " << p_rotor.transpose() << ", interfere with link" << j+1 << "&rotor : " << overlap_positions_.back().transpose());
                    }
                  else
                    {

                      if(overlap_inter)
                        {
                          double linear_inter_weight = (overlap_dist_inter_joint_thresh_ - dist_inter) / overlap_dist_inter_joint_thresh_;
                          overlap_positions_.push_back((linear_inter_weight * p_inter + linear_link_weight * p_link_overlap) / (linear_inter_weight + linear_link_weight));
                          overlap_weights_.push_back(weight);

                          overlap_segments_.push_back(std::string("link&inter_joint") + s);
                          overlap_rotors_.push_back(std::string("rotor") + std::to_string(i + 1) + std::string("_") + rotor_name);

                          /// ROS_INFO_STREAM(" rotor" << i + 1 << "_" << rotor_name << " " << p_rotor.transpose() << ", interfere with link & inter_joint" << j+1 << ": " << overlap_positions_.back().transpose());
                        }
                      else
                        {
                          overlap_positions_.push_back(p_link_overlap);
                          overlap_weights_.push_back(weight);

                          overlap_segments_.push_back(std::string("link") + s);
                          overlap_rotors_.push_back(std::string("rotor") + std::to_string(i + 1) + std::string("_") + rotor_name);

                          /// ROS_INFO_STREAM("rotor" << i + 1 << "_" << rotor_name << " " << p_rotor.transpose() << ", interfere with link" << j+1 << ": " << overlap_positions_.back().transpose());
                        }
                    }
                }

            }
        }
    };

  for(int i = 0; i < motor_num_; ++i)
    {
      std::string s = std::to_string(i + 1);
      KDL::Frame f_rotor  = cog_inv * seg_tf_map.at(std::string("edf") + s + std::string("_left"));
      rotorInterfere(i, aerial_robot_model::kdlToEigen(f_rotor.p), u.at(i), link_length, std::string("left"));
      f_rotor  = cog_inv * seg_tf_map.at(std::string("edf") + s + std::string("_right"));
      rotorInterfere(i, aerial_robot_model::kdlToEigen(f_rotor.p), u.at(i), link_length, std::string("right"));
    }


  if(overlap_positions_.size() == 0)
    {
      fz_bias_ = (1 - wrench_lpf_rate_) * fz_bias_ + wrench_lpf_rate_ * est_external_wrench_(2);
      tx_bias_ = (1 - wrench_lpf_rate_) * tx_bias_ + wrench_lpf_rate_ * est_external_wrench_(3);
      ty_bias_ = (1 - wrench_lpf_rate_) * ty_bias_ + wrench_lpf_rate_ * est_external_wrench_(4);

      rotor_interfere_comp_wrench_.segment(2, 3) = (1 - comp_wrench_lpf_rate_) * rotor_interfere_comp_wrench_.segment(2, 3) +  comp_wrench_lpf_rate_ * Eigen::VectorXd::Zero(3);
    }
  else
    {
      Eigen::Vector3d external_wrench(est_external_wrench_(2), est_external_wrench_(3) - tx_bias_, est_external_wrench_(4) - ty_bias_); //fz, mx,my
      if(fz_bias_thresh_ < fabs(fz_bias_)) external_wrench(0) -= fz_bias_;

      /// ROS_WARN_STREAM("compensate rotor overlap interfere: fz_bias: " << fz_bias_ << " external wrench: " << external_wrench.transpose());

      if(external_wrench(0) >= 0)
        {
          rotor_interfere_comp_wrench_.segment(2, 3) = (1 - comp_wrench_lpf_rate_) * rotor_interfere_comp_wrench_.segment(2, 3) +  comp_wrench_lpf_rate_ * Eigen::VectorXd::Zero(3);
          rotor_interfere_force_.setZero(0);
          return;
        }

      // for(int i = 0; i < overlap_rotors_.size(); i++) std::cout << overlap_rotors_.at(i) << " -> " << overlap_segments_.at(i) << "; ";
      // std::cout << std::endl;
      // for(int i = 0; i < overlap_positions_.size(); i++) std::cout << overlap_positions_.at(i).transpose() << "; ";
      // std::cout << std::endl;

      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, overlap_positions_.size());
      for(int j = 0; j < overlap_positions_.size(); j++)
        A.col(j) = Eigen::Vector3d(1, overlap_positions_.at(j).y(), -overlap_positions_.at(j).x());

      Eigen::MatrixXd dev_weight_mat = Eigen::MatrixXd::Identity(A.cols(), A.cols()) - Eigen::MatrixXd::Ones(A.cols(), A.cols())/A.cols();
      Eigen::MatrixXd W_dev = Eigen::MatrixXd::Identity(A.cols(), A.cols());
      for(int j = 0; j < overlap_weights_.size(); j++)
        W_dev(j,j) = overlap_weights_.at(j);

      Eigen::MatrixXd W_diff = rotor_interfere_torque_xy_weight_ * Eigen::MatrixXd::Identity(A.rows(), A.rows());
      W_diff(0,0) = 1;
      rotor_interfere_force_ = (A.transpose() * W_diff * A  + rotor_interfere_force_dev_weight_ * dev_weight_mat * W_dev * dev_weight_mat).inverse() * A.transpose() * W_diff * external_wrench;

      if(rotor_interfere_force_.maxCoeff() > 0)
        {
          while(1)
            {
              std::vector<Eigen::VectorXd> overlap_positions_tmp = overlap_positions_;
              std::vector<double> overlap_weights_tmp = overlap_weights_;
              std::vector<std::string> overlap_segments_tmp = overlap_segments_;
              std::vector<std::string> overlap_rotors_tmp = overlap_rotors_;
              overlap_positions_.clear();
              overlap_weights_.clear();
              overlap_segments_.clear();
              overlap_rotors_.clear();

              for(int i = 0; i < rotor_interfere_force_.size(); i++)
                {
                  if(rotor_interfere_force_(i) < 0)
                    {
                      overlap_positions_.push_back(overlap_positions_tmp.at(i));
                      overlap_weights_.push_back(overlap_weights_tmp.at(i));
                      overlap_segments_.push_back(overlap_segments_tmp.at(i));
                      overlap_rotors_.push_back(overlap_rotors_tmp.at(i));
                    }
                }
              if(overlap_positions_.size() == 0)
                {
                  rotor_interfere_comp_wrench_.segment(2, 3) = (1 - comp_wrench_lpf_rate_) * rotor_interfere_comp_wrench_.segment(2, 3) +  comp_wrench_lpf_rate_ * Eigen::VectorXd::Zero(3);
                  rotor_interfere_force_.setZero();
                  ROS_DEBUG_STREAM("no rotor_interfere from recalculate");
                  break;
                }
              else
                {
                  /// ROS_WARN_STREAM("rotor_interfere force before recalculate: " << rotor_interfere_force_.transpose());
                  A = Eigen::MatrixXd::Zero(3, overlap_positions_.size());
                  for(int j = 0; j < overlap_positions_.size(); j++)
                    A.col(j) = Eigen::Vector3d(1, overlap_positions_.at(j).y(), -overlap_positions_.at(j).x());

                  Eigen::MatrixXd dev_weight_mat = Eigen::MatrixXd::Identity(A.cols(), A.cols()) - Eigen::MatrixXd::Ones(A.cols(), A.cols())/A.cols();
                  Eigen::MatrixXd W_dev = Eigen::MatrixXd::Identity(A.cols(), A.cols());
                  for(int j = 0; j < overlap_weights_.size(); j++)
                    W_dev(j,j) = overlap_weights_.at(j);

                  Eigen::MatrixXd W_diff = rotor_interfere_torque_xy_weight_ * Eigen::MatrixXd::Identity(A.rows(), A.rows());
                  W_diff(0,0) = 1;
                  rotor_interfere_force_ = (A.transpose() * W_diff * A  + rotor_interfere_force_dev_weight_ * dev_weight_mat * W_dev * dev_weight_mat).inverse() * A.transpose() * W_diff * external_wrench;


                  if(rotor_interfere_force_.maxCoeff() > 0)
                    {
                      ROS_DEBUG_STREAM("invalid rotor_interfere force: " << rotor_interfere_force_.transpose()); // loop
                    }
                  else
                    {
                      ROS_DEBUG_STREAM("rotor_interfere force recalculate: " << rotor_interfere_force_.transpose());
                      break;
                    }
                }
            }
        }
      else
        ROS_DEBUG_STREAM("rotor_interfere force: " << rotor_interfere_force_.transpose());

      //rotor_interfere_comp_wrench_.segment(2, 3) = (1 - comp_wrench_lpf_rate_) * rotor_interfere_comp_wrench_.segment(2, 3) +  comp_wrench_lpf_rate_ * (- A * rotor_interfere_force_);
      rotor_interfere_comp_wrench_.segment(2, 3) =  - A * rotor_interfere_force_;

      ROS_DEBUG_STREAM("rotor_interfere_wrench: " << -rotor_interfere_comp_wrench_.segment(2, 3).transpose());
    }

  // visualize the interference
  visualization_msgs::MarkerArray interference_marker_msg;
  if(overlap_positions_.size() > 0)
    {
      int id = 0;
      for(int i = 0; i < overlap_positions_.size(); i++)
        {
          visualization_msgs::Marker segment_sphere;
          segment_sphere.header.stamp = ros::Time::now();
          segment_sphere.header.frame_id = nh_.getNamespace() + std::string("/cog"); //overlap_segments_.at(i);
          segment_sphere.id = id++;
          segment_sphere.action = visualization_msgs::Marker::ADD;
          segment_sphere.type = visualization_msgs::Marker::SPHERE;
          segment_sphere.pose.position.x = overlap_positions_.at(i).x();
          segment_sphere.pose.position.y = overlap_positions_.at(i).y();
          segment_sphere.pose.position.z = overlap_positions_.at(i).z();
          segment_sphere.pose.orientation.w = 1;
          segment_sphere.scale.x = 0.15;
          segment_sphere.scale.y = 0.15;
          segment_sphere.scale.z = 0.15;
          segment_sphere.color.g = 1.0;
          segment_sphere.color.a = 0.5;

          interference_marker_msg.markers.push_back(segment_sphere);

          visualization_msgs::Marker force_arrow;
          force_arrow.header.stamp = ros::Time::now();
          force_arrow.header.frame_id = nh_.getNamespace() + std::string("/cog"); //overlap_segments_.at(i);
          force_arrow.id = id++;
          force_arrow.action = visualization_msgs::Marker::ADD;
          force_arrow.type = visualization_msgs::Marker::ARROW;
          force_arrow.pose.position.x = overlap_positions_.at(i).x();
          force_arrow.pose.position.y = overlap_positions_.at(i).y();
          force_arrow.pose.position.z = overlap_positions_.at(i).z() - 0.02;
          force_arrow.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI/2, 0);
          force_arrow.scale.x = rotor_interfere_force_(i) / 10.0;
          force_arrow.scale.y = 0.02;
          force_arrow.scale.z = 0.02;
          force_arrow.color.g = 1.0;
          force_arrow.color.a = 0.5;

          interference_marker_msg.markers.push_back(force_arrow);
        }

      // remove the old marker
      visualization_msgs::Marker delete_operation;
      delete_operation.id = id++;
      delete_operation.action = visualization_msgs::Marker::DELETE;
      interference_marker_msg.markers.push_back(delete_operation);
      delete_operation.id = id++;
      interference_marker_msg.markers.push_back(delete_operation);
    }
  else
    {
      visualization_msgs::Marker delete_operation;
      delete_operation.action = visualization_msgs::Marker::DELETEALL;
      interference_marker_msg.markers.push_back(delete_operation);
    }

  interfrence_marker_pub_.publish(interference_marker_msg);
}

void DragonFullVectoringController::controlCore()
{
  /* TODO: saturation of z control */
  PoseLinearController::controlCore();

  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
  target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();
  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();


  if(navigator_->getForceLandingFlag() && target_acc_w.z() < 5.0) // heuristic measures to avoid to large gimbal angles after force land
    start_rp_integration_ = false;

  Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  double mass_inv =  1 / robot_model_->getMass();

  // rotor interference compensation
  rotorInterfereCompensation();

  Eigen::VectorXd rotor_interfere_comp_acc = Eigen::VectorXd::Zero(6);
  rotor_interfere_comp_acc(2) = mass_inv * rotor_interfere_comp_wrench_(2);

  bool torque_comp = false;
  if(overlap_positions_.size() == 1)
    {
      ROS_INFO_STREAM("compsensate the torque resulted from rotor interference: " << overlap_rotors_.at(0) << " to " << overlap_segments_.at(0));
      torque_comp = true;
    }

  if(overlap_positions_.size() == 2)
    {
      if(overlap_rotors_.at(0).substr(0, 6) == overlap_rotors_.at(1).substr(0, 6))
        {
          ROS_INFO_STREAM("do rotor interference torque compensation: " << overlap_rotors_.at(0) << " and " << overlap_rotors_.at(1));
          torque_comp = true;
        }
    }

  if(torque_comp)
    {
      rotor_interfere_comp_acc.tail(3) = inertia_inv * rotor_interfere_comp_wrench_.tail(3);
    }

  if(rotor_interfere_compensate_) // TODO move this scope
    target_wrench_acc_cog += rotor_interfere_comp_acc;

  std::stringstream ss;
  for(int i = 0; i < overlap_rotors_.size(); i++) ss << overlap_rotors_.at(i) << " -> " << overlap_segments_.at(i) << "; ";
  if(overlap_rotors_.size() > 0) ROS_DEBUG_STREAM("rotor interference: " << ss.str());


  // external wrench compensation
  // TODO: should be included in every iteration
  Eigen::MatrixXd cog_rot_inv = aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(rpy_.x(), rpy_.y(), rpy_.z()).Inverse());
  Eigen::MatrixXd extended_cog_rot_inv = Eigen::MatrixXd::Zero(6, 6);
  extended_cog_rot_inv.topLeftCorner(3,3) = cog_rot_inv;
  extended_cog_rot_inv.bottomRightCorner(3,3) = cog_rot_inv;
  std::map<std::string, Dragon::ExternalWrench> external_wrench_map = dragon_robot_model_->getExternalWrenchMap();
  for(auto& wrench: external_wrench_map) wrench.second.wrench = extended_cog_rot_inv * wrench.second.wrench;
  Eigen::VectorXd sum_external_wrench = dragon_robot_model_->calcExternalWrenchSum(external_wrench_map);
  Eigen::VectorXd sum_external_acc = Eigen::VectorXd::Zero(6);
  sum_external_acc.head(3) = mass_inv * sum_external_wrench.head(3);
  sum_external_acc.tail(3) = inertia_inv * sum_external_wrench.tail(3);
  target_wrench_acc_cog += sum_external_acc;


  setTargetWrenchAccCog(target_wrench_acc_cog);
  // TODO: we need to compensate a nonlinear term w x (Jw) for rotational motion.
  // solution1: using the raw angular velocity: https://ieeexplore.ieee.org/document/5717652, problem is the noisy of raw omega
  // solution2: using the target angular velocity: https://ieeexplore.ieee.org/document/6669644, problem is the larget gap between true  omega and target one. but this paper claim this is oK
  // solution3: ignore this term for low angular motion <- current is this

  // iteratively find the target force and target gimbal angles
  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_control_->setCogDesireOrientation(cog_desire_orientation); // update the cog orientation
  KDL::JntArray gimbal_processed_joint = dragon_robot_model_->getJointPositions();
  robot_model_for_control_->updateRobotModel(gimbal_processed_joint);

  roll_locked_gimbal_ = dragon_robot_model_->getRollLockedGimbal();
  gimbal_nominal_angles_ = dragon_robot_model_->getGimbalNominalAngles();
  const auto& joint_index_map = dragon_robot_model_->getJointIndexMap();

  // Note: the update of dragon_robot_model_ may be (absolutely in gazebo) slowed than robot_model_for_control_.
  //       so we can not trust the kinematics updated by dragon_robot_model_, only trust robot_model_for_control_.
  //       do not use getLinksRotationFromCog() from dragon_robot_model_.
  // const auto links_rotation_from_cog = dragon_robot_model_->getLinksRotationFromCog<Eigen::Matrix3d>();
  const auto seg_tf_map = robot_model_for_control_->getSegmentsTf();
  const KDL::Rotation cog_rot = seg_tf_map.at(robot_model_for_control_->getBaselinkName()).M * cog_desire_orientation.Inverse();
  std::vector<Eigen::Matrix3d> links_rotation_from_cog; // workaround to solve the sync issue of kinematics update
  for(int i = 0; i < motor_num_; i++)
    {
      std::string name = std::string("link") + std::to_string(i + 1);
      links_rotation_from_cog.push_back(kdlToEigen(cog_rot.Inverse() * seg_tf_map.at(name).M));
    }


  int gimbal_lock_num = std::accumulate(roll_locked_gimbal_.begin(), roll_locked_gimbal_.end(), 0);
  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3 * motor_num_ - gimbal_lock_num);

  std::vector<float> init_thrust_force;
  std::vector<double> init_gimbal_angles;

  double t = ros::Time::now().toSec();
  for(int j = 0; j < allocation_refine_max_iteration_; j++)
    {
      /* 5.2.1. update the wrench allocation matrix  */
      std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_for_control_->getRotorsOriginFromCog<Eigen::Vector3d>();

      Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
      wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
      Eigen::MatrixXd mask(3,2);
      mask << 1, 0, 0, 0, 0, 1;
      int last_col = 0;
      for(int i = 0; i < motor_num_; i++)
        {
          wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i));

          if(roll_locked_gimbal_.at(i) == 0)
            {
              /* 3DoF */
              full_q_mat.middleCols(last_col, 3) = wrench_map * links_rotation_from_cog.at(i);
              last_col += 3;
            }
          else
            {
              /* gimbal lock: 2Dof */
              full_q_mat.middleCols(last_col, 2) = wrench_map * links_rotation_from_cog.at(i) * aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(gimbal_nominal_angles_.at(i * 2), 0, 0)) * mask;
              last_col += 2;
            }
        }

      // TODO: check!!
      inertia_inv = robot_model_for_control_->getInertia<Eigen::Matrix3d>().inverse(); // update
      full_q_mat.topRows(3) =  mass_inv * full_q_mat.topRows(3) ;
      full_q_mat.bottomRows(3) =  inertia_inv * full_q_mat.bottomRows(3);
      Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat);
      target_vectoring_f_ = full_q_mat_inv * target_wrench_acc_cog;

      if(control_verbose_) ROS_DEBUG_STREAM("vectoring force for control in iteration "<< j+1 << ": " << target_vectoring_f_.transpose());
      last_col = 0;
      for(int i = 0; i < motor_num_; i++)
        {
          if(roll_locked_gimbal_.at(i) == 0)
            {
              Eigen::Vector3d f_i = target_vectoring_f_.segment(last_col, 3);
              target_base_thrust_.at(i) = f_i.norm();

              /* before takeoff, the form is level -> joint_pitch = 0*/
              if(!start_rp_integration_)
                f_i.z() = dragon_robot_model_->getHoverVectoringF()[last_col + 2]; // approximation: stable state

              double gimbal_i_roll = atan2(-f_i.y(), f_i.z());
              double gimbal_i_pitch = atan2(f_i.x(), -f_i.y() * sin(gimbal_i_roll) + f_i.z() * cos(gimbal_i_roll));

              target_gimbal_angles_.at(2 * i) = gimbal_i_roll;
              target_gimbal_angles_.at(2 * i + 1) = gimbal_i_pitch;

              last_col += 3;
            }
          else
            {
              Eigen::VectorXd f_i = target_vectoring_f_.segment(last_col, 2);
              target_base_thrust_.at(i) = f_i.norm();

              /* before takeoff, the form is level -> joint_pitch = 0*/
              if(!start_rp_integration_)
                f_i(1) = dragon_robot_model_->getHoverVectoringF()[last_col + 1]; // approximation: stable state

              target_gimbal_angles_.at(2 * i) = gimbal_nominal_angles_.at(2 * i); // lock the gimbal roll
              target_gimbal_angles_.at(2 * i + 1) = atan2(f_i(0), f_i(1));

              last_col += 2;
            }
        }

      std::vector<Eigen::Vector3d> prev_rotors_origin_from_cog = rotors_origin_from_cog;
      for(int i = 0; i < motor_num_; ++i)
        {
          std::string s = std::to_string(i + 1);
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = target_gimbal_angles_.at(i * 2);
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = target_gimbal_angles_.at(i * 2 + 1);
        }
      robot_model_for_control_->updateRobotModel(gimbal_processed_joint);
      rotors_origin_from_cog = robot_model_for_control_->getRotorsOriginFromCog<Eigen::Vector3d>();

      if (j == 0)
        {
          init_thrust_force = target_base_thrust_;
          init_gimbal_angles = target_gimbal_angles_;
        }

      double max_diff = 1e-6;
      for(int i = 0; i < motor_num_; i++)
        {
          double diff = (rotors_origin_from_cog.at(i) - prev_rotors_origin_from_cog.at(i)).norm();
          if(diff > max_diff) max_diff = diff;
        }

      if(control_verbose_) ROS_DEBUG_STREAM("refine rotor origin in control: iteration "<< j+1 << ", max_diff: " << max_diff);

      if(max_diff < allocation_refine_threshold_)
        {
          if(control_verbose_) ROS_INFO_STREAM("refine rotor origin in control: converge in iteration " << j+1 << " max_diff " << max_diff << ", use " << ros::Time::now().toSec() - t << "sec");
          break;
        }

      if(j == allocation_refine_max_iteration_ - 1)
        {
          ROS_WARN_STREAM("refine rotor origin in control: can not converge in iteration " << j+1 << " max_diff " << max_diff);
        }
    }

#if 1
  // Alternative method: Calcualte strictly from nonlinear optimization:
  // Note: please do setGimbalNominalAngles(), setRollLockedGimbal() first, and also update robot_model_for_plan first.
  std::vector<double> thrust_force_gimbal_angles(motor_num_ * 3  - gimbal_lock_num, 0); // thrust_force: rotor_num + gimbal_angles: rotor_num x 2
  Eigen::VectorXd static_thrust =  dragon_robot_model_->getStaticThrust();

  // initialize
  int col = 0;
  for(int i = 0; i < motor_num_; i++)
    {
      // thrust force
      // thrust_force_gimbal_angles.at(i) = target_wrench_acc_cog(2) * robot_model_->getMass() / motor_num_; // from hovering force
      thrust_force_gimbal_angles.at(i) = init_thrust_force.at(i); // from iteration 1 of static allocation

      if(roll_locked_gimbal_.at(i) == 0)
        {
          // from approximate hovering condition
          // thrust_force_gimbal_angles.at(motor_num_ + col) = gimbal_nominal_angles_.at(2 * i); // gimbal roll
          // thrust_force_gimbal_angles.at(motor_num_ + col + 1) = gimbal_nominal_angles_.at(2 * i + 1); // gimbal pitch

          thrust_force_gimbal_angles.at(motor_num_ + col) = init_gimbal_angles.at(2 * i); // gimbal roll
          thrust_force_gimbal_angles.at(motor_num_ + col + 1) = init_gimbal_angles.at(2 * i + 1); // gimbal pitch

          col += 2;
        }
      else
        {
          // from approximate hovering condition
          // thrust_force_gimbal_angles.at(motor_num_ + col) = gimbal_nominal_angles_.at(2 * i + 1); // gimbal pitch
          thrust_force_gimbal_angles.at(motor_num_ + col) = init_gimbal_angles.at(2 * i + 1); // gimbal pitch
          col += 1;
        }
    }

  // calculate sqp, check whether the inverse allocation equals to the desired wrench.
  double thrust_force_sum = 0;
  cnt = 0;
  double start_time = ros::Time::now().toSec();
  nlopt::opt full_vectoring_allocation_solver(nlopt::LD_SLSQP, motor_num_ * 3 - gimbal_lock_num);
  full_vectoring_allocation_solver.set_xtol_rel(1e-4); // => IMPORTANT: related to computation time
  full_vectoring_allocation_solver.set_maxeval(100); // 100 times

  std::vector<double>lower_bounds(motor_num_ * 3  - gimbal_lock_num, -1e6);
  nominal_inertia_ = robot_model_->getInertia<Eigen::Matrix3d>();
  for(int i = 0; i < motor_num_; i++) lower_bounds.at(i) = 0;
  full_vectoring_allocation_solver.set_lower_bounds(lower_bounds);
  full_vectoring_allocation_solver.set_upper_bounds(std::vector<double>(motor_num_ * 3 - gimbal_lock_num, 1e6)); // TODO: no bounds
  full_vectoring_allocation_solver.set_min_objective(thrustSumFunc, robot_model_for_control_.get());
  std::vector<double>tol(6, 1e-5); // => IMPORTANT: related to computation time (1e-8 is too stric)
  full_vectoring_allocation_solver.add_equality_mconstraint(wrenchAllocationEqCons, this, tol);
  auto result = full_vectoring_allocation_solver.optimize(thrust_force_gimbal_angles, thrust_force_sum);
  for(int i = 0; i < thrust_force_gimbal_angles.size() - motor_num_; i++)
    {
      double val = thrust_force_gimbal_angles.at(i + motor_num_);
      if(val > M_PI) thrust_force_gimbal_angles.at(i + motor_num_) -= 2 * M_PI;
      if(val < -M_PI) thrust_force_gimbal_angles.at(i + motor_num_) += 2 * M_PI;
    }
  prev_thrust_force_gimbal_angles_ = thrust_force_gimbal_angles;

  // evaluate result
  {
    std::stringstream ss_orig;
    std::stringstream ss_nonlinear;
    ss_orig << "thrust: ";
    ss_nonlinear << "thrust: ";
    double orig_thrust_sum = 0;
    double nonlinear_thrust_sum = 0;
    for(int i = 0; i < motor_num_; i++)
      {
        ss_orig << target_base_thrust_.at(i) << ", ";
        orig_thrust_sum += (target_base_thrust_.at(i) * target_base_thrust_.at(i));
        ss_nonlinear << thrust_force_gimbal_angles.at(i) << ", ";
        nonlinear_thrust_sum += (thrust_force_gimbal_angles.at(i) * thrust_force_gimbal_angles.at(i));
      }
    ss_orig << "gimbal: ";
    ss_nonlinear << "gimbal: ";
    col = 0;
    for(int i = 0; i < motor_num_; i++)
      {
        ss_orig << "(" << target_gimbal_angles_.at(2 * i) << ", " << target_gimbal_angles_.at(2 * i + 1) << ") ";

        if(roll_locked_gimbal_.at(i) == 0)
          {
            ss_nonlinear << "(" << thrust_force_gimbal_angles.at(motor_num_ + col) << ", " << thrust_force_gimbal_angles.at(motor_num_ + col + 1) << ") ";
            col += 2;
          }
        else
          { // roll is locked
            ss_nonlinear << "(" << gimbal_nominal_angles_.at(2 * i)  << ", " << thrust_force_gimbal_angles.at(motor_num_ + col) << ") ";
            col += 1;
          }
      }

    if(ave_cnt == 0) ave_cnt = cnt;
    else{
      ave_cnt = 0.6 * ave_cnt + 0.4 * cnt;
    }


    ROS_INFO_STREAM_THROTTLE(1.0, "constraint result: " <<  result << "; average count: " << ave_cnt << "; time: " << ros::Time::now().toSec() - start_time
                             <<  "\n original result: " << ss_orig.str() << "; " << orig_thrust_sum
                             << "\n nonlinear result: " << ss_nonlinear.str() << "; " << nonlinear_thrust_sum);

    // debug
    if(cnt > 50)
      {
        ROS_WARN_STREAM("constraint result: " <<  result << "; count: " << cnt << "; time: " << ros::Time::now().toSec() - start_time
                        <<  "\n original result: " << ss_orig.str() << "; " << orig_thrust_sum
                        << "\n nonlinear result: " << ss_nonlinear.str() << "; " << nonlinear_thrust_sum);
      }

    // check the allocation
    gimbal_processed_joint = robot_model_for_control_->getJointPositions();
    col = 0;
    for(int i = 0; i < motor_num_; i++)
      {
        std::string s = std::to_string(i + 1);
        if(roll_locked_gimbal_.at(i) == 0)
          {
            gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = thrust_force_gimbal_angles[motor_num_ + col];
            gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = thrust_force_gimbal_angles[motor_num_ + col + 1];
            col += 2;
          }
        else
          {
            gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = thrust_force_gimbal_angles[motor_num_ + col];
            col +=1;
          }
      }
    robot_model_for_control_->updateRobotModel(gimbal_processed_joint);

    Eigen::MatrixXd Q = robot_model_for_control_->calcWrenchMatrixOnCoG();
    Eigen::VectorXd target_wrench_cog = Eigen::VectorXd::Zero(6);
    target_wrench_cog.head(3) = robot_model_for_control_->getMass() * target_wrench_acc_cog.head(3);
    target_wrench_cog.tail(3) = robot_model_for_control_->getInertia<Eigen::Matrix3d>() * target_wrench_acc_cog.tail(3);
    Eigen::VectorXd wrench_diff = Q * Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(thrust_force_gimbal_angles.data(), motor_num_) - target_wrench_cog;

    if(wrench_diff.cwiseAbs().maxCoeff() > 1e-4)
      {
        ROS_WARN_STREAM("The nonlinear full vectoring allocation has bad result with wrench diff of " << wrench_diff.cwiseAbs().maxCoeff() << "; vector: " << wrench_diff.transpose() << "; count: " << cnt << "; target wrench:" << target_wrench_cog.transpose()  <<  "\n Q: \n" << Q);
        Eigen::VectorXd result = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(thrust_force_gimbal_angles.data(), thrust_force_gimbal_angles.size());
        ROS_WARN_STREAM("original result: " << ss_orig.str() << "; " << orig_thrust_sum  << "\n nonlinear result: " << ss_nonlinear.str() << "; " << nonlinear_thrust_sum);
        std::cout << "roll_locked_gimbal_:";
        for(int i = 0; i < motor_num_; i++) std::cout << roll_locked_gimbal_.at(i) << ", ";
        std::cout << std::endl;

        if(wrench_diff.cwiseAbs().maxCoeff() > 1e-2)
          throw;
      }
  }

  // assign to control input
  col = 0;
  for(int i = 0; i < motor_num_; i++)
    {
      target_base_thrust_.at(i) = thrust_force_gimbal_angles.at(i);
      if(!start_rp_integration_)
        {
          target_gimbal_angles_.at(2 * i) = gimbal_nominal_angles_.at(2 * i);
          target_gimbal_angles_.at(2 * i + 1) = gimbal_nominal_angles_.at(2 * i + 1);
        }
      else
        {
          if(roll_locked_gimbal_.at(i) == 0)
            {
              target_gimbal_angles_.at(2 * i) = thrust_force_gimbal_angles.at(motor_num_ + col);
              target_gimbal_angles_.at(2 * i + 1) = thrust_force_gimbal_angles.at(motor_num_ + col + 1);
              col += 2;
            }
          else
            {
              target_gimbal_angles_.at(2 * i) = gimbal_nominal_angles_.at(2 * i); // lock the gimbal roll
              target_gimbal_angles_.at(2 * i + 1) = thrust_force_gimbal_angles.at(motor_num_ + col);

              col += 1;
            }
        }
    }

#endif
}

void DragonFullVectoringController::externalWrenchEstimate()
{
  if(navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE &&
     navigator_->getNaviState() != aerial_robot_navigation::LAND_STATE)
    {
      prev_est_wrench_timestamp_ = 0;
      integrate_term_ = Eigen::VectorXd::Zero(6);
      return;
    }

  Eigen::Vector3d vel_w, omega_cog; // workaround: use the filtered value
  auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::DragonImu>(estimator_->getImuHandler(0));
  tf::vectorTFToEigen(imu_handler->getFilteredVelCog(), vel_w);
  tf::vectorTFToEigen(imu_handler->getFilteredOmegaCog(), omega_cog);
  Eigen::Matrix3d cog_rot;
  tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);

  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  double mass = robot_model_->getMass();

  Eigen::VectorXd sum_momentum = Eigen::VectorXd::Zero(6);
  sum_momentum.head(3) = mass * vel_w;
  sum_momentum.tail(3) = inertia * omega_cog;

  Eigen::MatrixXd J_t = Eigen::MatrixXd::Identity(6,6);
  J_t.topLeftCorner(3,3) = cog_rot;

  Eigen::VectorXd N = mass * robot_model_->getGravity();
  N.tail(3) = aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);

  const Eigen::VectorXd target_wrench_acc_cog = getTargetWrenchAccCog();
  Eigen::VectorXd target_wrench_cog = Eigen::VectorXd::Zero(6);
  target_wrench_cog.head(3) = mass * target_wrench_acc_cog.head(3);
  target_wrench_cog.tail(3) = inertia * target_wrench_acc_cog.tail(3);

  if(prev_est_wrench_timestamp_ == 0)
    {
      prev_est_wrench_timestamp_ = ros::Time::now().toSec();
      init_sum_momentum_ = sum_momentum; // not good
    }

  double dt = ros::Time::now().toSec() - prev_est_wrench_timestamp_;

  integrate_term_ += (J_t * target_wrench_cog - N + est_external_wrench_) * dt;

  est_external_wrench_ = momentum_observer_matrix_ * (sum_momentum - init_sum_momentum_ - integrate_term_);

  Eigen::VectorXd est_external_wrench_cog = est_external_wrench_;
  est_external_wrench_cog.head(3) = cog_rot.inverse() * est_external_wrench_.head(3);

  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
  wrench_msg.wrench.force.x = est_external_wrench_(0);
  wrench_msg.wrench.force.y = est_external_wrench_(1);
  wrench_msg.wrench.force.z = est_external_wrench_(2);
  wrench_msg.wrench.torque.x = est_external_wrench_(3);
  wrench_msg.wrench.torque.y = est_external_wrench_(4);
  wrench_msg.wrench.torque.z = est_external_wrench_(5);
  estimate_external_wrench_pub_.publish(wrench_msg);

  prev_est_wrench_timestamp_ = ros::Time::now().toSec();
}

/* external wrench */
void DragonFullVectoringController::addExternalWrenchCallback(const gazebo_msgs::ApplyBodyWrenchRequest::ConstPtr& msg)
{
  dragon_robot_model_->addExternalStaticWrench(msg->body_name, msg->reference_frame, msg->reference_point, msg->wrench);
}

bool DragonFullVectoringController::clearExternalWrenchCallback(gazebo_msgs::BodyRequest::Request& req, gazebo_msgs::BodyRequest::Response& res)
{
  dragon_robot_model_->removeExternalStaticWrench(req.body_name);
  return true;
}


void DragonFullVectoringController::sendCmd()
{
  PoseLinearController::sendCmd();

  /* send base throttle command */
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);

  /* send gimbal control command */
  sensor_msgs::JointState gimbal_control_msg;
  gimbal_control_msg.header.stamp = ros::Time::now();
  if (gimbal_vectoring_check_flag_)
    {
      gimbal_control_msg.position = dragon_robot_model_->getGimbalNominalAngles();
    }
  else
    {
      for(int i = 0; i < motor_num_ * 2; i++)
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
    }
  gimbal_control_pub_.publish(gimbal_control_msg);


  std_msgs::Float32MultiArray target_vectoring_force_msg;
  for(int i = 0; i < target_vectoring_f_.size(); i++)
    target_vectoring_force_msg.data.push_back(target_vectoring_f_(i));
  target_vectoring_force_pub_.publish(target_vectoring_force_msg);

  /* rotor interfere wrench */
  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
  wrench_msg.wrench.force.z = -rotor_interfere_comp_wrench_(2);
  wrench_msg.wrench.torque.x = -rotor_interfere_comp_wrench_(3);
  wrench_msg.wrench.torque.y = -rotor_interfere_comp_wrench_(4);
  rotor_interfere_wrench_pub_.publish(wrench_msg);


  sensor_msgs::Joy force_msg; // can only publish 3 elements
  force_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
  for(int i = 0; i < rotor_interfere_force_.size(); i++)
    force_msg.axes.push_back(rotor_interfere_force_(i));
}

void DragonFullVectoringController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "decoupling", decoupling_, false);
  getParam<bool>(control_nh, "gimbal_vectoring_check_flag", gimbal_vectoring_check_flag_, false);
  getParam<double>(control_nh, "allocation_refine_threshold", allocation_refine_threshold_, 0.01);
  getParam<int>(control_nh, "allocation_refine_max_iteration", allocation_refine_max_iteration_, 1);

  momentum_observer_matrix_ = Eigen::MatrixXd::Identity(6,6);
  double force_weight, torque_weight;
  getParam<double>(control_nh, "momentum_observer_force_weight", force_weight, 10.0);
  getParam<double>(control_nh, "momentum_observer_torque_weight", torque_weight, 10.0);
  momentum_observer_matrix_.topRows(3) *= force_weight;
  momentum_observer_matrix_.bottomRows(3) *= torque_weight;

  getParam<bool>(control_nh, "rotor_interfere_compensate", rotor_interfere_compensate_, true);
  getParam<double>(control_nh, "external_wrench_lpf_rate", wrench_lpf_rate_, 0.5);
  getParam<double>(control_nh, "external_fz_bias_thresh", fz_bias_thresh_, 1.0);
  getParam<double>(control_nh, "rotor_interfere_comp_wrench_lpf_rate", comp_wrench_lpf_rate_, 0.5);
  getParam<double>(control_nh, "rotor_interfere_force_dev_weight", rotor_interfere_force_dev_weight_, 1.0);
  getParam<double>(control_nh, "rotor_interfere_torque_xy_weight", rotor_interfere_torque_xy_weight_, 1.0);

  getParam<double>(control_nh, "overlap_dist_link_thresh", overlap_dist_link_thresh_, 0.08);
  getParam<double>(control_nh, "overlap_dist_rotor_thresh", overlap_dist_rotor_thresh_, 0.08);
  getParam<double>(control_nh, "overlap_dist_link_relax_thresh", overlap_dist_link_relax_thresh_, 0.08);
  getParam<double>(control_nh, "overlap_dist_rotor_relax_thresh", overlap_dist_rotor_relax_thresh_, 0.08);
  getParam<double>(control_nh, "overlap_dist_inter_joint_thresh", overlap_dist_inter_joint_thresh_, 0.08);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::DragonFullVectoringController, aerial_robot_control::ControlBase);

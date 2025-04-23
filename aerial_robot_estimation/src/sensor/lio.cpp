#include <aerial_robot_estimation/sensor/lio.h>

namespace sensor_plugin
{
  void LidarInertialOdometry::initialize(ros::NodeHandle nh,
                                         boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                         boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                         string sensor_name, int index)
  {
    SensorBase::initialize(nh, robot_model, estimator, sensor_name, index);

    std::string topic_name;
    getParam<std::string>("imu_topic_name", topic_name, std::string("imu"));
    imu_sub_ = nh_.subscribe<spinal::Imu>(topic_name, 10, &LidarInertialOdometry::ImuCallback, this);

    getParam<std::string>("lidar_imu_topic_name", topic_name, std::string("livox/imu_spinal"));
    lidar_imu_pub_ = nh_.advertise<sensor_msgs::Imu>(topic_name, 1);
    getParam<std::string>("lidar_frame", lidar_frame_, std::string("lidar_imu"));
  }

  void LidarInertialOdometry::ImuCallback(const spinal::ImuConstPtr& imu_msg)
  {
    if(!robot_model_->initialized()) return;

    ros::Time imu_stamp = imu_msg->stamp;

    tf::Vector3 acc_b;
    tf::Vector3 gyro_b;
    tf::Vector3 acc_lidar;
    tf::Vector3 gyro_lidar;

    for(int i = 0; i < 3; i++)
      {
        acc_b[i] = imu_msg->acc[i];
        gyro_b[i] = imu_msg->gyro[i];
      }

    tf::Transform lidar2baselink_tf;
    auto seg_tf_map = robot_model_->getSegmentsTf();
    tf::transformKDLToTF(seg_tf_map.at(lidar_frame_).Inverse() * seg_tf_map.at(robot_model_->getBaselinkName()), lidar2baselink_tf);

    acc_lidar = lidar2baselink_tf.getBasis() * acc_b / robot_model_->getGravity()(2);
    gyro_lidar = lidar2baselink_tf.getBasis() * gyro_b;

    sensor_msgs::Imu imu_ros_converted_msg;
    imu_ros_converted_msg.header.stamp = imu_msg->stamp;
    tf::vector3TFToMsg(gyro_lidar, imu_ros_converted_msg.angular_velocity);
    tf::vector3TFToMsg(acc_lidar, imu_ros_converted_msg.linear_acceleration);
    lidar_imu_pub_.publish(imu_ros_converted_msg);
  }
};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::LidarInertialOdometry, sensor_plugin::SensorBase);


[1mdiff --git a/aerial_robot_estimation/CMakeLists.txt b/aerial_robot_estimation/CMakeLists.txt[m
[1mindex dee70dc0..5ad49505 100644[m
[1m--- a/aerial_robot_estimation/CMakeLists.txt[m
[1m+++ b/aerial_robot_estimation/CMakeLists.txt[m
[36m@@ -51,8 +51,7 @@[m [madd_library(sensor_pluginlib[m
   src/sensor/mocap.cpp[m
   src/sensor/gps.cpp[m
   src/sensor/imu.cpp[m
[31m-  src/sensor/plane_detection.cpp[m
[31m-  src/sensor/lio.cpp)[m
[32m+[m[32m  src/sensor/plane_detection.cpp)[m
 [m
 target_link_libraries(sensor_pluginlib aerial_robot_estimation ${catkin_LIBRARIES})[m
 add_dependencies(sensor_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp)[m
[1mdiff --git a/aerial_robot_estimation/include/aerial_robot_estimation/sensor/lio.h b/aerial_robot_estimation/include/aerial_robot_estimation/sensor/lio.h[m
[1mdeleted file mode 100644[m
[1mindex 1d2c10a3..00000000[m
[1m--- a/aerial_robot_estimation/include/aerial_robot_estimation/sensor/lio.h[m
[1m+++ /dev/null[m
[36m@@ -1,31 +0,0 @@[m
[31m-#pragma once[m
[31m-[m
[31m-#include <aerial_robot_estimation/sensor/base_plugin.h>[m
[31m-#include <sensor_msgs/Imu.h>[m
[31m-#include <spinal/Imu.h>[m
[31m-[m
[31m-[m
[31m-namespace sensor_plugin[m
[31m-{[m
[31m-  class LidarInertialOdometry : public SensorBase[m
[31m-  {[m
[31m-  public:[m
[31m-    LidarInertialOdometry(){};[m
[31m-    ~LidarInertialOdometry(){};[m
[31m-[m
[31m-    void initialize(ros::NodeHandle nh,[m
[31m-                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,[m
[31m-                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,[m
[31m-                    string sensor_name, int index) override;[m
[31m-[m
[31m-[m
[31m-  private:[m
[31m-    ros::Subscriber imu_sub_;[m
[31m-    ros::Publisher lidar_imu_pub_;[m
[31m-[m
[31m-    std::string lidar_frame_;[m
[31m-[m
[31m-    void ImuCallback(const spinal::ImuConstPtr& imu_msg);[m
[31m-  };[m
[31m-};[m
[31m-[m
[1mdiff --git a/aerial_robot_estimation/include/aerial_robot_estimation/state_estimation.h b/aerial_robot_estimation/include/aerial_robot_estimation/state_estimation.h[m
[1mindex cd36468e..8ceb41cc 100644[m
[1m--- a/aerial_robot_estimation/include/aerial_robot_estimation/state_estimation.h[m
[1m+++ b/aerial_robot_estimation/include/aerial_robot_estimation/state_estimation.h[m
[36m@@ -441,7 +441,6 @@[m [mnamespace aerial_robot_estimation[m
     }[m
 [m
     inline uint8_t getUnhealthLevel() { return unhealth_level_; }[m
[31m-    std::string getTFPrefix() {return tf_prefix_;}[m
 [m
     const vector<boost::shared_ptr<sensor_plugin::SensorBase> >& getImuHandlers() const { return imu_handlers_;}[m
     const vector<boost::shared_ptr<sensor_plugin::SensorBase> >& getAltHandlers() const { return alt_handlers_;}[m
[36m@@ -470,7 +469,6 @@[m [mnamespace aerial_robot_estimation[m
     vector<boost::shared_ptr<sensor_plugin::SensorBase> > vo_handlers_;[m
     vector<boost::shared_ptr<sensor_plugin::SensorBase> > gps_handlers_;[m
     vector<boost::shared_ptr<sensor_plugin::SensorBase> > plane_detection_handlers_;[m
[31m-    vector<boost::shared_ptr<sensor_plugin::SensorBase> > lio_handlers_;[m
 [m
     /* mutex */[m
     boost::mutex state_mutex_;[m
[1mdiff --git a/aerial_robot_estimation/plugins/sensor_plugins.xml b/aerial_robot_estimation/plugins/sensor_plugins.xml[m
[1mindex b6f98989..a91f2e6c 100644[m
[1m--- a/aerial_robot_estimation/plugins/sensor_plugins.xml[m
[1m+++ b/aerial_robot_estimation/plugins/sensor_plugins.xml[m
[36m@@ -28,8 +28,4 @@[m
     <description>Plane Detection sensor plugin.</description>[m
   </class>[m
 [m
[31m-  <class name="sensor_plugin/lio" type="sensor_plugin::LidarInertialOdometry" base_class_type="sensor_plugin::SensorBase">[m
[31m-    <description>Lidar Inertial Odometry sensor plugin.</description>[m
[31m-  </class>[m
[31m-[m
 </library>[m
[1mdiff --git a/aerial_robot_estimation/src/sensor/imu.cpp b/aerial_robot_estimation/src/sensor/imu.cpp[m
[1mindex 3a69c26e..254f16c6 100644[m
[1m--- a/aerial_robot_estimation/src/sensor/imu.cpp[m
[1m+++ b/aerial_robot_estimation/src/sensor/imu.cpp[m
[36m@@ -422,7 +422,6 @@[m [mnamespace sensor_plugin[m
   {[m
     sensor_msgs::Imu imu_data;[m
     imu_data.header.stamp = imu_stamp_;[m
[31m-    imu_data.header.frame_id = tf::resolve(estimator_->getTFPrefix(), robot_model_->getBaselinkName());[m
     tf::Quaternion q;[m
     raw_rot_.getRotation(q);[m
     tf::quaternionTFToMsg(q, imu_data.orientation);[m
[1mdiff --git a/aerial_robot_estimation/src/sensor/lio.cpp b/aerial_robot_estimation/src/sensor/lio.cpp[m
[1mdeleted file mode 100644[m
[1mindex 354953c5..00000000[m
[1m--- a/aerial_robot_estimation/src/sensor/lio.cpp[m
[1m+++ /dev/null[m
[36m@@ -1,56 +0,0 @@[m
[31m-#include <aerial_robot_estimation/sensor/lio.h>[m
[31m-[m
[31m-namespace sensor_plugin[m
[31m-{[m
[31m-  void LidarInertialOdometry::initialize(ros::NodeHandle nh,[m
[31m-                                         boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,[m
[31m-                                         boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,[m
[31m-                                         string sensor_name, int index)[m
[31m-  {[m
[31m-    SensorBase::initialize(nh, robot_model, estimator, sensor_name, index);[m
[31m-[m
[31m-    std::string topic_name;[m
[31m-    getParam<std::string>("imu_topic_name", topic_name, std::string("imu"));[m
[31m-    imu_sub_ = nh_.subscribe<spinal::Imu>(topic_name, 10, &LidarInertialOdometry::ImuCallback, this);[m
[31m-[m
[31m-    getParam<std::string>("lidar_imu_topic_name", topic_name, std::string("livox/imu_spinal"));[m
[31m-    lidar_imu_pub_ = nh_.advertise<sensor_msgs::Imu>(topic_name, 1);[m
[31m-    getParam<std::string>("lidar_frame", lidar_frame_, std::string("lidar_imu"));[m
[31m-  }[m
[31m-[m
[31m-  void LidarInertialOdometry::ImuCallback(const spinal::ImuConstPtr& imu_msg)[m
[31m-  {[m
[31m-    if(!robot_model_->initialized()) return;[m
[31m-[m
[31m-    ros::Time imu_stamp = imu_msg->stamp;[m
[31m-[m
[31m-    tf::Vector3 acc_b;[m
[31m-    tf::Vector3 gyro_b;[m
[31m-    tf::Vector3 acc_lidar;[m
[31m-    tf::Vector3 gyro_lidar;[m
[31m-[m
[31m-    for(int i = 0; i < 3; i++)[m
[31m-      {[m
[31m-        acc_b[i] = imu_msg->acc[i];[m
[31m-        gyro_b[i] = imu_msg->gyro[i];[m
[31m-      }[m
[31m-[m
[31m-    tf::Transform lidar2baselink_tf;[m
[31m-    auto seg_tf_map = robot_model_->getSegmentsTf();[m
[31m-    tf::transformKDLToTF(seg_tf_map.at(lidar_frame_).Inverse() * seg_tf_map.at(robot_model_->getBaselinkName()), lidar2baselink_tf);[m
[31m-[m
[31m-    acc_lidar = lidar2baselink_tf.getBasis() * acc_b / robot_model_->getGravity()(2);[m
[31m-    gyro_lidar = lidar2baselink_tf.getBasis() * gyro_b;[m
[31m-[m
[31m-    sensor_msgs::Imu imu_ros_converted_msg;[m
[31m-    imu_ros_converted_msg.header.stamp = imu_msg->stamp;[m
[31m-    tf::vector3TFToMsg(gyro_lidar, imu_ros_converted_msg.angular_velocity);[m
[31m-    tf::vector3TFToMsg(acc_lidar, imu_ros_converted_msg.linear_acceleration);[m
[31m-    lidar_imu_pub_.publish(imu_ros_converted_msg);[m
[31m-  }[m
[31m-};[m
[31m-[m
[31m-/* plugin registration */[m
[31m-#include <pluginlib/class_list_macros.h>[m
[31m-PLUGINLIB_EXPORT_CLASS(sensor_plugin::LidarInertialOdometry, sensor_plugin::SensorBase);[m
[31m-[m
[1mdiff --git a/aerial_robot_estimation/src/state_estimation.cpp b/aerial_robot_estimation/src/state_estimation.cpp[m
[1mindex 7ecf7cc6..b7ca3e57 100644[m
[1m--- a/aerial_robot_estimation/src/state_estimation.cpp[m
[1m+++ b/aerial_robot_estimation/src/state_estimation.cpp[m
[36m@@ -330,12 +330,6 @@[m [mvoid StateEstimator::rosParamInit()[m
               sensor_index.back() = plane_detection_handlers_.size();[m
             }[m
 [m
[31m-          if(name.find("lio") != std::string::npos)[m
[31m-            {[m
[31m-              lio_handlers_.push_back(sensors_.back());[m
[31m-              sensor_index.back() = lio_handlers_.size();[m
[31m-            }[m
[31m-[m
 [m
           sensors_.back()->initialize(nh_, robot_model_, shared_from_this(), name, sensor_index.back());[m
           break;[m

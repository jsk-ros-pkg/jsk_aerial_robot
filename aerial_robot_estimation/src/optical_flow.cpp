#include "aerial_robot_estimation/optical_flow.h"
#include <pluginlib/class_list_macros.h>

namespace
{
  Eigen::Matrix3d delta_r_;
  tf::Matrix3x3 prev_r_;
  tf::Vector3 prev_ang_vel_;
  double prev_z_pos_;
  double prev_z_vel_;
  ros::Time prev_stamp_;
  void  epipolarFit(const CMatrixDouble  &allData,
                    const vector_size_t  &useIndices,
                    std::vector< CMatrixDouble > &fitModels )
  {
    ASSERT_(useIndices.size() == 2);

    Eigen::Vector3d p_1_orig; p_1_orig << allData(0, useIndices[0]), allData(1, useIndices[0]), 1;
    Eigen::Vector3d p_2_orig; p_2_orig << allData(0, useIndices[1]), allData(1, useIndices[1]), 1;

    Eigen::Vector3d p_dash_1; p_dash_1 << allData(2, useIndices[0]), allData(3, useIndices[0]), 1;
    Eigen::Vector3d p_dash_2; p_dash_2 << allData(2, useIndices[1]), allData(3, useIndices[1]), 1;
    Eigen::Vector3d p_dash_1_orig = delta_r_ * p_dash_1;
    Eigen::Vector3d p_dash_2_orig = delta_r_ * p_dash_2;

    // std::cout << "p_1_orig: \n" << p_1_orig << std::endl;
    // std::cout << "p_2_orig: \n" << p_2_orig << std::endl;
    // std::cout << "p_dash_1: \n" << p_dash_1 << std::endl;
    // std::cout << "p_dash_2: \n" << p_dash_2 << std::endl;
    // std::cout << "p_dash_1_orig: \n" << p_dash_1_orig << std::endl;
    // std::cout << "p_dash_2_orig: \n" << p_dash_2_orig << std::endl;

    try
      {
        fitModels.resize(1);
        CMatrixDouble &M = fitModels[0];
        M.setSize(1,3);

        Eigen::Vector3d p1_cross = (p_1_orig.cross(p_dash_1_orig));
        Eigen::Vector3d p2_cross = (p_2_orig.cross(p_dash_2_orig));
        Eigen::Vector3d delta_t = p1_cross.cross(p2_cross);
        //std::cout << "dleta_t: \n" << delta_t << std::endl;
        delta_t.normalize();
        M(0, 0) = delta_t(0);
        M(0, 1) = delta_t(1);
        M(0, 2) = delta_t(2);

      }
    catch(std::exception &)
      {
        fitModels.clear();
        return;
      }
  }

  void epipolarDistance(const CMatrixDouble &allData,
                        const std::vector< CMatrixDouble > & testModels,
                        const double distanceThreshold,
                        unsigned int & out_bestModelIndex,
                        vector_size_t & out_inlierIndices )
  {
    ASSERT_(testModels.size() == 1)
      out_bestModelIndex = 0;
    const CMatrixDouble &M = testModels[0];

    const size_t N = allData.cols();
    out_inlierIndices.clear();
    out_inlierIndices.reserve(100);

    /* t = (t_x, t_y, 1) */
    Eigen::Matrix3d delta_t_cross;
    delta_t_cross << 0, -M(0, 2), M(0, 1), M(0, 2), 0, -M(0, 0), -M(0, 1), M(0, 0), 0;
    Eigen::Matrix3d E =  delta_t_cross * delta_r_;

    for (size_t i = 0; i < N; i++)
      {
        Eigen::Vector3d x1;
        x1 << allData.get_unsafe(0, i), allData.get_unsafe(1, i), 1.;
        Eigen::Vector3d x2;
        x2 << allData.get_unsafe(2, i), allData.get_unsafe(3, i), 1.;

        Eigen::Vector3d Ex2 = E * x2;
        Eigen::Vector3d Etx1 = E.transpose() * x1;
        double x1tEx2 = x1.dot(Ex2);

        double a = Ex2[0] * Ex2[0];
        double b = Ex2[1] * Ex2[1];
        double c = Etx1[0] * Etx1[0];
        double d = Etx1[1] * Etx1[1];

        const double d_normailized_square = x1tEx2 * x1tEx2 / (a + b + c + d);

        if (d_normailized_square < distanceThreshold * distanceThreshold)
          {
            //printf("d_normailized_square: %f, distanceThreshold: %f \n", d_normailized_square, distanceThreshold);
            out_inlierIndices.push_back(i);
          }
      }
  }

  /** Return "true" if the selected points are a degenerate (invalid) case.
   */
  bool epipolarDegenerate(const CMatrixDouble &allData,
                          const mrpt::vector_size_t &useIndices )
  {
    return false;
  }

};

namespace {
#if USE_GPU
  void download(const cv::gpu::GpuMat& d_mat, std::vector<cv::Point2f>& vec)
  {
    vec.resize(d_mat.cols);
    cv::Mat mat(1, d_mat.cols, CV_32FC2, (void*)&vec[0]);
    d_mat.download(mat);
  }

  void download(const cv::gpu::GpuMat& d_mat, std::vector<uchar>& vec)
  {
    vec.resize(d_mat.cols);
    cv::Mat mat(1, d_mat.cols, CV_8UC1, (void*)&vec[0]);
    d_mat.download(mat);
  }
#endif
}

namespace aerial_robot_estimation
{
  void OpticalFlow::onInit()
  {
    nh_ = this->getNodeHandle();
    nhp_ = this->getPrivateNodeHandle();

    /* ros param */
    nhp_.param("downward_camera_image_topic_name", downward_camera_image_topic_name_, std::string("/downward_cam/camera/image"));
    nhp_.param("downward_camera_info_topic_name", downward_camera_info_topic_name_, std::string("/downward_cam/camera/camera_info"));
    nhp_.param("odometry_topic_name", odometry_topic_name_, std::string("/ground_truth/state"));
    nhp_.param("camera_vel_topic_name", camera_vel_topic_name_, std::string("camera_velocity"));
    nhp_.param("optical_flow_image_topic_name", optical_flow_image_topic_name_, std::string("optical_flow_image"));

    nhp_.param("debug", debug_, false);
    nhp_.param("max_count", max_count_, 100);
    nhp_.param("alt_offset", alt_offset_, 0.0);
    nhp_.param("image_crop_scale", image_crop_scale_, 1.0);
    nhp_.param("epipolar_thresh", epipolar_thresh_, 1.0);

    /* subscriber */
    downward_camera_image_sub_ = nh_.subscribe(downward_camera_image_topic_name_, 1, &OpticalFlow::downwardCameraImageCallback, this);
    downward_camera_info_sub_ = nh_.subscribe(downward_camera_info_topic_name_, 1, &OpticalFlow::downwardCameraInfoCallback, this);
    odometry_sub_ = nh_.subscribe(odometry_topic_name_, 1, &OpticalFlow::odometryCallback, this);

    /* publisher */
    camera_vel_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(camera_vel_topic_name_, 10);
    baselink_vel_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("baselink_vel", 10);
    optical_flow_image_pub_ = nh_.advertise<sensor_msgs::Image>(optical_flow_image_topic_name_, 1);

#if USE_GPU
    ROS_WARN("optical flow gpu mode");
#else
    ROS_WARN("optical flow no gpu mode");
#endif
  }

  void OpticalFlow::downwardCameraImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    //if (!camera_info_update_ || !imu_update_ || !image_stamp_update_) {
    if (!camera_info_update_ || !image_stamp_update_) {
      prev_r_ = uav_rotation_mat_;
      prev_ang_vel_ = ang_vel_;
      prev_z_vel_ = z_vel_;
      prev_stamp_ = msg->header.stamp;

      /* TODO: should be merge into the sensor fusion */
      /* obtain the transform from baselink to optical flow sensor */
      try
        {
          std::string camera_frame, baselink_frame;
          nhp_.param("camera_frame", camera_frame, std::string("downwards_cam_optical_frame"));
          nhp_.param("baselink_frame", baselink_frame, std::string("fc"));

          tf::TransformListener listener;
          listener.waitForTransform(camera_frame, baselink_frame, ros::Time(0), ros::Duration(10.0) );
          listener.lookupTransform(camera_frame, baselink_frame, ros::Time(0), tf_fc2camera_);
        } catch (tf::TransformException ex)
        {
          ROS_ERROR("%s",ex.what());
        }

      image_stamp_update_ = true;
      return;
    }

    ros::Time start_time = ros::Time().now();
    cv::Mat src_img = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    if(debug_)  ROS_INFO("image copy: %f[sec]", ros::Time().now().toSec() - start_time.toSec());

    start_time = ros::Time().now();
    if (image_crop_scale_ != 1.0) {
      src_img = src_img(cv::Range(src_img.rows * (1 - image_crop_scale_) / 2, src_img.rows * (1 + image_crop_scale_) / 2), cv::Range(src_img.cols * (1 - image_crop_scale_) / 2, src_img.cols * (1 + image_crop_scale_) / 2));
    }
    if(debug_) ROS_INFO("image cropping: %f[sec]", ros::Time().now().toSec() - start_time.toSec());
    std::vector<cv::Point2f> points[2];

    //calc features
#if USE_GPU
    start_time = ros::Time().now();
    cv::gpu::GpuMat d_src_img(src_img);
    if (src_img.channels() > 1) {
      cv::gpu::cvtColor(d_src_img, d_frame1Gray, cv::COLOR_BGR2GRAY);
    } else {
      d_frame1Gray = d_src_img;
    }
    if(debug_)  ROS_INFO("image convert color: %f[sec]", ros::Time().now().toSec() - start_time.toSec());

    start_time = ros::Time().now();
    if (d_frame0Gray.empty()) d_frame1Gray.copyTo(d_frame0Gray);
    cv::gpu::GoodFeaturesToTrackDetector_GPU detector(max_count_, 0.01, 10, 5, false, 0.04);
    cv::gpu::GpuMat d_prevPts;
    detector(d_frame0Gray, d_prevPts);
    if(debug_)  ROS_INFO("image find features: %f[sec]", ros::Time().now().toSec() - start_time.toSec());
#else
    start_time = ros::Time().now();
    cv::Mat gray_img;
    if (src_img.channels() > 1) {
      cv::cvtColor(src_img, gray_img, cv::COLOR_BGR2GRAY);
    } else {
      src_img.copyTo(gray_img);
    }
    if(prev_gray_img_.empty()) gray_img.copyTo(prev_gray_img_);
    if(debug_) ROS_INFO("image convert color: %f[sec]", ros::Time().now().toSec() - start_time.toSec());

    start_time = ros::Time().now();
    cv::TermCriteria termcrit(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 0.03);
    cv::Size subPixWinSize(10,10), winSize(31,31);
    cv::goodFeaturesToTrack(prev_gray_img_, points[0], max_count_, 0.01, 10, cv::Mat(), 3, false, 0.04);
    //cv::cornerSubPix(prev_gray_img_, points[0], subPixWinSize, cv::Size(-1,-1), termcrit);
    if(debug_)  ROS_INFO("image find features: %f[sec]", ros::Time().now().toSec() - start_time.toSec());
#endif

    //calc optical flow
    std::vector<uchar> status;
    start_time = ros::Time().now();
#if USE_GPU
    cv::gpu::PyrLKOpticalFlow d_pyrLK;

    d_pyrLK.winSize.width = 31;
    d_pyrLK.winSize.height = 31;
    d_pyrLK.maxLevel = 3;
    d_pyrLK.iters = 30;

    cv::gpu::GpuMat d_nextPts, d_status;

    d_pyrLK.sparse(d_frame0Gray, d_frame1Gray, d_prevPts, d_nextPts, d_status);

    points[0].resize(d_prevPts.cols);
    download(d_prevPts, points[0]);

    points[1].resize(d_nextPts.cols);
    download(d_nextPts, points[1]);

    status.resize(d_status.cols);
    download(d_status, status);
#else
    std::vector<float> err;
    if (!points[0].empty())
      cv::calcOpticalFlowPyrLK(prev_gray_img_, gray_img, points[0], points[1], status, err, winSize, 3, termcrit, 0, 0.001);
#endif
    if(debug_) ROS_INFO("image feature matching by PyrLK : %f[sec]", ros::Time().now().toSec() - start_time.toSec());

    /* ransac filter based on the epipolar constraint */
    start_time = ros::Time().now();
    //cv::Mat mask;
    //cv::Mat essential_matrix = cv::findEssentialMat(points[0], points[1], camera_f_, cv::Point2d(camera_cx_, camera_cy_), cv::LMEDS, 0.999, 3.0, mask);
    //int cnt = 0;
    //for(int i = 0; i < (int)mask.size(); i ++)
      //{
      //if(mask[i] == 1) cnt ++;
    //}
    //ROS_INFO("whole point: %d, valid point: %d, ratio: %f", mask.size(), cnt, cnt / mask.size());
    //std::cout << "mask: \n" << mask  << std::endl;
    //cv::SVD essential_matrix_svd(essential_matrix);
    //std::cout << "essential_matrix: \n" << essential_matrix << std::endl;
    //std::cout << "essential_matrix U: \n" << essential_matrix_svd.u << std::endl;
    //std::cout << "essential_matrix W: \n" << essential_matrix_svd.w << std::endl;
    //std::cout << "essential_matrix V: \n" << essential_matrix_svd.vt << std::endl;
    //cv::Mat w_dash = (cv::Mat_<double>(3,3) << 0, -1, 0, 1, 0, 0, 0, 0, 1);
    //std::cout << "delta R: \n" << essential_matrix_svd.u * w_dash * essential_matrix_svd.vt  << std::endl;
    //cv::Mat R, t;
    //recoverPose(essential_matrix, points[0], points[1], R, t, camera_f_, cv::Point2d(camera_cx_, camera_cy_), mask);
    //std::cout << "delta R: \n" << R  << std::endl;
    //std::cout << "delta t: \n" << t  << std::endl;

    /* start translation only RANSAC */
    tf::matrixTFToEigen(prev_r_.inverse() * uav_rotation_mat_, delta_r_);
    CMatrixDouble all_data(4, points[0].size());

    for(int i = 0; i < all_data.cols(); i++)
      {
        all_data(0, i) = (points[0].at(i).x - camera_cx_) / camera_f_;
        all_data(1, i) = (points[0].at(i).y - camera_cy_) / camera_f_;
        all_data(2, i) = (points[1].at(i).x - camera_cx_) / camera_f_;
        all_data(3, i) = (points[1].at(i).y - camera_cy_) / camera_f_;
      }

    double tx, ty, tz;
    vector_size_t best_inliers;
    ransac(all_data, tx, ty, tz, best_inliers);

    if(debug_)  ROS_INFO("image epipolar ransac : %f[sec]", ros::Time().now().toSec() - start_time.toSec());


    double time = (msg->header.stamp - prev_stamp_).toSec();
    prev_stamp_ = msg->header.stamp;
#if 0 // differential motion based BAD!!!
    tf::Vector3 camera_vel(tx / tz, ty / tz, 1);
    camera_vel *= ((-z_pos_ + prev_z_pos_) / time);

#else // optical based vel calculation
    tf::Vector3 local_ang_vel = tf_fc2camera_.getBasis().inverse() * ( ang_vel_ + prev_ang_vel_) / 2;
    local_ang_vel = tf_fc2camera_.getBasis().inverse() * ang_vel_;
    tf::Vector3 camera_vel;
    double camera_x_vel = 0.0, camera_y_vel = 0.0;
    for(size_t i = 0; i < best_inliers.size(); i++)
      {
        double x = points[1][best_inliers.at(i)].x - camera_cx_, prev_x = points[0][best_inliers.at(i)].x - camera_cx_;
        double y = points[1][best_inliers.at(i)].y - camera_cy_, prev_y = points[0][best_inliers.at(i)].y - camera_cy_;

        camera_x_vel += z_vel_ * x / camera_f_ + (-(x - prev_x) / time - local_ang_vel.y() * camera_f_ + local_ang_vel.z() * y + (local_ang_vel.x() * x * y - local_ang_vel.y() * x * x) / camera_f_) * z_pos_ / camera_f_;
        camera_y_vel += z_vel_ * y / camera_f_ + (-(y - prev_y) / time + local_ang_vel.x() * camera_f_ - local_ang_vel.z() * x + (local_ang_vel.x() * y * y - local_ang_vel.y() * x * y) / camera_f_) * z_pos_ / camera_f_;
        if (debug_)
          {
            cv::circle(src_img, points[0][best_inliers.at(i)], 3, cv::Scalar(0,255,0), -1, 8);
            cv::line(src_img, points[1][best_inliers.at(i)], points[0][best_inliers.at(i)], cv::Scalar(0,255,0), 1, 8, 0);
          }
    }

    if (best_inliers.size() != 0) camera_vel.setValue(camera_x_vel / best_inliers.size(), camera_y_vel / best_inliers.size(), z_vel_);
    else camera_vel.setValue(0.0, 0.0, 0.0);

#endif

    /* TODO: should be merge into the sensor fusion */
    /* camera frame vel w.r.t world */
    camera_vel = uav_rotation_mat_ * tf_fc2camera_.getBasis() * camera_vel;

    /* baselink frame vel w.r.t world */
    tf::Vector3 baselink_vel = camera_vel - uav_rotation_mat_ * ang_vel_.cross(tf_fc2camera_.getOrigin());

    if (debug_)
      optical_flow_image_pub_.publish(cv_bridge::CvImage(msg->header, msg->encoding, src_img).toImageMsg());

    geometry_msgs::Vector3Stamped camera_vel_msg;
    camera_vel_msg.header = msg->header;
    camera_vel_msg.vector.x = camera_vel.x();
    camera_vel_msg.vector.y = camera_vel.y();
    camera_vel_msg.vector.z = camera_vel.z();
    camera_vel_pub_.publish(camera_vel_msg);

    geometry_msgs::Vector3Stamped baselink_vel_msg;
    baselink_vel_msg.header = msg->header;
    baselink_vel_msg.vector.x = baselink_vel.x();
    baselink_vel_msg.vector.y = baselink_vel.y();
    baselink_vel_msg.vector.z = baselink_vel.z();
    baselink_vel_pub_.publish(baselink_vel_msg);


    start_time = ros::Time().now();
#if USE_GPU
    d_frame0Gray.swap(d_frame1Gray);
#else
    cv::swap(prev_gray_img_, gray_img);
#endif
    if(debug_)  ROS_INFO("image swapping : %f[sec]", ros::Time().now().toSec() - start_time.toSec());

    prev_r_ = uav_rotation_mat_;
    prev_ang_vel_ = ang_vel_;
    prev_z_vel_ = z_vel_;
    prev_z_pos_ = z_pos_;
  }

  void OpticalFlow::ransac(const CMatrixDouble& all_data, double& t_x, double& t_y, double& t_z, vector_size_t& best_inliers)
  {
    CMatrixDouble best_model;

    double start_t = ros::Time::now().toSec();

    ransac_.execute(all_data,
                    epipolarFit,
                    epipolarDistance,
                    epipolarDegenerate,
                    epipolar_thresh_ / camera_f_,
                    2,  // Minimum set of points
                    best_inliers,
                    best_model,
                    debug_);

    if(size(best_model,1) !=1 && size(best_model,2) !=3) return;

    if(debug_)
      std::cout << "RANSAC finished in " << ros::Time::now().toSec() - start_t
                << "[sec]: Best model: " << best_model << "; inlier size: "<< best_inliers.size() << std::endl;

    t_x = best_model(0,0);
    t_y = best_model(0,1);
    t_z = best_model(0,2);
  }

  void OpticalFlow::downwardCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    if (camera_info_update_) return;
    camera_f_ = msg->K[0];
    camera_cx_ = msg->K[2] * image_crop_scale_;
    camera_cy_ = msg->K[5] * image_crop_scale_;
    if (msg->K[0] > 0) {
      camera_info_update_ = true;
    }
  }

  void OpticalFlow::odometryCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    tf::Quaternion uav_q(msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z,
                         msg->pose.pose.orientation.w);
    uav_rotation_mat_.setRotation(uav_q);
    double r, p, y;
    uav_rotation_mat_.getRPY(r, p, y);

    sensor_msgs::RangePtr sonar_msg(new sensor_msgs::Range);
    sonar_msg->range = msg->pose.pose.position.z / (cos(r) * cos(p));
    altCallback(sonar_msg);
    z_vel_ = -msg->twist.twist.linear.z;

    tf::vector3MsgToTF(msg->twist.twist.angular, ang_vel_);

    odom_update_ = true;
  }

  void OpticalFlow::altCallback(const sensor_msgs::RangeConstPtr& msg)
  {
    z_pos_ = msg->range - alt_offset_;
  }
} //namespace aerial_robot_estimation


PLUGINLIB_EXPORT_CLASS(aerial_robot_estimation::OpticalFlow, nodelet::Nodelet)

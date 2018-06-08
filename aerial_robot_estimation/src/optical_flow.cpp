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
  ros::Time imu_stamp_;

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
    if(allData(4, useIndices[0]) == 0 || allData(4, useIndices[1]) == 0) return true;
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
    //nh_ = this->getNodeHandle();
    //nhp_ = this->getPrivateNodeHandle();
    nh_ = this->getMTNodeHandle();
    nhp_ = this->getMTPrivateNodeHandle();

    /* ros param */
    nhp_.param("downward_camera_image_topic_name", downward_camera_image_topic_name_, std::string("/downward_cam/camera/image"));
    nhp_.param("downward_camera_info_topic_name", downward_camera_info_topic_name_, std::string("/downward_cam/camera/camera_info"));
    nhp_.param("imu_topic_name", imu_topic_name_, std::string("/imu"));
    nhp_.param("odometry_topic_name", odometry_topic_name_, std::string("/ground_truth/state"));
    nhp_.param("camera_vel_topic_name", camera_vel_topic_name_, std::string("/camera_velocity"));
    nhp_.param("optical_flow_image_topic_name", optical_flow_image_topic_name_, std::string("optical_flow_image"));

    nhp_.param("debug", debug_, false);
    nhp_.param("max_count", max_count_, 100);
    nhp_.param("alt_offset", alt_offset_, 0.0);
    nhp_.param("image_crop_scale", image_crop_scale_, 1.0);
    nhp_.param("epipolar_thresh", epipolar_thresh_, 1.0);
    nhp_.param("feature_min_dist", feature_min_dist_, 20);
    nhp_.param("optical_flow_win_size", optical_flow_win_size_, 21);
    nhp_.param("inlier_rate", inlier_rate_, 0.2); // 40 in 200
    nhp_.param("delta_rot_thresh", delta_rot_thresh_, 3.14); // 40 in 200

    /* subscriber */
    //downward_camera_image_sub_ = nh_.subscribe(downward_camera_image_topic_name_, 1, &OpticalFlow::downwardCameraImageCallback, this);
    downward_camera_info_sub_ = nh_.subscribe(downward_camera_info_topic_name_, 1, &OpticalFlow::downwardCameraInfoCallback, this);
    odometry_sub_ = nh_.subscribe(odometry_topic_name_, 1, &OpticalFlow::odometryCallback, this);
    //imu_sub_ = nh_.subscribe(imu_topic_name_, 1, &OpticalFlow::imuCallback, this);

    sub_image_.subscribe(nh_, downward_camera_image_topic_name_, 10, ros::TransportHints().tcpNoDelay());
    sub_imu_.subscribe(nh_, imu_topic_name_, 10, ros::TransportHints().tcpNoDelay());
    sync_ = boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10)));
    sync_->connectInput(sub_image_, sub_imu_);
    sync_->registerCallback(&OpticalFlow::opticalFlowCallback, this);

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

  void OpticalFlow::opticalFlowCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::ImuConstPtr& imu_msg)
  {
    imuCallback(imu_msg);
    downwardCameraImageCallback(image_msg);
  }


  void OpticalFlow::downwardCameraImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    //ROS_INFO("start callback");
    curr_image_stamp_ = msg->header.stamp;

    ros::Time start_time = ros::Time().now();
    cv::Mat src_img = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    if(debug_)  ROS_INFO("image copy: %f[sec]", ros::Time().now().toSec() - start_time.toSec());

    start_time = ros::Time().now();
    if (image_crop_scale_ != 1.0) {
      src_img = src_img(cv::Range(src_img.rows * (1 - image_crop_scale_) / 2, src_img.rows * (1 + image_crop_scale_) / 2), cv::Range(src_img.cols * (1 - image_crop_scale_) / 2, src_img.cols * (1 + image_crop_scale_) / 2));
    }
    if(debug_) ROS_INFO("image cropping: %f[sec]", ros::Time().now().toSec() - start_time.toSec());

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
    std::vector<cv::Point2f> curr_raw_feature_points;
    start_time = ros::Time().now();
    cv::Mat gray_img;
    if (src_img.channels() > 1) {
      cv::cvtColor(src_img, gray_img, cv::COLOR_BGR2GRAY);
    } else {
      src_img.copyTo(gray_img);
    }
    gray_img.copyTo(curr_gray_img_);

    if(debug_) ROS_INFO("image convert color and copy: %f[sec]", ros::Time().now().toSec() - start_time.toSec());

    /* another thread to do motion estimation based on optical flow */
    auto motion_estimation_th = std::thread([this]{ this->motionEstimation(); });

    /* extract good feature from current image */
    start_time = ros::Time().now();
    /* TODO: shift the crop function to MASK */
    cv::goodFeaturesToTrack(gray_img, curr_raw_feature_points, max_count_, 0.01, feature_min_dist_, cv::Mat(), 3, false, 0.04);
    if(debug_) ROS_INFO("image find features: %f[sec]", ros::Time().now().toSec() - start_time.toSec());
    // start_time = ros::Time().now();
    // cv::Size subPixWinSize(10,10), winSize(31,31);
    // cv::TermCriteria termcrit(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 0.03);
    // cv::cornerSubPix(gray_img, curr_feature_points, subPixWinSize, cv::Size(-1,-1), termcrit);
    // if(debug_) ROS_INFO("image find features: %f[sec]", ros::Time().now().toSec() - start_time.toSec());
#endif

    /* finish the thread */
    motion_estimation_th.join();

    /* update the stamp,  image and feature points */
    prev_image_stamp_ = curr_image_stamp_;
    start_time = ros::Time().now();
#if USE_GPU
    d_frame0Gray.swap(d_frame1Gray); //what is this?
#else
    gray_img.copyTo(prev_gray_img_);
#endif
    if(debug_)  ROS_INFO("image copy : %f[sec]", ros::Time().now().toSec() - start_time.toSec());

    /* update */
    prev_raw_feature_points_ = curr_raw_feature_points;
    prev_r_ = uav_rotation_mat_;
    prev_ang_vel_ = ang_vel_;
    prev_z_vel_ = z_vel_;
    prev_z_pos_ = z_pos_;

    //optical_flow_image_pub_.publish(cv_bridge::CvImage(msg->header, msg->encoding, src_img).toImageMsg());

  }

  void OpticalFlow::motionEstimation()
  {
#if 0 // test the thread
    ROS_INFO("start motion estimation");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ROS_INFO("finish motion estimation");
#endif

    /* init processing */
    if (!camera_info_update_ || !odom_update_ || !prev_image_ready_)
      {
        prev_r_ = uav_rotation_mat_;
        prev_ang_vel_ = ang_vel_;
        prev_z_vel_ = z_vel_;

        /* TODO: should be merge into the sensor fusion */
        /* obtain the transform from baselink to optical flow sensor */
        try
          {
            std::string camera_frame, baselink_frame;
            nhp_.param("camera_frame", camera_frame, std::string("downwards_cam_optical_frame"));
            nhp_.param("baselink_frame", baselink_frame, std::string("fc"));

            tf::TransformListener listener;
            listener.waitForTransform(baselink_frame, camera_frame, ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform(baselink_frame, camera_frame, ros::Time(0), tf_fc2camera_);

            double r, p, y;
            tf_fc2camera_.getBasis().getRPY(r, p, y);
             ROS_ERROR("rpy: [%f, %f, %f], pos: [%f, %f, %f]", r, p, y,
                       tf_fc2camera_.getOrigin().x(),
                       tf_fc2camera_.getOrigin().y(),
                       tf_fc2camera_.getOrigin().z());

          } catch (tf::TransformException ex)
          {
            ROS_ERROR("%s",ex.what());
          }

        prev_image_ready_ = true;
        return;
      }

    /* feature matching by calc optical flow */
    std::vector<uchar> status;
    ros::Time start_time = ros::Time().now();

#if USE_GPU
    cv::gpu::PyrLKOpticalFlow d_pyrLK;

    d_pyrLK.winSize.width = 31;
    d_pyrLK.winSize.height = 31;
    d_pyrLK.maxLevel = 3;
    d_pyrLK.iters = 30;

    cv::gpu::GpuMat d_nextPts, d_status;

    d_pyrLK.sparse(d_frame0Gray, d_frame1Gray, d_prevPts, d_nextPts, d_status);

    prev_raw_feature_points_.resize(d_prevPts.cols);
    download(d_prevPts, prev_raw_feature_points_);

    points[1].resize(d_nextPts.cols);
    download(d_nextPts, points[1]);

    status.resize(d_status.cols);
    download(d_status, status);
#else
    if(prev_raw_feature_points_.size() == 0) return;
    //assert(!prev_raw_feature_points_.empty()); // maybe there is really no features
    std::vector<cv::Point2f> curr_raw_feature_points;
    std::vector<float> err;
    // cv::TermCriteria termcrit(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 0.03); //anzai tuning
    //cv::calcOpticalFlowPyrLK(prev_gray_img_, curr_gray_img_, prev_raw_feature_points_, curr_feature_points, status, err, cv::Size (optical_flow_win_size_, optical_flow_win_size_), 3, termcrit, 0, 0.001);
    cv::calcOpticalFlowPyrLK(prev_gray_img_, curr_gray_img_, prev_raw_feature_points_, curr_raw_feature_points, status, err, cv::Size (optical_flow_win_size_, optical_flow_win_size_));

#endif
    if(debug_) ROS_INFO("image feature matching by PyrLK : %f[sec]", ros::Time().now().toSec() - start_time.toSec());


    /* undistortion */
    start_time = ros::Time().now();
    std::vector<cv::Point2f> prev_feature_points, curr_feature_points;
    //prev_feature_points = prev_raw_feature_points_;
    //curr_feature_points = curr_raw_feature_points;
    cv::undistortPoints(prev_raw_feature_points_, prev_feature_points, camera_mat_, camera_d_, cv::Mat(), camera_mat_);
    cv::undistortPoints(curr_raw_feature_points, curr_feature_points, camera_mat_, camera_d_, cv::Mat(), camera_mat_);

    /*
    for(int i = 0; i < prev_feature_points.size(); i++)
      {
        ROS_INFO("%d: prev raw [%f, %f] => [%f, %f]", i+1, prev_raw_feature_points_.at(i).x,
                 prev_raw_feature_points_.at(i).y,
                 prev_feature_points.at(i).x, prev_feature_points.at(i).y);
      }
    */
    if(debug_)
      ROS_INFO("image feature distortion : %f[sec]", ros::Time().now().toSec() - start_time.toSec());


    size_t valid_point = 0;
    for(auto itr: status)
      {
        if(itr == 1) valid_point++;
      }
    //ROS_ERROR("valid_point: %d", valid_point);
    if( (float)valid_point / (float)max_count_ < inlier_rate_)
      {
        if(debug_) ROS_WARN("stop calculate differential motion since the valid point is not enough.: %d", (int)valid_point);
        return;
      }

    /* ransac filter based on the epipolar constraint */
    start_time = ros::Time().now();

    /* start translation only RANSAC */
    double prev_r, prev_p, prev_y, r, p, y;
    prev_r_.getRPY(prev_r, prev_p, prev_y);
    uav_rotation_mat_.getRPY(r, p, y);
    //ROS_WARN("prev: [%f, %f, %f,], curr: [%f, %f, %f]",prev_r, prev_p, prev_y, r, p, y);

    tf::matrixTFToEigen(tf_fc2camera_.getBasis().inverse() * prev_r_.inverse() * uav_rotation_mat_ * tf_fc2camera_.getBasis(), delta_r_);
    CMatrixDouble all_data(5, prev_feature_points.size());

    for(int i = 0; i < all_data.cols(); i++)
      {
        all_data(0, i) = (prev_feature_points.at(i).x - camera_cx_) / camera_f_;
        all_data(1, i) = (prev_feature_points.at(i).y - camera_cy_) / camera_f_;
        all_data(2, i) = (curr_feature_points.at(i).x - camera_cx_) / camera_f_;
        all_data(3, i) = (curr_feature_points.at(i).y - camera_cy_) / camera_f_;
        all_data(4, i) = status.at(i);
      }

    double tx, ty, tz;
    vector_size_t best_inliers;
    ransac(all_data, tx, ty, tz, best_inliers);

    if(debug_)  ROS_INFO("image epipolar ransac : %f[sec]", ros::Time().now().toSec() - start_time.toSec());

    tf::Vector3 local_ang_vel = tf_fc2camera_.getBasis().inverse() * prev_ang_vel_; // best!!
    //tf::Vector3 local_ang_vel = tf_fc2camera_.getBasis().inverse() * ang_vel_;
    //tf::Vector3 local_ang_vel = tf_fc2camera_.getBasis().inverse() * (prev_ang_vel_ + ang_vel_) / 2;

    tf::Vector3 camera_vel;
    double camera_x_vel = 0.0, camera_y_vel = 0.0;

    double delta_time = (curr_image_stamp_ - prev_image_stamp_).toSec();
    if(debug_) ROS_INFO("delta_time: %f", delta_time);

#if 0 // SUM method
    double image_cut_pixel = 10;
    for(size_t i = 0; i < prev_feature_points.size(); i++) {
      if(!status[i]) continue;

      double x = curr_feature_points.at(i).x - camera_cx_, prev_x = prev_feature_points.at(i).x - camera_cx_;
      double y = curr_feature_points.at(i).y - camera_cy_, prev_y = prev_feature_points.at(i).y - camera_cy_;

      if(fabs(x) > (camera_cx_ - image_cut_pixel)  || fabs(y) > (camera_cy_ - image_cut_pixel)) continue;

      camera_x_vel += z_vel_ * x / camera_f_ + (-(x - prev_x) / delta_time - local_ang_vel.y() * camera_f_ + local_ang_vel.z() * y + (local_ang_vel.x() * x * y - local_ang_vel.y() * x * x) / camera_f_) * z_pos_ / camera_f_;
      camera_y_vel += z_vel_ * y / camera_f_ + (-(y - prev_y) / delta_time + local_ang_vel.x() * camera_f_ - local_ang_vel.z() * x + (local_ang_vel.x() * y * y - local_ang_vel.y() * x * y) / camera_f_) * z_pos_ / camera_f_;
      /*
      if (debug_) {
        cv::circle(src_img, points[0][i], 3, cv::Scalar(0,255,0), -1, 8);
        cv::line(src_img, curr_feature_points[i], points[0][i], cv::Scalar(0,255,0), 1, 8, 0);
      }
      */
    }
    camera_vel.setValue(camera_x_vel / valid_point, camera_y_vel / valid_point, z_vel_);

    //inliear_rate = (float)valid_point / prev_feature_points.size();

#else // optical based vel calculation based on ransac reusult

    if(debug_)
      ROS_WARN("inliers %d -> %d -> %d",  max_count_, (int)valid_point, (int)best_inliers.size());

    if((float)best_inliers.size() / max_count_ < inlier_rate_) return;

    double camera_x_vel_trans = 0;
    double camera_x_vel_rot = 0;

    tf::Vector3 local_ang_vel_prev = tf_fc2camera_.getBasis().inverse() * prev_ang_vel_;
    tf::Vector3 local_ang_vel_curr = tf_fc2camera_.getBasis().inverse() * ang_vel_ ;
    double camera_x_vel_curr = 0;
    double camera_x_vel_prev = 0;
    for(size_t i = 0; i < best_inliers.size(); i++)
      {
        double x = curr_feature_points.at(best_inliers.at(i)).x - camera_cx_, prev_x = prev_feature_points.at(best_inliers.at(i)).x - camera_cx_;
        double y = curr_feature_points.at(best_inliers.at(i)).y - camera_cy_, prev_y = prev_feature_points[best_inliers.at(i)].y - camera_cy_;

        camera_x_vel += (z_vel_ * x / camera_f_ + (-(x - prev_x) / delta_time - local_ang_vel.y() * camera_f_ + local_ang_vel.z() * y + (local_ang_vel.x() * x * y - local_ang_vel.y() * x * x) / camera_f_) * z_pos_ / camera_f_);
        camera_y_vel += z_vel_ * y / camera_f_ + (-(y - prev_y) / delta_time + local_ang_vel.x() * camera_f_ - local_ang_vel.z() * x + (local_ang_vel.x() * y * y - local_ang_vel.y() * x * y) / camera_f_) * z_pos_ / camera_f_;

        camera_x_vel_trans += (- local_ang_vel.y() * camera_f_ + ( - local_ang_vel.y() * x * x) / camera_f_) * z_pos_ / camera_f_;
        camera_x_vel_rot += z_vel_ * x / camera_f_ + (-(x - prev_x) / delta_time ) * z_pos_ / camera_f_;

        // camera_x_vel_prev += (z_vel_ * x / camera_f_ + (-(x - prev_x) / delta_time - local_ang_vel_prev.y() * camera_f_ + local_ang_vel_prev.z() * y + (local_ang_vel_prev.x() * x * y - local_ang_vel_prev.y() * x * x) / camera_f_) * z_pos_ / camera_f_);
        // camera_x_vel_curr += (z_vel_ * x / camera_f_ + (-(x - prev_x) / delta_time - local_ang_vel_curr.y() * camera_f_ + local_ang_vel_curr.z() * y + (local_ang_vel_curr.x() * x * y - local_ang_vel_curr.y() * x * x) / camera_f_) * z_pos_ / camera_f_);
    }

    camera_vel.setValue(camera_x_vel / best_inliers.size(), camera_y_vel / best_inliers.size(), z_vel_);

#endif

    //camera_vel.setValue(tx, ty, tz); //bad
    /* TODO: should be merge into the sensor fusion */
    /* camera frame vel w.r.t world */
    //camera_vel = uav_rotation_mat_ * tf_fc2camera_.getBasis() * camera_vel;
    //camera_vel = tf_fc2camera_.getBasis() * camera_vel;
    /* baselink frame vel w.r.t world */
    tf::Vector3 baselink_vel = uav_rotation_mat_ * (tf_fc2camera_.getBasis() * camera_vel - ang_vel_.cross(tf_fc2camera_.getOrigin()));

    geometry_msgs::Vector3Stamped camera_vel_msg;
    camera_vel_msg.header.stamp = curr_image_stamp_;
    camera_vel_msg.vector.x = camera_vel.x();
    camera_vel_msg.vector.y = camera_vel.y();
    camera_vel_msg.vector.z = camera_vel.z();
    camera_vel_pub_.publish(camera_vel_msg);

    geometry_msgs::Vector3Stamped baselink_vel_msg;
    baselink_vel_msg.header.stamp = curr_image_stamp_;
    baselink_vel_msg.vector.x = baselink_vel.x();
    baselink_vel_msg.vector.y = baselink_vel.y();
    baselink_vel_msg.vector.z = baselink_vel.z();

    // baselink_vel_msg.vector.x = camera_x_vel_prev / best_inliers.size();
    // baselink_vel_msg.vector.y = camera_x_vel_curr / best_inliers.size();

    baselink_vel_pub_.publish(baselink_vel_msg);
    //ROS_INFO("imu stamp - image stamp: %f", imu_stamp_.toSec() - curr_image_stamp_.toSec());
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
                    false/* debug_ */);

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

    camera_mat_ = (cv::Mat_<double>(3,3) << msg->K[0], msg->K[1], msg->K[2] * image_crop_scale_,
                   msg->K[3], msg->K[4], msg->K[5] * image_crop_scale_,
                   msg->K[6], msg->K[7], msg->K[8]);
    camera_d_ = (cv::Mat_<double>(5,1) << msg->D[0], msg->D[1], msg->D[2], msg->D[3], msg->D[4]);
    if (msg->K[0] > 0) {
      camera_info_update_ = true;
    }
  }

  void OpticalFlow::odometryCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    tf::Matrix3x3 uav_rotation_mat(q);
    double r, p, y;
    uav_rotation_mat.getRPY(r, p, y);

    z_pos_ = ( msg->pose.pose.position.z + tf_fc2camera_.getOrigin().z())  / (cos(r) * cos(p));
    z_vel_ = -msg->twist.twist.linear.z;

    odom_update_ = true;
  }

  void OpticalFlow::imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->orientation, q);
    uav_rotation_mat_.setRotation(q);
    tf::vector3MsgToTF(msg->angular_velocity, ang_vel_);

    imu_stamp_ = msg->header.stamp;
  }

} //namespace aerial_robot_estimation


PLUGINLIB_EXPORT_CLASS(aerial_robot_estimation::OpticalFlow, nodelet::Nodelet)

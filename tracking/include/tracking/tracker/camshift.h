
#ifndef CAM_SHIFT_H
#define CAM_SHIFT_H

#include <ros/ros.h>
#include <tracking/basic_tracking.h>
#include <aerial_robot_base/digital_filter.h>
#include <tracking/tracking.h>
#include <sensor_msgs/CameraInfo.h>
#include <jsk_recognition_msgs/RotatedRectStamped.h>
#include <image_processing/StartTracking.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <aerial_robot_base/FlightNav.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


class CamShift
{
 public:


 CamShift(ros::NodeHandle nh, ros::NodeHandle nh_private, BasicTracking* tracker)
   : nh_(nh), nh_private_(nh_private, "cam_shift")
    {
      rosParamInit();
      
      tracker_ = tracker;

      camshift_sub_ = nh_.subscribe<jsk_recognition_msgs::RotatedRectStamped>("result", 1, &CamShift::trackerCallback, this, ros::TransportHints().tcpNoDelay());

      camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>(cam_info_topic_, 1, &CamShift::cameraInfoCallback, this, ros::TransportHints().tcpNoDelay());

      start_tracking_client_ = nh_.serviceClient<image_processing::StartTracking>("/start_tracking");

      world_cam_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("world_cam_coord", 1); 

      lpf_image_x_ =  IirFilter((float)rx_freq_, (float)cutoff_freq_);
      lpf_image_y_ =  IirFilter((float)rx_freq_, (float)cutoff_freq_);
      lpf_image_area_ =  IirFilter((float)rx_freq_, (float)cutoff_freq_);

      alt_control_flag_ = false;

      //trial, start tracking
      startTracking();
 }

  ~CamShift()
    {
      /* navigator_->setTargetVelX(0); */
      /* navigator_->setTargetVelY(0); */
      /* navigator_->setTargetPsi(0); */
      aerial_robot_base::FlightNav navi_command;
      navi_command.header.stamp = ros::Time::now();
      navi_command.command_mode = aerial_robot_base::FlightNav::VEL_FLIGHT_MODE_COMMAND;
      navi_command.target_vel_x = 0;
      navi_command.target_vel_y = 0;
      navi_command.target_psi = 0;
      navi_command.pos_z_navi_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
      //tracker_->navi_pub_.publish(navi_command);
      tracker_->navigation(navi_command);


      camshift_sub_.shutdown();
      start_tracking_client_.shutdown();
    }

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber camshift_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Publisher world_cam_pub_;
  ros::Publisher navi_pub_;
  ros::Subscriber state_sub_;

  ros::ServiceClient start_tracking_client_;
  BasicTracking* tracker_;


  IirFilter lpf_image_x_, lpf_image_y_, lpf_image_area_;
  double rx_freq_, cutoff_freq_;

  Eigen::Matrix3d intrinsic_matrix_, intrinsic_matrix_inverse_;

  double target_area_;
  double target_y_;
  double target_z_;
  double thre_z_;
  double thre_psi_;

  double gain_forward_x_;
  double gain_backward_x_;
  double gain_y_;
  double gain_z_;
  double gain_psi_;
  std::string cam_info_topic_;

  bool alt_control_flag_;

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
  {
    Eigen::Matrix<double, 3, 4> projection_matrix_;
    projection_matrix_ << msg->P[0], msg->P[1], msg->P[2], msg->P[3],
      msg->P[4], msg->P[5], msg->P[6], msg->P[7],
      msg->P[8], msg->P[9], msg->P[10], msg->P[11];
    intrinsic_matrix_  = projection_matrix_.block<3,3>(0,0);
    intrinsic_matrix_inverse_ = intrinsic_matrix_.inverse();
    std::cout << "Intrinsic Matrix is :\n" << intrinsic_matrix_ << std::endl; 

    //param
    target_area_ = msg->height * msg->width * target_area_;
    target_y_ = msg->width * target_y_;
    target_z_ = msg->height * target_z_;
    thre_z_  = msg->height  * thre_z_; 
    thre_psi_  = msg->width / 2 * thre_psi_; //half of image frame

    camera_info_sub_.shutdown();
  }

  void trackerCallback(const jsk_recognition_msgs::RotatedRectStampedConstPtr& msg)
  {
    /* double pitch = estimator_->getStateTheta(); */
    /* double roll  = estimator_->getStatePhy(); */

    double pitch = (double)tracker_->getTheta();
    double roll  = tracker_->getPhy();

    //IMPORTANT : We need transform system!! => TODO
    Eigen::Quaternion<double> q = Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitX()) 
      * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()) ;
    Eigen::Matrix3d rotation = q.matrix();

    Eigen::Matrix<double, 3, 1> local_coord; 
    local_coord << (float)msg->rect.x, (float)msg->rect.y, 1;
    Eigen::Matrix<double, 3, 1> world_coord; 
    world_coord << intrinsic_matrix_ * rotation * intrinsic_matrix_inverse_ * local_coord;

    //std::cout << " intrinsic_matrix_ * rotation * intrinsic_matrix_inverse_ is :\n" << intrinsic_matrix_ * rotation * intrinsic_matrix_inverse_ << std::endl; 
    //std::cout << "world_coord is :\n" << world_coord << std::endl; 

    double x_dash, y_dash, ball_area;
    // x_dash = world_coord(0);
    // y_dash = world_coord(1);
    // ball_area = msg->rect.width * msg->rect.height;
    lpf_image_x_.filterFunction((double)world_coord(0), x_dash);
    lpf_image_y_.filterFunction((double)world_coord(1), y_dash);
    lpf_image_area_.filterFunction((double)(msg->rect.width * msg->rect.height), ball_area);

    geometry_msgs::Vector3Stamped world_cam_coord;
    world_cam_coord.header.stamp = msg->header.stamp;
    //world_cam_coord.vector.x = world_coord(0);
    //world_cam_coord.vector.y = world_coord(1);
    //world_cam_coord.vector.z = local_coord(1);
    world_cam_coord.vector.x = x_dash;
    world_cam_coord.vector.y = y_dash;
    world_cam_coord.vector.z = ball_area;
    world_cam_pub_.publish(world_cam_coord);

    //tracking process using vel control
    aerial_robot_base::FlightNav navi_command;
    navi_command.header.stamp = msg->header.stamp;
    navi_command.command_mode = aerial_robot_base::FlightNav::VEL_FLIGHT_MODE_COMMAND;

    //* x
    float dif_area =  - (ball_area - target_area_) / target_area_;
    float target_vel_x = 0;
    if(dif_area > 0) // forward
      target_vel_x = dif_area * gain_forward_x_;
    else
      target_vel_x = dif_area * gain_backward_x_;
    //navigator_->setTargetVelX(target_vel_x);
    navi_command.target_vel_x = target_vel_x;

    //* z
    navi_command.pos_z_navi_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;

    int dif_z = - (y_dash - target_z_);
    //TODO: should add the factor of area of ball, 
    //      same with the alt control func of joy stick navigator.
    float target_dif_pos_z = dif_z / abs(dif_z) * gain_z_;
    float pos_z = tracker_->getPosZ();
    if(abs(dif_z) > thre_z_)
      {
        navi_command.pos_z_navi_mode = aerial_robot_base::FlightNav::VEL_FLIGHT_MODE_COMMAND;
        navi_command.target_pos_diff_z = target_dif_pos_z;
        alt_control_flag_ = true;
      }
    else
      {
        if(alt_control_flag_)
          {
            alt_control_flag_ = false;
            navi_command.pos_z_navi_mode = aerial_robot_base::FlightNav::POS_FLIGHT_MODE_COMMAND;
            navi_command.target_pos_z = pos_z;
            //navigator_->setTargetPosZ(estimator_->getStatePosZ());
          }
      }
    if(pos_z < 0.35) 
      {
        navi_command.pos_z_navi_mode = aerial_robot_base::FlightNav::POS_FLIGHT_MODE_COMMAND;
        navi_command.target_pos_z = 0.35;
        //navigator_->setTargetPosZ(0.35);
      }
    if(pos_z > 3) 
      {
        navi_command.pos_z_navi_mode = aerial_robot_base::FlightNav::POS_FLIGHT_MODE_COMMAND;
        navi_command.target_pos_z = 3;
        // navigator_->setTargetPosZ(3);
      }

    //* y & psi
    int dif_y = - (x_dash - target_y_);
    float target_vel_y = dif_y / target_y_ * gain_y_;
    int dif_psi = abs(dif_y) - thre_psi_;
    float target_psi = dif_y * gain_psi_;
    if(dif_psi > 0)
      {
        navi_command.target_psi = target_psi; //bad-> should be velocity control
        navi_command.target_vel_y = 0; //bad-> should be velocity control
        /* navigator_->setTargetPsi(target_psi); */
        /* navigator_->setTargetVelY(0); */
      }
    else 
      {
        navi_command.target_psi = 0; //bad-> should be velocity control
        navi_command.target_vel_y = target_vel_y; 
        /* navigator_->setTargetPsi(0); */
        /* navigator_->setTargetVelY(target_vel_y); */
      }

    tracker_->navigation(navi_command);
  }

  void startTracking()
  {
    image_processing::StartTracking srv;
    srv.request.tracking_req = true;
    if(start_tracking_client_.call(srv))
      {
        if(srv.response.tracking_res)
          ROS_INFO("start tracking from jsk_quadcopter");
      }
    else
      {
        ROS_ERROR("Filaed to call service");
      }
  }


  void rosParamInit()
  {
    std::string ns = nh_private_.getNamespace();

    //x
    if (!nh_private_.getParam ("target_area", target_area_))
      target_area_ = 0.05; // 125 * 125 / (640 * 480)  => 1.5m
    printf("%s: target_area_ is %.3f\n", ns.c_str(), target_area_);

    if (!nh_private_.getParam ("gain_backward_x", gain_backward_x_))
      gain_backward_x_ = 0.3; //old: 0.2
    printf("%s: gain_backward_x_ is %.3f\n", ns.c_str(), gain_backward_x_);

    if (!nh_private_.getParam ("gain_forward_x", gain_forward_x_))
      gain_forward_x_ = 0.4; //old: 0.2
    printf("%s: gain_forward_x_ is %.3f\n", ns.c_str(), gain_forward_x_);


    //y
    if (!nh_private_.getParam ("target_y", target_y_))
      target_y_ = 0.5;
    printf("%s: target_y_ is %.3f\n", ns.c_str(), target_y_);

    if (!nh_private_.getParam ("gain_y", gain_y_))
      gain_y_ = 0.6; //old: 0.4
    printf("%s: gain_y_ is %.3f\n", ns.c_str(), gain_y_);

    //z
    if (!nh_private_.getParam ("target_z", target_z_))
      target_z_ = 0.5;
    printf("%s: target_z_ is %.3f\n", ns.c_str(), target_z_);

    if (!nh_private_.getParam ("gain_z", gain_z_))
      gain_z_ = 0.008; //old 0.004
    printf("%s: gain_z_ is %.3f\n", ns.c_str(), gain_z_);

    if (!nh_private_.getParam ("thre_z", thre_z_))
      thre_z_ = 0.25;
    printf("%s: thre_z_ is %.3f\n", ns.c_str(), thre_z_);

    //psi
    if (!nh_private_.getParam ("thre_psi", thre_psi_))
      thre_psi_ = 0.6;
    printf("%s: thre_psi_ is %.3f\n", ns.c_str(), thre_psi_);

    if (!nh_private_.getParam ("gain_psi", gain_psi_))
      gain_psi_ = 0.35;
    printf("%s: gain_psi_ is %.3f\n", ns.c_str(), gain_psi_);

    nh_private_.param("cam_info_topic", cam_info_topic_, std::string("/camera/camera_info"));
    printf("%s: cam_info_topic_ is %s\n", ns.c_str(), cam_info_topic_.c_str());
    nh_private_.param("rx_freq", rx_freq_, 30.0);
    nh_private_.param("cutoff_freq", cutoff_freq_, 5.0);

  }
  
};


#endif



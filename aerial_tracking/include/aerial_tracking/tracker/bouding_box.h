#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <ros/ros.h>
#include <aerial_tracking/basic_tracking.h>
#include <aerial_robot_base/digital_filter.h>
#include <aerial_tracking/basic_tracking.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_processing/RotatedRectStamped.h>
#include <image_processing/StartTracking.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


class BoundingBox
{
 public:

 BoundingBox(ros::NodeHandle nh, ros::NodeHandle nh_private, BasicTracking* tracker)
   : nh_(nh), nh_private_(nh_private, "bounding_box")
    {
      rosParamInit();

      tracker_ = tracker;

      bounding_box_sub_ = nh_.subscribe<image_processing::RotatedRectStamped>("result", 1, &BoundingBox::trackerCallback, this, ros::TransportHints().tcpNoDelay());

      camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>(cam_info_topic_, 1, &BoundingBox::cameraInfoCallback, this, ros::TransportHints().tcpNoDelay());

      world_cam_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("world_cam_coord", 1); 

      lpf_image_x_ =  IirFilter((float)rx_freq_, (float)cutoff_freq_);
      lpf_image_y_ =  IirFilter((float)rx_freq_, (float)cutoff_freq_);
      lpf_image_area_ =  IirFilter((float)rx_freq_, (float)cutoff_freq_);

      alt_control_flag_ = false;
      start_tracking_flag_ = false;
 }

  ~BoundingBox()
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
      navi_command.pos_z_navi_mode = aerial_robot_base::FlightNav::POS_FLIGHT_MODE_COMMAND;
      navi_command.target_pos_z = tracker_->getPosZ();;

      //tracker_->navi_pub_.publish(navi_command);
      tracker_->navigation(navi_command);

      bounding_box_sub_.shutdown();
      //start_tracking_client_.shutdown();
      ROS_WARN("shutdown the once bounding box tracker");
    }

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber bounding_box_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Publisher world_cam_pub_;
  ros::Publisher navi_pub_;
  ros::Subscriber state_sub_;


  //ros::ServiceClient start_tracking_client_;
  BasicTracking* tracker_;

  IirFilter lpf_image_x_, lpf_image_y_, lpf_image_area_;
  double rx_freq_, cutoff_freq_;

  Eigen::Matrix3d intrinsic_r_matrix_, r_intrinsic_inverse_matrix_;

  double target_area_;
  double target_y_;
  double target_z_;
  double thre_z_;
  double thre_psi_;
  double z_real_lower_pos_thre_;
  double z_real_upper_pos_thre_;

  double gain_forward_x_;
  double gain_backward_x_;
  double gain_y_;
  double gain_z_;
  double gain_psi_;
  std::string cam_info_topic_;

  bool alt_control_flag_;

  bool start_tracking_flag_;

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
  {
    Eigen::Matrix<double, 3, 4> projection_matrix_;
    projection_matrix_ << msg->P[0], msg->P[1], msg->P[2], msg->P[3],
      msg->P[4], msg->P[5], msg->P[6], msg->P[7],
      msg->P[8], msg->P[9], msg->P[10], msg->P[11];

    Eigen::Matrix<double, 3, 3> r_zero;
    r_zero << 0, -1, 0, 0, 0, -1, 1, 0, 0;

    /* intrinsic_r_matrix_  = projection_matrix_.block<3,3>(0,0) * r_zero; */
    /* r_intrinsic_inverse_matrix_ =  r_zero.inverse() * intrinsic_r_matrix_.inverse(); */
    intrinsic_r_matrix_  = projection_matrix_.block<3,3>(0,0);
    r_intrinsic_inverse_matrix_ =  intrinsic_r_matrix_.inverse();

    std::cout << "Intrinsic Matrix is :\n" << intrinsic_r_matrix_ << std::endl; 


    //param
    //target_area_ = msg->height * msg->width * target_area_;
    target_y_ = msg->width * target_y_;
    target_z_ = msg->height * target_z_;
    thre_z_  = msg->height  * thre_z_; 
    thre_psi_  = msg->width / 2 * thre_psi_; //half of image frame

    ROS_INFO("target_y_: %f, target_z_: %f", target_y_, target_z_);

    start_tracking_flag_ = true;

    camera_info_sub_.shutdown();

  }

  void trackerCallback(const image_processing::RotatedRectStampedConstPtr& msg)
  {
    static bool first_flag = true;

    if(!start_tracking_flag_) return;

    /* double pitch = estimator_->getStateTheta(); */
    /* double roll  = estimator_->getStatePhy(); */
    double pitch = (double)tracker_->getTheta();
    double roll  = tracker_->getPhy();

    //IMPORTANT : We need transform system!! => TODO
#if 1
    Eigen::Quaternion<double> q = Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitX()) 
      * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()) ;
#else
    Eigen::Quaternion<double> q = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) 
      * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) ;
#endif

    Eigen::Matrix3d rotation = q.matrix();

    Eigen::Matrix<double, 3, 1> local_coord; 
    local_coord << (float)msg->x, (float)msg->y, 1;
    Eigen::Matrix<double, 3, 1> temp_coord; 
    temp_coord << intrinsic_r_matrix_ * rotation * r_intrinsic_inverse_matrix_ * local_coord;
    float scale = temp_coord(2);
    Eigen::Matrix<double, 3, 1> world_coord; 
    world_coord = temp_coord / scale;

    //std::cout << " intrinsic_r_matrix_ * rotation * intrinsic_r_matrix_inverse_ is :\n" << intrinsic_r_matrix_ * rotation * r_intrinsic_inverse_matrix_ << std::endl;
    //std::cout << "rotation is :\n" << rotation << std::endl;  
    //std::cout << "world_coord is :\n" << world_coord << std::endl; 
    //std::cout << "local_coord is :\n" << local_coord << std::endl; 

    if(first_flag) 
      {
        target_area_ = msg->width * msg->height;
        lpf_image_x_.setPosInitState((double)world_coord(0));
        lpf_image_y_.setPosInitState((double)world_coord(1));
        lpf_image_area_.setPosInitState((double)target_area_);
        first_flag = false;
        return;
      }


    double x_dash, y_dash, ball_area;
    // x_dash = world_coord(0);
    // y_dash = world_coord(1);
    // ball_area = msg->width * msg->height;
    lpf_image_x_.filterFunction((double)world_coord(0), x_dash);
    lpf_image_y_.filterFunction((double)world_coord(1), y_dash);
    lpf_image_area_.filterFunction((double)(msg->width * msg->height), ball_area);


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

    double dif_z =  -(y_dash - target_z_); //sign should be opposite, according to the camera  coord
    //ROS_INFO("ok1-5: dif_z:%f, y_dash:%lf, target_z:%lf", dif_z, y_dash, target_z_);
    //TODO: should add the factor of area of ball, 
    //      same with the alt control func of joy stick navigator.
    float target_dif_pos_z =  dif_z / fabs(dif_z) * gain_z_;
    float pos_z = tracker_->getPosZ();


    //ROS_INFO("navi_command.target_pos_diff_z :%f", target_dif_pos_z);
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

    if(pos_z < z_real_lower_pos_thre_) 
      {
        navi_command.pos_z_navi_mode = aerial_robot_base::FlightNav::POS_FLIGHT_MODE_COMMAND;
        navi_command.target_pos_z = z_real_lower_pos_thre_;
        //navigator_->setTargetPosZ(0.35);
      }
    if(pos_z > z_real_upper_pos_thre_) 
      {
        navi_command.pos_z_navi_mode = aerial_robot_base::FlightNav::POS_FLIGHT_MODE_COMMAND;
        navi_command.target_pos_z = z_real_upper_pos_thre_;
        // navigator_->setTargetPosZ(3);
      }

    //* y & psi
    int dif_y = - (x_dash - target_y_);

    //ROS_INFO("x:%f, y:%f, aera:%f", x_dash, y_dash, ball_area);
    float target_vel_y = dif_y / target_y_ * gain_y_;
    int dif_psi = abs(dif_y) - thre_psi_;
    float target_psi = dif_y /target_y_ * gain_psi_;
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
      thre_z_ = 0.2;
    printf("%s: thre_z_ is %.3f\n", ns.c_str(), thre_z_);

    if (!nh_private_.getParam ("z_real_lower_pos_thre", z_real_lower_pos_thre_))
      z_real_lower_pos_thre_ = 0.35;
    printf("%s: z_real_lower_pos_thre_ is %.3f\n", ns.c_str(), z_real_lower_pos_thre_);

    if (!nh_private_.getParam ("z_real_upper_pos_thre", z_real_upper_pos_thre_))
      z_real_upper_pos_thre_ = 3.0;
    printf("%s: z_real_upper_pos_thre_ is %.3f\n", ns.c_str(), z_real_upper_pos_thre_);


    //psi
    if (!nh_private_.getParam ("thre_psi", thre_psi_))
      thre_psi_ = 0.6;
    printf("%s: thre_psi_ is %.3f\n", ns.c_str(), thre_psi_);

    if (!nh_private_.getParam ("gain_psi", gain_psi_))
      gain_psi_ = 0.4;
    printf("%s: gain_psi_ is %.3f\n", ns.c_str(), gain_psi_);

    nh_private_.param("cam_info_topic", cam_info_topic_, std::string("/camera/camera_info"));
    printf("%s: cam_info_topic_ is %s\n", ns.c_str(), cam_info_topic_.c_str());

    //for IIR filter
    nh_private_.param("rx_freq", rx_freq_, 30.0); //cam hz
    nh_private_.param("cutoff_freq", cutoff_freq_, 5.0); //cutoff hz

  }

};

#endif



#include "aerial_robot_base/aerial_robot_base.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AerialRobotBase
// Class for creating basic methods that are common to all aerial robots and are used in the main loop.
// These methods are connected to the robot's methods by ROS's plugin service '<pluginlib/class_loader.h>',
// which allows the use of different implementations to be used interchangably at runtime.
// The main loop is responsible for updating the navigation and control of the robot.
////////////////////////////////////////////////////////////////////////////////////////////////////////////

AerialRobotBase::AerialRobotBase(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nhp_(nh_private), callback_spinner_(4), main_loop_spinner_(1, &main_loop_queue_),    // Use 4 threads for callback_spinner_ and 1 thread for main_loop_spinner_
    navigator_loader_("aerial_robot_control", "aerial_robot_navigation::BaseNavigator"),
    controller_loader_("aerial_robot_control", "aerial_robot_control::ControlBase")
{

    nhp_.param ("param_verbose", param_verbose_, true);
    nhp_.param ("main_rate", main_rate_, 0.0);
    double main_loop_dt = 1 / main_rate_;

    // Create Robot model
    robot_model_ros_ = boost::make_shared<aerial_robot_model::RobotModelRos>(nh_, nhp_);
    auto robot_model = robot_model_ros_->getRobotModel();

    // Initialize Estimator
    estimator_ = boost::make_shared<aerial_robot_estimation::StateEstimator>();
    estimator_->initialize(nh_, nhp_, robot_model);

    // Initialize Navigator
    std::string navi_plugin_name;
    if(nh_.getParam("flight_navigation_plugin_name", navi_plugin_name))
    {
        try
        {
            navigator_ = navigator_loader_.createInstance(navi_plugin_name);
        }
        catch(pluginlib::PluginlibException& ex)
        {
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }
    }
    else
    {
        ROS_DEBUG("[Using default class for flight navigation: aerial_robot_navigation::BaseNavigator...]");
        navigator_ = boost::make_shared<aerial_robot_navigation::BaseNavigator>();
    }
    
    if(!navigator_) 
    {
        ROS_ERROR("Failed to create navigator plugin.");
    }
    
    navigator_->initialize(nh_, nhp_, robot_model, estimator_, main_loop_dt);

    // Initialize Controller
    std::string aerial_robot_control_name;
    if(nh_.getParam("aerial_robot_control_name", aerial_robot_control_name))
        {
            try
                {
                    controller_ = controller_loader_.createInstance(aerial_robot_control_name);
                }
            catch(pluginlib::PluginlibException& ex)
                {
                    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
                }
        }
    else
        {
            ROS_DEBUG("[Using default class for control: aerial_robot_control::ControlBase...]");
            aerial_robot_control_name = "aerial_robot_control/flatness_pid";
            controller_ = controller_loader_.createInstance(aerial_robot_control_name);
        }
    if(!controller_) {
        ROS_ERROR("Failed to create controller plugin.");
    }
    controller_->initialize(nh_, nhp_, robot_model, estimator_, navigator_, main_loop_dt);

    // Additional information
    if(param_verbose_) cout << nhp_.getNamespace() << ": main loop rate is " << main_rate_ << endl;
    if(main_rate_ <= 0) ROS_ERROR_STREAM("Main rate is negative, cannot run main timer!");
    
    // Threading
    /* Note 1: Separate the thread for main control (including navigation) loop to guarantee a relatively stable loop rate */
    ros::TimerOptions ops(ros::Duration(main_loop_dt),
                          boost::bind(&AerialRobotBase::mainFunc, this, _1),
                          &main_loop_queue_);
    main_timer_ = nhp_.createTimer(ops);
    main_loop_spinner_.start();

    /* Note 2: callback_spinner_ calls following items with 4 threads
    - all subscribers (joint state for robot model, sensor for state estimation, uav/nav for navigation)
    - statePublish timer in state estimator for publish odometry and tf
    - service server */
    callback_spinner_.start();
}

AerialRobotBase::~AerialRobotBase()
{
    // Stop manually to avoid following error (message)
    // Terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
    // What():    boost: mutex lock failed in pthread_mutex_lock: Invalid argument
    main_timer_.stop();
    main_loop_spinner_.stop();
    callback_spinner_.stop();
}

// Function that manages every computation loop
// Main loop function that triggers through the ROS timer
void AerialRobotBase::mainFunc(const ros::TimerEvent & e)
{
    // Check if control loop is running on time
    if (!e.last_real.isZero())
    {
            double dt_real = e.current_real.toSec() - e.last_real.toSec();
            double tolerance = 0.05;    // 5% tolerance. This value is set based on experience.
            double dt_desire = (1.0 + tolerance) * 1.0 / main_rate_;
            if (dt_real > dt_desire)
            {
                    ROS_WARN("Control loop running behind schedule! Actual main loop rate is too low: (ts_real) %f s > (ts_desire with %2f%% tolerance) %f s ", 
                              dt_real, tolerance * 100, dt_desire);
            }
    }

    // Update the robot's navigation and control
    navigator_->update();
    controller_->update();
}
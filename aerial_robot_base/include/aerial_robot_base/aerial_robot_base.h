#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <pluginlib/class_loader.h>
#include "aerial_robot_model/model/aerial_robot_model_ros.h"
#include "aerial_robot_estimation/state_estimation.h"
#include "aerial_robot_control/flight_navigation.h"
#include "aerial_robot_control/control/base/base.h"

class AerialRobotBase
{
    public:
        AerialRobotBase(ros::NodeHandle nh, ros::NodeHandle nh_private);
        ~AerialRobotBase();

        void mainFunc(const ros::TimerEvent & e);

    private:
        bool param_verbose_;
        double main_rate_;

        ros::NodeHandle nh_;
        ros::NodeHandle nhp_;
        ros::Timer main_timer_;

        boost::shared_ptr<aerial_robot_model::RobotModelRos> robot_model_ros_;
        boost::shared_ptr<aerial_robot_estimation::StateEstimator>  estimator_;

        // Dynamically load classes without having to explicitly link against them at compile time. 
        // Enables changing functionality without modifying existing code.
        // Smart pointers will eventually hold instances of concrete implementations loaded through these loaders.
        pluginlib::ClassLoader<aerial_robot_navigation::BaseNavigator> navigator_loader_;
        boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator_;

        pluginlib::ClassLoader<aerial_robot_control::ControlBase> controller_loader_;
        boost::shared_ptr<aerial_robot_control::ControlBase> controller_;

        ros::CallbackQueue main_loop_queue_;
        ros::AsyncSpinner callback_spinner_;  // Use 4 threads
        ros::AsyncSpinner main_loop_spinner_; // Use 1 threads
};

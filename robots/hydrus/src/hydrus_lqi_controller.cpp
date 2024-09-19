#include <hydrus/hydrus_lqi_controller.h>

using namespace aerial_robot_control;

HydrusLQIController::HydrusLQIController():
  UnderActuatedLQIController()
{
}


void HydrusLQIController::initialize(ros::NodeHandle nh,
                                     ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  UnderActuatedLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
}

bool HydrusLQIController::checkRobotModel()
{
  boost::shared_ptr<HydrusRobotModel> hydrus_robot_model = boost::dynamic_pointer_cast<HydrusRobotModel>(robot_model_);
  lqi_mode_ = hydrus_robot_model->getWrenchDof();

  if(!robot_model_->initialized())
    {
      ROS_DEBUG_NAMED("LQI gain generator", "LQI gain generator: robot model is not initiliazed");
      return false;
    }

  if(!robot_model_->stabilityCheck(verbose_))
    {
      ROS_ERROR_NAMED("LQI gain generator", "LQI gain generator: invalid pose, stability is invalid");
      if(hydrus_robot_model->getWrenchDof() == 4 && hydrus_robot_model->getFeasibleControlRollPitchMin() > hydrus_robot_model->getFeasibleControlRollPitchMinThre())
        {
          ROS_WARN_NAMED("LQI gain generator", "LQI gain generator: change to three axis stable mode");
          lqi_mode_ = 3;
          return true;
        }

      return false;
    }
  return true;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusLQIController, aerial_robot_control::ControlBase);

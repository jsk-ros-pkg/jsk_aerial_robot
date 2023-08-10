#include <ros/ros.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <aerial_robot_simulation/spinal_interface.h>
#include <flight_control/flight_control.h>

class MujocoAttitudeController
{
public:
  MujocoAttitudeController();
  ~MujocoAttitudeController() {}

  bool init();
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);


private:
  boost::shared_ptr<FlightControl> controller_core_;
};

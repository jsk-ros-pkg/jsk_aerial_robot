#include <aerial_robot_model/numerical_jacobians.h>
#include <hydrus_xi/hydrus_xi_fully_actuated_robot_model.h>
#include <gtest/gtest.h>

using namespace aerial_robot_model;

class JacobianTest : public testing::Test
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::AsyncSpinner* spinner_;

  NumericalJacobian* numerical_sovler_;

  virtual void SetUp()
  {
    ::testing::Test::SetUp();

    nhp_ = ros::NodeHandle ("~");
    spinner_ = new ros::AsyncSpinner(0);

    numerical_sovler_ = new NumericalJacobian(nh_, nhp_, std::move(std::make_unique<HydrusXiFullyActuatedRobotModel>(true)));

    spinner_->start();
  }

  virtual void TearDown()
  {
    delete numerical_sovler_;
    ros::shutdown();
    delete spinner_;
    ::testing::Test::TearDown();
  }


  bool checkJacobians()
  {
    while(!numerical_sovler_->getInitialized())
      {
        ROS_WARN("wait");
        ros::Duration(1.0).sleep();
        continue;
      }

    return numerical_sovler_->checkJacobians();
  }
};

TEST_F(JacobianTest, CheckJacobians)
{
  ASSERT_TRUE(checkJacobians());
}


int main(int argc, char **argv)
{
  ros::init (argc, argv, "jacobian_test");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  bool rostest;
  nhp.param("rostest", rostest, true);

  if(rostest)
    {
      testing::InitGoogleTest(&argc, argv);
      return RUN_ALL_TESTS();
    }
  else
    {
      NumericalJacobian numerical_sovler(nh, nhp, std::move(std::make_unique<HydrusXiFullyActuatedRobotModel>(true)));
      ros::spin();
    }
}



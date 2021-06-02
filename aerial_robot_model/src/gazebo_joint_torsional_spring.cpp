#ifndef _TORTIONAL_SPRING_PLIGIN_CPP_
#define _TORTIONAL_SPRING_PLIGIN_CPP_ value

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class TorsionalSpringPlugin : public ModelPlugin
  {
    public: TorsionalSpringPlugin() {}

    private:
      physics::ModelPtr model;
      sdf::ElementPtr sdf;

      physics::JointPtr joint;

      // Set point
      double setPoint;

      // Spring constant
      double kx;

      // Pointer to update event connection
      event::ConnectionPtr updateConnection;

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        // Safety check
        if (_model->GetJointCount() == 0)
        {
          gzerr << "You have zero joints! Something is wrong! Not loading plugin." << std::endl;
          return;
        }

        // Store model pointer
        this->model = _model;

        // Store the SDF pointer
        this->sdf = _sdf;

        if (_sdf->HasElement("joint"))
          this->joint = _model->GetJoint(_sdf->Get<std::string>("joint"));
        else
          gzerr << "Must specify joint to apply a torsional spring at!\n";

        this->kx = 0.0;
        if (_sdf->HasElement("kx"))
          this->kx = _sdf->Get<double>("kx");
        else
          gzerr << "Torsional spring coefficient not specified! Defaulting to: " << this->kx << std::endl;

        this->setPoint = 0.0;

        if (_sdf->HasElement("set_point"))
          this->setPoint = _sdf->Get<double>("set_point");
        else
          gzerr << "Set point not specified! Defaulting to: %f\n" << this->setPoint << std::endl;

        // Listen to update event
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind (&TorsionalSpringPlugin::OnUpdate, this) );

        gzdbg << "Loaded gazebo_joint_torsional_spring." << std::endl;
      }

    protected: void OnUpdate()
      {
        double current_angle = this->joint->Position();
        this->joint->SetForce(0, this->kx*(this->setPoint-current_angle));			
        //gzdbg << "Applying force:" << this->kx*(this->setPoint-current_angle) << std::endl;
      }

  };

  GZ_REGISTER_MODEL_PLUGIN(TorsionalSpringPlugin)
}
#endif /* ifndef _TORTIONAL_SPRING_PLIGIN_CPP_ */
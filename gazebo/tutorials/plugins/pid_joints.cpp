#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/PID.hh"

namespace gazebo
{
  class PIDJoints : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      this->model_ = _model;
      // initialize a PID class
      this->target_position_ = 1.8;
      this->pid.Init(100, 0, 1, 0, 0, 100, -100);
      this->pid.SetCmd(this->target_position_);
      this->joint_ = this->model_->GetJoint("my_joint");
      this->last_update_time_ = this->model_->GetWorld()->GetSimTime();
      this->update_connection_ = event::Events::ConnectWorldUpdateStart(
        boost::bind(&PIDJoints::UpdatePID, this));
    }
    void UpdatePID()
    {
      common::Time current_time = this->model_->GetWorld()->GetSimTime();
      double error = this->joint_->GetAngle(0).Radian()
                   - target_position_;
      double dt    = current_time.Double()
                   - this->last_update_time_.Double();
      this->pid.Update(error, dt);
      // Change from torque to velocity, see what happens
      this->joint_->SetForce(0, this->pid.GetCmd());
      this->last_update_time_ = current_time;
      gzdbg << "error [" << error
            << "] cmd [" << this->pid.GetCmd() << "]\n";
    }
    common::PID pid;
    double target_position_;
    physics::JointPtr joint_;
    physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;
    common::Time last_update_time_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PIDJoints)
}

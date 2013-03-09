/**
 * @file foreleg_jointControl.h
 */

#ifndef _FORELEG_JOINTCONTROL_H_
#define _FORELEG_JOINTCONTROL_H_

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <controls/controlBundle.h>
#include <gaiter/gaiter.h>

namespace gazebo {

  /**
   * @class foreleg_jointControl
   */
  class foreleg_jointControl : public ModelPlugin
  {
  public:
    foreleg_jointControl();
    ~foreleg_jointControl();
    
    void Load( physics::ModelPtr _parent, 
	       sdf::ElementPtr _sdf );

    void SetInitialPose();
    void SetFootParallelToFloor();
    void CalculateBoundaryValues();
    void UpdateStates();

  private:
    physics::WorldPtr mWorld;
    physics::ModelPtr mModel;
    
    int mNumActuatedJoints;
    std::vector<std::string> mActuatedJointNames;
    std::vector<physics::JointPtr> mActuatedJoints;

    controlBundle mCb;
    gaiter mG;

    common::Time mLastUpdateTime;

    boost::mutex mUpdate_Mutex;

    // Pointer to the update event connection
    event::ConnectionPtr mUpdateConnection;
  };

}

#endif /** _FORELEG_JOINTCONTROL_H_ */
